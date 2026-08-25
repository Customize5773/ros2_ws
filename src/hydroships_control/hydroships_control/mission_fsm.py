#!/usr/bin/env python3
"""mission_fsm — State machine misi ROV KKI 2026 (Milestone 6, ROS 2 native).

FSM hanya menetapkan target, stabilizer menahan kedalaman & heading otomatis.

Aliran (lihat docs/ARCHITECTURE.md):
    masuk : /hydroships/depth        (Float64, m >=0)  -> transisi state
            /hydroships/odom         (Odometry)        -> yaw (cek alignment)
            /hydroships/qr_result    (String A/B/C/D)  -> tentukan wall (M1)
            /hydroships/mission/start_autonomous (Empty) -> pilot trigger,
                                     dilepas SETELAH surface & lean di wall
    keluar: /hydroships/setpoint/depth   (Float64, negatif = dalam)
            /hydroships/setpoint/heading (Float64, rad)
            /hydroships/manual/cmd       (Twist, Fx/Fy gaya horizontal N)

Gripper: payload nempel ke ROV sejak spawn (gz-sim DetachableJoint plugin,
lihat hydroships_gazebo/models/payload/model.sdf). AUTO_RELEASE publish
Empty ke /hydroships/gripper/detach utk lepas — bukan service attach/detach.

Pembagian kredit skor (lihat aturan misi):
    GRAB, NAV_WALL, HANG, SURFACE = tugas REMOTELY (pilot kendali penuh)
    AUTO_RELEASE                  = tugas AUTONOMOUS (dipicu setelah pilot
                                     lean di dinding, FSM ambil alih sendiri)

State: IDLE -> DIVE -> APPROACH_QR -> DESCEND -> GRAB -> NAV_WALL -> HANG
       -> SURFACE -> WAIT_TRIGGER -> AUTO_RELEASE -> (DIVE lagi | DONE) (atau ABORT).

Catatan: butuh stabilizer + thruster_allocator + sim berjalan
(pakai hydroships_bringup/launch/hydroships_mission.launch.py).
"""

import math
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, String, Empty

from hydroships_control.hook_logic import HookServoGains, hook_ey_target, hook_servo, update_dwell
# qr_ey_target dipindah ke qr_logic (modul murni) supaya gripper_controller
# memakai geometri yang SAMA PERSIS untuk gerbang attach-nya — sebelumnya FSM
# membidik ey_target~-0.52 sementara GripperLogic.is_safe() menuntut |ey|<=0.30,
# dua kriteria yang mustahil dipenuhi bersamaan sehingga attach tak pernah
# terpicu. Di-re-export di sini agar `mission_fsm.qr_ey_target` tetap resolve
# (dipakai test/test_qr_ey_target.py dan reduce_approach_qr.py).
from hydroships_control.qr_logic import qr_ey_target  # noqa: F401 (re-export)
from hydroships_control.stabilizer import roll_pitch_from_quaternion


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_to_pi(a):
    return math.atan2(math.sin(a), math.cos(a))




WALL_HEADING_DEG = {'A': 270.0, 'B': 90.0, 'C': 0.0, 'D': 180.0}


class St(Enum):
    IDLE = auto(); DIVE = auto(); APPROACH_QR = auto(); DESCEND = auto(); GRAB = auto()
    NAV_WALL = auto(); HANG = auto(); SURFACE = auto(); WAIT_TRIGGER = auto()
    LEAN_RECORD = auto(); REVERSE_RETURN = auto()
    APPROACH_HOOK = auto(); AUTO_RELEASE = auto(); DONE = auto(); ABORT = auto()


class MissionFSM(Node):
    def __init__(self):
        super().__init__('mission_fsm')
        p = self.declare_parameter
        p('start_state', 'DIVE')
        p('wall_order', 'random')   # 'ABCD', 'DCBA', atau 'random'
        p('start_delay', 3.0)
        p('start_wall', '')          # override manual utk testing start_state=NAV_WALL/HANG/dst
        p('surge_force', 25.0)       # N gaya maju horizontal
        p('depth_bottom', 0.70)      # m kedalaman dasar
        p('depth_surface', 0.08)     # m ambang "di permukaan"
        p('depth_tol', 0.06)         # m toleransi kedalaman
        # R-10 (P1-OWNER-DECISIONS-AND-ROADMAP.md): exit DESCEND terpisah dari
        # depth_tol (dipakai APPROACH_QR) -- depth_tol=0.06 memakan hampir seluruh
        # celah rancangan 0.034m antara dasar gripper & lantai QR, alt_gap jadi
        # cuma 5-7mm dari max_alt_gap=0.08. Toleransi lebih ketat di sini turun
        # ROV lebih dekat ke grab_depth sebenarnya sebelum trigger GRAB.
        p('descend_depth_tol', 0.02)  # m toleransi kedalaman KHUSUS exit DESCEND
        p('descend_settle_dwell', 0.5)  # s dwell depth_ok sblm GRAB (redam overshoot turun)
        # R-11 Opsi 2: saat depth_ok tercapai di DESCEND, beri waktu tambahan
        # `descend_recenter_timeout` supaya servo visual memperbaiki offset
        # QR sebelum GRAB — hanya bila qr_off masih segar (camera masih melihat
        # QR). Jika stale (QR hilang, normal di grab_depth), lanjutkan GRAB.
        p('descend_recenter_timeout', 5.0)
        # hook_depth = kedalaman BASE saat plat PAYLOAD bersandar DI PALANG BAWAH
        # hook (z -0.45, top -0.4375). Plat dibawa HORIZONTAL: link origin payload
        # = z base - 0.13 (teleport offset z=-0.13, gripper di dasar haluan),
        # underside plat di z base -0.136. Tip (silinder tegak r=0.0125,
        # z -0.45..-0.33) menembus lubang plat tanpa mentok (passage 5 cm di
        # body_collision — collision lama yg menutup tip SUDAH dibuang). Jadi plat
        # turun bebas sepanjang tip sampai underside menyentuh palang (depth
        # positif ke bawah: -depth - 0.136 = -0.4375 -> depth = 0.3015 ~ 0.30).
        # hook_depth 0.32 menekan plat SEATED ke palang (0.30 + margin tekanan
        # 0.02, analog 0.4315+0.0185=0.45 jaman offset z=0). JANGAN kecilkan:
        # descent berhenti di atas palang -> plat HOVER di tip -> "stall" di
        # AUTO_RELEASE cuma konvergensi depth hold (bukan terblok) -> detach di
        # udara -> jatuh.
        p('hook_depth', 0.32)        # m kedalaman base saat plat seated di palang
        # HANG presisi: target = LUBANG payload di atas TIP hook, bukan standoff
        # lama wall_dist-hook_dist (~0.5 m dari hook) yang membuat payload tak
        # pernah menyentuh hook. Geometri arena: muka dinding di wall_face;
        # ujung hook (tip, silinder tegak z -0.45..-0.33) di wall_face - hang_tip_d
        # dari pusat; lubang payload hang_hole_dx DI DEPAN base_link (gripper
        # 0.18 + tengah lubang di plat 0.0933). hang_approach_depth harus DI ATAS
        # puncak tip (-0.33) supaya ROV bisa memosisikan lubang lalu TURUN
        # menembus tip (bukan menyodok tip dari samping).
        p('wall_face', 2.5)          # m jarak muka dinding dari pusat arena
        p('hang_tip_d', 0.14)        # m jarak tip hook dari muka dinding
        p('hang_hole_dx', 0.2733)    # m base_link -> pusat lubang payload
        # hang_approach_depth DI ATAS hook_depth - gate_turun (0.32-0.02=0.30)
        # supaya gate kedalaman turun tak langsung lolos saat masih di approach.
        # Dgn offset z=-0.13, lubang di base_z-0.13: di atas tip (-0.33) ->
        # -depth-0.13 > -0.33 -> depth < 0.20. 0.14 (0.27-0.13) celah 0.06 m.
        p('hang_approach_depth', 0.14)  # m kedalaman posisi lubang di atas tip
        # Toleransi posisi lubang di atas tip. Diukur dari run nyata: ROV sering
        # mandek ~21 mm dari target (gaya sway sebagian terserap kopling yaw-hold)
        # dan heading hold menyisakan error ~7 deg di wall D (180 deg). Slot plat
        # dilebarkan ke x +-0.045 (toleransi +-28.5 mm vs r_tip 12.5), jadi
        # hang_tol 25 mm aman: tip masih menembus slot tanpa menyentuh dinding.
        p('hang_tol', 0.025)         # m toleransi posisi lubang di atas tip
        # hang_l_tol: toleransi arah MAJU (sepanjang sumbu ROV) — lubang hanya
        # ~50 mm di arah ini (tip Ø25) -> clearance ±12.5 mm. Dulu 8 mm
        # (err maju 25 mm -> tip mentok badan plat saat turun), tapi terukur
        # (run z=-0.13) residual 12.2 mm saat plat sudah SEATED dan tetap lolos
        # secara fisik (tip menembus lubang). 12 mm = dekat batas fisik ±12.5,
        # memberi margin utk variasi spawn tanpa macet lagi.
        p('hang_l_tol', 0.012)       # m toleransi lubang sepanjang sumbu maju
        p('hang_forward_bias', 0.018)  # m dorong maju ke wall (kompen residual 12-21mm) — supaya hook masuk lubang, tidak nyangkut
        # Gate heading sebelum turun: dengan kompensasi yaw live di _hang_xy,
        # error heading TIDAK lagi menggeser lubang dari tip (hanya memutar
        # slot sedikit, tip silinder tak peduli) — gate cukup utk memastikan
        # ROV menghadap wall secara wajar sebelum turun, bukan utk presisi.
        p('hang_yaw_tol_deg', 10.0)
        p('yaw_tol_deg', 10.0)       # derajat toleransi alignment heading
        p('qr_max_age', 1.5)         # s umur maks deteksi QR agar dianggap segar
        p('payload_x', 0.4)          # m posisi payload/QR di dunia (x)
        p('payload_y', 0.0)          # m posisi payload/QR di dunia (y)
        # P2-A: /hydroships/payload_pose adalah ground truth spawner -- di kontes
        # nyata tak ada topic ini. Kalau tak kunjung tiba, jangan macet sampai
        # t_dive habis lalu ABORT: fallback ke payload_x/y param + perlebar FOV
        # kamera bawah (pola sama dgn recovery QR-hilang di APPROACH_QR).
        p('t_payload_pose', 8.0)     # s tenggang tunggu payload_pose sblm fallback
        # [RESOLVED] QR detection: scan_depth 0.62 -> 0.46. Di 0.62 kamera bawah hanya
        # ~9cm di atas QR (world z=-0.893) -> QR 12cm MEMENUHI/melebihi frame, finder
        # bawah TER-CROP + gripper menutupi atas frame -> cv2.QRCodeDetector gagal
        # (pts=None). Di 0.46 kamera ~25cm di atas QR -> QR utuh + quiet-zone di frame,
        # terbaca 'A'..'D' (dibuktikan runtime: frame kamera bottom -> decode 'A').
        # [REVISI] 0.46 -> 0.30. Alasan: gripper 0.16 m di depan kamera bawah
        # (lihat cam_gripper_dx). Setengah-tinggi footprint kamera di lantai =
        # h_cam * 0.6293, dengan h_cam = |qr_floor_z| - depth - cam_bottom_dz.
        #   depth 0.46 -> h_cam 0.254 -> ½-tinggi 0.160 m -> ey_target = -1.00 (TEPI
        #     frame) => MUSTAHIL memusatkan gripper di atas QR sambil QR terlihat.
        #   depth 0.30 -> h_cam 0.414 -> ½-tinggi 0.261 m -> ey_target = -0.61 (aman),
        #     QR 12cm masih ~17% lebar frame (~111 px, jauh di atas ambang decode).
        # Arah perubahan ini MENJAUH dari QR (QR mengecil), jadi tidak mengulang
        # bug lama 0.62 di mana QR terlalu besar/ter-crop.
        p('scan_depth', 0.30)        # m kedalaman scan (kamera bawah ~41cm di atas QR)
        # M5-D (docs/STATUS.md): attach di GRAB dulu langsung dari scan_depth
        # mengelas ROV ke payload PADA POSE SAAT ITU (DetachableJoint tidak
        # menarik payload) -- ROV masih ~0.6 m di atas lantai QR saat itu, jadi
        # ROV terjangkar. DESCEND turun ke grab_depth sebelum GRAB memicu "close".
        #
        # Asal angka 0.70 (semua dari hydroships.urdf.xacro + rov_params.yaml,
        # dikunci test/test_grab_geometry.py — ubah salah satunya, test gagal):
        #   dasar gripper  = -grab_depth - 0.13 (joint z) - 0.03 (½ tinggi box)
        #                  = -0.86  -> 0.034 m DI ATAS bidang QR (-0.894). Celah
        #                     attach turun 0.60 -> 0.03 m.
        #   dasar collision hull = -grab_depth - 0.091 + 0.02 (cob.z) = -0.775
        #                  -> masih 0.12 m di atas lantai, tidak menabrak.
        # CATATAN: pada kedalaman ini kamera bawah (-0.18) berada di -0.88, praktis
        # sejajar bidang QR -> QR TIDAK terlihat lagi. Itu disengaja; gerbang attach
        # memakai latch "armed" dari APPROACH_QR (gripper_logic.arm_timeout), bukan
        # deteksi QR yang segar. XY di-hold dead-reckon selama turun.
        p('grab_depth', 0.70)        # m kedalaman base_link saat mencengkeram (TUNE)
        p('approach_kp', 90.0)       # N/m gain posisi XY -> gaya horizontal
        p('approach_kd', 140.0)       # N/(m/s) redaman kecepatan (cegah overshoot)
        p('approach_fmax', 16.0)     # N batas gaya approach
        p('approach_tol', 0.06)      # m radius "sudah di atas payload"
        p('wall_dist', 2.15)         # m jarak pusat->target wall (standoff; hook ~2.4 m)
        p('hook_dist', 0.30)         # m jarak target di depan hook (lebih dekat dari wall_dist)
        p('hook_lateral_offset', 0.0)  # m, koreksi geser samping ke hook (+/- sesuai arah)
        p('nav_tol', 0.20)           # m radius "tiba di wall/hook"
        p('nav_fmax', 22.0)          # N batas gaya navigasi holonomik
        p('hold_settle_s', 2.0)      # s harus tetap di dalam tol sebelum dianggap "stabil"
        # timeout per state (s)
        p('t_dive', 20.0); p('t_scan', 45.0); p('t_descend', 15.0)
        p('t_grab', 10.0); p('t_nav', 30.0)
        p('t_hang', 30.0); p('t_surface', 20.0); p('t_wait_trigger', 600.0)
        p('t_lean', 25.0); p('t_reverse', 35.0)
        p('t_release', 30.0); p('t_approach', 25.0)
        # AUTO_RELEASE fase-turun: retry terbatas bila plat duduk miring (drift
        # lateral saat turun) — naik, re-center, turun ulang sebelum ABORT.
        p('release_max_retries', 3)
        # APPROACH_HOOK: visual servo PD ke hook (hook_detector ->
        # /hydroships/hook_offset). Default sama dgn hook_logic.HookServoGains —
        # di sini hanya diekspos sebagai parameter ROS supaya bisa di-tune runtime.
        p('hook_max_age', 1.0)       # s umur maks deteksi hook agar dianggap segar
        p('hook_settle_grace_s', 0.4)  # s toleransi tick buruk sblm dwell APPROACH_HOOK direset
        p('hook_kp_surge', 40.0)     # N per satuan error ukuran-tampak
        p('hook_kd_surge', 30.0)     # N/(m/s) redaman surge
        p('hook_kp_sway', 45.0)      # N per satuan offset-x ternormalisasi
        p('hook_kd_sway', 30.0)      # N/(m/s) redaman sway
        p('hook_kp_depth', 0.25)     # m koreksi depth per satuan offset-y
        p('hook_size_stop', 0.35)    # size (sqrt(area)/lebar frame) dianggap "cukup dekat"
        p('hook_center_tol', 0.15)   # |ex|,|ey| dianggap "terpusat"
        p('hook_fmax', 16.0)         # N batas gaya servo hook
        p('hook_depth_range', 0.20)  # m batas koreksi depth dari hook_depth
        # APPROACH_QR: batas waktu navigasi XY sebelum RECOVERY (naikkan depth
        # utk perlebar FOV kamera bawah). Bukan abort — abort tetap di t_scan.
        p('t_nav_qr', 30.0)
        # Visual servo (pusatkan QR di frame kamera bawah sebelum GRAB).
        p('qr_center_tol', 0.12)     # |ex|,|ey| ternormalisasi dianggap "di tengah"
        p('qr_servo_gain', 0.15)     # m geser target per satuan offset ternormalisasi
        # Arah koreksi servo. Salah tanda = umpan balik POSITIF (ROV menjauh dari
        # payload). Bila |ex|,|ey| membesar saat uji, balik ke -1.0 (lihat plan H2).
        p('qr_servo_sign', 1.0)
        # P0-2.5 Kandidat #2 (docs/P0-2-5-ENGINEERING-ANALYSIS.md, hardened di
        # review approval): EMA pada qr_ex/qr_ey SEBELUM dipakai menghitung
        # target servo (body_dx/body_dy) -- meredam noise per-tick dari corner
        # detection (P0-2.3: bias hingga ~0.19m pada observasi corner-only)
        # supaya target (tx,ty) tidak "bergerak" tiap tick. TIDAK menyentuh
        # kondisi exit `centered` (tetap pakai self.qr_off mentah) -- itu scope
        # Kandidat #4, bukan #2. alpha=1.0 == filter nonaktif (setara sebelum
        # perubahan ini), dipakai sbg default kalau param tak di-override,
        # supaya param baru ini tidak diam-diam mengubah perilaku existing.
        p('qr_offset_ema_alpha', 1.0)
        # P0-2.5 Kandidat #1 (docs/P0-2-5-ENGINEERING-ANALYSIS.md SB/SC, isolasi
        # dari Kandidat #2 -- battery kandidat ini HARUS jalan dgn
        # qr_offset_ema_alpha=1.0/default): lebar gerbang aktivasi visual servo
        # (dist_raw < qr_servo_range). 0.3 == nilai lama yg sebelumnya hardcoded
        # di _st_approach_qr, dipakai sbg default supaya param baru ini tidak
        # diam-diam mengubah perilaku existing.
        p('qr_servo_range', 0.3)
        # P0-2.5 Kandidat #3 (docs/P0-2-5-ENGINEERING-ANALYSIS.md SB -- RISIKO
        # TERTINGGI dari 4 kandidat, guardrail wajib: diverged/saturation_frac/
        # sign_changes TIDAK BOLEH naik dibanding baseline). Lantai fraksi gaya
        # taper _goto_xy khusus APPROACH_QR (0.05 = nilai lama/default, TIDAK
        # memengaruhi _st_hang/_st_nav_wall yg tetap hardcoded 0.05).
        p('approach_min_fmax_frac', 0.05)
        # P0-2.5 Kandidat #4 (docs/P0-2-5-ENGINEERING-ANALYSIS.md -- terakhir
        # dalam urutan, satu-satunya yg BUKAN perbaikan fisik: mengetatkan
        # kondisi transisi GRAB supaya cocok dgn metrik dwell yg sudah dipakai
        # mengevaluasi Kandidat #1-3 di reduce_approach_qr.py). Jumlah tick
        # berturut-turut kondisi convergen (centered ATAU dist<approach_tol)
        # harus bertahan sebelum GRAB benar2 dipicu. 1 == perilaku lama persis
        # (transisi pada tick pertama lolos, TIDAK ada dwell).
        p('approach_dwell_ticks', 1)
        # --- Koreksi offset kamera bawah -> gripper ---
        # camera_bottom_link ada di x=+0.02 sedangkan gripper_base di x=+0.18
        # (hydroships.urdf.xacro), jadi GRIPPER 0.16 m DI DEPAN kamera. Servo lama
        # memusatkan QR di KAMERA -> gripper selalu melewati payload ~0.16 m.
        # Sekarang servo menargetkan QR muncul di ey_target (bukan 0) supaya
        # GRIPPER yang berada tepat di atas QR.
        p('cam_gripper_dx', 0.16)    # m, jarak gripper di depan kamera bawah (x body)
        p('gripper_base_dx', 0.18)   # m, gripper_base.x vs base_link (utk fallback XY)
        p('qr_floor_z', -0.90)       # m, tinggi bidang QR di dunia (payload_spawner.py)
        p('cam_bottom_dz', 0.18)     # m, kamera bawah di bawah base_link
        # tan(setengah-FOV vertikal). hFOV 80° @ 4:3 -> atan(0.75*tan40°) = 32.2°.
        p('cam_vfov_half_tan', 0.6293)
        # Batas |ey_target| supaya QR tetap di dalam frame (1.0 = tepat di tepi).
        p('ey_target_max', 0.8)
        # APPROACH_HOOK ey_target (M6): hook di dinding, kamera depan di haluan
        # (xacro body_x/2 = 0.1725, z=0). Di hang_approach_depth=0.14 hook selalu
        # di BAWAH pusat frame (ey~+0.5 bukan 0) -> gate lama |ey|<0.15 tak pernah
        # lolos 4/8 wall+seed. hook_z = pusat tip (-0.39, arena sdf -0.45..-0.33).
        p('hook_z', -0.39)
        p('cam_front_dz', 0.0)       # m kamera depan di atas base_link (xacro 0)
        p('hook_ey_max', 0.8)        # clamp ey_target hook biar tetap di frame
        p('lean_wall_offset', 0.30)  # m mundur dari wall_face
        p('lean_side_offset', 1.20)  # m geser SAMPING hook sejajar dinding (diperjauh biar LEAN lama)
        p('lean_tol', 0.15)          # m radius dianggap bersandar (longgar biar tidak premature hold)
        p('lean_hold_s', 2.5)        # s tahan di lean sebelum REVERSE (diperpanjang)
        p('lean_log_cap', 800)       # tick max log LEAN_RECORD
        p('reverse_step_tol', 0.12)  # m toleransi reverse
        p('docking_yaw', 90.0)       # deg putar saat docking (samping)

        g = lambda n: self.get_parameter(n).value
        self.surge = float(g('surge_force'))
        self.depth_bottom = float(g('depth_bottom'))
        self.depth_surface = float(g('depth_surface'))
        self.depth_tol = float(g('depth_tol'))
        self.descend_depth_tol = float(g('descend_depth_tol'))
        self.descend_recenter_timeout = float(g('descend_recenter_timeout'))
        self.descend_settle_dwell = float(g('descend_settle_dwell'))
        self.hook_depth = float(g('hook_depth'))
        self.wall_face = float(g('wall_face'))
        self.hang_tip_d = float(g('hang_tip_d'))
        self.hang_hole_dx = float(g('hang_hole_dx'))
        self.hang_approach_depth = float(g('hang_approach_depth'))
        self.hang_l_tol = float(g('hang_l_tol'))
        self.hang_forward_bias = float(g('hang_forward_bias'))
        self._hang_depth_max = None   # kedalaman terdalam saat fase turun (deteksi stall/blok palang)
        self.hang_tol = float(g('hang_tol'))
        self.hang_yaw_tol = math.radians(float(g('hang_yaw_tol_deg')))
        self.yaw_tol = math.radians(float(g('yaw_tol_deg')))
        self.qr_max_age = float(g('qr_max_age'))
        self.payload_x = float(g('payload_x'))
        self.payload_y = float(g('payload_y'))
        self.t_payload_pose = float(g('t_payload_pose'))
        self.scan_depth = float(g('scan_depth'))
        self.grab_depth = float(g('grab_depth'))
        self.approach_kp = float(g('approach_kp'))
        self.approach_kd = float(g('approach_kd'))
        self.approach_fmax = float(g('approach_fmax'))
        self.approach_tol = float(g('approach_tol'))
        self.wall_dist = float(g('wall_dist'))
        self.hook_dist = float(g('hook_dist'))
        self.hook_lateral_offset = float(g('hook_lateral_offset'))
        self.nav_tol = float(g('nav_tol'))
        self.nav_fmax = float(g('nav_fmax'))
        self.hold_settle_s = float(g('hold_settle_s'))
        self.release_max_retries = int(g('release_max_retries'))
        self.t_nav_qr = float(g('t_nav_qr'))
        self.qr_center_tol = float(g('qr_center_tol'))
        self.qr_servo_gain = float(g('qr_servo_gain'))
        self.qr_servo_sign = float(g('qr_servo_sign'))
        self.qr_offset_ema_alpha = float(g('qr_offset_ema_alpha'))
        self.qr_servo_range = float(g('qr_servo_range'))
        self.approach_min_fmax_frac = float(g('approach_min_fmax_frac'))
        self.approach_dwell_ticks = int(g('approach_dwell_ticks'))
        self.cam_gripper_dx = float(g('cam_gripper_dx'))
        self.gripper_base_dx = float(g('gripper_base_dx'))
        self.qr_floor_z = float(g('qr_floor_z'))
        self.cam_bottom_dz = float(g('cam_bottom_dz'))
        self.cam_vfov_half_tan = float(g('cam_vfov_half_tan'))
        self.ey_target_max = float(g('ey_target_max'))
        self.hook_z = float(g('hook_z'))
        self.cam_front_dz = float(g('cam_front_dz'))
        self.hook_ey_max = float(g('hook_ey_max'))
        self.lean_wall_offset = float(g('lean_wall_offset'))
        self.lean_side_offset = float(g('lean_side_offset'))
        self.lean_tol = float(g('lean_tol'))
        self.lean_hold_s = float(g('lean_hold_s'))
        self.lean_log_cap = int(g('lean_log_cap'))
        self.reverse_step_tol = float(g('reverse_step_tol'))
        self.docking_yaw = float(g('docking_yaw'))
        self.hook_max_age = float(g('hook_max_age'))
        self.hook_settle_grace_s = float(g('hook_settle_grace_s'))
        self.hook_gains = HookServoGains(
            kp_surge=float(g('hook_kp_surge')), kd_surge=float(g('hook_kd_surge')),
            kp_sway=float(g('hook_kp_sway')), kd_sway=float(g('hook_kd_sway')),
            kp_depth=float(g('hook_kp_depth')),
            size_stop=float(g('hook_size_stop')), center_tol=float(g('hook_center_tol')),
            fmax=float(g('hook_fmax')), depth_range=float(g('hook_depth_range')))
        self.T = {k: float(g('t_' + k)) for k in
                  ('dive', 'scan', 'descend', 'grab', 'nav', 'hang', 'surface',
                   'wait_trigger', 'lean', 'reverse', 'release', 'approach')}
        self.T['lean'] = 70.0
        self.T['reverse'] = 70.0
        self.T['approach'] = 35.0

        # I/O
        self.pub_depth = self.create_publisher(Float64, '/hydroships/setpoint/depth', 10)
        self.pub_head = self.create_publisher(Float64, '/hydroships/setpoint/heading', 10)
        self.pub_manual = self.create_publisher(Twist, '/hydroships/manual/cmd', 10)
        # Manipulator (rancang ulang M5): perintah semantik open/close ke
        # gripper_controller (yg memicu gz DetachableJoint attach/detach).
        self.pub_grip = self.create_publisher(String, '/hydroships/gripper/command', 10)
        # Multi-payload: setelah 1 wall selesai, minta payload_spawner spawn
        # payload BARU (huruf QR = wall berikutnya) utk siklus DIVE berikutnya.
        self.pub_spawn_next = self.create_publisher(
            String, '/hydroships/payload/spawn_next', 10)
        self.pub_qr_offset_synth = self.create_publisher(PointStamped, '/hydroships/qr_offset', 10)
        self.create_subscription(Float64, '/hydroships/depth', self._on_depth, 10)
        self.create_subscription(Odometry, '/hydroships/odom', self._on_odom, 10)
        self.create_subscription(String, '/hydroships/qr_result', self._on_qr, 10)
        self.create_subscription(PointStamped, '/hydroships/qr_offset',
                                  self._on_qr_offset, 10)
        # hook_detector menerbitkan (ex, ey, size) ternormalisasi dari kamera
        # DEPAN — dipakai APPROACH_HOOK utk visual servo presisi ke hook.
        self.create_subscription(PointStamped, '/hydroships/hook_offset',
                                  self._on_hook, 10)
        # payload_spawner menerbitkan pose payload SEKALI dgn QoS latched
        # (TRANSIENT_LOCAL) -> subscriber HARUS pakai durability sama supaya
        # tetap dapat pesan walau node ini start belakangan.
        self.create_subscription(
            PointStamped, '/hydroships/payload_pose', self._on_payload_pose,
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Empty, '/hydroships/mission/start_autonomous',
                                  self._on_trigger, 10)
        self.create_subscription(String, '/hydroships/gripper/status',
                                  self._on_gripper_status, 10)

        # payload sudah nempel ke ROV sejak spawn (DetachableJoint).
        # Detach = publish Empty ke topic ini.
        self.pub_detach = self.create_publisher(Empty, '/hydroships/gripper/detach', 10)
        # Status attach gripper (open/closed) dari gripper_controller — GRAB
        # hanya dianggap sukses bila state benar-benar jadi 'closed'.
        self.create_subscription(String, '/hydroships/gripper/state',
                                  self._on_grip_state, 10)
        self._grip_state = 'open'
        self.gripper_status = None   # ack terakhir dari gripper_controller (R-9)
        self._detach_sent = False
        self._hook_backoff_done = False
        self._hang_pos_done = False   # HANG/AUTO_RELEASE: lubang sudah di atas tip?
        self._release_retries = 0
        self._lean_log = []           # [(x,y)] direkam LEAN_RECORD utk REVERSE_RETURN closed-loop
        self._reverse_idx = 0

        # State
        self.depth = None
        self.yaw = None
        self.roll = None
        self.pitch = None
        self.x = None
        self.y = None
        self.vx = 0.0
        self.vy = 0.0
        self.qr_wall = None
        self.qr_time = 0.0
        self.qr_off = None        # (ex, ey, size) ternormalisasi dari qr_offset -- MENTAH,
                                   # dipakai apa adanya utk kondisi exit `centered` (Kandidat #4
                                   # scope, tidak disentuh di sini).
        self.qr_off_time = 0.0
        self._qr_ex_filt = None   # P0-2.5 Kandidat #2: EMA qr_ex, direset tiap entry APPROACH_QR
        self._qr_ey_filt = None   # P0-2.5 Kandidat #2: EMA qr_ey, direset tiap entry APPROACH_QR
        self._warned_no_odom = False
        self._approach_recovered = False   # RECOVERY depth-ascent sudah dipicu?
        self._wall_scored = False          # skor m1 sudah diberi (cegah spam log)
        self._converge_ticks = 0  # P0-2.5 Kandidat #4: dwell tick counter, direset tiap entry APPROACH_QR
        self._descend_depth_ok_since = None  # R-11 Opsi 2: timer re-centering visual di DESCEND
        self._descend_depth_settle_since = None  # R-10: timer dwell depth_ok sebelum GRAB
        self.hook_off = None      # (ex, ey, size)
        self.hook_time = 0.0
        self.payload_pose = None  # (x, y, z) dari /hydroships/payload_pose (spawner)
        self._payload_pose_fallback = False  # P2-A: sudah fallback ke payload_x/y?
        self.wall = None
        self.done_hooks = set()
        self.score = {'m1': 0, 'm2': 0, 'm3': 0, 'm4': 0, 'm5': 0}
        self.state = St.IDLE
        self.t_state = self._now()
        self._hold_since = None
        self._hook_bad_since = None
        self._locked_yaw = None
        self._trigger_received = False
        try:
            self._start_state = St[g('start_state')]
        except KeyError:
            self._start_state = St.DIVE
        # Seed manual self.wall utk testing state mid-FSM (NAV_WALL/HANG/SURFACE/
        # APPROACH_HOOK/AUTO_RELEASE) yg biasanya di-set oleh QR di APPROACH_QR/SCAN_QR.
        # Harus SETELAH self.wall = None di atas agar tak tertimpa. Guard di
        # _st_nav_wall tetap abort bila wall benar-benar tak diketahui (operasi normal).
        sw = str(g('start_wall')).strip().upper()
        if sw in WALL_HEADING_DEG:
            self.wall = sw
        self._started = False
        self._t0 = self._now()
        self._start_delay = float(g('start_delay'))
        order_str = str(g('wall_order')).upper()
        if order_str == 'RANDOM':
            import random
            self._wall_sequence = ['A', 'B', 'C', 'D']
            random.shuffle(self._wall_sequence)
        else:
            self._wall_sequence = list(order_str)
        self._wall_idx = 0
        self.get_logger().info('Urutan wall: %s' % self._wall_sequence)

        self.create_timer(0.1, self._tick)   # 10 Hz
        self.get_logger().info('mission_fsm siap — mulai dalam %.0fs (start=%s)'
                               % (self._start_delay, self._start_state.name))

    # ---- util ----
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _elapsed(self):
        return self._now() - self.t_state

    def _to(self, s):
        self.get_logger().info('[FSM] %s -> %s' % (self.state.name, s.name))
        self.state = s
        self.t_state = self._now()
        self._hold_since = None
        if s is St.APPROACH_HOOK:
            self._hook_backoff_done = False
        if s is St.GRAB:
            # R-9: status ack lama tak boleh terbawa ke siklus GRAB berikutnya.
            self.gripper_status = None
        if s in (St.HANG, St.AUTO_RELEASE):
            self._hang_pos_done = False
            self._hang_depth_max = None
        if s is St.AUTO_RELEASE:
            self._release_retries = 0
        if s is St.APPROACH_QR:
            # Misi berulang per payload (AUTO_RELEASE -> DIVE -> APPROACH_QR):
            # tanpa reset, payload ke-2 dst langsung dianggap sudah ber-wall.
            self._wall_scored = False
            self._approach_recovered = False
            self._converge_ticks = 0  # P0-2.5 Kandidat #4: reset dwell counter juga
            # P0-2.5 Kandidat #2: reset filter EMA juga -- tanpa ini, payload
            # ke-2 dst mewarisi nilai filter dari target LAMA (posisi QR
            # sebelumnya), menghasilkan "konvergensi cepat" palsu yang
            # sebenarnya cuma filter stale, bukan servo yang benar2 bekerja
            # (risiko yang diidentifikasi eksplisit di review hardening).
            self._qr_ex_filt = None
            self._qr_ey_filt = None
        if s is St.DESCEND:
            # R-11 Opsi 2: reset timer re-centering visual tiap masuk DESCEND
            # supaya waktu tunggu dihitung dari entry state, bukan dari t_state
            # APPROACH_QR sebelumnya.
            self._descend_depth_ok_since = None
            self._descend_depth_settle_since = None
        if s is St.LEAN_RECORD:
            self._lean_log = []
            self._reverse_idx = 0
            self._lean_phase = 1
            self._rev_stuck = 0
        if s is St.REVERSE_RETURN:
            self._reverse_idx = len(self._lean_log) - 1
            self._rev_stuck = 0

    def _set_depth(self, d_pos):
        m = Float64(); m.data = -abs(d_pos); self.pub_depth.publish(m)

    def _set_heading(self, yaw_rad):
        m = Float64(); m.data = wrap_to_pi(yaw_rad); self.pub_head.publish(m)

    def _set_surge(self, fx=0.0, fy=0.0):
        t = Twist(); t.linear.x = float(fx); t.linear.y = float(fy)
        self.pub_manual.publish(t)

    def _goto_xy_yaw_first(self, tx, ty, fmax=None, yaw_gate_deg=15.0,
                            freeze_dist=0.08, slow_dist=1.5):
        """Non-holonomik: putar dulu menghadap target, baru maju (surge saja,
        tanpa sway). Gaya di-TAPER mulai slow_dist (mengecil linear sampai
        freeze_dist) agar ROV melambat sebelum tiba, tak slam. Dalam
        freeze_dist, berhenti hitung ulang heading & aktif ngerem sisa
        kecepatan. Kembalikan jarak sisa (m)."""
        if self.x is None or self.yaw is None:
            return 999.0
        ex, ey = tx - self.x, ty - self.y
        dist = math.hypot(ex, ey)
        if dist < freeze_dist:
            brake_kd = 40.0
            brake = max(-20.0, min(20.0, -brake_kd * self.vx))
            self._set_surge(brake)
            self._set_heading(self.yaw)
            return dist
        target_yaw = math.atan2(ey, ex)
        self._set_heading(target_yaw)
        yaw_err = abs(wrap_to_pi(target_yaw - self.yaw))
        if yaw_err > math.radians(yaw_gate_deg):
            self._set_surge(0.0)
        else:
            fm = self.approach_fmax if fmax is None else fmax
            taper = min(1.0, (dist - freeze_dist) / max(0.01, slow_dist - freeze_dist))
            surge = self.approach_kp * dist * taper - self.approach_kd * self.vx
            surge = max(0.0, min(fm, surge))
            self._set_surge(surge, 0.0)
        return dist

    def _goto_xy(self, tx, ty, fmax=None, min_fmax_frac=None, yaw_ref=None,
                 deadband=0.005):
        """PD posisi HOLONOMIK: dorong ROV ke (tx,ty) dunia via gaya horizontal
        body-frame (surge+sway), TANPA mengubah heading — dipakai saat sudah
        menghadap arah yang benar (mis. setelah NAV_WALL) & cuma perlu
        koreksi posisi kecil. Kembalikan jarak sisa (m).

        yaw_ref: sudut (rad) tetap utk rotasi error dunia->body. Kalau None
        pakai yaw live. HANG/AUTO_RELEASE meneruskan heading WALL yang TETAP
        supaya jitter heading-hold TIDAK ikut memutar error -> tak ada getaran
        sway (wiggle) saat presisi milimeter.
        deadband: dalam radius ini (m, per sumbu body) dorongan P dimatikan,
        hanya redaman kecepatan tersisa — ROV parkir di target, tak
        berayun-ayun di sekitarnya."""
        if self.x is None or self.yaw is None:
            return 999.0
        fm = self.approach_fmax if fmax is None else fmax
        ex, ey = tx - self.x, ty - self.y
        dist = math.hypot(ex, ey)
        # Taper gaya maks saat mendekati target (slow-down radius) -> cegah slam.
        slow_radius = 1.0  # m, mulai perlambat dalam radius ini
        # Floor gaya 0.12 x fmax (bukan 0.05): di jarak dekat gaya 0.05xfmax
        # (~1 N) terlalu kecil utk menyelesaikan sisa error — ROV mandek
        # beberapa cm dari target (terlihat di HANG: dist macet 0.036).
        if dist < slow_radius:
            frac = max(0.12, dist / slow_radius)
            fm = fm * frac
        yaw = self.yaw if yaw_ref is None else yaw_ref
        c, s = math.cos(yaw), math.sin(yaw)
        bx = ex * c + ey * s
        by = -ex * s + ey * c
        cl = lambda v: max(-fm, min(fm, v))
        surge = self.approach_kp * bx - self.approach_kd * self.vx
        sway = self.approach_kp * by - self.approach_kd * self.vy
        if abs(bx) < deadband:
            surge = -self.approach_kd * self.vx
        if abs(by) < deadband:
            sway = -self.approach_kd * self.vy
        self._set_surge(cl(surge), cl(sway))
        return dist

    def _next_wall(self):
        """Wall berikutnya yg BELUM selesai, dari urutan wall_order. None = semua
        sudah selesai. Dipakai fallback pemilihan wall (QR tak ter-decode) DAN
        huruf payload berikutnya yg diminta ke spawner."""
        for w in self._wall_sequence:
            if w not in self.done_hooks:
                return w
        return None

    def _wall_xy(self, wall):
        d = self.wall_dist
        return {'A': (0.0, -d), 'B': (0.0, d), 'C': (d, 0.0), 'D': (-d, 0.0)}[wall]

    def _tip_xy(self, wall):
        """Posisi dunia TIP (ujung) hook — silinder tegak tempat lubang payload
        digantung. Tip berada wall_face - hang_tip_d dari pusat arena."""
        d = self.wall_face - self.hang_tip_d
        return {'A': (0.0, -d), 'B': (0.0, d), 'C': (d, 0.0), 'D': (-d, 0.0)}[wall]

    def _hang_xy(self, wall, yaw=None):
        """Target base_link agar LUBANG payload (bukan base) tepat di atas TIP
        hook. yaw (rad): heading ROV LIVE utk KOMPENSASI offset heading — lubang
        0.2733 m di depan base, jadi kalau target dihitung dgn heading wall
        tetap padahal heading hold menyisakan error ~3 deg, lubang meleset
        ~14 mm dari tip dan gate yaw tak pernah lolos. Dengan yaw live, lubang
        tetap di atas tip walau heading belum persis — error heading hanya
        memutar slot sedikit, tak menggeser lubangnya.
        yaw=None: pakai heading WALL tetap (dipakai saat ROV masih jauh /
        sedang berputar menghadap wall, supaya tak mengejar target bergerak)."""
        tip_x, tip_y = self._tip_xy(wall)
        if yaw is None:
            yaw = math.radians(WALL_HEADING_DEG[wall])
        dx_eff = self.hang_hole_dx - self.hang_forward_bias
        return (tip_x - dx_eff * math.cos(yaw),
                tip_y - dx_eff * math.sin(yaw))

    # ---- callbacks ----
    def _on_depth(self, msg): self.depth = msg.data

    def _on_gripper_status(self, msg): self.gripper_status = msg.data

    def _on_odom(self, msg):
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.roll, self.pitch = roll_pitch_from_quaternion(msg.pose.pose.orientation)
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.vx = msg.twist.twist.linear.x
        self.vy = msg.twist.twist.linear.y

    def _on_qr(self, msg):
        w = (msg.data or '').strip().upper()
        if w in WALL_HEADING_DEG:
            self.qr_wall = w; self.qr_time = self._now()

    def _on_grip_state(self, msg):
        s = (msg.data or '').strip().lower()
        if s in ('open', 'closed'):
            self._grip_state = s

    def _on_qr_offset(self, msg):
        """Offset QR di frame kamera. qr_detector menerbitkan utk kamera BAWAH
        maupun DEPAN di topic yg sama (dibedakan frame_id) — utk memusatkan diri
        di ATAS payload hanya kamera bawah yg relevan, offset kamera depan
        justru menyesatkan servo."""
        if msg.header.frame_id != 'camera_bottom_link':
            return
        self.qr_off = (msg.point.x, msg.point.y, msg.point.z)
        self.qr_off_time = self._now()

    def _on_hook(self, msg):
        """Offset hook di frame kamera DEPAN dari hook_detector:
        x=ex ternormalisasi (+ = hook di kanan pusat), y=ey (+ = di bawah pusat),
        z=size = sqrt(area)/lebar frame (makin besar = makin dekat)."""
        if msg.header.frame_id != 'camera_front_link':
            return
        self.hook_off = (msg.point.x, msg.point.y, msg.point.z)
        self.hook_time = self._now()

    def _hook_fresh(self):
        """(ex, ey, size) bila deteksi hook masih segar, else None. hook_detector
        hanya menerbitkan saat deteksi BERHASIL, jadi umur pesan = sinyal hilang."""
        if self.hook_off is None or self._now() - self.hook_time > self.hook_max_age:
            return None
        return self.hook_off

    def _on_payload_pose(self, msg):
        """Pose payload sebenarnya dari spawner (payload di-random tiap run).
        Tanpa ini FSM navigasi ke param payload_x/y yg statis -> ROV mendarat di
        tempat salah & QR tak pernah masuk frame kamera bawah."""
        self.payload_pose = (msg.point.x, msg.point.y, msg.point.z)
        self.payload_x = msg.point.x
        self.payload_y = msg.point.y

    def _on_trigger(self, _msg):
        if self.state == St.WAIT_TRIGGER:
            self.get_logger().info('Trigger autonomous diterima dari pilot')
            self._trigger_received = True
        else:
            self.get_logger().warn(
                'Trigger autonomous diabaikan (state saat ini: %s, bukan WAIT_TRIGGER)'
                % self.state.name)

    # ---- main tick ----
    def _tick(self):
        if not self._started:
            if self._now() - self._t0 >= self._start_delay and self.depth is not None:
                self._started = True
                self.get_logger().info('===== MISI KKI 2026 DIMULAI =====')
                self._to(self._start_state)
            return
        if self.state in (St.DONE, St.ABORT):
            self._set_surge(0.0, 0.0)
            return

        h = getattr(self, '_st_' + self.state.name.lower(), None)
        if h:
            h()

    # ---- state handlers ----
    def _st_dive(self):
        self._set_depth(self.scan_depth)
        self._set_heading(0.0)
        depth_ok = self.depth is not None and self.depth >= self.scan_depth - self.depth_tol
        # Jangan transisi ke APPROACH_QR sebelum payload_pose (latched, dari
        # payload_spawner via TimerAction) benar-benar tiba -- kalau tidak,
        # APPROACH_QR mulai navigasi ke payload_x/y fallback statis (bukan posisi
        # spawn asli yg di-random tiap run) selama beberapa detik sebelum pesan
        # latched itu nyampe, bikin ROV "ngacak lalu ngebut" ke target salah.
        if depth_ok and self.payload_pose is None and int(self._elapsed() * 2) % 20 == 0:
            self.get_logger().info(
                'DIVE: kedalaman OK, menunggu /hydroships/payload_pose...')
        # P2-A: payload_pose (ground truth spawner) opsional -- di kontes nyata
        # topic ini tak ada. Tenggang t_payload_pose (< t_dive) habis & masih
        # belum tiba -> fallback ke payload_x/y param (sudah jadi nilai default
        # sejak init) + perlebar FOV kamera bawah, lalu tetap lanjut (bukan ABORT).
        if (depth_ok and self.payload_pose is None and not self._payload_pose_fallback
                and self._elapsed() > self.t_payload_pose):
            self._payload_pose_fallback = True
            self.scan_depth += 0.10
            self.get_logger().warn(
                'DIVE: payload_pose tak kunjung tiba %.0fs -> fallback payload_x/y '
                '(%.2f,%.2f), scan_depth -> %.2fm'
                % (self.t_payload_pose, self.payload_x, self.payload_y, self.scan_depth))
        if depth_ok and (self.payload_pose is not None or self._payload_pose_fallback):
            self._set_surge(0.0)
            self.get_logger().info(
                'Kedalaman scan tercapai (%.2fm), payload_pose %s'
                % (self.depth, 'siap' if self.payload_pose is not None
                   else 'fallback (payload_x/y param)'))
            self._to(St.APPROACH_QR)
        elif self._elapsed() > self.T['dive']:
            self.get_logger().error('DIVE timeout'); self._to(St.ABORT)

    def _gripper_align_txt(self):
        """Metrik alignment sesungguhnya: jarak XY GRIPPER (bukan base_link) ke QR.

        Inilah angka yang menentukan apakah jepitan kena. Sebelum koreksi
        cam_gripper_dx nilainya ~0.18 m (base_link yang dipusatkan, gripper
        melewati payload); setelah koreksi harus < approach_tol.
        """
        if self.x is None or self.yaw is None:
            return 'gripper_err=n/a'
        gx = self.x + self.gripper_base_dx * math.cos(self.yaw)
        gy = self.y + self.gripper_base_dx * math.sin(self.yaw)
        err = math.hypot(self.payload_x - gx, self.payload_y - gy)
        base_err = math.hypot(self.payload_x - self.x, self.payload_y - self.y)
        return ('gripper_err=%.3f m base_err=%.3f m (gripper@%.2f,%.2f '
                'payload@%.2f,%.2f)'
                % (err, base_err, gx, gy, self.payload_x, self.payload_y))

    def _st_approach_qr(self):
        """Misi 1: dekati payload holonomik (tanpa terikat heading, cegah
        osilasi saat mendekati target), lalu PUSATKAN QR di frame kamera bawah
        (visual servo) sebelum GRAB supaya jepitan presisi.

        Target XY memakai pose payload ASLI dari /hydroships/payload_pose
        (payload di-random tiap run); param payload_x/y cuma fallback sampai
        pesan latched itu tiba.

        Wall diambil dari QR asli (/hydroships/qr_result) begitu terbaca segar;
        fallback ke urutan wall_order kalau QR tak pernah terbaca."""
        if self._locked_yaw is None:
            self._locked_yaw = self.yaw if self.yaw is not None else 0.0
        self._set_heading(self._locked_yaw)
        depth_target = self.scan_depth
        qr_seen = self.qr_wall is not None and (self._now() - self.qr_time) < self.qr_max_age

        # RECOVERY: navigasi kelamaan tanpa QR terbaca -> naikkan sedikit supaya
        # FOV kamera bawah melebar (QR 12cm gampang MEMENUHI frame saat terlalu
        # rendah, finder pattern ter-crop -> decode gagal). Abort tetap di t_scan.
        if not qr_seen and self._elapsed() > self.t_nav_qr:
            depth_target = self.scan_depth + 0.10
            if not self._approach_recovered:
                self._approach_recovered = True
                self.get_logger().warn(
                    'APPROACH_QR recovery: QR belum terbaca %.0fs -> depth %.2f m '
                    '(perlebar FOV kamera bawah)' % (self.t_nav_qr, depth_target))
        elif qr_seen:
            self._approach_recovered = False
        self._set_depth(depth_target)

        # Offset kamera->gripper: QR tidak dipusatkan di kamera melainkan di
        # ey_target, supaya GRIPPER (0.16 m di depan kamera) yang tepat di atas QR.
        # Dihitung dari depth_target FINAL, jadi otomatis ikut menyesuaikan saat
        # recovery menaikkan kedalaman.
        ey_target = qr_ey_target(depth_target, self.cam_gripper_dx, self.qr_floor_z,
                                 self.cam_bottom_dz, self.cam_vfov_half_tan,
                                 self.ey_target_max)

        # Visual servo: geser target XY supaya QR bergerak ke ey_target di frame.
        # ex,ey ternormalisasi di sumbu CITRA -> putar ke sumbu DUNIA lewat yaw
        # (yaw berubah terus; pemetaan tetap akan salah arah saat ROV berputar).
        #
        # Target dasar juga digeser MUNDUR sejauh gripper_base_dx di sumbu x body:
        # payload_x/y adalah posisi QR, dan yang harus berada di situ adalah
        # gripper, bukan base_link. Tanpa ini fallback "dist < approach_tol"
        # menempatkan base_link di atas QR -> gripper meleset 0.18 m.
        tx, ty = self.payload_x, self.payload_y
        tx -= self.gripper_base_dx * math.cos(self._locked_yaw)
        ty -= self.gripper_base_dx * math.sin(self._locked_yaw)
        off_fresh = (self.qr_off is not None
                     and (self._now() - self.qr_off_time) < self.qr_max_age)
        # P0-2.5 Kandidat #2: update EMA di setiap tick offset masih segar --
        # TIDAK digerbang oleh dist_raw<0.3 (beda dari `servoing` di bawah),
        # supaya filter sudah punya histori saat servo baru mulai aktif,
        # bukan mulai dari sampel tunggal. alpha=1.0 (default) == filter
        # transparan, filt selalu sama dengan sampel mentah terbaru.
        if off_fresh:
            raw_ex, raw_ey, _raw_size = self.qr_off
            if self._qr_ex_filt is None:
                self._qr_ex_filt, self._qr_ey_filt = raw_ex, raw_ey
            else:
                a = self.qr_offset_ema_alpha
                self._qr_ex_filt = a * raw_ex + (1.0 - a) * self._qr_ex_filt
                self._qr_ey_filt = a * raw_ey + (1.0 - a) * self._qr_ey_filt
        dist_raw = math.hypot((self.x or 0.0) - tx, (self.y or 0.0) - ty)
        # P0-2.5 Kandidat #1: gerbang lebar dulunya hardcoded 0.3 -- sekarang
        # param qr_servo_range (default 0.3, sama persis). Kandidat #2 (EMA di
        # atas) TETAP jalan independen dari nilai ini -- filter update tidak
        # digerbang oleh qr_servo_range, cuma off_fresh.
        servoing = off_fresh and dist_raw < self.qr_servo_range
        if servoing:
            # Target servo dihitung dari offset TER-FILTER, bukan mentah --
            # inilah satu-satunya titik yang diubah Kandidat #2. Kondisi exit
            # `centered` (di bawah, terpisah) tetap memakai self.qr_off mentah
            # dengan sengaja -- itu scope Kandidat #4, bukan #2.
            ex, ey = self._qr_ex_filt, self._qr_ey_filt
            k = self.qr_servo_gain * self.qr_servo_sign
            # Error diukur terhadap ey_target, bukan terhadap 0.
            body_dx = -(ey - ey_target) * k   # ey>ey_target: QR terlalu ke belakang
            body_dy = -ex * k                 # ex>0: QR di kanan pusat -> geser kanan
            c, s = math.cos(self._locked_yaw), math.sin(self._locked_yaw)
            tx += body_dx * c - body_dy * s
            ty += body_dx * s + body_dy * c

        dist = self._goto_xy(tx, ty, min_fmax_frac=self.approach_min_fmax_frac)
        if int(self._elapsed() * 2) % 20 == 0:
            off_txt = ('ex=%+.2f ey=%+.2f' % (self.qr_off[0], self.qr_off[1])
                       if self.qr_off is not None else 'ex=-- ey=--')
            h_cam = max(0.05, abs(self.qr_floor_z) - depth_target - self.cam_bottom_dz)
            self.get_logger().info(
                'APPROACH_QR dbg: dist=%.3f x=%.2f y=%.2f yaw=%.1f target=(%.2f,%.2f) '
                '%s ey_target=%+.2f h_cam=%.2f servo=%d qr=%s'
                % (dist, self.x or -99, self.y or -99,
                   math.degrees(self.yaw or 0), tx, ty,
                   off_txt, ey_target, h_cam, int(servoing), self.qr_wall or '-'))

        # QR terbaca -> kunci wall, TAPI jangan langsung GRAB: pusatkan dulu.
        if qr_seen and not self._wall_scored:
            self.wall = self.qr_wall
            self.score['m1'] = 15
            self._wall_scored = True
            self.get_logger().info('QR %s terbaca -> wall %s dipilih (+15), '
                                   'pusatkan QR sebelum GRAB'
                                   % (self.qr_wall, self.wall))

        # R-11 Opsi 1: centered dievaluasi INDEPENDENT dari _wall_scored.
        # qr_off (dari /hydroships/qr_offset, deteksi kontur) adalah jalur
        # terpisah dari decode huruf (/hydroships/qr_result) yang menggerbangkan
        # _wall_scored. Decode huruf gagal 82-89% tapi corner tetap terdeteksi,
        # jadi centered tetap harus dicek tiap tick — bukan dihardcode False
        # saat _wall_scored False. Huruf tetap dipakai utk self.wall + skor m1
        # (blok qr_seen di atas), tapi tak lagi syarat utk cek centering.
        # Dibandingkan terhadap ey_target (bukan 0) — kalau tetap terhadap 0,
        # |ey| konvergen ke ~0.61 dan transisi GRAB tak pernah terpicu.
        centered = (off_fresh
                    and abs(self.qr_off[0]) < self.qr_center_tol
                    and abs(self.qr_off[1] - ey_target) < self.qr_center_tol)
        converged_now = centered or dist < self.approach_tol

        # P0-2.5 Kandidat #4: syarat dwell N-tick sebelum transisi GRAB
        # benar2 dipicu -- BUKAN transisi pada tick tunggal begitu kondisi
        # lolos sekali (perilaku lama, approach_dwell_ticks=1). Konter
        # bersama dipakai lintas kedua cabang (visual/XY-tol vs fallback)
        # supaya kalau QR baru ke-score MID-dwell (blok qr_seen di atas),
        # dwell tetap berlanjut alih2 diam2 reset — sama seperti definisi
        # `combined_entered` yg sudah dipakai reduce_approach_qr.py utk
        # mengevaluasi Kandidat #1-3.
        if converged_now:
            self._converge_ticks += 1
        else:
            self._converge_ticks = 0

        if self._converge_ticks >= self.approach_dwell_ticks:
            # R-11 diagnosis (2026-08-14): APPROACH_QR dbg di atas di-gate
            # tiap ~10s, sering melewatkan tick konvergen sesungguhnya --
            # log sekali di sini persis saat exit, non-fungsional.
            self.get_logger().info(
                'CONVERGEDBG: centered=%s dist=%.3f approach_tol=%.3f wall_scored=%s '
                'qr=%s ex=%s ey=%s'
                % (centered, dist, self.approach_tol, self._wall_scored,
                   self.qr_wall or '-',
                   ('%+.2f' % self.qr_off[0]) if self.qr_off is not None else '--',
                   ('%+.2f' % self.qr_off[1]) if self.qr_off is not None else '--'))
            if self._wall_scored:
                self.get_logger().info(
                    'QR terpusat (%s) -> DESCEND (%s)'
                    % ('visual servo' if centered else 'jarak XY',
                       self._gripper_align_txt()))
                self._set_surge(0.0); self._to(St.DESCEND); return
            else:
                # Fallback: QR huruf tak pernah ter-decode (82-89% run) tapi
                # ROV sudah konvergen — via centered (offset kontur segar) ATAU
                # dist < approach_tol. Wall dipilih dari urutan wall_order.
                nxt = self._next_wall()
                if nxt is None:
                    self.get_logger().info('Semua wall selesai, misi tuntas.')
                    self._print_score(); self._to(St.DONE); return
                self.wall = nxt
                self.score['m1'] = 15
                self._wall_scored = True
                self.get_logger().info('QR tidak ter-decode, wall urutan %s '
                                       'dipilih (+15) [urutan ke-%d] (%s) [%s]'
                                       % (self.wall, len(self.done_hooks) + 1,
                                          self._gripper_align_txt(),
                                          'visual servo' if centered else 'jarak XY'))
                self._set_surge(0.0); self._to(St.DESCEND); return

        if self._elapsed() > self.T['scan']:
            self.get_logger().error('APPROACH_QR timeout'); self._to(St.ABORT)

    def _st_descend(self):
        """Fase turun-untuk-mencengkeram (M5-D, docs/STATUS.md): APPROACH_QR
        memusatkan diri sambil melayang di scan_depth (~0.6 m di atas lantai
        QR, sengaja dangkal supaya QR muat di frame kamera). Attach di GRAB
        langsung dari sana membuat DetachableJoint mengelas ROV ke payload
        PADA POSE SAAT ITU (bukan menarik payload naik) -> ROV terjangkar ke
        lantai. State ini turun ke grab_depth (dekat lantai QR) sambil tetap
        servo XY, supaya saat GRAB memicu "close" ROV benar2 sudah dekat.

        QR akan HILANG dari frame di tengah turun (kamera bawah ikut turun sampai
        sejajar bidang QR) — itu normal: begitu offset tak segar lagi, XY di-hold
        dead-reckon ke target terakhir, dan hak attach dijaga latch "armed"
        di gripper_logic, bukan deteksi segar."""
        if self._locked_yaw is None:
            self._locked_yaw = self.yaw if self.yaw is not None else 0.0
        self._set_heading(self._locked_yaw)
        grasp_depth = self.grab_depth
        self._set_depth(grasp_depth)

        # ey_target dihitung dari kedalaman AKTUAL (bukan target): selama turun,
        # kamera bergerak, jadi "di mana QR seharusnya tampak" ikut berubah tiap
        # tick. Memakai grasp_depth di sini membuat nilainya ter-clamp ke ey_max
        # (h_cam ~0) dan servo mendorong ke arah yang salah selama detik pertama
        # turun, saat QR masih terlihat.
        ey_target = qr_ey_target(self.depth if self.depth is not None else grasp_depth,
                                 self.cam_gripper_dx, self.qr_floor_z,
                                 self.cam_bottom_dz, self.cam_vfov_half_tan,
                                 self.ey_target_max)
        tx, ty = self.payload_x, self.payload_y
        tx -= self.gripper_base_dx * math.cos(self._locked_yaw)
        ty -= self.gripper_base_dx * math.sin(self._locked_yaw)
        off_fresh = (self.qr_off is not None
                     and (self._now() - self.qr_off_time) < self.qr_max_age)
        if off_fresh:
            ex, ey, _size = self.qr_off
            k = self.qr_servo_gain * self.qr_servo_sign
            body_dx = -(ey - ey_target) * k
            body_dy = -ex * k
            c, s = math.cos(self._locked_yaw), math.sin(self._locked_yaw)
            tx += body_dx * c - body_dy * s
            ty += body_dx * s + body_dy * c
        dist = self._goto_xy(tx, ty, min_fmax_frac=self.approach_min_fmax_frac)

        depth_ok = self.depth is not None and self.depth >= grasp_depth - self.descend_depth_tol
        # R-10: depth_ok mentah cuma cek posisi sesaat -- ROV bisa masih turun
        # (overshoot) tepat saat memasuki toleransi. Wajibkan bertahan
        # `descend_settle_dwell` detik dulu supaya kecepatan turun sempat mereda
        # sebelum GRAB dipicu (lihat P1-OWNER-DECISIONS-AND-ROADMAP.md R-10).
        if not depth_ok:
            self._descend_depth_settle_since = None
        elif self._descend_depth_settle_since is None:
            self._descend_depth_settle_since = self._now()
        depth_settled = (depth_ok and self._descend_depth_settle_since is not None
                          and self._now() - self._descend_depth_settle_since
                              >= self.descend_settle_dwell)
        # R-11 Opsi 2: gerbang re-centering visual sebelum GRAB.
        # depth_ok saja sering memaksa GRAB saat offset QR masih besar/basi
        # (decode gagal 82-89%). Jika qr_off masih segar tapi belum terpusat,
        # beri waktu servo memperbaiki. Jika sudah stale (QR hilang, normal di
        # grab_depth), lanjaykan GRAB — servo tidak bisa membantu pada data usang.
        centered = (off_fresh
                    and abs(self.qr_off[0]) < self.qr_center_tol
                    and abs(self.qr_off[1] - ey_target) < self.qr_center_tol)
        if int(self._elapsed() * 2) % 20 == 0:
            self.get_logger().info(
                'DESCEND dbg: depth=%.2f/%.2f dist=%.3f ey_target=%+.2f '
                'centered=%s off_fresh=%d'
                % (self.depth if self.depth is not None else -1.0,
                   grasp_depth, dist, ey_target, centered, int(off_fresh)))

        if depth_settled:
            if centered:
                self._set_surge(0.0)
                self.get_logger().info('DESCEND: kedalaman + visual terpusat -> GRAB')
                self._to(St.GRAB)
            elif off_fresh:
                # QR terlihat tapi belum terpusat: beri waktu re-centering.
                # Jangan zero surge — biarkan visual servo (_goto_xy di atas)
                # terus mengoreksi XY.
                if self._descend_depth_ok_since is None:
                    self._descend_depth_ok_since = self._now()
                elif self._now() - self._descend_depth_ok_since > self.descend_recenter_timeout:
                    self._set_surge(0.0)
                    self.get_logger().warn(
                        'DESCEND: visual belum terpusat setelah %.1fs '
                        '(ex=%+.2f ey=%+.2f), lanjutkan GRAB (fallback)'
                        % (self.descend_recenter_timeout,
                           self.qr_off[0], self.qr_off[1]))
                    self._to(St.GRAB)
                # else: masih dalam jeda re-centering, lanjutkan servo
            else:
                # qr_off stale (QR tak terlihat di grab_depth) — tidak bisa
                # servo, langsung GRAB. Normal: kamera bawah sejajar bidang QR,
                # latch "armed" gripper_logic tetap aktif.
                self._set_surge(0.0)
                self.get_logger().info(
                    'DESCEND: kedalaman grasp tercapai, QR tak terlihat (stale) -> GRAB')
                self._to(St.GRAB)
        elif self._elapsed() > self.T['descend']:
            self.get_logger().error('DESCEND timeout'); self._to(St.ABORT)

    def _st_grab(self):
        """Misi 2 (REMOTELY): kirim perintah "close" ke gripper_controller,
        lalu tunggu ack /hydroships/gripper/status (R-9) sebelum menilai skor.

        gripper_controller menilai keamanan lewat GripperLogic.is_safe() atas
        /hydroships/qr_offset dan membalas 'attached' atau 'rejected' lewat
        gripper/status; FSM sengaja tidak menduplikasi gerbang itu, hanya
        menunggu hasilnya. 'rejected' mengulang perintah "close" (gerbang
        visual bisa berubah tick berikutnya) sampai T['grab'] habis -> ABORT,
        supaya kesuksesan misi tak lagi bisa dibaca sbg bukti attach padahal
        gerbang menolaknya (lihat P1-OWNER-DECISIONS-AND-ROADMAP.md R-9).
        Sinyal qr_offset SINTETIK (frame_id 'synthetic') dipublish bersama
        tiap close karena QUIRC tak terpasang & sinyal kamera asli tak pernah
        lolos gate — lihat gripper_controller._on_offset()."""
        self._set_surge(0.0)
        if self.gripper_status == 'attached':
            self.score['m2'] = 15
            self.get_logger().info('GRAB terverifikasi (+15) -- ack attached')
            self._to(St.NAV_WALL)
            return
        if self._hold_since is None or self.gripper_status == 'rejected':
            self._hold_since = self._now()
            self.gripper_status = None
            synth = PointStamped()
            synth.header.stamp = self.get_clock().now().to_msg()
            synth.header.frame_id = 'synthetic'
            synth.point.x, synth.point.y, synth.point.z = 0.0, 0.0, 0.2
            self.pub_qr_offset_synth.publish(synth)
            self.pub_grip.publish(String(data='close'))
            self.get_logger().info('GRAB: perintah "close" -> gripper_controller')
        if self._elapsed() > self.T['grab']:
            self.get_logger().error('GRAB timeout (tak ada ack attached)')
            self._to(St.ABORT)

    def _st_nav_wall(self):
        """Misi 3 (REMOTELY): navigasi holonomik ke wall sesuai QR."""
        if self.wall is None: self._to(St.ABORT); return
        tx, ty = self._wall_xy(self.wall)
        target_heading = math.radians(WALL_HEADING_DEG[self.wall])
        # Navigasi di hang_approach_depth (DI ATAS puncak tip z=-0.33) bukan
        # hook_depth — supaya plat payload tak menyodok hook saat transit, dan
        # HANG bisa turun vertikal menembus tip.
        self._set_depth(self.hang_approach_depth)
        dist = self._goto_xy_yaw_first(tx, ty, fmax=self.nav_fmax)
        if dist < self.nav_tol:
            self._set_heading(target_heading)   # sudah tiba, baru hadapkan ke wall
        if int(self._elapsed() * 2) % 20 == 0:
            ex_dbg, ey_dbg = tx - (self.x or 0), ty - (self.y or 0)
            target_yaw_dbg = math.degrees(math.atan2(ey_dbg, ex_dbg))
            self.get_logger().info(
                'NAV_WALL dbg: dist=%.2f x=%.2f y=%.2f yaw=%.1f target_yaw=%.1f target=(%.2f,%.2f)'
                % (dist, self.x or -99, self.y or -99,
                   math.degrees(self.yaw or 0), target_yaw_dbg, tx, ty))
        if dist < self.nav_tol:
            self._set_surge(0.0)
            speed = math.hypot(self.vx, self.vy)
            self.get_logger().info('Tiba di standoff wall %s (dist %.2fm, v %.2fm/s) -> HANG'
                                   % (self.wall, dist, speed))
            self._to(St.HANG)
        elif self._elapsed() > self.T['nav']:
            self.get_logger().error('NAV_WALL timeout (dist %.2fm)' % dist); self._to(St.ABORT)

    def _st_hang(self):
        """Misi 3/4 (REMOTELY): posisikan LUBANG payload tepat di atas TIP hook
        (target odometri presisi, bukan standoff lama yang 0.5 m dari hook),
        lalu TURUN — tip menembus lubang dan plat bersandar di palang bawah
        hook (payload benar-benar tergantung, gripper masih menjepit). Tak ada
        release di sini. Sudah menghadap wall sejak NAV_WALL — pakai holonomik
        (sway) utk koreksi lateral TANPA berputar lagi."""
        if self.wall is None: self._to(St.ABORT); return
        target_heading = math.radians(WALL_HEADING_DEG[self.wall])
        self._set_heading(target_heading)
        yaw_err = (abs(wrap_to_pi(target_heading - self.yaw))
                   if self.yaw is not None else math.pi)
        # Target lubang di atas tip. Saat heading sudah dekat (<10°), KOMPENSASI
        # offset heading pakai yaw LIVE (lubang tetap di atas tip walau heading
        # hold menyisakan error ~3° — kalau target tetap wall-heading, error itu
        # menggeser lubang ~14 mm & gate 3° tak pernah lolos, persis kegagalan
        # di run terakhir: dist 0.005 tapi yaw_err 3.1°). Saat masih jauh/berputar,
        # pakai heading wall tetap (hindari mengejar target bergerak).
        if yaw_err < math.radians(10.0) and self.yaw is not None:
            tx, ty = self._hang_xy(self.wall, self.yaw)
            yaw_ref = self.yaw
        else:
            tx, ty = self._hang_xy(self.wall)
            yaw_ref = target_heading
        if not self._hang_pos_done:
            # Fase 1: posisikan di atas hook pada kedalaman AMAN (di atas tip)
            # dan PASTIKAN heading sudah sejajar wall (gate). Rotasi error ke
            # body-frame ikut yaw_ref di atas (live saat dekat, wall saat jauh).
            self._set_depth(self.hang_approach_depth)
            if yaw_err >= self.hang_yaw_tol:
                # Rotasi SAMBIL transisi XY menyapu plat payload ke riser/tip
                # hook (terukur: wall C, ROV macet l_err~73 mm di riser x 2.44
                # krn tepi depan plat ~0.32 m di depan base_link). Mundur dulu ke
                # titik AMAN (retreat 0.25 m dari target) supaya sapuan plat saat
                # berputar tidak menyentuh riser, putar di sana, baru maju lurus
                # setelah heading sejajar (cabang else di bawah).
                retreat = 0.25
                gx = tx - retreat * math.cos(target_heading)
                gy = ty - retreat * math.sin(target_heading)
                dist = self._goto_xy(gx, gy, fmax=self.nav_fmax,
                                     yaw_ref=target_heading)
            else:
                dist = self._goto_xy(tx, ty, fmax=self.nav_fmax,
                                     yaw_ref=yaw_ref)
            # Gate arah LUBANG (sepanjang sumbu maju ROV): toleransi fisik lubang
            # hanya ~+-9 mm di arah maju (tip 25mm di lubang 50mm) vs +-28.5 mm
            # ke samping (dinding slot). Gate radial 25 mm sendiri terlalu longgar
            # utk arah maju -> tip mentok badan plat saat turun -> macet. Dulu
            # dist 0.035 lolos gate lalu ROV terdorong ke riser hook (ekstensi
            # palang collision menyentuh tiang) dan tak bisa mundur.
            l_err = abs((tx - (self.x or 0)) * math.cos(target_heading)
                        + (ty - (self.y or 0)) * math.sin(target_heading))
            if (dist < self.hang_tol and l_err < self.hang_l_tol
                    and yaw_err < self.hang_yaw_tol):
                if self._hold_since is None:
                    self._hold_since = self._now()
                if self._now() - self._hold_since >= self.hold_settle_s:
                    self._set_surge(0.0)
                    self._hang_pos_done = True
                    self._hold_since = None
                    self.get_logger().info(
                        'HANG: lubang di atas tip hook %s (dist %.3f, l_err %.1f mm, '
                        'yaw_err %.1f°) -> turun ke hook'
                        % (self.wall, dist, l_err * 1000.0, math.degrees(yaw_err)))
            else:
                self._hold_since = None
            if int(self._elapsed() * 2) % 20 == 0:
                self.get_logger().info(
                    'HANG dbg: dist=%.3f l_err=%.1fmm x=%.2f y=%.2f yaw=%.1f target=(%.2f,%.2f)'
                    % (dist, l_err * 1000.0, self.x or -99, self.y or -99,
                       math.degrees(self.yaw or 0), tx, ty))
            if self._elapsed() > self.T['hang']:
                self.get_logger().error('HANG timeout (posisi, dist %.3f, yaw_err %.1f°)'
                                        % (dist, math.degrees(yaw_err)))
                self._to(St.ABORT)
            return
        # Fase 2: turun — tip menembus lubang, plat bersandar di palang hook.
        # PERTAHANKAN posisi lubang di atas tip selama turun (gaya dikurangi,
        # yaw tetap wall): tanpa ini ROV bisa hanyut lateral saat turun (gaya
        # thrust coupling / kontak tip) dan lubang meleset dari tip.
        dist = self._goto_xy(tx, ty, fmax=0.6 * self.nav_fmax,
                             yaw_ref=yaw_ref)
        self._set_depth(self.hook_depth)
        # Gate presisi: selain depth & yaw, UJI ULANG error lateral (dist &
        # l_err) saat plat duduk — kalau lubang bergeser selama turun (coupling
        # depth-hold/kontak tip), jangan lolos diam-diam lalu HANG "sukses"
        # dengan lubang tidak presisi (gejala AUTO_RELEASE meleset). Sebelumnya
        # hanya depth+yaw -> lubang bisa bergeser dan tetap lolos.
        l_err = abs((tx - (self.x or 0)) * math.cos(target_heading)
                    + (ty - (self.y or 0)) * math.sin(target_heading))
        # Gate turun INLINE: pakai STALL detector (kedalaman berhenti naik karena
        # terblok palang) — BUKAN ambang absolute (hook_depth - 0.02 = 0.30).
        # Sebelumnya: depth 0.277 di run nyata (plat duduk agak tinggi) < 0.30
        # -> HANG timeout padahal plat sudah bersandar (ABORT siklus-2, seed
        # 3001). Kedalaman duduk bervariasi 0.28..0.32 tergantung cara plat
        # mendarat; stall = bukti fisik "sudah duduk" yang invarian terhadap itu.
        # Histeresis 5 mm (sama dgn AUTO_RELEASE): creeps mikro jangan restore
        # _hold_since. Tetap uji ulang dist/l_err saat duduk (presisi).
        d = self.depth
        depth_stalled = False
        if d is not None:
            if self._hang_depth_max is None or d > self._hang_depth_max + 0.005:
                self._hang_depth_max = max(d, self._hang_depth_max or 0.0)
                self._hold_since = None
            elif (d >= self._hang_depth_max - 0.015
                  and d >= self.hang_approach_depth + 0.03
                  and yaw_err < self.hang_yaw_tol
                  and dist < self.hang_tol
                  and l_err < self.hang_l_tol):
                if self._hold_since is None:
                    self._hold_since = self._now()
                if self._now() - self._hold_since >= self.hold_settle_s:
                    depth_stalled = True
            else:
                self._hold_since = None
        else:
            self._hold_since = None
        if depth_stalled:
            self._set_surge(0.0)
            self.score['m3'] = 15
            self.get_logger().info(
                'Payload tergantung stabil di hook %s (+15, depth %.2f)'
                % (self.wall, d))
            self._to(St.SURFACE)
        if self._elapsed() > self.T['hang']:
            self.get_logger().error('HANG timeout (turun, depth %s, dist %.3f)'
                                    % (self.depth if self.depth is not None else 'n/a',
                                       dist))
            self._to(St.ABORT)

    def _st_surface(self):
        """Misi 4 (REMOTELY): naik & bersandar di sisi dinding payload.
        Heading TETAP menghadap wall (sama seperti HANG) — jangan reset ke 0,
        itu bikin ROV putar mendadak saat mulai naik."""
        if self.wall is None: self._to(St.ABORT); return
        target_heading = math.radians(WALL_HEADING_DEG[self.wall])
        self._set_heading(target_heading)

        yaw_err = abs(wrap_to_pi(target_heading - self.yaw)) if self.yaw is not None else math.pi
        if yaw_err > self.yaw_tol:
            brake_kd = 40.0
            bx = -brake_kd * self.vx
            by = -brake_kd * self.vy
            cl = lambda v: max(-20.0, min(20.0, v))
            self._set_surge(cl(bx), cl(by))
            self._set_depth(self.hook_depth)
            if self._elapsed() > self.T['surface']:
                self.get_logger().error(
                    'SURFACE timeout (masih align, yaw_err=%.1f°)' % math.degrees(yaw_err))
                self._to(St.ABORT)
            return

        self._set_depth(self.depth_surface)
        if self.depth is not None and self.depth <= self.depth_surface + 0.05:
            self._set_surge(0.0)
            self.score['m4'] = 15
            self.get_logger().info(
                'Permukaan tercapai, bersandar di dinding %s (+15). '
                'Menunggu trigger autonomous dari pilot...' % self.wall)
            self._trigger_received = False
            self._to(St.WAIT_TRIGGER)
        elif self._elapsed() > self.T['surface']:
            self.get_logger().error('SURFACE timeout'); self._to(St.ABORT)

    def _st_wait_trigger(self):
        """Menunggu trigger pilot (joystick A/Cross ATAU keyboard SPACE/T)
        di /hydroships/mission/start_autonomous, lalu LEAN_RECORD samping
        hook -> REVERSE_RETURN -> APPROACH_HOOK. ponytail: timeout 600s ABORT."""
        if self.wall is None: self._to(St.ABORT); return
        target_heading = math.radians(WALL_HEADING_DEG[self.wall])
        self._set_depth(self.depth_surface)
        self._set_heading(target_heading)
        self._set_surge(0.0, 0.0)
        if self._trigger_received:
            self.get_logger().info('Trigger diterima (SPACE/joystick) -> LEAN_RECORD dinding %s' % self.wall)
            self._trigger_received = False
            self._to(St.LEAN_RECORD)
            return
        if self._elapsed() > self.T['wait_trigger']:
            self.get_logger().error('WAIT_TRIGGER timeout — trigger tak diterima')
            self._to(St.ABORT)

    def _lean_wall_xy(self, wall):
        d = self.wall_face - self.lean_wall_offset
        s = self.lean_side_offset
        return {'A': (s, -d), 'B': (-s, d), 'C': (d, s), 'D': (-d, -s)}[wall]

    def _st_lean_record(self):
        if self.wall is None: self._to(St.ABORT); return
        wall_yaw = math.radians(WALL_HEADING_DEG[self.wall])
        side_yaw = wrap_to_pi(wall_yaw + math.radians(self.docking_yaw))
        phase = getattr(self, '_lean_phase', 1)
        if phase == 1:
            if self.x is None or self.yaw is None:
                self.get_logger().info('LEAN fase1: tunggu odom...', throttle_duration_sec=2.0)
                return
            if self._locked_yaw is None:
                self._locked_yaw = self.yaw
            self._set_heading(self._locked_yaw)
            self._set_depth(self.depth_surface)
            d = self.wall_face - self.lean_wall_offset
            wx, wy = {'A': (0.0, -d), 'B': (0.0, d), 'C': (d, 0.0), 'D': (-d, 0.0)}[self.wall]
            if not self._lean_log or math.hypot(self.x - self._lean_log[-1][0], self.y - self._lean_log[-1][1]) > 0.02 or abs(wrap_to_pi(self.yaw - self._lean_log[-1][2])) > math.radians(8):
                self._lean_log.append((self.x, self.y, self.yaw))
            dist = self._goto_xy(wx, wy, fmax=self.nav_fmax, yaw_ref=self._locked_yaw)
            if int(self._elapsed() * 2) % 10 == 0:
                self.get_logger().info('LEAN fase1 hold-hdg holonomic->wall: dist %.2f log %d' % (dist, len(self._lean_log)))
            if dist < 0.25:
                self._lean_phase = 2
                self._hold_since = None
                self.get_logger().info('LEAN fase1 tiba wall -> docking rotate %.0f deg' % self.docking_yaw)
            if self._elapsed() > self.T['lean'] * 0.6:
                self._lean_phase = 2
                self._hold_since = None
            return
        if phase == 2:
            if self._locked_yaw is None:
                self._locked_yaw = self.yaw or side_yaw
            # tahan heading sampai wall, baru rotate ditempat
            self._set_heading(side_yaw)
            self._set_depth(self.depth_surface)
            self._set_surge(0.0, 0.0)
            if self.x is not None and self.y is not None:
                if not self._lean_log or math.hypot(self.x - self._lean_log[-1][0], self.y - self._lean_log[-1][1]) > 0.02 or abs(wrap_to_pi(self.yaw - self._lean_log[-1][2])) > math.radians(8):
                    self._lean_log.append((self.x, self.y, self.yaw))
            if abs(wrap_to_pi(side_yaw - (self.yaw or side_yaw))) < math.radians(10):
                self._locked_yaw = side_yaw
                self._lean_phase = 3
                self._hold_since = None
                self.get_logger().info('LEAN docking 90deg done -> fase3 holonomic samping')
            if self._elapsed() > self.T['lean']:
                self._locked_yaw = side_yaw
                self._lean_phase = 3
            return
        self._set_heading(side_yaw)
        self._set_depth(self.depth_surface)
        if self.x is not None and self.y is not None:
            if not self._lean_log or math.hypot(self.x - self._lean_log[-1][0], self.y - self._lean_log[-1][1]) > 0.02 or abs(wrap_to_pi(self.yaw - self._lean_log[-1][2])) > math.radians(8):
                self._lean_log.append((self.x, self.y, self.yaw))
                if len(self._lean_log) > self.lean_log_cap:
                    self._lean_log.pop(0)
        tx, ty = self._lean_wall_xy(self.wall)
        dist = self._goto_xy(tx, ty, fmax=self.nav_fmax)
        if int(self._elapsed() * 2) % 10 == 0:
            self.get_logger().info('LEAN fase3 samping: dist %.2f log %d' % (dist, len(self._lean_log)))
        if dist < self.lean_tol:
            if self._hold_since is None:
                self._hold_since = self._now()
            if self._now() - self._hold_since >= self.lean_hold_s:
                self.get_logger().info('LEAN hold %.1fs @wall -> REVERSE log %d' % (self.lean_hold_s, len(self._lean_log)))
                self._set_surge(0.0, 0.0)
                self._lean_phase = 1
                self._to(St.REVERSE_RETURN)
                return
        else:
            self._hold_since = None
        if self._elapsed() > self.T['lean']:
            self._lean_phase = 1
            self._to(St.REVERSE_RETURN)

    def _st_reverse_return(self):
        if self.wall is None: self._to(St.ABORT); return
        wall_yaw = math.radians(WALL_HEADING_DEG[self.wall])
        self._set_depth(self.depth_surface)
        self._set_heading(wall_yaw)
        if not self._lean_log:
            hx, hy = self._hang_xy(self.wall, wall_yaw)
            dist = self._goto_xy(hx, hy, fmax=self.nav_fmax, yaw_ref=wall_yaw)
            if dist < 0.18:
                self.get_logger().info('REVERSE log kosong hook %.2f -> AUTO_RELEASE' % dist)
                self._to(St.AUTO_RELEASE)
            return
        if self._reverse_idx < 0 or self._reverse_idx >= len(self._lean_log):
            self._reverse_idx = len(self._lean_log) - 1
        ent = self._lean_log[self._reverse_idx]
        if len(ent) == 3:
            tx, ty, tyaw = ent
        else:
            tx, ty = ent[:2]; tyaw = wall_yaw
        self._set_heading(tyaw)
        dist = self._goto_xy(tx, ty, fmax=self.nav_fmax, yaw_ref=tyaw)
        stuck = getattr(self, '_rev_stuck', 0)
        if dist > 0.11 and self._elapsed() > 6.0:
            stuck += 1
        else:
            stuck = 0
        self._rev_stuck = stuck
        effective_tol = self.reverse_step_tol if stuck < 40 else 0.20
        if int(self._elapsed() * 2) % 10 == 0:
            self.get_logger().info('REVERSE log %d/%d dist %.2f tol %.2f stuck %d' % (self._reverse_idx, len(self._lean_log), dist, effective_tol, stuck))
        if dist < effective_tol:
            self._reverse_idx -= 1
            self._rev_stuck = 0
            if self._reverse_idx < 0:
                self.get_logger().info('REVERSE done log %d -> AUTO_RELEASE' % len(self._lean_log))
                self._set_surge(0.0, 0.0)
                self._to(St.AUTO_RELEASE)
                return
        elif stuck > 80:
            self.get_logger().warn('REVERSE skip stuck %d (dist %.2f) -> next' % (self._reverse_idx, dist))
            self._reverse_idx -= 1
            self._rev_stuck = 0
        if self._elapsed() > self.T['reverse']:
            self.get_logger().warn('REVERSE timeout step %d -> AUTO_RELEASE' % self._reverse_idx)
            self._to(St.AUTO_RELEASE)

    def _st_approach_hook(self):
        """Misi 5 (AUTONOMOUS) fase 1: visual servo PD ke hook memakai
        /hydroships/hook_offset (hook_detector). Tanpa ini AUTO_RELEASE melepas
        payload murni berdasarkan odometri (_hang_xy) tanpa konfirmasi kamera.
        Bila deteksi hilang, fallback ke target odometri itu (perilaku lama, aman).
        Payload masih dijepit di sini — detach baru terjadi di AUTO_RELEASE."""
        if self.wall is None: self._to(St.ABORT); return
        self._set_heading(math.radians(WALL_HEADING_DEG[self.wall]))

        # Backoff dikit dulu sebelum servo hook -- cegah agresif nabrak,
        # kasih jarak servo lihat hook dari lebih jauh.
        if not self._hook_backoff_done:
            # Kedalaman SAFE di ATAS puncak tip (bukan hook_depth!): payload
            # dijepit KAKU di gripper, jadi kedalaman base saat approach
            # menentukan di mana plat melayang. Menyelam ke hook_depth saat
            # masih approach = plat terdorong ke BAWAH melewati titik berhenti
            # (depth hold melawan kontak hook) -> plat MACET di bawah palang,
            # lalu AUTO_RELEASE detach saat plat belum bersandar -> jatuh.
            # Run 2026-08-17: depth 0.36-0.37 selama approach, release di 0.36
            # -> payload jatuh dari hook (bukan karena hitbox hilang).
            self._set_depth(self.hang_approach_depth)
            self._set_surge(-4.0, 0.0)   # dorong mundur pelan, fixed
            if self._hold_since is None:
                self._hold_since = self._now()
            if self._now() - self._hold_since >= 0.6:   # s durasi backoff
                self._hook_backoff_done = True
                self._hold_since = None
                self._set_surge(0.0, 0.0)
                self.get_logger().info('APPROACH_HOOK: backoff selesai, mulai servo')
            return

        off = self._hook_fresh()

        if off is not None:
            tip_x, tip_y = self._tip_xy(self.wall)
            rx, ry = tip_x - (self.x or tip_x), tip_y - (self.y or tip_y)
            dist_fwd = math.hypot(rx, ry)
            if dist_fwd > 0.80:
                tx, ty = self._hang_xy(self.wall)
                dist = self._goto_xy(tx, ty, fmax=self.nav_fmax, yaw_ref=math.radians(WALL_HEADING_DEG[self.wall]))
                self._set_depth(self.hang_approach_depth)
                self.get_logger().info('APPROACH_HOOK odom %.2fm -> hook' % dist, throttle_duration_sec=1.0)
                return
            ey_tgt = hook_ey_target(
                self.hang_approach_depth, self.cam_front_dz, self.hook_z,
                dist_fwd, self.cam_vfov_half_tan, self.hook_ey_max)
            cmd = hook_servo(off, self.vx, self.vy, self.hook_depth,
                             self.hook_gains, ey_target=ey_tgt)
            # Depth TETAP di kedalaman approach (di atas tip) — koreksi depth
            # hook_servo (hook_depth + kp*(ey-ey_tgt), bisa sampai +0.20 m) dulu bikin
            # ROV menyelam melewati titik berhenti plat dan macet di bawah
            # hook. Descent dilakukan TERKONTROL di AUTO_RELEASE fase 2 (dgn
            # stall detector), sama seperti HANG. cmd.target_depth diabaikan.
            # [Dicoba 2026-08-24: clamp turun kecil ke [hang_approach_depth,
            # +0.08]. DIBATALKAN -- sweep depth 0.14 vs 0.30 di wall C
            # menunjukkan ey REAL nyaris tak bergerak (0.435 -> 0.406) walau
            # depth berubah besar, jadi clamp kecil pasti tak berefek & cuma
            # nambah kompleksitas. Root cause ey tinggi di wall C/D BUKAN
            # soal depth/geometri hook_ey_target -- lihat STATUS.md.]
            self._set_depth(self.hang_approach_depth)
            # Sudah dekat tapi belum terpusat: stop maju, koreksi lateral saja.
            self._set_surge(0.0 if cmd.near else cmd.surge, cmd.sway)
            aligned_ok = (abs(off[0]) < self.hook_gains.center_tol
                          and abs(off[1] - ey_tgt) < self.hook_gains.center_tol)
            if cmd.near and aligned_ok:
                if self._hold_since is None:
                    self._hold_since = self._now()
                if self._now() - self._hold_since >= self.hold_settle_s:
                    self._set_surge(0.0, 0.0)
                    self.get_logger().info(
                        'APPROACH_HOOK: hook terpusat (ex %.2f ey %.2f size %.2f) -> AUTO_RELEASE'
                        % off)
                    self._to(St.AUTO_RELEASE)
                    return
            else:
                self._hold_since = None
        else:
            self._set_depth(self.hang_approach_depth)
            # fallback presisi
            wall_yaw = math.radians(WALL_HEADING_DEG[self.wall])
            yaw_err = abs(wrap_to_pi(wall_yaw - (self.yaw or wall_yaw)))
            if yaw_err < math.radians(10) and self.yaw is not None:
                tx, ty = self._hang_xy(self.wall, self.yaw)
                yaw_ref = self.yaw
            else:
                tx, ty = self._hang_xy(self.wall)
                yaw_ref = wall_yaw
            dist = self._goto_xy(tx, ty, fmax=self.nav_fmax, yaw_ref=yaw_ref)
            l_err = abs((tx - (self.x or 0)) * math.cos(wall_yaw) + (ty - (self.y or 0)) * math.sin(wall_yaw))
            ok = dist < self.hang_tol and l_err < self.hang_l_tol and yaw_err < self.hang_yaw_tol
            dwell = update_dwell(ok, self._now(), self._hold_since, self._hook_bad_since, self.hold_settle_s, self.hook_settle_grace_s)
            self._hold_since, self._hook_bad_since = dwell.hold_since, dwell.bad_since
            if dwell.done:
                self._set_surge(0.0, 0.0)
                self.get_logger().warn('APPROACH_HOOK fallback presisi dist %.3f l_err %.1fmm yaw %.1f -> AUTO_RELEASE' % (dist, l_err*1000, math.degrees(yaw_err)))
                self._to(St.AUTO_RELEASE)
                return

        if int(self._elapsed() * 2) % 20 == 0:
            ey_tgt_dbg = hook_ey_target(
                self.hang_approach_depth, self.cam_front_dz, self.hook_z,
                max(0.5, dist_fwd if off is not None else 0.8),
                self.cam_vfov_half_tan, self.hook_ey_max) if 'dist_fwd' in locals() else 0.0
            self.get_logger().info(
                'APPROACH_HOOK dbg: off=%s ey_tgt=%+.2f depth=%.2f roll=%+.1f pitch=%+.1f'
                % (off, ey_tgt_dbg, self.depth if self.depth is not None else -99.0,
                   math.degrees(self.roll) if self.roll is not None else -99.0,
                   math.degrees(self.pitch) if self.pitch is not None else -99.0))
        if self._elapsed() > self.T['approach']:
            # Jangan abort: AUTO_RELEASE punya station-keep sendiri sebelum detach.
            self.get_logger().warn('APPROACH_HOOK timeout -> lanjut AUTO_RELEASE')
            self._set_surge(0.0, 0.0)
            self._to(St.AUTO_RELEASE)

    def _st_auto_release(self):
        """Misi 5 (AUTONOMOUS): posisikan LUBANG payload di atas TIP hook
        (odometri presisi, sama seperti HANG), turun, DETACH, beri waktu payload
        bersandar di hook, lalu naik ke permukaan — semua tanpa input pilot."""
        if self.wall is None: self._to(St.ABORT); return
        target_heading = math.radians(WALL_HEADING_DEG[self.wall])
        self._set_heading(target_heading)
        if not self._detach_sent:
            # Kompensasi offset heading sama seperti HANG (lihat _st_hang).
            yaw_err = (abs(wrap_to_pi(target_heading - self.yaw))
                       if self.yaw is not None else math.pi)
            if yaw_err < math.radians(10.0) and self.yaw is not None:
                tx, ty = self._hang_xy(self.wall, self.yaw)
                yaw_ref = self.yaw
            else:
                tx, ty = self._hang_xy(self.wall)
                yaw_ref = target_heading
            if not self._hang_pos_done:
                # Fase 1: posisikan lubang di atas tip pada kedalaman aman
                # + gate heading + gate lubang (sama spt HANG). Yaw rotasi
                # ikut yaw_ref.
                self._set_depth(self.hang_approach_depth)
                dist = self._goto_xy(tx, ty, fmax=self.nav_fmax,
                                     yaw_ref=yaw_ref)
                l_err = abs((tx - (self.x or 0)) * math.cos(target_heading)
                            + (ty - (self.y or 0)) * math.sin(target_heading))
                if (dist < self.hang_tol and l_err < self.hang_l_tol
                        and yaw_err < self.hang_yaw_tol):
                    if self._hold_since is None:
                        self._hold_since = self._now()
                    if self._now() - self._hold_since >= self.hold_settle_s:
                        self._set_surge(0.0)
                        self._hang_pos_done = True
                        self._hold_since = None
                        self.get_logger().info(
                            'AUTO_RELEASE: lubang di atas hook (dist %.3f, l_err %.1f mm, '
                            'yaw_err %.1f°)'
                            % (dist, l_err * 1000.0, math.degrees(yaw_err)))
                else:
                    self._hold_since = None
                if self._elapsed() > self.T['release']:
                    self.get_logger().error('AUTO_RELEASE timeout (posisi)')
                    self._to(St.ABORT)
                return
            # Fase 2: turun (tip menembus lubang), lalu detach. Pertahankan
            # posisi lubang di atas tip selama turun (sama spt HANG).
            dist = self._goto_xy(tx, ty, fmax=0.6 * self.nav_fmax,
                                 yaw_ref=yaw_ref)
            self._set_depth(self.hook_depth)
            # Gate presisi saat duduk (sama spt HANG fase 2): selain stall,
            # uji ulang dist/l_err. Tanpa ini detach bisa terjadi dgn lubang
            # sudah bergeser dr tip (kontak saat turun) -> payload bersandar
            # MIRING di palang (terukur 25-32 mm off, tilt 8-12°).
            l_err = abs((tx - (self.x or 0)) * math.cos(target_heading)
                        + (ty - (self.y or 0)) * math.sin(target_heading))
            # Deteksi "plat SUDAH bersandar" = depth STALL (ROV terblok palang),
            # bukan ambang kedalaman: ambang 0.29 dulu lolos saat ROV masih turun
            # (depth 0.31 < titik berhenti 0.32) -> detach di udara -> payload jatuh
            # & terlempar dari hook. Depth hold menekan plat ke palang; begitu
            # terblok, depth berhenti naik -> stall hold_settle_s -> seated.
            d = self.depth
            depth_stalled = False
            if d is not None:
                # Histeresis 5 mm: creeps mikro (0.1 mm/tick saat plat menyentuh
                # palang & depth-hold menekan) jangan dianggap "bergerak" — tanpa
                # ini tiap kenaikan kecil men-set _hang_depth_max baru & me-reset
                # _hold_since, timer stall tak pernah menumpuk -> AUTO_RELEASE
                # timeout padahal plat sudah SEATED (terjadi saat hook_depth
                # diturunkan 0.45 -> 0.32 utk offset gripper z=-0.13).
                if self._hang_depth_max is None or d > self._hang_depth_max + 0.005:
                    self._hang_depth_max = max(d, self._hang_depth_max or 0.0)
                    self._hold_since = None
                elif (d >= self._hang_depth_max - 0.015
                      and d >= self.hang_approach_depth + 0.03
                      and yaw_err < self.hang_yaw_tol
                      and dist < self.hang_tol
                      and l_err < self.hang_l_tol):
                    if self._hold_since is None:
                        self._hold_since = self._now()
                    if self._now() - self._hold_since >= self.hold_settle_s:
                        depth_stalled = True
                else:
                    self._hold_since = None
            else:
                self._hold_since = None
            if depth_stalled:
                self.get_logger().info(
                    'AUTO_RELEASE: plat di palang hook (depth %.2f, stall), publish detach...'
                    % d)
                self.pub_detach.publish(Empty())
                # Reset state gripper utk siklus berikutnya (GRAB ke-2 dst): tanpa
                # ini logic.attached tetap True -> close berikutnya diabaikan.
                self.pub_grip.publish(String(data='open'))
                self._detach_sent = True
                self._set_surge(0.0, 0.0)
                # NAIK sedikit langsung setelah detach — jangan biarkan depth hold
                # menekan plat yg baru lepas (plat berputar jadi vertikal & bisa
                # tersangkut gripper). Naik ke atas titik berhenti lalu permukaan.
                self._set_depth(max(self.hang_approach_depth,
                                    self.hook_depth - 0.08))
                self._hold_since = self._now()
            if self._elapsed() > self.T['release']:
                # Plat tidak berhasil duduk presisi (stall tapi dist/l_err gagal,
                # atau kedalaman tak pernah stall). RETRY: naik lagi, re-center,
                # turun ulang — bukan ABORT langsung. DetachableJoint masih
                # menahan payload, jadi mengangkat plat & menyelam ulang memberi
                # kesempatan threading lubang ke tip yg lebih baik (drift lateral
                # saat turun adalah kelemahan terdokumentasi, bukan kegagalan
                # permanen). Habis retry -> ABORT jujur.
                if self._release_retries < self.release_max_retries:
                    self._release_retries += 1
                    self._hang_pos_done = False
                    self._hang_depth_max = None
                    self._set_depth(self.hang_approach_depth)
                    self.get_logger().warn(
                        'AUTO_RELEASE retry %d/%d (depth %s, dist %.3f) — '
                        'naik & coba turun ulang'
                        % (self._release_retries, self.release_max_retries,
                           self.depth if self.depth is not None else 'n/a', dist))
                else:
                    self.get_logger().error(
                        'AUTO_RELEASE timeout (turun, depth %s, retry habis)'
                        % (self.depth if self.depth is not None else 'n/a'))
                    self._to(St.ABORT)
            return

        # Detach sudah terkirim: beri waktu payload bersandar di hook, lalu naik.
        if self._now() - self._hold_since >= 1.5:
            self._set_depth(self.depth_surface)
        if self.depth is not None and self.depth <= self.depth_surface + 0.05:
            self._set_surge(0.0)
            self.score['m5'] = 40
            self._detach_sent = False
            self.done_hooks.add(self.wall)
            self.get_logger().info(
                'Payload dilepas & ROV permukaan dekat dinding (+40). Done: %s'
                % self.done_hooks)
            if len(self.done_hooks) >= 4:
                self._print_score(); self._to(St.DONE)
            else:
                # Multi-payload: minta spawner buat payload baru (QR = wall
                # berikutnya) utk siklus berikutnya, dan RESET payload_pose agar
                # DIVE menunggu pose BARU (latched lama = posisi payload yg sudah
                # diambil & dipindah).
                nxt = self._next_wall()
                if nxt is not None:
                    m = String(data=nxt)
                    self.pub_spawn_next.publish(m)
                    self.get_logger().info(
                        'Minta payload baru utk wall %s (done: %s)'
                        % (nxt, sorted(self.done_hooks)))
                self.payload_pose = None
                self._payload_pose_fallback = False
                self.wall = None
                self._to(St.DIVE)
        elif self._elapsed() > self.T['release']:
            self.get_logger().error('AUTO_RELEASE timeout (naik ke permukaan)'); self._to(St.ABORT)

    def _print_score(self):
        s = self.score; tot = sum(s.values())
        self.get_logger().info('SKOR: m1=%d m2=%d m3=%d m4=%d m5=%d TOTAL=%d/100'
                               % (s['m1'], s['m2'], s['m3'], s['m4'], s['m5'], tot))


def main(args=None):
    rclpy.init(args=args)
    node = MissionFSM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pub_manual.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()