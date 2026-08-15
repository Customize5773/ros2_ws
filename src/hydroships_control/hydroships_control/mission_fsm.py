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

from hydroships_control.hook_logic import HookServoGains, hook_servo, update_dwell
# qr_ey_target dipindah ke qr_logic (modul murni) supaya gripper_controller
# memakai geometri yang SAMA PERSIS untuk gerbang attach-nya — sebelumnya FSM
# membidik ey_target~-0.52 sementara GripperLogic.is_safe() menuntut |ey|<=0.30,
# dua kriteria yang mustahil dipenuhi bersamaan sehingga attach tak pernah
# terpicu. Di-re-export di sini agar `mission_fsm.qr_ey_target` tetap resolve
# (dipakai test/test_qr_ey_target.py dan reduce_approach_qr.py).
from hydroships_control.qr_logic import qr_ey_target  # noqa: F401 (re-export)


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
        # R-11 Opsi 2: saat depth_ok tercapai di DESCEND, beri waktu tambahan
        # `descend_recenter_timeout` supaya servo visual memperbaiki offset
        # QR sebelum GRAB — hanya bila qr_off masih segar (camera masih melihat
        # QR). Jika stale (QR hilang, normal di grab_depth), lanjutkan GRAB.
        p('descend_recenter_timeout', 5.0)
        p('hook_depth', 0.45)        # m kedalaman hook (lihat arena)
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
        p('t_hang', 20.0); p('t_surface', 20.0); p('t_wait_trigger', 600.0)
        p('t_release', 30.0); p('t_approach', 25.0)
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
        p('qr_floor_z', -0.894)      # m, tinggi bidang QR di dunia (payload_spawner.py)
        p('cam_bottom_dz', 0.18)     # m, kamera bawah di bawah base_link
        # tan(setengah-FOV vertikal). hFOV 80° @ 4:3 -> atan(0.75*tan40°) = 32.2°.
        p('cam_vfov_half_tan', 0.6293)
        # Batas |ey_target| supaya QR tetap di dalam frame (1.0 = tepat di tepi).
        p('ey_target_max', 0.8)

        g = lambda n: self.get_parameter(n).value
        self.surge = float(g('surge_force'))
        self.depth_bottom = float(g('depth_bottom'))
        self.depth_surface = float(g('depth_surface'))
        self.depth_tol = float(g('depth_tol'))
        self.descend_depth_tol = float(g('descend_depth_tol'))
        self.descend_recenter_timeout = float(g('descend_recenter_timeout'))
        self.hook_depth = float(g('hook_depth'))
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
                   'wait_trigger', 'release', 'approach')}

        # I/O
        self.pub_depth = self.create_publisher(Float64, '/hydroships/setpoint/depth', 10)
        self.pub_head = self.create_publisher(Float64, '/hydroships/setpoint/heading', 10)
        self.pub_manual = self.create_publisher(Twist, '/hydroships/manual/cmd', 10)
        # Manipulator (rancang ulang M5): perintah semantik open/close ke
        # gripper_controller (yg memicu gz DetachableJoint attach/detach).
        self.pub_grip = self.create_publisher(String, '/hydroships/gripper/command', 10)
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
        self._detach_sent = False
        self._hook_backoff_done = False
        self.gripper_status = None   # ack terakhir dari gripper_controller (observability, tak memicu transisi)

        # State
        self.depth = None
        self.yaw = None
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
            self._hook_bad_since = None
        if s is St.GRAB:
            # R-9: status ack lama tak boleh terbawa ke siklus GRAB berikutnya.
            self.gripper_status = None
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

    def _goto_xy(self, tx, ty, fmax=None, min_fmax_frac=None):
        """PD posisi HOLONOMIK: dorong ROV ke (tx,ty) dunia via gaya horizontal
        body-frame (surge+sway), TANPA mengubah heading — dipakai saat sudah
        menghadap arah yang benar (mis. setelah NAV_WALL) & cuma perlu
        koreksi posisi kecil. Kembalikan jarak sisa (m)."""
        if self.x is None or self.yaw is None:
            return 999.0
        fm = self.approach_fmax if fmax is None else fmax
        ex, ey = tx - self.x, ty - self.y
        dist = math.hypot(ex, ey)
        # Taper gaya maks saat mendekati target (slow-down radius) -> cegah slam.
        slow_radius = 1.0  # m, mulai perlambat dalam radius ini
        # P0-2.5 Kandidat #3 (docs/P0-2-5-ENGINEERING-ANALYSIS.md, isolasi dari
        # Kandidat #1/#2): default 0.05 tetap hardcoded di sini SUPAYA caller
        # lain (_st_hang L720, _st_nav_wall-adjacent L830) TIDAK ikut berubah —
        # hanya _st_approach_qr yang meneruskan min_fmax_frac non-default.
        if min_fmax_frac is None:
            min_fmax_frac = 0.05  # jangan sampai gaya nol total (masih perlu lawan drag/arus)
        if dist < slow_radius:
            frac = max(min_fmax_frac, dist / slow_radius)
            fm = fm * frac
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        bx = ex * c + ey * s
        by = -ex * s + ey * c
        cl = lambda v: max(-fm, min(fm, v))
        surge = self.approach_kp * bx - self.approach_kd * self.vx
        sway = self.approach_kp * by - self.approach_kd * self.vy
        self._set_surge(cl(surge), cl(sway))
        return dist

    def _wall_xy(self, wall):
        d = self.wall_dist
        return {'A': (0.0, -d), 'B': (0.0, d), 'C': (d, 0.0), 'D': (-d, 0.0)}[wall]

    def _hook_xy(self, wall):
        d = self.wall_dist - self.hook_dist
        lat = self.hook_lateral_offset
        return {'A': (lat, -d), 'B': (lat, d), 'C': (d, lat), 'D': (-d, lat)}[wall]

    # ---- callbacks ----
    def _on_depth(self, msg): self.depth = msg.data

    def _on_gripper_status(self, msg): self.gripper_status = msg.data

    def _on_odom(self, msg):
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.vx = msg.twist.twist.linear.x
        self.vy = msg.twist.twist.linear.y

    def _on_qr(self, msg):
        w = (msg.data or '').strip().upper()
        if w in WALL_HEADING_DEG:
            self.qr_wall = w; self.qr_time = self._now()

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
                if self._wall_idx >= len(self._wall_sequence):
                    self.get_logger().info('Semua wall selesai, misi tuntas.')
                    self._print_score(); self._to(St.DONE); return
                self.wall = self._wall_sequence[self._wall_idx]
                self._wall_idx += 1
                self.score['m1'] = 15
                self._wall_scored = True
                self.get_logger().info('QR tidak ter-decode, wall urutan %s '
                                       'dipilih (+15) [urutan ke-%d] (%s) [%s]'
                                       % (self.wall, self._wall_idx,
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

        if depth_ok:
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
        gerbang menolaknya (lihat P1-OWNER-DECISIONS-AND-ROADMAP.md R-9)."""
        self._set_surge(0.0)
        if self.gripper_status == 'attached':
            self.score['m2'] = 15
            self.get_logger().info('GRAB terverifikasi (+15) -- ack attached')
            self._to(St.NAV_WALL)
            return
        if self._hold_since is None or self.gripper_status == 'rejected':
            self._hold_since = self._now()
            self.gripper_status = None
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
        self._set_depth(self.hook_depth)
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
        """Misi 3/4 (REMOTELY): dekati hook presisi & tahan stabil (payload
        tergantung, gripper masih menjepit). Tak ada release di sini. Sudah
        menghadap wall sejak NAV_WALL — pakai holonomik (sway) utk koreksi
        lateral kecil TANPA berputar lagi."""
        if self.wall is None: self._to(St.ABORT); return
        tx, ty = self._hook_xy(self.wall)
        target_heading = math.radians(WALL_HEADING_DEG[self.wall])
        self._set_depth(self.hook_depth)
        self._set_heading(target_heading)
        dist = self._goto_xy(tx, ty, fmax=self.nav_fmax)
        if dist < self.nav_tol:
            if self._hold_since is None:
                self._hold_since = self._now()
            if self._now() - self._hold_since >= self.hold_settle_s:
                self._set_surge(0.0)
                self.score['m3'] = 15
                self.get_logger().info('Payload tergantung stabil di hook %s (+15)' % self.wall)
                self._to(St.SURFACE)
        else:
            self._hold_since = None
        if self._elapsed() > self.T['hang']:
            self.get_logger().error('HANG timeout'); self._to(St.ABORT)

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
        """Menunggu pilot menekan trigger setelah bersandar di dinding.
        Selama menunggu, tahan posisi (depth permukaan, heading TETAP
        menghadap wall — sama seperti SURFACE/HANG, jangan reset ke 0)."""
        if self.wall is None: self._to(St.ABORT); return
        target_heading = math.radians(WALL_HEADING_DEG[self.wall])
        self._set_depth(self.depth_surface)
        self._set_heading(target_heading)
        self._set_surge(0.0, 0.0)
        if self._trigger_received:
            self.get_logger().info('Mulai misi pelepasan payload AUTONOMOUS')
            self._to(St.APPROACH_HOOK)
        elif self._elapsed() > self.T['wait_trigger']:
            self.get_logger().error('WAIT_TRIGGER timeout — trigger tak diterima')
            self._to(St.ABORT)

    def _st_approach_hook(self):
        """Misi 5 (AUTONOMOUS) fase 1: visual servo PD ke hook memakai
        /hydroships/hook_offset (hook_detector). Tanpa ini AUTO_RELEASE melepas
        payload murni berdasarkan odometri (_hook_xy) tanpa konfirmasi kamera.
        Bila deteksi hilang, fallback ke target odometri itu (perilaku lama, aman).
        Payload masih dijepit di sini — detach baru terjadi di AUTO_RELEASE."""
        if self.wall is None: self._to(St.ABORT); return
        self._set_heading(math.radians(WALL_HEADING_DEG[self.wall]))

        # Backoff dikit dulu sebelum servo hook -- cegah agresif nabrak,
        # kasih jarak servo lihat hook dari lebih jauh.
        if not self._hook_backoff_done:
            self._set_depth(self.hook_depth)
            self._set_surge(-8.0, 0.0)   # dorong mundur pelan, fixed
            if self._hold_since is None:
                self._hold_since = self._now()
            if self._now() - self._hold_since >= 1.2:   # s durasi backoff
                self._hook_backoff_done = True
                self._hold_since = None
                self._set_surge(0.0, 0.0)
                self.get_logger().info('APPROACH_HOOK: backoff selesai, mulai servo')
            return

        off = self._hook_fresh()

        if off is not None:
            cmd = hook_servo(off, self.vx, self.vy, self.hook_depth, self.hook_gains)
            self._set_depth(cmd.target_depth)
            # Sudah dekat tapi belum terpusat: stop maju, koreksi lateral saja.
            self._set_surge(0.0 if cmd.near else cmd.surge, cmd.sway)
            dwell = update_dwell(cmd.near and cmd.aligned, self._now(),
                                  self._hold_since, self._hook_bad_since,
                                  self.hold_settle_s, self.hook_settle_grace_s)
            self._hold_since, self._hook_bad_since = dwell.hold_since, dwell.bad_since
            if dwell.done:
                self._set_surge(0.0, 0.0)
                self.get_logger().info(
                    'APPROACH_HOOK: hook terpusat (ex %.2f ey %.2f size %.2f) -> AUTO_RELEASE'
                    % off)
                self._to(St.AUTO_RELEASE)
                return
        else:
            # Fallback open-loop: target odometri, sama seperti AUTO_RELEASE lama.
            self._set_depth(self.hook_depth)
            tx, ty = self._hook_xy(self.wall)
            dist = self._goto_xy(tx, ty, fmax=self.nav_fmax)
            dwell = update_dwell(dist < self.nav_tol, self._now(),
                                  self._hold_since, self._hook_bad_since,
                                  self.hold_settle_s, self.hook_settle_grace_s)
            self._hold_since, self._hook_bad_since = dwell.hold_since, dwell.bad_since
            if dwell.done:
                self._set_surge(0.0, 0.0)
                self.get_logger().warn(
                    'APPROACH_HOOK: tak ada deteksi hook, pakai target odometri '
                    '(dist %.2fm) -> AUTO_RELEASE' % dist)
                self._to(St.AUTO_RELEASE)
                return

        if int(self._elapsed() * 2) % 20 == 0:
            self.get_logger().info(
                'APPROACH_HOOK dbg: off=%s depth=%.2f'
                % (off, self.depth if self.depth is not None else -99.0))
        if self._elapsed() > self.T['approach']:
            # Jangan abort: AUTO_RELEASE punya station-keep sendiri sebelum detach.
            self.get_logger().warn('APPROACH_HOOK timeout -> lanjut AUTO_RELEASE')
            self._set_surge(0.0, 0.0)
            self._to(St.AUTO_RELEASE)

    def _st_auto_release(self):
        """Misi 5 (AUTONOMOUS) fase 2: tahan posisi hasil servo APPROACH_HOOK,
        lepas payload (publish ke detach topic — DetachableJoint), lalu naik ke
        permukaan dekat dinding — semua tanpa input pilot."""
        if self.wall is None: self._to(St.ABORT); return
        target_heading = math.radians(WALL_HEADING_DEG[self.wall])
        self._set_heading(target_heading)
        if not self._detach_sent:
            # APPROACH_HOOK sudah memosisikan ROV di atas hook. Jangan navigasi
            # ulang ke _hook_xy — itu justru menarik ROV kembali menjauh dari
            # posisi yg baru dikonfirmasi kamera. Cukup station-keep (redam
            # kecepatan sisa) selama hold_settle_s, lalu detach.
            self._set_depth(self.hook_depth)
            brake_kd = 40.0
            cl = lambda v: max(-self.nav_fmax, min(self.nav_fmax, v))
            self._set_surge(cl(-brake_kd * self.vx), cl(-brake_kd * self.vy))
            if self._hold_since is None:
                self._hold_since = self._now()
            if self._now() - self._hold_since >= self.hold_settle_s:
                self.get_logger().info('AUTO_RELEASE: posisi stabil, publish detach...')
                self.pub_detach.publish(Empty())
                self._detach_sent = True
                self._set_surge(0.0, 0.0)
            if self._elapsed() > self.T['release']:
                self.get_logger().error('AUTO_RELEASE timeout (belum detach)'); self._to(St.ABORT)
            return

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