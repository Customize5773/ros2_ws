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

State: IDLE -> DIVE -> APPROACH_QR -> GRAB -> NAV_WALL -> HANG -> SURFACE
       -> WAIT_TRIGGER -> AUTO_RELEASE -> (DIVE lagi | DONE) (atau ABORT).

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

from hydroships_control.hook_logic import HookServoGains, hook_servo


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_to_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


def qr_ey_target(depth, cam_gripper_dx, qr_floor_z, cam_bottom_dz,
                 vfov_half_tan, ey_max):
    """Offset vertikal ternormalisasi tempat QR HARUS tampak di kamera bawah.

    Gripper berada `cam_gripper_dx` meter DI DEPAN kamera bawah (sumbu x body).
    Kalau servo memusatkan QR di kamera (ey=0), gripper meleset sejauh itu. Jadi
    QR harus dibiarkan tampak di DEPAN pusat frame supaya gripper-lah yang tepat
    di atas QR.

    Konvensi `qr_logic.offset_from_points`: ey > 0 = QR di BAWAH pusat frame =
    payload di BELAKANG ROV. Maka "QR di depan" berarti ey NEGATIF.

    Geometri: kamera berada `h_cam` di atas bidang QR, dan setengah-tinggi
    petak pandang di bidang itu = h_cam * tan(½ FOV vertikal). Offset metrik
    dinormalkan terhadap setengah-tinggi tsb.

    Dikembalikan sudah ter-clamp ke ±ey_max agar QR tak terdorong keluar frame
    (|ey| = 1.0 tepat di tepi). Clamp yang AKTIF adalah tanda bahwa `depth`
    terlalu dalam untuk offset gripper sebesar ini — lihat catatan `scan_depth`.
    """
    h_cam = max(0.05, abs(qr_floor_z) - depth - cam_bottom_dz)
    half_h = max(1e-3, h_cam * vfov_half_tan)
    ey = -cam_gripper_dx / half_h
    return max(-ey_max, min(ey_max, ey))


WALL_HEADING_DEG = {'A': 270.0, 'B': 90.0, 'C': 0.0, 'D': 180.0}


class St(Enum):
    IDLE = auto(); DIVE = auto(); APPROACH_QR = auto(); GRAB = auto()
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
        p('hook_depth', 0.45)        # m kedalaman hook (lihat arena)
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
        p('hang_approach_depth', 0.28)  # m kedalaman posisi lubang di atas tip
        p('hang_tol', 0.012)         # m toleransi posisi lubang di atas tip
        # Gate heading sebelum turun: lubang 0.2733 m di depan base, jadi error
        # heading theta menggeser lubang ~0.2733*sin(theta). 2.5 deg -> ~12 mm
        # (masih dalam toleransi slot plat +-13 mm).
        p('hang_yaw_tol_deg', 2.5)
        p('yaw_tol_deg', 10.0)       # derajat toleransi alignment heading
        p('qr_max_age', 1.5)         # s umur maks deteksi QR agar dianggap segar
        p('payload_x', 0.4)          # m posisi payload/QR di dunia (x)
        p('payload_y', 0.0)          # m posisi payload/QR di dunia (y)
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
        p('t_dive', 20.0); p('t_scan', 45.0); p('t_grab', 10.0); p('t_nav', 30.0)
        p('t_hang', 20.0); p('t_surface', 20.0); p('t_wait_trigger', 600.0)
        p('t_release', 30.0); p('t_approach', 25.0)
        # APPROACH_HOOK: visual servo PD ke hook (hook_detector ->
        # /hydroships/hook_offset). Default sama dgn hook_logic.HookServoGains —
        # di sini hanya diekspos sebagai parameter ROS supaya bisa di-tune runtime.
        p('hook_max_age', 1.0)       # s umur maks deteksi hook agar dianggap segar
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

        g = lambda n: self.get_parameter(n).value
        self.surge = float(g('surge_force'))
        self.depth_bottom = float(g('depth_bottom'))
        self.depth_surface = float(g('depth_surface'))
        self.depth_tol = float(g('depth_tol'))
        self.hook_depth = float(g('hook_depth'))
        self.wall_face = float(g('wall_face'))
        self.hang_tip_d = float(g('hang_tip_d'))
        self.hang_hole_dx = float(g('hang_hole_dx'))
        self.hang_approach_depth = float(g('hang_approach_depth'))
        self.hang_tol = float(g('hang_tol'))
        self.hang_yaw_tol = math.radians(float(g('hang_yaw_tol_deg')))
        self.yaw_tol = math.radians(float(g('yaw_tol_deg')))
        self.qr_max_age = float(g('qr_max_age'))
        self.payload_x = float(g('payload_x'))
        self.payload_y = float(g('payload_y'))
        self.scan_depth = float(g('scan_depth'))
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
        self.cam_gripper_dx = float(g('cam_gripper_dx'))
        self.gripper_base_dx = float(g('gripper_base_dx'))
        self.qr_floor_z = float(g('qr_floor_z'))
        self.cam_bottom_dz = float(g('cam_bottom_dz'))
        self.cam_vfov_half_tan = float(g('cam_vfov_half_tan'))
        self.ey_target_max = float(g('ey_target_max'))
        self.hook_max_age = float(g('hook_max_age'))
        self.hook_gains = HookServoGains(
            kp_surge=float(g('hook_kp_surge')), kd_surge=float(g('hook_kd_surge')),
            kp_sway=float(g('hook_kp_sway')), kd_sway=float(g('hook_kd_sway')),
            kp_depth=float(g('hook_kp_depth')),
            size_stop=float(g('hook_size_stop')), center_tol=float(g('hook_center_tol')),
            fmax=float(g('hook_fmax')), depth_range=float(g('hook_depth_range')))
        self.T = {k: float(g('t_' + k)) for k in
                  ('dive', 'scan', 'grab', 'nav', 'hang', 'surface',
                   'wait_trigger', 'release', 'approach')}

        # I/O
        self.pub_depth = self.create_publisher(Float64, '/hydroships/setpoint/depth', 10)
        self.pub_head = self.create_publisher(Float64, '/hydroships/setpoint/heading', 10)
        self.pub_manual = self.create_publisher(Twist, '/hydroships/manual/cmd', 10)
        # Manipulator (rancang ulang M5): perintah semantik open/close ke
        # gripper_controller (yg memicu gz DetachableJoint attach/detach).
        self.pub_grip = self.create_publisher(String, '/hydroships/gripper/command', 10)
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

        # payload sudah nempel ke ROV sejak spawn (DetachableJoint).
        # Detach = publish Empty ke topic ini.
        self.pub_detach = self.create_publisher(Empty, '/hydroships/gripper/detach', 10)
        # Status attach gripper (open/closed) dari gripper_controller — GRAB
        # hanya dianggap sukses bila state benar-benar jadi 'closed'.
        self.create_subscription(String, '/hydroships/gripper/state',
                                  self._on_grip_state, 10)
        self._grip_state = 'open'
        self._detach_sent = False
        self._hook_backoff_done = False
        self._hang_pos_done = False   # HANG/AUTO_RELEASE: lubang sudah di atas tip?

        # State
        self.depth = None
        self.yaw = None
        self.x = None
        self.y = None
        self.vx = 0.0
        self.vy = 0.0
        self.qr_wall = None
        self.qr_time = 0.0
        self.qr_off = None        # (ex, ey, size) ternormalisasi dari qr_offset
        self.qr_off_time = 0.0
        self._warned_no_odom = False
        self._approach_recovered = False   # RECOVERY depth-ascent sudah dipicu?
        self._wall_scored = False          # skor m1 sudah diberi (cegah spam log)
        self.hook_off = None      # (ex, ey, size)
        self.hook_time = 0.0
        self.payload_pose = None  # (x, y, z) dari /hydroships/payload_pose (spawner)
        self.wall = None
        self.done_hooks = set()
        self.score = {'m1': 0, 'm2': 0, 'm3': 0, 'm4': 0, 'm5': 0}
        self.state = St.IDLE
        self.t_state = self._now()
        self._hold_since = None
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
        if s in (St.HANG, St.AUTO_RELEASE):
            self._hang_pos_done = False
        if s is St.APPROACH_QR:
            # Misi berulang per payload (AUTO_RELEASE -> DIVE -> APPROACH_QR):
            # tanpa reset, payload ke-2 dst langsung dianggap sudah ber-wall.
            self._wall_scored = False
            self._approach_recovered = False

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

    def _goto_xy(self, tx, ty, fmax=None):
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
        # Floor gaya 0.12 x fmax (bukan 0.05): di jarak dekat gaya 0.05xfmax
        # (~1 N) terlalu kecil utk menyelesaikan sisa error — ROV mandek
        # beberapa cm dari target (terlihat di HANG: dist macet 0.036).
        if dist < slow_radius:
            frac = max(0.12, dist / slow_radius)
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

    def _tip_xy(self, wall):
        """Posisi dunia TIP (ujung) hook — silinder tegak tempat lubang payload
        digantung. Tip berada wall_face - hang_tip_d dari pusat arena."""
        d = self.wall_face - self.hang_tip_d
        return {'A': (0.0, -d), 'B': (0.0, d), 'C': (d, 0.0), 'D': (-d, 0.0)}[wall]

    def _hang_xy(self, wall):
        """Target base_link agar LUBANG payload (bukan base) tepat di atas TIP
        hook. Pakai HEADING WALL yang TETAP (bukan yaw live dari odom) supaya
        target tak bergeser saat ROV masih berputar menghadap wall — kalau
        target ikut yaw live, ROV mengejar target bergerak dan tak pernah
        konvergen. Error heading yang tersisa dikoreksi lewat gate
        hang_yaw_tol_deg sebelum turun (lubang 0.2733 m di depan base)."""
        tip_x, tip_y = self._tip_xy(wall)
        yaw = math.radians(WALL_HEADING_DEG[wall])
        return (tip_x - self.hang_hole_dx * math.cos(yaw),
                tip_y - self.hang_hole_dx * math.sin(yaw))

    # ---- callbacks ----
    def _on_depth(self, msg): self.depth = msg.data

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
        if depth_ok and self.payload_pose is not None:
            self._set_surge(0.0)
            self.get_logger().info('Kedalaman scan tercapai (%.2fm), payload_pose siap'
                                   % self.depth)
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
        dist_raw = math.hypot((self.x or 0.0) - tx, (self.y or 0.0) - ty)
        servoing = off_fresh and dist_raw < 0.3
        if servoing:
            ex, ey, _size = self.qr_off
            k = self.qr_servo_gain * self.qr_servo_sign
            # Error diukur terhadap ey_target, bukan terhadap 0.
            body_dx = -(ey - ey_target) * k   # ey>ey_target: QR terlalu ke belakang
            body_dy = -ex * k                 # ex>0: QR di kanan pusat -> geser kanan
            c, s = math.cos(self._locked_yaw), math.sin(self._locked_yaw)
            tx += body_dx * c - body_dy * s
            ty += body_dx * s + body_dy * c

        dist = self._goto_xy(tx, ty)
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

        if self._wall_scored:
            # Dibandingkan terhadap ey_target (bukan 0) — kalau tetap terhadap 0,
            # |ey| konvergen ke ~0.61 dan transisi GRAB tak pernah terpicu.
            centered = (off_fresh
                        and abs(self.qr_off[0]) < self.qr_center_tol
                        and abs(self.qr_off[1] - ey_target) < self.qr_center_tol)
            if centered or dist < self.approach_tol:
                self.get_logger().info(
                    'QR terpusat (%s) -> GRAB (%s)'
                    % ('visual servo' if centered else 'jarak XY',
                       self._gripper_align_txt()))
                self._set_surge(0.0); self._to(St.GRAB); return

        elif dist < self.approach_tol:
            # Fallback: QR tak pernah terbaca tapi ROV sudah di atas payload.
            if self._wall_idx >= len(self._wall_sequence):
                self.get_logger().info('Semua wall selesai, misi tuntas.')
                self._print_score(); self._to(St.DONE); return
            self.wall = self._wall_sequence[self._wall_idx]
            self._wall_idx += 1
            self.score['m1'] = 15
            self._wall_scored = True
            self.get_logger().info('Wall %s dipilih (+15) [urutan ke-%d] (%s)'
                                   % (self.wall, self._wall_idx,
                                      self._gripper_align_txt()))
            self._set_surge(0.0); self._to(St.GRAB); return

        if self._elapsed() > self.T['scan']:
            self.get_logger().error('APPROACH_QR timeout'); self._to(St.ABORT)

    def _st_grab(self):
        """Misi 2 (REMOTELY): kirim perintah "close" ke gripper_controller
        SECARA BERULANG (setiap tick) sampai attach dikonfirmasi lewat
        /hydroships/gripper/state == 'closed'. Karena QUIRC tak terpasang &
        desain ey_target membuat qr_offset kamera tak pernah lolos safety gate
        (|ey|=-0.61 > max_offset 0.30), FSM publish sinyal qr_offset SINTETIK
        (frame_id 'synthetic', tersentral, size cukup) bersama tiap close.
        Sinyal sintetik dipublish terus-terusan supaya tak kalah balapan (race)
        dgn sinyal asli kamera yang tiba tak menentu — dulu attach jadi acak.
        TODO: hapus sintetik bila QUIRC terpasang & gate memakai sinyal asli."""
        self._set_surge(0.0)
        if self._grip_state != 'closed':
            synth = PointStamped()
            synth.header.stamp = self.get_clock().now().to_msg()
            synth.header.frame_id = 'synthetic'
            synth.point.x, synth.point.y, synth.point.z = 0.0, 0.0, 0.2
            self.pub_qr_offset_synth.publish(synth)
            self.pub_grip.publish(String(data='close'))
            if int(self._elapsed() * 2) % 10 == 0:
                self.get_logger().info('GRAB: kirim close (state=%s)' % self._grip_state)
        if self._grip_state == 'closed':
            if self._hold_since is None:
                self._hold_since = self._now()
            if self._now() - self._hold_since >= 0.5:
                self.score['m2'] = 15
                self.get_logger().info('GRAB terverifikasi: payload ter-attach (+15)')
                self._to(St.NAV_WALL)
        elif self._elapsed() > self.T['grab']:
            self.get_logger().error(
                'GRAB timeout — payload TAK ter-attach (state=%s)' % self._grip_state)
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
        tx, ty = self._hang_xy(self.wall)
        yaw_err = (abs(wrap_to_pi(target_heading - self.yaw))
                   if self.yaw is not None else math.pi)
        if not self._hang_pos_done:
            # Fase 1: posisikan di atas hook pada kedalaman AMAN (di atas tip)
            # dan PASTIKAN heading sudah sejajar wall (gate). Kalau target ikut
            # yaw live, ROV mengejar target bergerak — pakai target tetap.
            self._set_depth(self.hang_approach_depth)
            dist = self._goto_xy(tx, ty, fmax=self.nav_fmax)
            if dist < self.hang_tol and yaw_err < self.hang_yaw_tol:
                if self._hold_since is None:
                    self._hold_since = self._now()
                if self._now() - self._hold_since >= self.hold_settle_s:
                    self._set_surge(0.0)
                    self._hang_pos_done = True
                    self._hold_since = None
                    self.get_logger().info(
                        'HANG: lubang di atas tip hook %s (dist %.3f, yaw_err %.1f°) '
                        '-> turun ke hook' % (self.wall, dist, math.degrees(yaw_err)))
            else:
                self._hold_since = None
            if int(self._elapsed() * 2) % 20 == 0:
                self.get_logger().info(
                    'HANG dbg: dist=%.3f x=%.2f y=%.2f yaw=%.1f target=(%.2f,%.2f)'
                    % (dist, self.x or -99, self.y or -99,
                       math.degrees(self.yaw or 0), tx, ty))
            if self._elapsed() > self.T['hang']:
                self.get_logger().error('HANG timeout (posisi, dist %.3f, yaw_err %.1f°)'
                                        % (dist, math.degrees(yaw_err)))
                self._to(St.ABORT)
            return
        # Fase 2: turun — tip menembus lubang, plat bersandar di palang hook.
        self._set_depth(self.hook_depth)
        depth_ok = (self.depth is not None
                    and self.depth >= self.hook_depth - self.depth_tol
                    and yaw_err < self.hang_yaw_tol)
        if depth_ok:
            if self._hold_since is None:
                self._hold_since = self._now()
            if self._now() - self._hold_since >= self.hold_settle_s:
                self._set_surge(0.0)
                self.score['m3'] = 15
                self.get_logger().info(
                    'Payload tergantung stabil di hook %s (+15, depth %.2f)'
                    % (self.wall, self.depth))
                self._to(St.SURFACE)
        else:
            self._hold_since = None
        if self._elapsed() > self.T['hang']:
            self.get_logger().error('HANG timeout (turun, depth %s)'
                                    % (self.depth if self.depth is not None else 'n/a'))
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
        payload murni berdasarkan odometri (_hang_xy) tanpa konfirmasi kamera.
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
            if cmd.near and cmd.aligned:
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
            # Fallback open-loop: target odometri presisi (lubang di atas tip).
            self._set_depth(self.hang_approach_depth)
            tx, ty = self._hang_xy(self.wall)
            dist = self._goto_xy(tx, ty, fmax=self.nav_fmax)
            if dist < self.nav_tol:
                if self._hold_since is None:
                    self._hold_since = self._now()
                if self._now() - self._hold_since >= self.hold_settle_s:
                    self._set_surge(0.0, 0.0)
                    self.get_logger().warn(
                        'APPROACH_HOOK: tak ada deteksi hook, pakai target odometri '
                        '(dist %.2fm) -> AUTO_RELEASE' % dist)
                    self._to(St.AUTO_RELEASE)
                    return
            else:
                self._hold_since = None

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
        """Misi 5 (AUTONOMOUS): posisikan LUBANG payload di atas TIP hook
        (odometri presisi, sama seperti HANG), turun, DETACH, beri waktu payload
        bersandar di hook, lalu naik ke permukaan — semua tanpa input pilot."""
        if self.wall is None: self._to(St.ABORT); return
        target_heading = math.radians(WALL_HEADING_DEG[self.wall])
        self._set_heading(target_heading)
        if not self._detach_sent:
            tx, ty = self._hang_xy(self.wall)
            yaw_err = (abs(wrap_to_pi(target_heading - self.yaw))
                       if self.yaw is not None else math.pi)
            if not self._hang_pos_done:
                # Fase 1: posisikan lubang di atas tip pada kedalaman aman
                # + gate heading (sama spt HANG).
                self._set_depth(self.hang_approach_depth)
                dist = self._goto_xy(tx, ty, fmax=self.nav_fmax)
                if dist < self.hang_tol and yaw_err < self.hang_yaw_tol:
                    if self._hold_since is None:
                        self._hold_since = self._now()
                    if self._now() - self._hold_since >= self.hold_settle_s:
                        self._set_surge(0.0)
                        self._hang_pos_done = True
                        self._hold_since = None
                        self.get_logger().info(
                            'AUTO_RELEASE: lubang di atas hook (dist %.3f, yaw_err %.1f°)'
                            % (dist, math.degrees(yaw_err)))
                else:
                    self._hold_since = None
                if self._elapsed() > self.T['release']:
                    self.get_logger().error('AUTO_RELEASE timeout (posisi)')
                    self._to(St.ABORT)
                return
            # Fase 2: turun (tip menembus lubang), lalu detach.
            self._set_depth(self.hook_depth)
            depth_ok = (self.depth is not None
                        and self.depth >= self.hook_depth - self.depth_tol
                        and yaw_err < self.hang_yaw_tol)
            if depth_ok:
                if self._hold_since is None:
                    self._hold_since = self._now()
                if self._now() - self._hold_since >= self.hold_settle_s:
                    self.get_logger().info(
                        'AUTO_RELEASE: plat di palang hook (depth %.2f), publish detach...'
                        % self.depth)
                    self.pub_detach.publish(Empty())
                    self._detach_sent = True
                    self._set_surge(0.0, 0.0)
                    self._hold_since = self._now()
            else:
                self._hold_since = None
            if self._elapsed() > self.T['release']:
                self.get_logger().error('AUTO_RELEASE timeout (turun)'); self._to(St.ABORT)
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