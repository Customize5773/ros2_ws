#!/usr/bin/env python3
"""mission_fsm — State machine misi ROV KKI 2026 (Milestone 6, ROS 2 native).

Menjalankan urutan misi autonomous dengan MENGENDALIKAN LEWAT stabilizer (M2):
FSM hanya menetapkan target, stabilizer menahan kedalaman & heading otomatis.

Aliran (lihat docs/ARCHITECTURE.md):
    masuk : /hydroships/depth      (Float64, m >=0)  -> transisi state
            /hydroships/odom       (Odometry)        -> yaw (cek alignment)
            /hydroships/qr_result  (String A/B/C/D)   -> tentukan wall (M1)
            /hydroships/qr_offset  (PointStamped)     -> centering visual servo (APPROACH_QR)
            /hydroships/payload_pose (PointStamped)   -> posisi payload spawn (APPROACH_QR)
    keluar: /hydroships/setpoint/depth   (Float64, negatif = dalam)
            /hydroships/setpoint/heading (Float64, rad)
            /hydroships/manual/cmd       (Twist, Fx/Fy gaya horizontal N)

Catatan: subsistem GRIPPER/manipulasi DIHAPUS (akan dirancang ulang). State
GRAB/HANG/AUTO_RELEASE kini hanya gerakan (depth/surge), tanpa perintah jepit.

State: IDLE -> DIVE -> SCAN_QR -> GRAB -> NAV_WALL -> HANG -> SURFACE -> DOCK
       -> APPROACH_HOOK -> AUTO_RELEASE -> DONE (atau ABORT).

Catatan: butuh stabilizer + thruster_allocator + sim berjalan
(pakai hydroships_bringup/launch/hydroships_mission.launch.py).
"""

import math
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, String

from hydroships_control.hook_logic import HookServoGains, hook_servo


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_to_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


WALL_HEADING_DEG = {'A': 270.0, 'B': 90.0, 'C': 0.0, 'D': 180.0}


class St(Enum):
    IDLE = auto(); DIVE = auto(); APPROACH_QR = auto(); SCAN_QR = auto(); GRAB = auto()
    NAV_WALL = auto(); HANG = auto(); SURFACE = auto(); DOCK = auto()
    APPROACH_HOOK = auto(); AUTO_RELEASE = auto(); DONE = auto(); ABORT = auto()


class MissionFSM(Node):
    def __init__(self):
        super().__init__('mission_fsm')
        p = self.declare_parameter
        p('start_state', 'DIVE')
        p('start_delay', 3.0)
        p('start_wall', '')          # override manual utk testing start_state=NAV_WALL/HANG/dst
        p('surge_force', 25.0)       # N gaya maju horizontal
        p('depth_bottom', 0.70)      # m kedalaman dasar
        p('depth_surface', 0.08)     # m ambang "di permukaan"
        p('depth_tol', 0.06)         # m toleransi kedalaman
        p('hook_depth', 0.45)        # m kedalaman hook (lihat arena)
        p('scan_rate', 0.4)          # rad/s laju sapuan heading saat scan
        p('yaw_tol_deg', 10.0)       # derajat toleransi alignment heading
        p('qr_max_age', 1.5)         # s umur maks deteksi QR agar dianggap segar
        # APPROACH payload (M1): posisikan ROV DI ATAS QR datar di dasar lalu hold
        p('payload_x', 0.4)          # m posisi payload/QR di dunia (x)
        p('payload_y', 0.0)          # m posisi payload/QR di dunia (y)
        # [RESOLVED] QR detection: scan_depth 0.62 -> 0.46. Di 0.62 kamera bawah hanya
        # ~9cm di atas QR (world z=-0.893) -> QR 12cm MEMENUHI/melebihi frame, finder
        # bawah TER-CROP + gripper menutupi atas frame -> cv2.QRCodeDetector gagal
        # (pts=None). Di 0.46 kamera ~25cm di atas QR -> QR utuh + quiet-zone di frame,
        # terbaca 'A'..'D' (dibuktikan runtime: frame kamera bottom -> decode 'A').
        p('scan_depth', 0.46)        # m kedalaman scan (kamera bawah ~25cm di atas QR)
        p('approach_kp', 90.0)       # N/m gain posisi XY -> gaya horizontal
        p('approach_kd', 140.0)       # N/(m/s) redaman kecepatan (cegah overshoot)
        p('approach_fmax', 16.0)     # N batas gaya approach
        p('approach_tol', 0.06)      # m radius "sudah di atas payload"
        # APPROACH_QR: timeout NAVIGASI internal (capai payload) sblm t_scan penuh;
        # bila lewat & masih jauh -> recovery (naik sedikit perluas FOV) lalu ABORT.
        p('t_nav_qr', 30.0)          # s batas navigasi ke atas payload
        # Visual servo centering QR (dari /hydroships/qr_offset, ternormalisasi
        # [-1..1]): saat QR terlihat tapi di pinggir frame, geser target hold agar
        # QR ke tengah -> decoder lebih mudah membaca. Gain kecil (nudge halus).
        p('qr_off_max_age', 1.0)     # s umur maks qr_offset agar segar
        p('qr_center_tol', 0.12)     # |offset| dianggap "cukup tengah" (tak koreksi)
        p('qr_servo_gain', 0.15)     # m nudge target per unit offset (kecil = halus)
        # Peta arah kamera-bawah -> dunia (image +x/+y ke world dx/dy). Tanda perlu
        # DIVERIFIKASI runtime (mounting kamera); default konservatif, ubah via param.
        p('qr_servo_sign_x', -1.0)   # world dx = sign_x * gain * offset.x
        p('qr_servo_sign_y', -1.0)   # world dy = sign_y * gain * offset.y
        # NAV_WALL HOLONOMIK (mitigasi yaw lemah — lihat PROBLEM.md): ROV translasi
        # ke posisi wall di dunia TANPA memutar badan (surge+sway), heading di-hold.
        # KEAMANAN DINDING: muka dalam dinding fisik (kki_arena) di x/y = +-2.5 m.
        # Target NAV_WALL = wall_face - wall_standoff (BUKAN mepet dinding). Sebelumnya
        # wall_dist=2.30 -> clearance cuma 0.20 m -> ROV overshoot & nabrak keras.
        p('wall_face', 2.50)         # m jarak pusat -> muka DALAM dinding fisik
        p('wall_standoff', 0.45)     # m jarak AMAN dari muka dinding (clearance ROV)
        p('hook_dist', 0.30)  # m jarak target di depan hook (lebih dekat dari wall)
        p('nav_tol', 0.25)           # m radius "tiba di standoff" (was 0.15; standoff cukup)
        p('nav_settle_vel', 0.10)    # m/s ambang kecepatan utk dianggap "settle" (berhenti)
        p('nav_fmax', 22.0)          # N batas gaya navigasi holonomik
        p('hang_hold', 6.0)          # s tahan di standoff (simulasi gantung) sebelum SURFACE
        # timeout per state (s)
        p('t_dive', 20.0); p('t_scan', 45.0); p('t_grab', 10.0); p('t_nav', 40.0)
        # t_scan 45->60: ROV sering spawn lebih DALAM dari scan_depth (mis. 0.73 vs 0.46)
        # -> DIVE lolos instan (depth>=scan_depth-tol) lalu APPROACH_QR harus NAIK ~0.27 m
        # sambil memusatkan; naik pelan memakan ~40 s (runtime), 45 s terlalu mepet. Beri
        # margin agar QR sempat terbaca sebelum timeout. Lihat docs/CHANGELOG.md [RESOLVED].
        p('t_dive', 20.0); p('t_scan', 60.0); p('t_grab', 10.0); p('t_nav', 30.0)
        p('t_hang', 15.0); p('t_surface', 20.0); p('t_dock', 12.0)
        p('t_approach', 20.0); p('t_release', 30.0)
        # APPROACH_HOOK visual servo PD (deteksi hook dari hook_detector; port GUI-ROV).
        # Holonomik: sway (offset-x) + surge (ukuran-tampak) + koreksi setpoint depth
        # (offset-y). Ganti servo proporsional-heading lama -> PD penuh (kp*err-kd*vel).
        p('hook_max_age', 1.0)       # s umur maks hook_offset agar segar
        p('hook_kp_surge', 40.0)     # N per unit error ukuran-tampak (maju bila jauh)
        p('hook_kd_surge', 30.0)     # N per (m/s) redaman surge (body vx)
        p('hook_kp_sway', 45.0)      # N per unit offset-x (koreksi lateral)
        p('hook_kd_sway', 30.0)      # N per (m/s) redaman sway (body vy)
        p('hook_kp_depth', 0.25)     # m per unit offset-y (geser setpoint depth)
        p('hook_fmax', 16.0)         # N batas gaya horizontal servo hook
        p('hook_depth_range', 0.20)  # m simpangan maks setpoint depth dari hook_depth
        p('hook_size_stop', 0.35)    # ukuran-tampak hook -> dianggap cukup dekat
        p('hook_center_tol', 0.15)   # |offset| dianggap "sejajar"

        g = lambda n: self.get_parameter(n).value
        self.surge = float(g('surge_force'))
        self.depth_bottom = float(g('depth_bottom'))
        self.depth_surface = float(g('depth_surface'))
        self.depth_tol = float(g('depth_tol'))
        self.done_hooks = set()
        self.hook_depth = float(g('hook_depth'))
        self.hook_dist = float(g('hook_dist'))
        self.scan_rate = float(g('scan_rate'))
        self.yaw_tol = math.radians(float(g('yaw_tol_deg')))
        self.qr_max_age = float(g('qr_max_age'))
        self.payload_x = float(g('payload_x'))
        self.payload_y = float(g('payload_y'))
        self.scan_depth = float(g('scan_depth'))
        self.approach_kp = float(g('approach_kp'))
        self.approach_kd = float(g('approach_kd'))
        self.approach_fmax = float(g('approach_fmax'))
        self.approach_tol = float(g('approach_tol'))
        self.t_nav_qr = float(g('t_nav_qr'))
        self.qr_off_max_age = float(g('qr_off_max_age'))
        self.qr_center_tol = float(g('qr_center_tol'))
        self.qr_servo_gain = float(g('qr_servo_gain'))
        self.qr_servo_sign_x = float(g('qr_servo_sign_x'))
        self.qr_servo_sign_y = float(g('qr_servo_sign_y'))
        self.wall_face = float(g('wall_face'))
        self.wall_standoff = float(g('wall_standoff'))
        # Jarak pusat->target NAV_WALL (di depan dinding, aman). Tak pernah negatif.
        self.wall_target_dist = max(0.0, self.wall_face - self.wall_standoff)
        self.nav_settle_vel = float(g('nav_settle_vel'))
        self.hang_hold = float(g('hang_hold'))
        self.nav_tol = float(g('nav_tol'))
        self.nav_fmax = float(g('nav_fmax'))
        self.T = {k: float(g('t_' + k)) for k in
                  ('dive', 'scan', 'grab', 'nav', 'hang', 'surface', 'dock',
                   'approach', 'release')}
        self.hook_max_age = float(g('hook_max_age'))
        self.hook_gains = HookServoGains(
            kp_surge=float(g('hook_kp_surge')), kd_surge=float(g('hook_kd_surge')),
            kp_sway=float(g('hook_kp_sway')), kd_sway=float(g('hook_kd_sway')),
            kp_depth=float(g('hook_kp_depth')),
            size_stop=float(g('hook_size_stop')), center_tol=float(g('hook_center_tol')),
            fmax=float(g('hook_fmax')), depth_range=float(g('hook_depth_range')))

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
        # qr_offset (visual servo centering APPROACH_QR; dari node qr_detector).
        self.create_subscription(PointStamped, '/hydroships/qr_offset', self._on_qr_offset, 10)
        # hook_offset (visual servo APPROACH_HOOK; dari node hook_detector).
        self.create_subscription(PointStamped, '/hydroships/hook_offset', self._on_hook, 10)
        # payload_pose (posisi spawn payload QR dari node payload_spawner; APPROACH_QR
        # navigasi ke sini bila tersedia, jatuh ke payload_x/payload_y bila belum).
        self.create_subscription(PointStamped, '/hydroships/payload_pose', self._on_payload_pose, 10)

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
        self._approach_move_t0 = self._now()  # baseline timeout nav APPROACH_QR
        self.hook_off = None      # (ex, ey, size)
        self.hook_time = 0.0
        self.payload_pose = None  # (x, y, z) dari /hydroships/payload_pose (spawner)
        self.wall = None
        self.done_hooks = set()
        self.score = {'m1': 0, 'm2': 0, 'm3': 0, 'm4': 0, 'm5': 0}
        self.state = St.IDLE
        self.t_state = self._now()
        self._scan_head0 = 0.0
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
        if s == St.APPROACH_QR:
            # Reset baseline timeout navigasi & flag warning odom tiap kali masuk
            # APPROACH_QR (mis. loop kembali dari recovery/DIVE).
            self._approach_move_t0 = self._now()
            self._warned_no_odom = False

    def _set_depth(self, d_pos):
        m = Float64(); m.data = -abs(d_pos); self.pub_depth.publish(m)

    def _set_heading(self, yaw_rad):
        m = Float64(); m.data = wrap_to_pi(yaw_rad); self.pub_head.publish(m)

    def _set_surge(self, fx=0.0, fy=0.0):
        t = Twist(); t.linear.x = float(fx); t.linear.y = float(fy)
        self.pub_manual.publish(t)

    def _grip(self, close):
        """Kirim perintah manipulator: close=True -> 'close' (attach payload bila
        di jangkauan), close=False -> 'open' (detach). gripper_controller yg
        memutuskan attach fisik via DetachableJoint (lihat gripper_logic)."""
        m = String(); m.data = 'close' if close else 'open'; self.pub_grip.publish(m)

    def _goto_xy(self, tx, ty, fmax=None):
        """PD posisi HOLONOMIK: dorong ROV ke (tx,ty) dunia via gaya horizontal
        body-frame (surge+sway), dgn redaman kecepatan agar tak overshoot.
        Kompensasi yaw tiap tick -> arah gerak tetap benar walau heading melenceng
        (mitigasi yaw lemah, lihat PROBLEM.md). Kembalikan jarak sisa (m)."""
        if self.x is None or self.yaw is None:
            return 999.0
        fm = self.approach_fmax if fmax is None else fmax
        ex, ey = tx - self.x, ty - self.y
        dist = math.hypot(ex, ey)
        # Taper gaya maks saat mendekati target (slow-down radius) -> cegah slam.
        slow_radius = 1.0  # m, mulai perlambat dalam radius ini
        min_fmax_frac = 0.05  # jangan sampai gaya nol total (masih perlu lawan drag/arus)
        if dist < slow_radius:
            frac = max(min_fmax_frac, dist / slow_radius)
            fm = fm * frac
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        bx = ex * c + ey * s      # error posisi di body +x (surge)
        by = -ex * s + ey * c     # error posisi di body +y (sway)
        cl = lambda v: max(-fm, min(fm, v))
        # vx,vy sudah body-frame dari odom twist -> pakai untuk redaman
        surge = self.approach_kp * bx - self.approach_kd * self.vx
        sway = self.approach_kp * by - self.approach_kd * self.vy
        self._set_surge(cl(surge), cl(sway))
        return dist

    def _move_world(self, wx, wy, force):
        """Set gaya horizontal menuju arah DUNIA (wx,wy) unit, kompensasi yaw
        (gerak tak bergantung heading — mitigasi yaw lemah)."""
        if self.yaw is None:
            self._set_surge(0.0, 0.0); return
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self._set_surge(force * (wx * c + wy * s), force * (-wx * s + wy * c))

    def _wall_xy(self, wall):
        """Posisi target XY (dunia) STANDOFF AMAN di depan wall A/B/C/D — di
        wall_face - wall_standoff dari pusat (bukan mepet muka dinding)."""
        d = self.wall_target_dist
        return {'A': (0.0, -d), 'B': (0.0, d), 'C': (d, 0.0), 'D': (-d, 0.0)}[wall]

    def _wall_clearance(self):
        """Sisa jarak (m) ROV ke MUKA dinding fisik sepanjang sumbu menuju wall.
        Kecil/negatif = terlalu dekat/menembus. None bila odom/wall belum ada."""
        if self.x is None or self.wall is None:
            return None
        ux, uy = self._wall_inward(self.wall)
        proj = self.x * ux + self.y * uy      # proyeksi posisi ROV ke sumbu inward
        return self.wall_face - proj
    
    def _hook_xy(self, wall):
        """Posisi target XY (dunia) di depan hook A/B/C/D (lebih dekat dari wall_dist)."""
        d = self.hook_dist
        return {'A': (0.0, -d), 'B': (0.0, d), 'C': (d, 0.0), 'D': (-d, 0.0)}[wall]
    
    def _hook_inward(self, wall):
        """Vektor satuan DUNIA dari pusat menuju hook."""
        return {'A': (0.0, -1.0), 'B': (0.0, 1.0), 'C': (1.0, 0.0), 'D': (-1.0, 0.0)}[wall]

    def _wall_inward(self, wall):
        """Vektor satuan DUNIA dari pusat menuju wall ('maju ke wall')."""
        return {'A': (0.0, -1.0), 'B': (0.0, 1.0), 'C': (1.0, 0.0), 'D': (-1.0, 0.0)}[wall]

    # ---- callbacks ----
    def _on_depth(self, msg): self.depth = msg.data

    def _on_odom(self, msg):
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.vx = msg.twist.twist.linear.x   # kecepatan body-frame (odom gz)
        self.vy = msg.twist.twist.linear.y

    def _on_qr(self, msg):
        w = (msg.data or '').strip().upper()
        if w in WALL_HEADING_DEG:
            self.qr_wall = w; self.qr_time = self._now()

    def _on_qr_offset(self, msg):
        # ex/ey ternormalisasi [-1..1] (+x=QR di kanan, +y=QR di bawah), z=ukuran.
        self.qr_off = (msg.point.x, msg.point.y, msg.point.z)
        self.qr_off_time = self._now()

    def _qr_offset_fresh(self):
        """(ex, ey, size) bila qr_offset masih segar, else None."""
        if self.qr_off is None:
            return None
        if self._now() - self.qr_off_time > self.qr_off_max_age:
            return None
        return self.qr_off

    def _qr_center_nudge(self, off):
        """Geseran world (dx, dy) kecil utk memusatkan QR di frame kamera bawah.
        Nol bila QR sudah cukup tengah (|offset| <= qr_center_tol)."""
        ex, ey = off[0], off[1]
        dx = 0.0 if abs(ex) <= self.qr_center_tol else self.qr_servo_sign_x * self.qr_servo_gain * ex
        dy = 0.0 if abs(ey) <= self.qr_center_tol else self.qr_servo_sign_y * self.qr_servo_gain * ey
        return dx, dy

    def _on_hook(self, msg):
        self.hook_off = (msg.point.x, msg.point.y, msg.point.z)
        self.hook_time = self._now()

    def _on_payload_pose(self, msg):
        pose = (msg.point.x, msg.point.y, msg.point.z)
        if pose != self.payload_pose:
            self.get_logger().info('Payload pose diterima: (%.2f, %.2f, %.2f)' % pose)
        self.payload_pose = pose

    def _hook_fresh(self):
        """Kembalikan (ex, ey, size) bila deteksi hook masih segar, else None."""
        if self.hook_off is None:
            return None
        if self._now() - self.hook_time > self.hook_max_age:
            return None
        return self.hook_off

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
        # menyelam ke KEDALAMAN SCAN (kamera bawah cukup tinggi utk lihat QR utuh)
        self._set_depth(self.scan_depth)
        self._set_heading(0.0)   # mulai putar balik menghadap QR sambil menyelam
        if self.depth is not None and self.depth >= self.scan_depth - self.depth_tol:
            self._set_surge(0.0)
            self.get_logger().info('Kedalaman scan tercapai (%.2fm)' % self.depth)
            self._to(St.APPROACH_QR)
        elif self._elapsed() > self.T['dive']:
            self.get_logger().error('DIVE timeout'); self._to(St.ABORT)

    def _st_approach_qr(self):
        """Misi 1a: posisikan ROV DI ATAS payload/QR datar & tahan (depth+XY hold)
        agar kamera bawah membaca QR. QR terbaca -> tetapkan wall -> GRAB."""
        self._set_depth(self.scan_depth)
        self._set_heading(0.0)

        yaw_err = abs(wrap_to_pi(0.0 - self.yaw)) if self.yaw is not None else math.pi
        if yaw_err > self.yaw_tol:
            self._set_surge(0.0, 0.0)
            if self._elapsed() > self.T['scan']:
                self.get_logger().error('APPROACH_QR timeout (masih align, yaw_err=%.1f°)' % math.degrees(yaw_err))
                self._to(St.ABORT)
            return

        dist = self._goto_xy(self.payload_x, self.payload_y)
        agar kamera bawah membaca QR. QR terbaca -> tetapkan wall -> GRAB.
        Navigasi ke posisi payload dinamis (/hydroships/payload_pose); visual servo
        centering dari /hydroships/qr_offset; timeout nav + recovery bila tak sampai."""
        self._set_heading(0.0)                       # heading tetap (QR di bawah, rotasi tak perlu)
        # GUARD odom: tanpa pose ROV tak bisa navigasi. Tunggu (log sekali), JANGAN
        # anggap "sudah sampai" & reset baseline timeout nav agar tak keburu ABORT.
        if self.x is None or self.yaw is None:
            self._set_depth(self.scan_depth)
            if not self._warned_no_odom:
                self.get_logger().warn('APPROACH_QR: menunggu odom (pose ROV belum ada)')
                self._warned_no_odom = True
            self._set_surge(0.0, 0.0)
            self._approach_move_t0 = self._now()
            return

        # Posisi payload dinamis dari payload_spawner; fallback ke param default.
        tx = self.payload_pose[0] if self.payload_pose is not None else self.payload_x
        ty = self.payload_pose[1] if self.payload_pose is not None else self.payload_y
        # Visual servo: bila QR terlihat tapi di pinggir frame, geser target hold
        # sedikit agar QR menuju tengah (decoder lebih andal).
        off = self._qr_offset_fresh()
        if off is not None:
            dx, dy = self._qr_center_nudge(off)
            tx += dx; ty += dy
        dist = self._goto_xy(tx, ty)

        # QR terbaca (dari qr_detector via kamera bawah) & segar?
        if self.qr_wall and (self._now() - self.qr_time) <= self.qr_max_age:
            self.wall = self.qr_wall; self.score['m1'] = 15
            self.get_logger().info('QR terbaca -> wall %s (+15) [dist %.2fm]' % (self.wall, dist))
            self._set_surge(0.0); self._to(St.GRAB); return

        if int(self._elapsed() * 2) % 4 == 0:
            self.get_logger().debug('APPROACH_QR dist=%.2fm off=%s' % (dist, off))

        # Timeout NAVIGASI: bila masih jauh dari payload setelah t_nav_qr, recovery
        # naik sedikit (perluas FOV kamera bawah) sambil tetap koreksi XY.
        stuck = dist > self.approach_tol and (self._now() - self._approach_move_t0) > self.t_nav_qr
        if stuck:
            self.get_logger().warn('APPROACH_QR: belum capai payload (dist %.2fm > tol %.2fm) '
                                   'setelah %.0fs — recovery naik sedikit'
                                   % (dist, self.approach_tol, self.t_nav_qr))
            self._set_depth(self.scan_depth - 0.10)
        else:
            self._set_depth(self.scan_depth)

        if self._elapsed() > self.T['scan']:
            self.get_logger().error('APPROACH_QR gagal capai payload / QR tak terbaca — ABORT '
                                    '[dist %.2fm]' % dist)
            self._to(St.ABORT)

    def _st_scan_qr(self):
        # QR segar?
        if self.qr_wall and (self._now() - self.qr_time) <= self.qr_max_age:
            self.wall = self.qr_wall; self.score['m1'] = 15
            self.get_logger().info('QR -> wall %s (+15)' % self.wall)
            self._set_surge(0.0); self._to(St.GRAB); return
        # sapu heading pelan
        self._set_heading(self._scan_head0 + self.scan_rate * self._elapsed())
        if self._elapsed() > self.T['scan']:
            self.get_logger().error('SCAN_QR timeout — tak ada QR'); self._to(St.ABORT)

    def _st_grab(self):
        # Grasp via DetachableJoint (rancang ulang M5): kirim 'close' tiap tick →
        # gripper_controller meng-attach payload begitu ROV di atasnya dalam
        # jangkauan aman (dinilai dari qr_offset). Datang dari APPROACH_QR yg sudah
        # memusatkan ROV di atas payload, jadi attach mestinya di tick awal.
        e = self._elapsed()
        self._grip(True)
        if e < 4.0: self._set_surge(self.surge)
        elif e < 7.0: self._set_surge(0.0)
        else:
            self.score['m2'] = 15; self.get_logger().info('GRAB selesai (attach dikirim)')
            self._to(St.NAV_WALL)
        if e > self.T['grab']: self.get_logger().error('GRAB timeout'); self._to(St.ABORT)

    def _st_nav_wall(self):
        if self.wall is None: self._to(St.ABORT); return
        tx, ty = self._wall_xy(self.wall)
        target_heading = math.radians(WALL_HEADING_DEG[self.wall])
        self._set_depth(self.hook_depth)
        self._set_heading(target_heading)
        yaw_err = abs(wrap_to_pi(target_heading - self.yaw)) if self.yaw is not None else math.pi
        if yaw_err > self.yaw_tol:
            # Belum sejajar: tahan posisi, jangan translasi dulu.
            self._set_surge(0.0, 0.0)
            if self._elapsed() > self.T['nav']:
                self.get_logger().error('NAV_WALL timeout (masih align, yaw_err=%.1f°)' % math.degrees(yaw_err))
                self._to(St.ABORT)
            return
        """Navigasi HOLONOMIK ke STANDOFF AMAN di depan wall (mitigasi yaw lemah):
        tahan heading, translasi surge+sway ke posisi standoff. SOFT-STOP: bila ROV
        terlalu dekat muka dinding (clearance < wall_standoff), dorong MENJAUH agar
        tak menabrak. Transisi ke HANG hanya saat dekat standoff DAN sudah settle."""
        if self.wall is None: self._to(St.ABORT); return
        self._set_depth(self.hook_depth)     # turun ke level hook sambil bergerak
        self._set_heading(0.0)               # HOLD heading (yaw lemah -> jangan slew)

        # SOFT-STOP dinding: terlalu dekat -> dorong menjauh (tak mendekat lagi).
        clr = self._wall_clearance()
        if clr is not None and clr < self.wall_standoff:
            ux, uy = self._wall_inward(self.wall)
            self._move_world(-ux, -uy, self.nav_fmax * 0.6)   # menjauhi dinding
            if int(self._elapsed() * 2) % 6 == 0:
                self.get_logger().warn('NAV_WALL soft-stop: clearance %.2fm < %.2fm '
                                       '-> mundur dari dinding' % (clr, self.wall_standoff))
            if self._elapsed() > self.T['nav']:
                self.get_logger().error('NAV_WALL timeout saat soft-stop (clr %.2fm)' % clr)
                self._to(St.ABORT)
            return

        tx, ty = self._wall_xy(self.wall)
        dist = self._goto_xy(tx, ty, fmax=self.nav_fmax)
        speed = math.hypot(self.vx, self.vy)
        # Transisi hanya saat dekat standoff DAN kecepatan kecil (settle) -> tak
        # transisi mid-osilasi.
        if dist < self.nav_tol and speed < self.nav_settle_vel:
            self._set_surge(0.0)
            self.get_logger().info('Tiba di standoff wall %s (dist %.2fm, v %.2fm/s) -> HANG'
                                   % (self.wall, dist, speed))
            self._to(St.HANG)
        elif self._elapsed() > self.T['nav']:
            self.get_logger().error('NAV_WALL timeout (dist %.2fm)' % dist); self._to(St.ABORT)

    def _st_hang(self):
        """Tahan di STANDOFF aman depan wall (depth+XY hold lembut) selama hang_hold
        detik sebagai simulasi 'gantung payload', lalu SURFACE. TIDAK ada gerak
        agresif ke dinding (placeholder lama yg menabrak sudah dihapus). SOFT-STOP
        tetap aktif agar drift tak menyeret ROV ke dinding."""
        e = self._elapsed()
        self._set_depth(self.hook_depth)   # naik/turun ke level hook
        self._set_heading(math.radians(WALL_HEADING_DEG[self.wall]))
        ux, uy = self._wall_inward(self.wall) if self.wall else (0.0, 0.0)
        # Manipulasi (jepit/lepas) DIHAPUS — placeholder gerakan WORLD-FRAME sampai
        # dirancang ulang: dekati wall lalu mundur (arah dunia, tak bergantung heading).
        if e < 5.0: self._set_surge(0.0)
        elif e < 8.0: self._move_world(ux, uy, 15.0)      # dekati wall
        elif e < 11.0: self._set_surge(0.0)
        elif e < 13.0: self._move_world(-ux, -uy, 15.0)   # mundur dari wall
        self._set_depth(self.hook_depth)   # tahan level hook
        self._set_heading(0.0)
        if e < 0.15:
            self.get_logger().info('HANG: tahan di standoff wall %s (%.1fs)'
                                   % (self.wall, self.hang_hold))
        # HOLD posisi: dorong menjauh bila terlalu dekat dinding, else XY-hold lembut
        # ke standoff (tak overshoot ke dinding).
        clr = self._wall_clearance()
        if clr is not None and clr < self.wall_standoff:
            ux, uy = self._wall_inward(self.wall)
            self._move_world(-ux, -uy, self.nav_fmax * 0.5)
        elif self.wall is not None:
            tx, ty = self._wall_xy(self.wall)
            self._goto_xy(tx, ty, fmax=self.nav_fmax)
        else:
            self._set_surge(0.0)

        if e >= self.hang_hold:
            self._set_surge(0.0); self.score['m3'] = 15
            self.get_logger().info('Payload tergantung di wall %s (+15)' % self.wall)
            self._to(St.SURFACE)
        elif e > self.T['hang']:
            self.get_logger().error('HANG timeout'); self._to(St.ABORT)

    def _st_surface(self):
        self._set_heading(0.0)   # putar balik menghadap depan sebelum naik

        yaw_err = abs(wrap_to_pi(0.0 - self.yaw)) if self.yaw is not None else math.pi
        if yaw_err > self.yaw_tol:
            self._set_surge(0.0, 0.0)
            self._set_depth(self.hook_depth)   # tahan kedalaman dulu, jangan naik saat masih align
            if self._elapsed() > self.T['surface']:
                self.get_logger().error('SURFACE timeout (masih align, yaw_err=%.1f°)' % math.degrees(yaw_err))
                self._to(St.ABORT)
            return

        self._set_depth(self.depth_surface)
        if self.depth is not None and self.depth <= self.depth_surface + 0.05:
            self._set_surge(0.0)
            self.done_hooks.add(self.wall)
            self.get_logger().info('Permukaan tercapai. Hook %s selesai. Done: %s' % (self.wall, self.done_hooks))
            if len(self.done_hooks) >= 4:
                self.score['m5'] = 40
                self.get_logger().info('Semua hook selesai (+40)!')
                self._print_score(); self._to(St.DONE)
            else:
                self.wall = None
                self._to(St.DIVE)
        elif self._elapsed() > self.T['surface']:
            self.get_logger().error('SURFACE timeout'); self._to(St.ABORT)


    def _st_dock(self):
        self._set_depth(self.depth_surface)
        if self._elapsed() < 8.0:
            self._set_surge(15.0)
        else:
            self._set_surge(0.0); self.score['m4'] = 15
            self.get_logger().info('Docking selesai (+15)'); self._to(St.APPROACH_HOOK)
        if self._elapsed() > self.T['dock']:
            self.get_logger().error('DOCK timeout'); self._to(St.ABORT)

    def _st_approach_hook(self):
        # Visual servo PD hook (deteksi contour dari node hook_detector ->
        # /hydroships/hook_offset). Bila deteksi segar: PD holonomik koreksi
        # sway (offset-x) + surge (ukuran-tampak) + setpoint depth (offset-y),
        # dgn redaman kecepatan (mirip _goto_xy), sampai cukup dekat & terpusat
        # -> AUTO_RELEASE. Bila TAK ada deteksi: fallback timed lama (aman).
        # Heading di-hold menghadap wall agar kamera depan tetap melihat hook.
        off = self._hook_fresh()
        if off is not None:
            cmd = hook_servo(off, self.vx, self.vy, self.hook_depth, self.hook_gains)
            self._set_depth(cmd.target_depth)
            if self.wall in WALL_HEADING_DEG:
                self._set_heading(math.radians(WALL_HEADING_DEG[self.wall]))
            if cmd.near and cmd.aligned:
                self.get_logger().info('APPROACH_HOOK: hook tercapai (PD servo, size %.2f)' % off[2])
                self._set_surge(0.0, 0.0); self._to(St.AUTO_RELEASE); return
            # Sudah dekat tapi belum terpusat: berhenti maju, hanya koreksi lateral/vert.
            surge = 0.0 if cmd.near else cmd.surge
            self._set_surge(surge, cmd.sway)
            if self._elapsed() > self.T['approach']:
                self.get_logger().warn('APPROACH_HOOK timeout (PD servo, size %.2f) -> lanjut' % off[2])
                self._set_surge(0.0, 0.0); self._to(St.AUTO_RELEASE)
            return
        # Fallback timed (tak ada hook_offset — hook_detector mati / tak terdeteksi).
        self._set_depth(self.hook_depth)
        self._set_heading(math.radians(WALL_HEADING_DEG[self.wall]))
        dist = self._goto_xy(tx, ty, fmax=self.nav_fmax)
        if dist < self.nav_tol:
            self._set_surge(0.0)
            self.get_logger().info('Tiba di hook %s (dist %.2fm)' % (self.wall, dist))
            self._to(St.AUTO_RELEASE)
        elif self._elapsed() > self.T['approach']:
            self.get_logger().error('APPROACH_HOOK timeout (dist %.2fm)' % dist); self._to(St.ABORT)
        self._set_surge(10.0 if self._elapsed() < 6.0 else 0.0)
        if self._elapsed() >= 6.0 or self._elapsed() > self.T['approach']:
            self.get_logger().warn('APPROACH_HOOK timed (tak ada deteksi hook)')
            self._set_surge(0.0, 0.0); self._to(St.AUTO_RELEASE)

    def _st_auto_release(self):
        e = self._elapsed()
        self._set_depth(self.hook_depth if e < 15.0 else self.depth_surface)
        # Grasp via DetachableJoint (rancang ulang M5): tetap tutup saat mendekati
        # hook, LEPAS (detach) saat menggantung, lalu mundur.
        if e < 11.0:
            self._grip(True); self._set_surge(15.0 if e >= 8.0 else 0.0)  # dekati hook (tetap tutup)
        elif e < 14.0:
            self._grip(False); self._set_surge(0.0)                        # LEPAS payload ke hook
        elif e < 17.0: self._set_surge(-15.0)                              # mundur dari hook
        elif e < 26.0: self._set_surge(0.0)   # naik (depth-hold ke permukaan)
        else:
            self._set_surge(0.0)
            self.done_hooks.add(self.wall)
            self.get_logger().info('Hook %s selesai. Done: %s' % (self.wall, self.done_hooks))
            if len(self.done_hooks) >= 4:
                self.score['m5'] = 40
                self.get_logger().info('Semua hook selesai (+40)!')
                self._print_score(); self._to(St.DONE)
            else:
                self.wall = None
                self._to(St.DIVE)

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
            node.pub_manual.publish(Twist())   # netralkan horizontal saat keluar
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
