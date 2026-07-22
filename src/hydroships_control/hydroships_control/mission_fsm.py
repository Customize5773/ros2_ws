#!/usr/bin/env python3
"""mission_fsm — State machine misi ROV KKI 2026 (Milestone 6, ROS 2 native).

Menjalankan urutan misi autonomous dengan MENGENDALIKAN LEWAT stabilizer (M2):
FSM hanya menetapkan target, stabilizer menahan kedalaman & heading otomatis.

Aliran (lihat docs/ARCHITECTURE.md):
    masuk : /hydroships/depth      (Float64, m >=0)  -> transisi state
            /hydroships/odom       (Odometry)        -> yaw (cek alignment)
            /hydroships/qr_result  (String A/B/C/D)   -> tentukan wall (M1)
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
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, String


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
        p('scan_depth', 0.62)        # m kedalaman scan (kamera bawah ~9cm di atas QR)
        p('approach_kp', 90.0)       # N/m gain posisi XY -> gaya horizontal
        p('approach_kd', 70.0)       # N/(m/s) redaman kecepatan (cegah overshoot)
        p('approach_fmax', 16.0)     # N batas gaya approach
        p('approach_tol', 0.06)      # m radius "sudah di atas payload"
        # NAV_WALL HOLONOMIK (mitigasi yaw lemah — lihat PROBLEM.md): ROV translasi
        # ke posisi wall di dunia TANPA memutar badan (surge+sway), heading di-hold.
        p('wall_dist', 2.30)         # m jarak pusat->target wall (standoff; hook ~2.4 m)
        p('hook_dist', 0.30)  # m jarak target di depan hook (lebih dekat dari wall_dist)
        p('nav_tol', 0.15)           # m radius "tiba di wall"
        p('nav_fmax', 22.0)          # N batas gaya navigasi holonomik
        # timeout per state (s)
        p('t_dive', 20.0); p('t_scan', 45.0); p('t_grab', 10.0); p('t_nav', 30.0)
        p('t_hang', 15.0); p('t_surface', 20.0); p('t_dock', 12.0)
        p('t_approach', 20.0); p('t_release', 30.0)

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
        self.wall_dist = float(g('wall_dist'))
        self.nav_tol = float(g('nav_tol'))
        self.nav_fmax = float(g('nav_fmax'))
        self.T = {k: float(g('t_' + k)) for k in
                  ('dive', 'scan', 'grab', 'nav', 'hang', 'surface', 'dock',
                   'approach', 'release')}

        # I/O
        self.pub_depth = self.create_publisher(Float64, '/hydroships/setpoint/depth', 10)
        self.pub_head = self.create_publisher(Float64, '/hydroships/setpoint/heading', 10)
        self.pub_manual = self.create_publisher(Twist, '/hydroships/manual/cmd', 10)
        self.create_subscription(Float64, '/hydroships/depth', self._on_depth, 10)
        self.create_subscription(Odometry, '/hydroships/odom', self._on_odom, 10)
        self.create_subscription(String, '/hydroships/qr_result', self._on_qr, 10)

        # State
        self.depth = None
        self.yaw = None
        self.x = None
        self.y = None
        self.vx = 0.0
        self.vy = 0.0
        self.qr_wall = None
        self.qr_time = 0.0
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

    def _set_depth(self, d_pos):
        m = Float64(); m.data = -abs(d_pos); self.pub_depth.publish(m)

    def _set_heading(self, yaw_rad):
        m = Float64(); m.data = wrap_to_pi(yaw_rad); self.pub_head.publish(m)

    def _set_surge(self, fx=0.0, fy=0.0):
        t = Twist(); t.linear.x = float(fx); t.linear.y = float(fy)
        self.pub_manual.publish(t)

    def _goto_xy(self, tx, ty, fmax=None):
        """PD posisi HOLONOMIK: dorong ROV ke (tx,ty) dunia via gaya horizontal
        body-frame (surge+sway), dgn redaman kecepatan agar tak overshoot.
        Kompensasi yaw tiap tick -> arah gerak tetap benar walau heading melenceng
        (mitigasi yaw lemah, lihat PROBLEM.md). Kembalikan jarak sisa (m)."""
        if self.x is None or self.yaw is None:
            return 999.0
        fm = self.approach_fmax if fmax is None else fmax
        ex, ey = tx - self.x, ty - self.y
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        bx = ex * c + ey * s      # error posisi di body +x (surge)
        by = -ex * s + ey * c     # error posisi di body +y (sway)
        cl = lambda v: max(-fm, min(fm, v))
        # vx,vy sudah body-frame dari odom twist -> pakai untuk redaman
        surge = self.approach_kp * bx - self.approach_kd * self.vx
        sway = self.approach_kp * by - self.approach_kd * self.vy
        self._set_surge(cl(surge), cl(sway))
        return math.hypot(ex, ey)

    def _move_world(self, wx, wy, force):
        """Set gaya horizontal menuju arah DUNIA (wx,wy) unit, kompensasi yaw
        (gerak tak bergantung heading — mitigasi yaw lemah)."""
        if self.yaw is None:
            self._set_surge(0.0, 0.0); return
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self._set_surge(force * (wx * c + wy * s), force * (-wx * s + wy * c))

    def _wall_xy(self, wall):
        """Posisi target XY (dunia) di depan wall A/B/C/D (dari geometri arena)."""
        d = self.wall_dist
        return {'A': (0.0, -d), 'B': (0.0, d), 'C': (d, 0.0), 'D': (-d, 0.0)}[wall]
    
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
        # QR terbaca (dari qr_detector via kamera bawah) & segar?
        if self.qr_wall and (self._now() - self.qr_time) <= self.qr_max_age:
            self.wall = self.qr_wall; self.score['m1'] = 15
            self.get_logger().info('QR -> wall %s (+15) [dist %.2fm]' % (self.wall, dist))
            self._set_surge(0.0); self._to(St.GRAB); return
        if int(self._elapsed() * 2) % 4 == 0:
            self.get_logger().debug('APPROACH_QR dist=%.2fm' % dist)
        if self._elapsed() > self.T['scan']:
            self.get_logger().error('APPROACH_QR timeout — QR tak terbaca'); self._to(St.ABORT)

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
        # Manipulasi (jepit) DIHAPUS — gripper belum ada, lewati langsung (instan).
        self._set_surge(0.0)
        self.score['m2'] = 15
        self.get_logger().info('GRAB dilewati (gripper dihapus, placeholder instan)')
        self._to(St.NAV_WALL)

    def _st_nav_wall(self):
        """Navigasi HOLONOMIK ke wall (mitigasi yaw lemah): tahan heading, translasi
        surge+sway ke posisi wall di dunia. Tak perlu memutar badan menghadap wall."""
        if self.wall is None: self._to(St.ABORT); return
        tx, ty = self._wall_xy(self.wall)
        self._set_depth(self.hook_depth)     # turun ke level hook sambil bergerak
        self._set_heading(0.0)               # HOLD heading (yaw lemah -> jangan slew)
        dist = self._goto_xy(tx, ty, fmax=self.nav_fmax)
        if dist < self.nav_tol:
            self._set_surge(0.0)
            self.get_logger().info('Tiba di wall %s (dist %.2fm)' % (self.wall, dist))
            self._to(St.HANG)
        elif self._elapsed() > self.T['nav']:
            self.get_logger().error('NAV_WALL timeout (dist %.2fm)' % dist); self._to(St.ABORT)

    def _st_hang(self):
        e = self._elapsed()
        self._set_depth(self.hook_depth)   # naik/turun ke level hook
        self._set_heading(0.0)
        ux, uy = self._wall_inward(self.wall) if self.wall else (0.0, 0.0)
        # Manipulasi (jepit/lepas) DIHAPUS — placeholder gerakan WORLD-FRAME sampai
        # dirancang ulang: dekati wall lalu mundur (arah dunia, tak bergantung heading).
        if e < 5.0: self._set_surge(0.0)
        elif e < 8.0: self._move_world(ux, uy, 15.0)      # dekati wall
        elif e < 11.0: self._set_surge(0.0)
        elif e < 13.0: self._move_world(-ux, -uy, 15.0)   # mundur dari wall
        else:
            self._set_surge(0.0); self.score['m3'] = 15
            self.get_logger().info('Payload tergantung di wall %s (+15)' % self.wall)
            self._to(St.SURFACE)
        if e > self.T['hang']: self.get_logger().error('HANG timeout'); self._to(St.ABORT)

    def _st_surface(self):
        self._set_heading(0.0)   # putar balik menghadap depan sebelum naik

        yaw_err = abs(wrap_to_pi(0.0 - self.yaw)) if self.yaw is not None else math.pi
        if yaw_err > self.yaw_tol:
            # Aktif ngerem sisa kecepatan (bukan cuma commit gaya nol) supaya
            # ROV benar-benar diam saat berputar, bukan drift sisa momentum HANG.
            brake_kd = 40.0
            bx = -brake_kd * self.vx
            by = -brake_kd * self.vy
            cl = lambda v: max(-20.0, min(20.0, v))
            self._set_surge(cl(bx), cl(by))
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
        """Navigasi HOLONOMIK ke hook (posisi sebenarnya, bukan timed)."""
        if self.wall is None: self._to(St.ABORT); return
        tx, ty = self._hook_xy(self.wall)
        self._set_depth(self.hook_depth)
        self._set_heading(0.0)
        dist = self._goto_xy(tx, ty, fmax=self.nav_fmax)
        if dist < self.nav_tol:
            self._set_surge(0.0)
            self.get_logger().info('Tiba di hook %s (dist %.2fm)' % (self.wall, dist))
            self._to(St.AUTO_RELEASE)
        elif self._elapsed() > self.T['approach']:
            self.get_logger().error('APPROACH_HOOK timeout (dist %.2fm)' % dist); self._to(St.ABORT)

    def _st_auto_release(self):
        e = self._elapsed()
        self._set_depth(self.hook_depth if e < 15.0 else self.depth_surface)
        # Manipulasi (jepit/lepas) DIHAPUS — placeholder gerakan sampai dirancang ulang.
        if e < 11.0: self._set_surge(15.0 if e >= 8.0 else 0.0)
        elif e < 14.0: self._set_surge(0.0)
        elif e < 17.0: self._set_surge(-15.0)
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
