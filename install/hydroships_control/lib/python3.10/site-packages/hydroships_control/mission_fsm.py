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
            /hydroships/gripper/command  (String open/close)

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
    IDLE = auto(); DIVE = auto(); SCAN_QR = auto(); GRAB = auto()
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
        # timeout per state (s)
        p('t_dive', 20.0); p('t_scan', 25.0); p('t_grab', 10.0); p('t_nav', 30.0)
        p('t_hang', 15.0); p('t_surface', 20.0); p('t_dock', 12.0)
        p('t_approach', 20.0); p('t_release', 30.0)

        g = lambda n: self.get_parameter(n).value
        self.surge = float(g('surge_force'))
        self.depth_bottom = float(g('depth_bottom'))
        self.depth_surface = float(g('depth_surface'))
        self.depth_tol = float(g('depth_tol'))
        self.hook_depth = float(g('hook_depth'))
        self.scan_rate = float(g('scan_rate'))
        self.yaw_tol = math.radians(float(g('yaw_tol_deg')))
        self.qr_max_age = float(g('qr_max_age'))
        self.T = {k: float(g('t_' + k)) for k in
                  ('dive', 'scan', 'grab', 'nav', 'hang', 'surface', 'dock',
                   'approach', 'release')}

        # I/O
        self.pub_depth = self.create_publisher(Float64, '/hydroships/setpoint/depth', 10)
        self.pub_head = self.create_publisher(Float64, '/hydroships/setpoint/heading', 10)
        self.pub_manual = self.create_publisher(Twist, '/hydroships/manual/cmd', 10)
        self.pub_grip = self.create_publisher(String, '/hydroships/gripper/command', 10)
        self.create_subscription(Float64, '/hydroships/depth', self._on_depth, 10)
        self.create_subscription(Odometry, '/hydroships/odom', self._on_odom, 10)
        self.create_subscription(String, '/hydroships/qr_result', self._on_qr, 10)

        # State
        self.depth = None
        self.yaw = None
        self.qr_wall = None
        self.qr_time = 0.0
        self.wall = None
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

    def _grip(self, close):
        m = String(); m.data = 'close' if close else 'open'; self.pub_grip.publish(m)

    # ---- callbacks ----
    def _on_depth(self, msg): self.depth = msg.data
    def _on_odom(self, msg): self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)

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
        self._set_depth(self.depth_bottom)
        if self.depth is not None and self.depth >= self.depth_bottom - self.depth_tol:
            self._set_surge(0.0); self.get_logger().info('Dasar tercapai (%.2fm)' % self.depth)
            self._to(St.SCAN_QR); self._scan_head0 = self.yaw or 0.0
        elif self._elapsed() > self.T['dive']:
            self.get_logger().error('DIVE timeout'); self._to(St.ABORT)

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
        e = self._elapsed()
        if e < 1.0: self._grip(False)
        elif e < 4.0: self._set_surge(self.surge); self._grip(False)
        elif e < 7.0: self._set_surge(0.0); self._grip(True)
        else:
            self.score['m2'] = 15; self.get_logger().info('Payload diambil (+15)')
            self._to(St.NAV_WALL)
        if e > self.T['grab']: self.get_logger().error('GRAB timeout'); self._to(St.ABORT)

    def _st_nav_wall(self):
        if self.wall is None: self._to(St.ABORT); return
        tgt = math.radians(WALL_HEADING_DEG[self.wall])
        self._set_heading(tgt)
        aligned = self.yaw is not None and abs(wrap_to_pi(tgt - self.yaw)) < self.yaw_tol
        self._set_surge(self.surge if (aligned and self._elapsed() > 4.0) else 0.0)
        self._grip(True)
        if self._elapsed() > 18.0:
            self.score['m3'] = 0  # skor diberi di HANG
            self._set_surge(0.0); self._to(St.HANG)
        elif self._elapsed() > self.T['nav']:
            self.get_logger().error('NAV_WALL timeout'); self._to(St.ABORT)

    def _st_hang(self):
        e = self._elapsed()
        self._set_depth(self.hook_depth)   # naik/turun ke level hook
        if e < 5.0: self._grip(True)
        elif e < 8.0: self._set_surge(15.0); self._grip(True)
        elif e < 11.0: self._set_surge(0.0); self._grip(False)
        elif e < 13.0: self._set_surge(-15.0); self._grip(False)
        else:
            self._set_surge(0.0); self.score['m3'] = 15
            self.get_logger().info('Payload tergantung di wall %s (+15)' % self.wall)
            self._to(St.SURFACE)
        if e > self.T['hang']: self.get_logger().error('HANG timeout'); self._to(St.ABORT)

    def _st_surface(self):
        self._set_depth(self.depth_surface)
        if self.depth is not None and self.depth <= self.depth_surface:
            self._set_surge(0.0); self.get_logger().info('Permukaan tercapai'); self._to(St.DOCK)
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
        # TODO (M3 lanjut): visual servo ArUco di ROS 2 belum ada -> sementara timed.
        self._set_depth(self.hook_depth)
        self._set_surge(10.0 if self._elapsed() < 6.0 else 0.0)
        if self._elapsed() >= 6.0 or self._elapsed() > self.T['approach']:
            self.get_logger().warn('APPROACH_HOOK timed (visual servo ROS2 belum ada)')
            self._set_surge(0.0); self._to(St.AUTO_RELEASE)

    def _st_auto_release(self):
        e = self._elapsed()
        self._set_depth(self.hook_depth if e < 15.0 else self.depth_surface)
        if e < 8.0: self._grip(False)
        elif e < 11.0: self._set_surge(15.0); self._grip(False)
        elif e < 14.0: self._set_surge(0.0); self._grip(True)
        elif e < 17.0: self._set_surge(-15.0); self._grip(True)
        elif e < 26.0: self._set_surge(0.0); self._grip(True)   # naik (depth-hold ke permukaan)
        else:
            self._grip(False); self._set_surge(0.0); self.score['m5'] = 40
            self.get_logger().info('Misi 5 AUTONOMOUS selesai (+40)!')
            self._print_score(); self._to(St.DONE)
        if e > self.T['release'] + 8.0:
            self.get_logger().error('AUTO_RELEASE timeout'); self._print_score(); self._to(St.DONE)

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
