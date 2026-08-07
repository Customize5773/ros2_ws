#!/usr/bin/env python3
"""Teleop gamepad ROV HYDROships (Logitech F310, skema gaya QGroundControl/ArduSub).

Routing (parameter `route_through_stabilizer`):
  true  -> SEMUA mode publish wrench 6-DOF ke /hydroships/manual/cmd, dan mode
           dikirim ke stabilizer lewat /hydroships/control_mode. Stabilizer yang
           menerbitkan wrench final ke /hydroships/cmd_vel. Dipakai bersama
           hydroships_stabilized.launch.py.
  false -> publish wrench langsung ke /hydroships/cmd_vel seperti teleop_keyboard.
           Dipakai bersama hydroships_sim.launch.py (tanpa stabilizer).

PENTING: jangan pernah publish ke /hydroships/cmd_vel saat stabilizer jalan —
stabilizer menerbitkan ke topic itu tiap tick tanpa syarat, sehingga dua
publisher membuat thruster_allocator melihat aliran berselang-seling (perintah
pilot cuma dapat sebagian duty cycle dan PID melawan perintah manual).

Mode (toggle, mutually exclusive) — stabilizer mematikan hold sesuai mode:
  manual      -> semua hold off; stick heave/yaw jadi Fz/Mz langsung.
  depth_hold  -> depth+pitch+roll hold; stick heave menggeser setpoint depth,
                 stick yaw tetap Mz langsung.
  poshold     -> depth+heading+pitch+roll hold; stick heave & yaw menggeser
                 setpoint depth & heading.

Perpindahan mode bersifat *bumpless*: setpoint di-seed dari odometry saat itu
(z & yaw aktual), bukan dari nilai hardcode, supaya tidak ada step besar
melawan PID.

emergency_stop (toggle) menolkan semua wrench dan mengunci di nol selama aktif.
Setpoint depth/heading TIDAK direset oleh emergency_stop.

Tombol tanpa backend (arm, disarm, mount_tilt_up/down, gain_inc/dec) hanya
di-log, tidak publish ke topic manapun.

Publikasi ulang berkala agar watchdog thruster_allocator (cmd_timeout) tidak
menolkan thruster saat stick diam.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64, String

from hydroships_control.stabilizer import yaw_from_quaternion

MODE_ACTIONS = ('mode_manual', 'mode_depth_hold', 'mode_poshold')
MODE_FOR_ACTION = {
    'mode_manual': 'manual',
    'mode_depth_hold': 'depth_hold',
    'mode_poshold': 'poshold',
}
STUB_ACTIONS = (
    'arm', 'disarm', 'mount_tilt_up', 'mount_tilt_down', 'gain_inc', 'gain_dec',
)
ONE_SHOT_ACTIONS = ('grip_open', 'grip_close')
# Batas dt integrasi setpoint (s) — menahan lompatan setelah jeda panjang.
MAX_INTEGRATION_DT = 0.2


class TeleopGamepad(Node):
    def __init__(self):
        super().__init__('teleop_gamepad')
        p = self.declare_parameter
        p('publish_rate', 20.0)
        p('joy_topic', '/joy')
        p('route_through_stabilizer', True)
        p('deadzone', 0.12)
        p('expo', 1.6)
        p('max_force', 20.0)
        p('max_torque', 5.0)
        p('max_heading_rate', 1.0)
        p('max_depth_rate', 0.5)

        # Konvensi ArduSub: Axis X = surge, Axis Y = sway, Axis Z = heave, Axis R = yaw.
        p('axes.yaw', 0)
        p('axes.heave', 1)
        p('axes.sway', 2)
        p('axes.surge', 3)
        p('axes_invert_yaw', False)
        p('axes_invert_heave', True)
        p('axes_invert_sway', False)
        p('axes_invert_surge', True)

        p('buttons_shift', 16)
        p('buttons_regular_mode_poshold', 0)
        p('buttons_regular_mode_manual', 1)
        p('buttons_regular_emergency_stop', 2)
        p('buttons_regular_mode_depth_hold', 3)
        p('buttons_regular_mount_tilt_up', 4)
        p('buttons_regular_grip_close', 5)
        p('buttons_regular_mount_tilt_down', 6)
        p('buttons_regular_grip_open', 7)
        p('buttons_regular_disarm', 8)
        p('buttons_regular_arm', 9)
        p('buttons_regular_gain_dec', 12)
        p('buttons_regular_gain_inc', 13)
        p('buttons_shift_gain_dec', 12)
        p('buttons_shift_gain_inc', 13)
        p('buttons_shift_mount_tilt_up', 14)
        p('buttons_shift_mount_tilt_down', 15)

        p('cmd_vel_topic', '/hydroships/cmd_vel')
        p('manual_cmd_topic', '/hydroships/manual/cmd')
        p('depth_setpoint_topic', '/hydroships/setpoint/depth')
        p('heading_setpoint_topic', '/hydroships/setpoint/heading')
        p('gripper_cmd_topic', '/hydroships/gripper/command')
        p('control_mode_topic', '/hydroships/control_mode')
        p('odom_topic', '/hydroships/odom')

        g = lambda n: self.get_parameter(n).value
        self.deadzone = float(g('deadzone'))
        self.expo = float(g('expo'))
        self.max_force = float(g('max_force'))
        self.max_torque = float(g('max_torque'))
        self.max_heading_rate = float(g('max_heading_rate'))
        self.max_depth_rate = float(g('max_depth_rate'))

        self.route_through_stabilizer = bool(g('route_through_stabilizer'))
        self.axes_idx = {
            'yaw': int(g('axes.yaw')),
            'heave': int(g('axes.heave')),
            'sway': int(g('axes.sway')),
            'surge': int(g('axes.surge')),
        }
        self.axes_invert = {
            'yaw': bool(g('axes_invert_yaw')),
            'heave': bool(g('axes_invert_heave')),
            'sway': bool(g('axes_invert_sway')),
            'surge': bool(g('axes_invert_surge')),
        }

        self.shift_idx = int(g('buttons_shift'))
        self.buttons_regular = {
            'mode_poshold': int(g('buttons_regular_mode_poshold')),
            'mode_manual': int(g('buttons_regular_mode_manual')),
            'emergency_stop': int(g('buttons_regular_emergency_stop')),
            'mode_depth_hold': int(g('buttons_regular_mode_depth_hold')),
            'mount_tilt_up': int(g('buttons_regular_mount_tilt_up')),
            'grip_close': int(g('buttons_regular_grip_close')),
            'mount_tilt_down': int(g('buttons_regular_mount_tilt_down')),
            'grip_open': int(g('buttons_regular_grip_open')),
            'disarm': int(g('buttons_regular_disarm')),
            'arm': int(g('buttons_regular_arm')),
            'gain_dec': int(g('buttons_regular_gain_dec')),
            'gain_inc': int(g('buttons_regular_gain_inc')),
        }
        self.buttons_shift = {
            'gain_dec': int(g('buttons_shift_gain_dec')),
            'gain_inc': int(g('buttons_shift_gain_inc')),
            'mount_tilt_up': int(g('buttons_shift_mount_tilt_up')),
            'mount_tilt_down': int(g('buttons_shift_mount_tilt_down')),
        }

        self.cmd_vel_pub = self.create_publisher(Twist, g('cmd_vel_topic'), 10)
        self.manual_pub = self.create_publisher(Twist, g('manual_cmd_topic'), 10)
        self.depth_pub = self.create_publisher(Float64, g('depth_setpoint_topic'), 10)
        self.heading_pub = self.create_publisher(Float64, g('heading_setpoint_topic'), 10)
        self.gripper_pub = self.create_publisher(String, g('gripper_cmd_topic'), 10)
        self.mode_pub = self.create_publisher(String, g('control_mode_topic'), 10)

        self.mode = 'manual'
        # wrench[0:3] = Fx,Fy,Fz ; wrench[3:6] = Mx,My,Mz
        self.wrench = [0.0] * 6
        # Setpoint di-seed dari odometry saat masuk mode ber-hold (bumpless);
        # None = belum pernah di-seed.
        self.depth_sp = None
        self.heading_sp = None
        self.cur_z = None
        self.cur_yaw = None
        self.emergency = False
        self._prev_buttons = []
        self._last_dt_ns = None

        self.create_subscription(Joy, g('joy_topic'), self.on_joy, 10)
        self.create_subscription(Odometry, g('odom_topic'), self.on_odom, 10)
        rate = float(g('publish_rate'))
        self.timer = self.create_timer(1.0 / rate, self.publish_cmd)

        route = ('stabilizer (/hydroships/manual/cmd)' if self.route_through_stabilizer
                 else 'langsung (/hydroships/cmd_vel)')
        self.get_logger().info(f'teleop_gamepad siap (mode awal: manual, rute: {route})')
        if self.route_through_stabilizer:
            self._publish_mode()

    def apply_deadzone_expo(self, x):
        if abs(x) < self.deadzone:
            return 0.0
        mag = (abs(x) - self.deadzone) / (1.0 - self.deadzone)
        return math.copysign(mag ** self.expo, x)

    @staticmethod
    def _wrap(a):
        return math.atan2(math.sin(a), math.cos(a))

    def on_odom(self, msg: Odometry):
        self.cur_z = msg.pose.pose.position.z
        self.cur_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def _pub_depth(self):
        if self.depth_sp is not None:
            self.depth_pub.publish(Float64(data=float(self.depth_sp)))

    def _pub_heading(self):
        if self.heading_sp is not None:
            self.heading_pub.publish(Float64(data=float(self.heading_sp)))

    def _publish_mode(self):
        self.mode_pub.publish(String(data=self.mode))

    def _seed_setpoints(self):
        """Kunci setpoint ke keadaan saat ini supaya perpindahan mode bumpless.

        Tanpa ini, setpoint lama (atau nilai awal) menimbulkan step besar
        melawan PID begitu hold aktif.
        """
        if self.cur_z is not None:
            self.depth_sp = self.cur_z
            self._pub_depth()
        if self.cur_yaw is not None:
            self.heading_sp = self.cur_yaw
            self._pub_heading()
        if self.cur_z is None or self.cur_yaw is None:
            self.get_logger().warn(
                'odometry belum tersedia — setpoint tidak bisa di-seed, '
                'stabilizer akan memakai target dari gains.yaml')

    def _pressed(self, buttons, action_map, action):
        idx = action_map[action]
        return idx < len(buttons) and bool(buttons[idx])

    def _rising_edges(self, buttons, action_map):
        edges = set()
        for action, idx in action_map.items():
            now = idx < len(buttons) and bool(buttons[idx])
            was = idx < len(self._prev_buttons) and bool(self._prev_buttons[idx])
            if now and not was:
                edges.add(action)
        return edges

    def on_joy(self, msg: Joy):
        buttons = msg.buttons
        shift_held = self.shift_idx < len(buttons) and bool(buttons[self.shift_idx])
        action_map = self.buttons_shift if shift_held else self.buttons_regular

        edges = self._rising_edges(buttons, action_map)

        if 'emergency_stop' in edges:
            self.emergency = not self.emergency
            self.get_logger().info(f'emergency_stop -> {self.emergency}')

        for action in MODE_ACTIONS:
            if action in edges:
                new_mode = MODE_FOR_ACTION[action]
                if new_mode != self.mode:
                    self.mode = new_mode
                    self.wrench = [0.0] * 6
                    self.get_logger().info(f'mode -> {self.mode}')
                    # Seed dulu (menerbitkan setpoint baru), baru umumkan mode,
                    # supaya stabilizer sudah punya target benar saat hold nyala.
                    if new_mode in ('depth_hold', 'poshold'):
                        self._seed_setpoints()
                    if self.route_through_stabilizer:
                        self._publish_mode()

        for action in STUB_ACTIONS:
            if action in edges:
                self.get_logger().info(f'{action} ditekan (belum ada backend, no-op)')

        for action in ONE_SHOT_ACTIONS:
            if action in edges:
                cmd = 'open' if action == 'grip_open' else 'close'
                self.gripper_pub.publish(String(data=cmd))
                self.get_logger().info(f'gripper -> {cmd}')

        self._prev_buttons = list(buttons)

        # dt selalu di-update, termasuk saat emergency, supaya saat emergency
        # dilepas integrator setpoint tidak melompat sebesar durasi emergency.
        # Di-clamp agar jeda panjang (sim di-pause, joy_node tersendat) tidak
        # menghasilkan lompatan setpoint.
        now_ns = self.get_clock().now().nanoseconds
        dt = 0.0 if self._last_dt_ns is None else (now_ns - self._last_dt_ns) * 1e-9
        dt = min(max(dt, 0.0), MAX_INTEGRATION_DT)
        self._last_dt_ns = now_ns

        if self.emergency:
            self.wrench = [0.0] * 6
            return

        axes = msg.axes
        yaw = self._axis(axes, 'yaw')
        heave = self._axis(axes, 'heave')
        surge = self._axis(axes, 'surge')
        sway = self._axis(axes, 'sway')

        # Fx/Fy selalu dari stick (stabilizer meneruskannya apa adanya).
        self.wrench[0] = surge * self.max_force
        self.wrench[1] = sway * self.max_force

        if self.mode == 'manual':
            # Semua hold off -> heave & yaw jadi gaya/momen langsung.
            self.wrench[2] = heave * self.max_force
            self.wrench[5] = yaw * self.max_torque
            return

        # Mode ber-hold: Fz dipegang PID depth, jadi jangan kirim Fz manual.
        self.wrench[2] = 0.0
        if self.mode == 'depth_hold':
            # Heading masih manual -> yaw tetap momen langsung.
            self.wrench[5] = yaw * self.max_torque
        else:  # poshold: heading dipegang PID.
            self.wrench[5] = 0.0

        if dt <= 0.0:
            return
        if self.depth_sp is not None and heave != 0.0:
            self.depth_sp += heave * self.max_depth_rate * dt
            self._pub_depth()
        if self.mode == 'poshold' and self.heading_sp is not None and yaw != 0.0:
            self.heading_sp = self._wrap(
                self.heading_sp + yaw * self.max_heading_rate * dt)
            self._pub_heading()

    def _axis(self, axes, name):
        """Nilai axis setelah deadzone, expo, dan inversi."""
        idx = self.axes_idx[name]
        raw = float(axes[idx]) if idx < len(axes) else 0.0
        value = self.apply_deadzone_expo(raw)
        return -value if self.axes_invert[name] else value

    def publish_cmd(self):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z = self.wrench[0:3]
        msg.angular.x, msg.angular.y, msg.angular.z = self.wrench[3:6]
        # Satu tujuan saja. Publish ke cmd_vel selagi stabilizer jalan akan
        # membuat allocator melihat dua aliran perintah yang saling berebut.
        if self.route_through_stabilizer:
            self.manual_pub.publish(msg)
        else:
            self.cmd_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopGamepad()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.wrench = [0.0] * 6
        node.publish_cmd()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
