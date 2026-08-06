#!/usr/bin/env python3
"""Teleop gamepad ROV HYDROships (Logitech F310, skema gaya QGroundControl/ArduSub).

Mode (toggle, mutually exclusive):
  manual      -> wrench 6-DOF penuh (Fx,Fy,Fz,Mz; Mx=My=0) ke /hydroships/cmd_vel,
                 setara teleop_keyboard.
  depth_hold  -> Fx/Fy ke /hydroships/manual/cmd + setpoint depth/heading
                 kontinyu, setara teleop_stabilized.
  poshold     -> sisi kontrol horizontal identik depth_hold (belum ada backend
                 position-hold penuh; mode ini dicatat sebagai stub).

emergency_stop (toggle) menolkan semua wrench/Fx/Fy dan mengunci di nol
selama aktif. Setpoint depth/heading TIDAK direset oleh emergency_stop.

Tombol tanpa backend (arm, disarm, mount_tilt_up/down, gain_inc/dec) hanya
di-log, tidak publish ke topic manapun.

Publikasi ulang 10 Hz agar watchdog thruster_allocator (cmd_timeout) tidak
menolkan thruster saat stick diam.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64, String

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


class TeleopGamepad(Node):
    def __init__(self):
        super().__init__('teleop_gamepad')
        p = self.declare_parameter
        p('publish_rate', 10.0)
        p('joy_topic', '/joy')
        p('deadzone', 0.12)
        p('expo', 1.6)
        p('max_force', 20.0)
        p('max_torque', 5.0)
        p('max_heading_rate', 1.0)
        p('max_depth_rate', 0.5)

        p('axes.yaw', 0)
        p('axes.heave', 1)
        p('axes.surge', 2)
        p('axes.sway', 3)
        p('axes_invert_heave', True)
        p('axes_invert_sway', True)

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

        g = lambda n: self.get_parameter(n).value
        self.deadzone = float(g('deadzone'))
        self.expo = float(g('expo'))
        self.max_force = float(g('max_force'))
        self.max_torque = float(g('max_torque'))
        self.max_heading_rate = float(g('max_heading_rate'))
        self.max_depth_rate = float(g('max_depth_rate'))

        self.axes_idx = {
            'yaw': int(g('axes.yaw')),
            'heave': int(g('axes.heave')),
            'surge': int(g('axes.surge')),
            'sway': int(g('axes.sway')),
        }
        self.axes_invert = {
            'heave': bool(g('axes_invert_heave')),
            'sway': bool(g('axes_invert_sway')),
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

        self.mode = 'manual'
        self.wrench = [0.0] * 6
        self.fx = 0.0
        self.fy = 0.0
        self.depth_sp = -1.0
        self.heading_sp = 0.0
        self.emergency = False
        self._prev_buttons = []
        self._last_dt_ns = None

        self.create_subscription(Joy, g('joy_topic'), self.on_joy, 10)
        rate = float(g('publish_rate'))
        self.timer = self.create_timer(1.0 / rate, self.publish_cmd)
        self.get_logger().info('teleop_gamepad siap (mode awal: manual)')

    def apply_deadzone_expo(self, x):
        if abs(x) < self.deadzone:
            return 0.0
        mag = (abs(x) - self.deadzone) / (1.0 - self.deadzone)
        return math.copysign(mag ** self.expo, x)

    @staticmethod
    def _wrap(a):
        return math.atan2(math.sin(a), math.cos(a))

    def _pub_depth(self):
        self.depth_pub.publish(Float64(data=float(self.depth_sp)))

    def _pub_heading(self):
        self.heading_pub.publish(Float64(data=float(self.heading_sp)))

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
                    self.fx = self.fy = 0.0
                    self.get_logger().info(f'mode -> {self.mode}')
                if action == 'mode_poshold':
                    self.get_logger().info(
                        'mode_poshold: kontrol horizontal setara depth_hold, '
                        'position-hold backend belum ada (no-op)')

        for action in STUB_ACTIONS:
            if action in edges:
                self.get_logger().info(f'{action} ditekan (belum ada backend, no-op)')

        for action in ONE_SHOT_ACTIONS:
            if action in edges:
                cmd = 'open' if action == 'grip_open' else 'close'
                self.gripper_pub.publish(String(data=cmd))
                self.get_logger().info(f'gripper -> {cmd}')

        self._prev_buttons = list(buttons)

        if self.emergency:
            self.wrench = [0.0] * 6
            self.fx = self.fy = 0.0
            return

        axes = msg.axes
        yaw = self.apply_deadzone_expo(self._axis(axes, 'yaw'))
        heave = self.apply_deadzone_expo(self._axis(axes, 'heave'))
        surge = self.apply_deadzone_expo(self._axis(axes, 'surge'))
        sway = self.apply_deadzone_expo(self._axis(axes, 'sway'))
        if self.axes_invert['heave']:
            heave = -heave
        if self.axes_invert['sway']:
            sway = -sway

        now_ns = self.get_clock().now().nanoseconds
        dt = 0.0 if self._last_dt_ns is None else max(0.0, (now_ns - self._last_dt_ns) * 1e-9)
        self._last_dt_ns = now_ns

        if self.mode == 'manual':
            self.wrench[0] = surge * self.max_force
            self.wrench[1] = sway * self.max_force
            self.wrench[2] = heave * self.max_force
            self.wrench[5] = yaw * self.max_torque
        else:
            self.fx = surge * self.max_force
            self.fy = sway * self.max_force
            if dt > 0.0:
                new_heading = self._wrap(self.heading_sp + yaw * self.max_heading_rate * dt)
                new_depth = self.depth_sp + heave * self.max_depth_rate * dt
                if abs(new_heading - self.heading_sp) > 1e-6:
                    self.heading_sp = new_heading
                    self._pub_heading()
                if abs(new_depth - self.depth_sp) > 1e-6:
                    self.depth_sp = new_depth
                    self._pub_depth()

    def _axis(self, axes, name):
        idx = self.axes_idx[name]
        return float(axes[idx]) if idx < len(axes) else 0.0

    def publish_cmd(self):
        if self.mode == 'manual':
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.linear.z = self.wrench[0:3]
            msg.angular.x, msg.angular.y, msg.angular.z = self.wrench[3:6]
            self.cmd_vel_pub.publish(msg)
        else:
            msg = Twist()
            msg.linear.x = self.fx
            msg.linear.y = self.fy
            self.manual_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopGamepad()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.wrench = [0.0] * 6
        node.fx = node.fy = 0.0
        node.publish_cmd()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
