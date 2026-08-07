#!/usr/bin/env python3
"""Stabilizer HYDROships (Milestone 2): depth-hold + heading-hold + pitch/roll-hold.

Membaca odometry, menahan KEDALAMAN (z), HEADING (yaw), PITCH & ROLL memakai PID,
lalu menggabung dengan perintah horizontal manual dari pilot/autonomy. Output
berupa wrench body ke /hydroships/cmd_vel (dikonsumsi thruster_allocator).

Aliran:
    /hydroships/odom          (Odometry)  -> pengukuran z, roll, pitch, yaw
    /hydroships/manual/cmd    (Twist)     -> Fx, Fy manual (pass-through)
    /hydroships/setpoint/depth   (Float64)-> target kedalaman (m, negatif = dalam)
    /hydroships/setpoint/heading (Float64)-> target yaw (rad)
        =>  /hydroships/cmd_vel (Twist = wrench)

Fz, Mx, My, Mz berasal dari PID (saat hold masing-masing aktif); Fx/Fy dari manual.
Pitch/roll-hold adalah lapisan robustness ringan; restoring moment pasif utama
berasal dari CoB di atas CoG (lihat rov_params.yaml: cob).
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64, String

from hydroships_control.pid import PID, wrap_to_pi

# Mode kendali runtime (dikirim teleop_gamepad lewat /hydroships/control_mode).
# Nilai = (depth_hold, heading_hold, pitch_hold, roll_hold). Pitch/roll selalu
# aktif di mode ber-hold sebagai lapisan robustness; di manual pilot pegang penuh.
CONTROL_MODES = {
    'manual': (False, False, False, False),
    'depth_hold': (True, False, True, True),
    'poshold': (True, True, True, True),
}


def yaw_from_quaternion(q):
    """Ekstrak yaw (rad) dari geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def roll_pitch_from_quaternion(q):
    """Ekstrak roll & pitch (rad) dari geometry_msgs/Quaternion."""
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    return roll, pitch


class Stabilizer(Node):
    def __init__(self):
        super().__init__('stabilizer')

        # ---- Parameter (bisa di-override dari gains.yaml) ----
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('depth.kp', 60.0)
        self.declare_parameter('depth.ki', 8.0)
        self.declare_parameter('depth.kd', 40.0)
        self.declare_parameter('depth.integral_limit', 30.0)
        self.declare_parameter('depth.out_limit', 60.0)
        self.declare_parameter('heading.kp', 8.0)
        self.declare_parameter('heading.ki', 0.5)
        self.declare_parameter('heading.kd', 4.0)
        self.declare_parameter('heading.integral_limit', 5.0)
        self.declare_parameter('heading.out_limit', 15.0)
        self.declare_parameter('pitch.kp', 6.0)
        self.declare_parameter('pitch.ki', 0.3)
        self.declare_parameter('pitch.kd', 3.0)
        self.declare_parameter('pitch.integral_limit', 5.0)
        self.declare_parameter('pitch.out_limit', 15.0)
        self.declare_parameter('roll.kp', 6.0)
        self.declare_parameter('roll.ki', 0.3)
        self.declare_parameter('roll.kd', 3.0)
        self.declare_parameter('roll.integral_limit', 5.0)
        self.declare_parameter('roll.out_limit', 15.0)
        # Feedforward untuk mengimbangi gaya apung bersih (N, negatif = dorong turun).
        # Lihat gains.yaml: near-neutral setelah collision box buoyancy diskalakan.
        self.declare_parameter('buoyancy_ff', -0.3)
        self.declare_parameter('target_depth', -0.1)
        self.declare_parameter('target_heading', 0.0)
        self.declare_parameter('target_pitch', 0.0)
        self.declare_parameter('target_roll', 0.0)
        self.declare_parameter('enable_depth_hold', True)
        self.declare_parameter('enable_heading_hold', True)
        self.declare_parameter('enable_pitch_hold', True)
        self.declare_parameter('enable_roll_hold', True)

        gp = self.get_parameter
        self.depth_pid = PID(
            gp('depth.kp').value, gp('depth.ki').value, gp('depth.kd').value,
            out_min=-gp('depth.out_limit').value,
            out_max=gp('depth.out_limit').value,
            integral_limit=gp('depth.integral_limit').value)
        self.heading_pid = PID(
            gp('heading.kp').value, gp('heading.ki').value, gp('heading.kd').value,
            out_min=-gp('heading.out_limit').value,
            out_max=gp('heading.out_limit').value,
            integral_limit=gp('heading.integral_limit').value)
        self.pitch_pid = PID(
            gp('pitch.kp').value, gp('pitch.ki').value, gp('pitch.kd').value,
            out_min=-gp('pitch.out_limit').value,
            out_max=gp('pitch.out_limit').value,
            integral_limit=gp('pitch.integral_limit').value)
        self.roll_pid = PID(
            gp('roll.kp').value, gp('roll.ki').value, gp('roll.kd').value,
            out_min=-gp('roll.out_limit').value,
            out_max=gp('roll.out_limit').value,
            integral_limit=gp('roll.integral_limit').value)

        self.buoyancy_ff = gp('buoyancy_ff').value
        self.target_depth = gp('target_depth').value
        self.target_heading = gp('target_heading').value
        self.target_pitch = gp('target_pitch').value
        self.target_roll = gp('target_roll').value
        self.enable_depth = gp('enable_depth_hold').value
        self.enable_heading = gp('enable_heading_hold').value
        self.enable_pitch = gp('enable_pitch_hold').value
        self.enable_roll = gp('enable_roll_hold').value

        # ---- State ----
        self.cur_z = None
        self.cur_yaw = None
        self.cur_pitch = None
        self.cur_roll = None
        self.manual = Twist()
        self.last_time = None

        # ---- I/O ----
        self.pub = self.create_publisher(Twist, '/hydroships/cmd_vel', 10)
        self.create_subscription(Odometry, '/hydroships/odom', self.on_odom, 10)
        self.create_subscription(Twist, '/hydroships/manual/cmd', self.on_manual, 10)
        self.create_subscription(
            Float64, '/hydroships/setpoint/depth', self.on_depth_sp, 10)
        self.create_subscription(
            Float64, '/hydroships/setpoint/heading', self.on_heading_sp, 10)
        # Mode kendali runtime. Tanpa pesan di topic ini perilaku tetap memakai
        # enable_*_hold dari gains.yaml, jadi teleop_stabilized & mission_fsm
        # yang sudah ada tidak berubah.
        self.create_subscription(
            String, '/hydroships/control_mode', self.on_control_mode, 10)

        rate = gp('rate').value
        self.timer = self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().info(
            f'Stabilizer aktif. depth_hold={self.enable_depth} '
            f'heading_hold={self.enable_heading} '
            f'pitch_hold={self.enable_pitch} roll_hold={self.enable_roll} '
            f'target_depth={self.target_depth} m.')

    # ---- Callback ----
    def on_odom(self, msg: Odometry):
        self.cur_z = msg.pose.pose.position.z
        self.cur_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.cur_roll, self.cur_pitch = roll_pitch_from_quaternion(
            msg.pose.pose.orientation)

    def on_manual(self, msg: Twist):
        self.manual = msg

    def on_control_mode(self, msg: String):
        mode = msg.data.strip()
        flags = CONTROL_MODES.get(mode)
        if flags is None:
            self.get_logger().warn(f'mode kendali tak dikenal: {msg.data!r}')
            return

        depth, heading, pitch, roll = flags
        if (depth, heading, pitch, roll) == (
                self.enable_depth, self.enable_heading,
                self.enable_pitch, self.enable_roll):
            return

        # Reset integrator tiap hold yang BERUBAH status supaya windup dari mode
        # sebelumnya tidak terbawa (mis. integral depth yang menumpuk saat manual).
        for was, now, pid in (
                (self.enable_depth, depth, self.depth_pid),
                (self.enable_heading, heading, self.heading_pid),
                (self.enable_pitch, pitch, self.pitch_pid),
                (self.enable_roll, roll, self.roll_pid)):
            if was != now:
                pid.reset()

        # Bumpless: saat hold baru dinyalakan, kunci target ke keadaan SAAT INI
        # supaya tidak ada step besar melawan PID. Pertahanan kedua — teleop
        # normalnya sudah mengirim setpoint sendiri saat ganti mode.
        if depth and not self.enable_depth and self.cur_z is not None:
            self.target_depth = self.cur_z
        if heading and not self.enable_heading and self.cur_yaw is not None:
            self.target_heading = self.cur_yaw

        self.enable_depth = depth
        self.enable_heading = heading
        self.enable_pitch = pitch
        self.enable_roll = roll
        self.get_logger().info(
            f'mode kendali -> {mode} (depth_hold={depth} heading_hold={heading} '
            f'pitch_hold={pitch} roll_hold={roll} '
            f'target_depth={self.target_depth:.2f} m)')

    def on_depth_sp(self, msg: Float64):
        self.target_depth = msg.data

    def on_heading_sp(self, msg: Float64):
        self.target_heading = wrap_to_pi(msg.data)

    def on_timer(self):
        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            return
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        out = Twist()
        # Horizontal: pass-through dari manual.
        out.linear.x = self.manual.linear.x
        out.linear.y = self.manual.linear.y

        # Heave (Fz): depth-hold atau manual.
        if self.enable_depth and self.cur_z is not None:
            err = self.target_depth - self.cur_z
            out.linear.z = self.depth_pid.update(err, self.cur_z, dt) + self.buoyancy_ff
        else:
            out.linear.z = self.manual.linear.z

        # Roll (Mx): roll-hold atau manual.
        if self.enable_roll and self.cur_roll is not None:
            err = wrap_to_pi(self.target_roll - self.cur_roll)
            out.angular.x = self.roll_pid.update(err, self.cur_roll, dt)
        else:
            out.angular.x = self.manual.angular.x

        # Pitch (My): pitch-hold atau manual.
        if self.enable_pitch and self.cur_pitch is not None:
            err = wrap_to_pi(self.target_pitch - self.cur_pitch)
            out.angular.y = self.pitch_pid.update(err, self.cur_pitch, dt)
        else:
            out.angular.y = self.manual.angular.y

        # Yaw (Mz): heading-hold atau manual.
        if self.enable_heading and self.cur_yaw is not None:
            err = wrap_to_pi(self.target_heading - self.cur_yaw)
            out.angular.z = self.heading_pid.update(err, self.cur_yaw, dt)
        else:
            out.angular.z = self.manual.angular.z

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = Stabilizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
