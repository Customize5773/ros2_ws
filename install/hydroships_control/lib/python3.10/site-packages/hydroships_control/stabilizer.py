#!/usr/bin/env python3
"""Stabilizer HYDROships (Milestone 2): depth-hold + heading-hold.

Membaca odometry, menahan KEDALAMAN (sumbu z) dan HEADING (yaw) memakai PID,
lalu menggabung dengan perintah horizontal manual dari pilot/autonomy. Output
berupa wrench body ke /hydroships/cmd_vel (dikonsumsi thruster_allocator).

Aliran:
    /hydroships/odom          (Odometry)  -> pengukuran z & yaw
    /hydroships/manual/cmd    (Twist)     -> Fx, Fy, Mx, My manual (pass-through)
    /hydroships/setpoint/depth   (Float64)-> target kedalaman (m, negatif = dalam)
    /hydroships/setpoint/heading (Float64)-> target yaw (rad)
        =>  /hydroships/cmd_vel (Twist = wrench)

Fz & Mz berasal dari PID (saat hold aktif); komponen lain dari manual.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64

from hydroships_control.pid import PID, wrap_to_pi


def yaw_from_quaternion(q):
    """Ekstrak yaw (rad) dari geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


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
        # Feedforward untuk mengimbangi gaya apung bersih (N, negatif = dorong turun).
        self.declare_parameter('buoyancy_ff', -1.45)
        self.declare_parameter('target_depth', -1.0)
        self.declare_parameter('target_heading', 0.0)
        self.declare_parameter('enable_depth_hold', True)
        self.declare_parameter('enable_heading_hold', True)

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

        self.buoyancy_ff = gp('buoyancy_ff').value
        self.target_depth = gp('target_depth').value
        self.target_heading = gp('target_heading').value
        self.enable_depth = gp('enable_depth_hold').value
        self.enable_heading = gp('enable_heading_hold').value

        # ---- State ----
        self.cur_z = None
        self.cur_yaw = None
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

        rate = gp('rate').value
        self.timer = self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().info(
            f'Stabilizer aktif. depth_hold={self.enable_depth} '
            f'heading_hold={self.enable_heading} '
            f'target_depth={self.target_depth} m.')

    # ---- Callback ----
    def on_odom(self, msg: Odometry):
        self.cur_z = msg.pose.pose.position.z
        self.cur_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def on_manual(self, msg: Twist):
        self.manual = msg

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
        # Horizontal & sikap lain: pass-through dari manual.
        out.linear.x = self.manual.linear.x
        out.linear.y = self.manual.linear.y
        out.angular.x = self.manual.angular.x
        out.angular.y = self.manual.angular.y

        # Heave (Fz): depth-hold atau manual.
        if self.enable_depth and self.cur_z is not None:
            err = self.target_depth - self.cur_z
            out.linear.z = self.depth_pid.update(err, self.cur_z, dt) + self.buoyancy_ff
        else:
            out.linear.z = self.manual.linear.z

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
