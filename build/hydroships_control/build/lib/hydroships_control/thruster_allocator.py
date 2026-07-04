#!/usr/bin/env python3
"""Alokasi thruster ROV HYDROships (Milestone 1, open-loop).

Menerima wrench body 6-DOF pada topic /hydroships/cmd_vel (geometry_msgs/Twist,
diinterpretasikan sebagai gaya & torsi, BUKAN kecepatan) lalu memetakannya ke
6 perintah gaya thruster (N) memakai pseudo-inverse dari Thrust Allocation
Matrix (TAM).

Konvensi wrench (frame body, REP-103 x-maju/y-kiri/z-atas):
  linear.x  -> Fx (surge)   angular.x -> Mx (roll)
  linear.y  -> Fy (sway)    angular.y -> My (pitch)
  linear.z  -> Fz (heave)   angular.z -> Mz (yaw)

Geometri thruster HARUS konsisten dengan urdf/hydroships.urdf.xacro dan
docs/thruster_config.md. Kolom ke-i TAM = [axis_i ; pos_i x axis_i].
"""

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64

from hydroships_control.allocation import (
    MAX_THRUST,
    MIN_THRUST,
    THRUSTERS,
    allocate,
    build_allocation_matrix,
)


class ThrusterAllocator(Node):
    def __init__(self):
        super().__init__('thruster_allocator')

        self.tam = build_allocation_matrix(THRUSTERS)
        # Pseudo-inverse: f = pinv(TAM) @ wrench.
        self.tam_pinv = np.linalg.pinv(self.tam)

        self.pubs = [
            self.create_publisher(Float64, f'/hydroships/thruster_{i + 1}/thrust', 10)
            for i in range(len(THRUSTERS))
        ]
        self.sub = self.create_subscription(
            Twist, '/hydroships/cmd_vel', self.on_cmd, 10)

        # Watchdog: kalau tidak ada perintah > timeout, matikan thruster.
        self.cmd_timeout = 0.5
        self.last_cmd_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.on_timer)

        self.get_logger().info(
            f'Thruster allocator siap. TAM rank = '
            f'{np.linalg.matrix_rank(self.tam)} / 6.')

    def on_cmd(self, msg: Twist):
        wrench = np.array([
            msg.linear.x, msg.linear.y, msg.linear.z,
            msg.angular.x, msg.angular.y, msg.angular.z,
        ])
        forces = allocate(wrench, self.tam_pinv, MIN_THRUST, MAX_THRUST)
        self.publish_forces(forces)
        self.last_cmd_time = self.get_clock().now()

    def on_timer(self):
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if elapsed > self.cmd_timeout:
            self.publish_forces(np.zeros(len(self.pubs)))

    def publish_forces(self, forces):
        for pub, f in zip(self.pubs, forces):
            msg = Float64()
            msg.data = float(np.clip(f, MIN_THRUST, MAX_THRUST))
            pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ThrusterAllocator()
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
