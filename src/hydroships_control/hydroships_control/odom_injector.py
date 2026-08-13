"""odom_injector — P2-B: suntik noise realistis ke /hydroships/odom (opsional).

Ground truth Gazebo dibridge ke /hydroships/odom_raw (lihat bridge.yaml); node
ini relay ke /hydroships/odom, +noise kalau odom_noise:=true. Default false =
passthrough identik (tak mengubah perilaku sim yang sudah ada). Memaksa
stabilizer & mission_fsm bekerja dgn estimasi tak sempurna — mendekati kondisi
tanpa DVL/GPS bawah air (lihat docs/ARCHITECTURE.md, P2).

Kontrak topic:
    /hydroships/odom_raw (nav_msgs/Odometry) -> masuk : ground truth Gazebo
    /hydroships/odom     (nav_msgs/Odometry) -> keluar: +noise (atau apa adanya)
"""

import math
import random

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

from hydroships_control.odom_noise_logic import (
    add_white_noise, perturb_heading, step_heading_bias)


class OdomInjector(Node):

    def __init__(self):
        super().__init__('odom_injector')

        def p(name, default):
            self.declare_parameter(name, default)

        def g(name):
            return self.get_parameter(name).value

        p('odom_noise', False)            # aktifkan noise (false = passthrough)
        p('pos_noise_std', 0.03)          # m, white noise posisi (sigma 0.02-0.05)
        p('vel_noise_std', 0.02)          # m/s, white noise kecepatan (0.01-0.03)
        p('heading_noise_std_deg', 1.0)   # deg/√s, random walk heading (0.5-2.0)
        p('noise_seed', 0)                # 0 = acak penuh; isi utk reproducible

        self.enabled = bool(g('odom_noise'))
        self.pos_std = float(g('pos_noise_std'))
        self.vel_std = float(g('vel_noise_std'))
        self.heading_std = math.radians(float(g('heading_noise_std_deg')))
        seed = int(g('noise_seed'))
        self.rng = random.Random(seed) if seed else random.Random()

        self._heading_bias = 0.0
        self._last_t = None

        self.pub = self.create_publisher(Odometry, '/hydroships/odom', 10)
        self.create_subscription(Odometry, '/hydroships/odom_raw', self._on_odom, 10)
        self.get_logger().info(
            'odom_injector siap (noise=%s pos_std=%.3fm vel_std=%.3fm/s '
            'heading_std=%.2fdeg/√s)'
            % (self.enabled, self.pos_std, self.vel_std,
               math.degrees(self.heading_std)))

    def _on_odom(self, msg):
        if not self.enabled:
            self.pub.publish(msg)
            return

        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = 0.0 if self._last_t is None else max(0.0, t - self._last_t)
        self._last_t = t
        self._heading_bias = step_heading_bias(
            self._heading_bias, dt, self.heading_std, self.rng)

        pos = msg.pose.pose.position
        pos.x = add_white_noise(pos.x, self.pos_std, self.rng)
        pos.y = add_white_noise(pos.y, self.pos_std, self.rng)
        pos.z = add_white_noise(pos.z, self.pos_std, self.rng)

        lin = msg.twist.twist.linear
        lin.x = add_white_noise(lin.x, self.vel_std, self.rng)
        lin.y = add_white_noise(lin.y, self.vel_std, self.rng)
        lin.z = add_white_noise(lin.z, self.vel_std, self.rng)

        q = msg.pose.pose.orientation
        nq = perturb_heading((q.w, q.x, q.y, q.z), self._heading_bias)
        q.w, q.x, q.y, q.z = nq

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = OdomInjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
