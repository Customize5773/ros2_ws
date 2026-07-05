"""depth_publisher — turunkan KEDALAMAN ROV dari odometry (Milestone 3).

Di simulasi tidak ada sensor tekanan; kedalaman diturunkan dari posisi z odom.
Konvensi dunia Gazebo: z ke atas, permukaan air di z = 0, jadi ROV terendam -> z < 0.
Kedalaman (meter, positif ke bawah) = max(0, -z), cocok dengan konvensi GUI/panduan.

Kontrak topic (lihat docs/ARCHITECTURE.md):
    /hydroships/odom    (nav_msgs/Odometry) -> masuk : sumber z
    /hydroships/depth   (std_msgs/Float64)  -> keluar: kedalaman (m, >= 0)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64


class DepthPublisher(Node):
    def __init__(self):
        super().__init__('depth_publisher')
        self.pub = self.create_publisher(Float64, '/hydroships/depth', 10)
        self.sub = self.create_subscription(
            Odometry, '/hydroships/odom', self._on_odom, 10)

    def _on_odom(self, msg: Odometry):
        z = msg.pose.pose.position.z
        out = Float64()
        out.data = max(0.0, -z)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = DepthPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
