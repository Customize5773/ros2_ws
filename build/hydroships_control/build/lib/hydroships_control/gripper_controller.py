"""gripper_controller — petakan perintah open/close ke 2 jari gripper (Milestone 5).

Menerima perintah semantik (sama dgn kontrak GUI/autonomy: "open"/"close") lalu
menerbitkan setpoint sudut (rad) untuk tiap jari ke JointPositionController Gazebo.

Kontrak topic:
    /hydroships/gripper/command   (std_msgs/String "open"|"close") -> masuk
    /hydroships/gripper_left/cmd   (std_msgs/Float64, rad)          -> keluar (bridge->gz)
    /hydroships/gripper_right/cmd  (std_msgs/Float64, rad)          -> keluar (bridge->gz)

Konvensi sudut (tunable via parameter; verifikasi arah buka/tutup di sim):
  - open  : jari terbuka (menjauh dari tengah)
  - close : jari menutup (mendekat ke tengah)
Jari kiri & kanan simetris: right = -left.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64


class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller')
        # sudut jari kiri saat open/close (kanan = kebalikannya)
        self.declare_parameter('open_angle', 0.5)    # rad — jari kiri saat terbuka
        self.declare_parameter('close_angle', -0.15)  # rad — jari kiri saat menutup

        self.pub_l = self.create_publisher(Float64, '/hydroships/gripper_left/cmd', 10)
        self.pub_r = self.create_publisher(Float64, '/hydroships/gripper_right/cmd', 10)
        self.sub = self.create_subscription(
            String, '/hydroships/gripper/command', self._on_cmd, 10)
        # default: mulai dalam keadaan terbuka
        self._left_target = float(self.get_parameter('open_angle').value)
        # Terbitkan setpoint berkala (2 Hz) agar tidak hilang bila bridge/gz
        # belum siap saat publish awal, dan controller selalu punya target terbaru.
        self._timer = self.create_timer(0.5, self._apply)
        self.get_logger().info('gripper_controller siap (perintah: open/close)')

    def _apply(self):
        l = Float64(); l.data = self._left_target
        r = Float64(); r.data = -self._left_target
        self.pub_l.publish(l)
        self.pub_r.publish(r)

    def _on_cmd(self, msg: String):
        cmd = (msg.data or '').strip().lower()
        if cmd in ('open', 'buka', '0', 'false'):
            self._left_target = float(self.get_parameter('open_angle').value)
            self._apply()
            self.get_logger().info('gripper: OPEN')
        elif cmd in ('close', 'tutup', '1', 'true'):
            self._left_target = float(self.get_parameter('close_angle').value)
            self._apply()
            self.get_logger().info('gripper: CLOSE')
        else:
            self.get_logger().warn(f'perintah gripper tak dikenal: {msg.data!r}')


def main():
    rclpy.init()
    node = GripperController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
