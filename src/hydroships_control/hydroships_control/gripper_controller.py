"""gripper_controller — node manipulator ROV (rancang ulang M5, DetachableJoint).

Menerima perintah semantik open/close (kontrak GUI/autonomy dipertahankan) lalu:
  * menggerakkan jari kosmetik 1 DOF (Float64 -> JointPositionController gz), dan
  * memicu **gz-sim DetachableJoint** attach/detach (std_msgs/Empty -> gz.msgs.Empty
    via bridge) untuk grasp fisik yang andal (bukan gesekan jari versi lama).

Attach hanya terjadi saat "close" DAN ROV berada di atas payload dalam jangkauan
aman (dinilai dari /hydroships/qr_offset). Logika keputusan ada di gripper_logic
(murni, teruji headless).

Kontrak topic (lihat docs/ARCHITECTURE.md — tak mengubah interface lama yg dipakai):
    /hydroships/gripper/command   (std_msgs/String  "open"|"close")   -> masuk
    /hydroships/qr_offset         (geometry_msgs/PointStamped)         -> masuk
    /hydroships/gripper_jaw/cmd   (std_msgs/Float64, rad)              -> keluar (bridge->gz)
    /hydroships/gripper/attach    (std_msgs/Empty)                     -> keluar (bridge->gz)
    /hydroships/gripper/detach    (std_msgs/Empty)                     -> keluar (bridge->gz)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64, Empty
from geometry_msgs.msg import PointStamped

from hydroships_control.gripper_logic import GripperLogic


class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller')
        p = self.declare_parameter
        p('max_offset', 0.30)       # |offset x/y| maks agar "di atas payload"
        p('min_size', 0.12)         # ukuran-tampak QR min (proxy dekat)
        p('offset_timeout', 1.5)    # umur maks qr_offset (s)
        p('jaw_open', 0.6)          # sudut jari terbuka (rad)
        p('jaw_close', 0.0)         # sudut jari menutup (rad)
        g = lambda n: self.get_parameter(n).value

        self.logic = GripperLogic(
            max_offset=float(g('max_offset')), min_size=float(g('min_size')),
            offset_timeout=float(g('offset_timeout')),
            jaw_open=float(g('jaw_open')), jaw_close=float(g('jaw_close')))

        self.pub_jaw = self.create_publisher(Float64, '/hydroships/gripper_jaw/cmd', 10)
        self.pub_attach = self.create_publisher(Empty, '/hydroships/gripper/attach', 10)
        self.pub_detach = self.create_publisher(Empty, '/hydroships/gripper/detach', 10)
        self.create_subscription(String, '/hydroships/gripper/command', self._on_cmd, 10)
        self.create_subscription(PointStamped, '/hydroships/qr_offset', self._on_offset, 10)

        # Terbitkan target jari berkala (2 Hz) agar tak hilang bila bridge/gz belum
        # siap saat publish awal — sama pola gripper lama.
        self._timer = self.create_timer(0.5, self._apply_jaw)
        self.get_logger().info('gripper_controller siap (DetachableJoint; perintah open/close)')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_offset(self, msg: PointStamped):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # Bila stamp kosong (0), pakai jam node agar tetap dianggap segar.
        if stamp <= 0.0:
            stamp = self._now()
        self.logic.update_offset(msg.point.x, msg.point.y, msg.point.z, stamp)

    def _apply_jaw(self):
        m = Float64(); m.data = float(self.logic.jaw_target)
        self.pub_jaw.publish(m)

    def _on_cmd(self, msg: String):
        action = self.logic.on_command(msg.data, self._now())
        if action is None:
            self.get_logger().warn('perintah gripper tak dikenal: %r' % msg.data)
            return
        self._apply_jaw()
        if action['joint'] == 'attach':
            self.pub_attach.publish(Empty())
        elif action['joint'] == 'detach':
            self.pub_detach.publish(Empty())
        self.get_logger().info('gripper %s: %s' % (action['state'], action['reason']))


def main(args=None):
    rclpy.init(args=args)
    node = GripperController()
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
