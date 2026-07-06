"""qr_detector — deteksi QR dari kamera bawah → sisi kolam A/B/C/D (Milestone 3).

Baca kamera bawah (dasar kolam), decode QR pakai cv2.QRCodeDetector (tanpa cv_bridge —
Image di-decode manual via numpy), ekstrak huruf sisi A/B/C/D, publish ke /hydroships/qr_result.
Dikonsumsi mission_fsm (SCAN_QR) untuk menentukan dinding target.

Kontrak topic (lihat docs/ARCHITECTURE.md):
    /hydroships/camera_bottom/image_raw  (sensor_msgs/Image)  -> masuk
    /hydroships/qr_result                (std_msgs/String A/B/C/D) -> keluar
"""

import re
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

# Huruf sisi A-D berdiri sendiri (mis. "A", "SIDE_B", "WALL-C") — sama dgn GUI/autonomy.
_WALL_RE = re.compile(r'(?:^|[^A-Z])([ABCD])(?![A-Z])')


class QRDetector(Node):
    def __init__(self):
        super().__init__('qr_detector')
        self.declare_parameter('image_topic', '/hydroships/camera_bottom/image_raw')
        self.declare_parameter('max_rate', 5.0)   # batas deteksi/detik (hemat CPU)
        topic = self.get_parameter('image_topic').value

        self.pub = self.create_publisher(String, '/hydroships/qr_result', 10)
        self.sub = self.create_subscription(Image, topic, self._on_image, 5)
        self.det = cv2.QRCodeDetector()
        self._last_t = 0.0
        self._last_data = None
        self.get_logger().info(f'qr_detector siap (subscribe {topic})')

    def _to_cv(self, msg: Image):
        try:
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            h, w, enc = msg.height, msg.width, msg.encoding
            if enc in ('rgb8', 'bgr8'):
                img = buf.reshape(h, w, 3)
                if enc == 'rgb8':
                    img = img[:, :, ::-1]           # RGB -> BGR
                return np.ascontiguousarray(img)
            if enc in ('mono8', '8UC1'):
                return buf.reshape(h, w)
            return buf.reshape(h, w, -1)[:, :, :3]   # fallback
        except Exception as e:
            self.get_logger().warn(f'decode image gagal: {e}', throttle_duration_sec=5.0)
            return None

    def _on_image(self, msg: Image):
        now = self.get_clock().now().nanoseconds * 1e-9
        period = 1.0 / max(0.1, float(self.get_parameter('max_rate').value))
        if now - self._last_t < period:
            return
        self._last_t = now

        img = self._to_cv(msg)
        if img is None:
            return
        try:
            data, _pts, _ = self.det.detectAndDecode(img)
        except Exception:
            return
        if not data:
            return

        m = _WALL_RE.search(data.upper())
        wall = m.group(1) if m else None
        out = String()
        out.data = wall if wall else data
        self.pub.publish(out)
        if data != self._last_data:
            self.get_logger().info(f'QR terbaca: "{data}" -> sisi {wall}')
            self._last_data = data


def main(args=None):
    rclpy.init(args=args)
    node = QRDetector()
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
