"""qr_detector — deteksi QR dari kamera → sisi kolam A/B/C/D + offset piksel (M3).

Baca kamera (bawah/depan), decode QR pakai cv2.QRCodeDetector (tanpa cv_bridge —
Image di-decode manual via numpy):
  * ekstrak huruf sisi A/B/C/D  -> /hydroships/qr_result  (dipakai FSM: tentukan wall)
  * hitung POSISI QR di frame    -> /hydroships/qr_offset  (sinyal VISUAL SERVO:
    align presisi ROV ke payload). Offset diterbitkan begitu QR TERDETEKSI, walau
    decode gagal, agar servo tetap bisa memusatkan.

Kontrak topic (lihat docs/ARCHITECTURE.md):
    /hydroships/camera_bottom/image_raw  (sensor_msgs/Image)          -> masuk
    /hydroships/camera_front/image_raw   (sensor_msgs/Image)          -> masuk
    /hydroships/qr_result                (std_msgs/String A/B/C/D)     -> keluar
    /hydroships/qr_offset                (geometry_msgs/PointStamped)  -> keluar
        point.x = offset horizontal ternormalisasi [-1..1] (+ = QR di KANAN pusat)
        point.y = offset vertikal   ternormalisasi [-1..1] (+ = QR di BAWAH pusat)
        point.z = ukuran-tampak QR (fraksi sisi frame, ~proxy jarak; besar = dekat)
        header.frame_id = camera_bottom_link / camera_front_link (sumber deteksi)
"""

import re
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped

# Huruf sisi A-D berdiri sendiri (mis. "A", "SIDE_B", "WALL-C") — sama dgn GUI/autonomy.
_WALL_RE = re.compile(r'(?:^|[^A-Z])([ABCD])(?![A-Z])')


class QRDetector(Node):
    def __init__(self):
        super().__init__('qr_detector')
        # QR payload bisa terlihat kamera bawah (bila datar) atau kamera depan
        # (bila payload berdiri, QR di muka vertikal) → dengarkan keduanya.
        self.declare_parameter('image_topics',
                               ['/hydroships/camera_bottom/image_raw',
                                '/hydroships/camera_front/image_raw'])
        self.declare_parameter('max_rate', 5.0)   # batas deteksi/detik (hemat CPU)
        topics = list(self.get_parameter('image_topics').value)

        self.pub = self.create_publisher(String, '/hydroships/qr_result', 10)
        self.pub_off = self.create_publisher(PointStamped, '/hydroships/qr_offset', 10)
        # Bind nama topik ke callback agar tahu kamera sumber (utk frame_id offset).
        self._subs = [self.create_subscription(
            Image, t, lambda msg, tp=t: self._on_image(msg, tp), 5) for t in topics]
        self.det = cv2.QRCodeDetector()
        self._last_t = 0.0
        self._last_data = None
        self.get_logger().info('qr_detector siap (subscribe %s)' % ', '.join(topics))

    @staticmethod
    def _frame_of(topic):
        if 'bottom' in topic:
            return 'camera_bottom_link'
        if 'front' in topic:
            return 'camera_front_link'
        return topic

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

    def _on_image(self, msg: Image, topic=''):
        now = self.get_clock().now().nanoseconds * 1e-9
        period = 1.0 / max(0.1, float(self.get_parameter('max_rate').value))
        if now - self._last_t < period:
            return
        self._last_t = now

        img = self._to_cv(msg)
        if img is None:
            return
        try:
            data, pts, _ = self.det.detectAndDecode(img)
        except Exception:
            return

        # Offset piksel: publish begitu QR TERDETEKSI (pts ada), walau decode gagal,
        # supaya visual servo tetap bisa memusatkan QR di frame.
        if pts is not None and len(pts) > 0:
            self._publish_offset(pts, img.shape, self._frame_of(topic))

        if not data:
            return
        m = _WALL_RE.search(data.upper())
        wall = m.group(1) if m else None
        out = String()
        out.data = wall if wall else data
        self.pub.publish(out)
        if data != self._last_data:
            self.get_logger().info(f'QR terbaca: "{data}" -> sisi {wall} [{topic}]')
            self._last_data = data

    def _publish_offset(self, pts, shape, frame_id):
        p = np.asarray(pts, dtype=float).reshape(-1, 2)   # 4 sudut (x,y) piksel
        h, w = float(shape[0]), float(shape[1])
        cx, cy = p[:, 0].mean(), p[:, 1].mean()
        bw = p[:, 0].max() - p[:, 0].min()
        bh = p[:, 1].max() - p[:, 1].min()
        ps = PointStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = frame_id
        ps.point.x = float((cx - w / 2.0) / (w / 2.0))    # + = QR di kanan pusat
        ps.point.y = float((cy - h / 2.0) / (h / 2.0))    # + = QR di bawah pusat
        ps.point.z = float(max(bw / w, bh / h))           # ukuran-tampak (proxy jarak)
        self.pub_off.publish(ps)


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
