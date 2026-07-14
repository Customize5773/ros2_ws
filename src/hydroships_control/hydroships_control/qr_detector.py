"""qr_detector — deteksi QR dari kamera → sisi kolam A/B/C/D + offset piksel (M3).

Baca kamera (bawah/depan), decode QR pakai cv2.QRCodeDetector (tanpa cv_bridge —
Image di-decode manual via numpy). Logika decode/offset murni ada di `qr_logic.py`
(testable headless):
  * ekstrak huruf sisi A/B/C/D  -> /hydroships/qr_result  (dipakai FSM: tentukan wall)
  * hitung POSISI QR di frame    -> /hydroships/qr_offset  (sinyal VISUAL SERVO:
    align presisi ROV ke payload). Offset diterbitkan begitu QR TERDETEKSI, walau
    decode gagal, agar servo tetap bisa memusatkan.

Preprocessing decode berjenjang (grayscale+CLAHE+adaptive-threshold+upscale) di
`qr_logic.robust_decode` mengatasi render kamera sim terdegradasi (lantai berfaset,
bayangan, QR kecil di frame) — lihat PROBLEM.md "QR belum terbaca meski posisi tepat".

Kontrak topic (lihat docs/ARCHITECTURE.md):
    /hydroships/camera_bottom/image_raw   (sensor_msgs/Image)          -> masuk
    /hydroships/camera_front/image_raw    (sensor_msgs/Image)          -> masuk
    /hydroships/camera_*/camera_info      (sensor_msgs/CameraInfo)     -> masuk (K disimpan)
    /hydroships/qr_result                 (std_msgs/String A/B/C/D)     -> keluar
    /hydroships/qr_offset                 (geometry_msgs/PointStamped)  -> keluar
        point.x = offset horizontal ternormalisasi [-1..1] (+ = QR di KANAN pusat)
        point.y = offset vertikal   ternormalisasi [-1..1] (+ = QR di BAWAH pusat)
        point.z = ukuran-tampak QR (fraksi sisi frame, ~proxy jarak; besar = dekat)
        header.frame_id = camera_bottom_link / camera_front_link (sumber deteksi)

CATATAN INTRINSICS (Tugas M3/M6): K matrix dari `camera_info` DISIMPAN untuk
konsumen visual-servo, TAPI intrinsics ini murni hasil kalkulasi Gazebo dari
FOV/resolusi SDF sim — BUKAN kalibrasi kamera fisik ROV asli. JANGAN dipakai
untuk estimasi jarak riil sampai kalibrasi hardware tersedia (lihat PROBLEM.md).
"""

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped

from hydroships_control.qr_logic import robust_decode, parse_wall, offset_from_points


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

        # CameraInfo (intrinsics): simpan K per-kamera. MURNI kalkulasi Gazebo dari
        # FOV/resolusi SDF — BUKAN kalibrasi kamera fisik ROV. Disimpan agar konsumen
        # visual-servo bisa memakainya, TAPI jangan untuk estimasi jarak riil sampai
        # kalibrasi hardware tersedia (lihat PROBLEM.md / docs/ARCHITECTURE.md).
        self.K = {}                # frame_id -> 3x3 K matrix (numpy) atau None
        for info_topic, frame in (('/hydroships/camera_bottom/camera_info', 'camera_bottom_link'),
                                  ('/hydroships/camera_front/camera_info', 'camera_front_link')):
            self.create_subscription(
                CameraInfo, info_topic,
                lambda msg, fr=frame: self._on_caminfo(msg, fr), 5)

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

    def _on_caminfo(self, msg: CameraInfo, frame):
        # K = [fx 0 cx; 0 fy cy; 0 0 1]. Simpan sekali (intrinsics statis di sim).
        if self.K.get(frame) is not None:
            return
        k = np.asarray(msg.k, dtype=float).reshape(3, 3)
        self.K[frame] = k
        self.get_logger().info(
            'camera_info %s: fx=%.1f fy=%.1f cx=%.1f cy=%.1f (%dx%d) '
            '[intrinsics SIM, bukan kalibrasi hardware]'
            % (frame, k[0, 0], k[1, 1], k[0, 2], k[1, 2], msg.width, msg.height))

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
        # Decode berjenjang (mentah -> CLAHE -> adaptive-threshold -> upscale).
        data, pts = robust_decode(img, self.det)

        # Offset piksel: publish begitu QR TERDETEKSI (pts ada), walau decode gagal,
        # supaya visual servo tetap bisa memusatkan QR di frame.
        if pts is not None and len(pts) > 0:
            self._publish_offset(pts, img.shape, self._frame_of(topic))

        if not data:
            return
        wall = parse_wall(data)
        out = String()
        out.data = wall if wall else data
        self.pub.publish(out)
        if data != self._last_data:
            self.get_logger().info(f'QR terbaca: "{data}" -> sisi {wall} [{topic}]')
            self._last_data = data

    def _publish_offset(self, pts, shape, frame_id):
        ex, ey, size = offset_from_points(pts, shape)
        ps = PointStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = frame_id
        ps.point.x = ex        # + = QR di kanan pusat
        ps.point.y = ey        # + = QR di bawah pusat
        ps.point.z = size      # ukuran-tampak (proxy jarak)
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
