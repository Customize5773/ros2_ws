"""hook_detector — deteksi hook (pipa-U) dari kamera depan -> offset (visual servo).

Node ROS 2 mengikuti pola qr_detector.py (baca image topic, publish offset), untuk
MENGGANTIKAN behavior timed di mission_fsm state APPROACH_HOOK dgn servo berbasis
penglihatan.

Pipeline deteksi DIPORT dari repo GUI tim: Customize5773/GUI-ROV
`autonomy/vision/hook_detect.py` (detect_hook: color->contour/CLAHE->Hough
berjenjang). Diadaptasi ke node ROS (decode Image manual via numpy, tanpa
cv_bridge — sama gaya qr_detector). Nilai ambang default = titik-awal uji-meja,
WAJIB di-tuning ulang di kolam (lihat catatan asli GUI-ROV & PROBLEM.md).

Kontrak topic:
    /hydroships/camera_front/image_raw  (sensor_msgs/Image)         -> masuk
    /hydroships/hook_offset             (geometry_msgs/PointStamped) -> keluar
        point.x = offset horizontal ternormalisasi [-1..1] (+ = hook di KANAN)
        point.y = offset vertikal   ternormalisasi [-1..1] (+ = hook di BAWAH)
        point.z = ukuran-tampak hook (fraksi sisi frame, proxy jarak; besar=dekat)
        header.frame_id = camera_front_link
"""

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped

from hydroships_control.hook_logic import normalize_hook_offset

try:
    import cv2
    CV2_OK = True
except ImportError:                       # pragma: no cover
    CV2_OK = False


# ---- Parameter deteksi (default uji-meja; tuning ulang di kolam) ----
HOOK_MIN_AREA = 150.0
HOOK_CLAHE_CLIP = 2.0
HOOK_CLAHE_TILE = 8
HOOK_CANNY_LO = 50
HOOK_CANNY_HI = 150
HOOK_ASPECT_MIN = 0.15
HOOK_ASPECT_MAX = 6.0


# ---- Deteksi (diport dari GUI-ROV hook_detect.py; butuh cv2) ----
def _to_gray_clahe(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
    clahe = cv2.createCLAHE(clipLimit=HOOK_CLAHE_CLIP,
                            tileGridSize=(HOOK_CLAHE_TILE, HOOK_CLAHE_TILE))
    return clahe.apply(gray)


def _score_contour(cnt, min_area):
    area = float(cv2.contourArea(cnt))
    if area < min_area:
        return None
    x, y, w, h = cv2.boundingRect(cnt)
    if w <= 0 or h <= 0:
        return None
    aspect = w / float(h)
    if not (HOOK_ASPECT_MIN <= aspect <= HOOK_ASPECT_MAX):
        return None
    solidity = area / float(w * h)
    conf = float(np.clip(0.6 * solidity + 0.4 * min(1.0, area / 4000.0), 0.0, 1.0))
    return area, (x + w / 2.0, y + h / 2.0), conf


def _best_contour(mask, min_area):
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for cnt in cnts:
        scored = _score_contour(cnt, min_area)
        if scored is None:
            continue
        if best is None or scored[2] > best[2]:
            best = scored
    return best


def detect_hook(frame, min_area=HOOK_MIN_AREA):
    """Deteksi hook -> (center, area) atau None. Jenjang: contour/CLAHE lalu Hough.
    (Jalur warna GUI-ROV dilewati: warna PVC hook tak pasti — lihat catatan asli.)"""
    if not CV2_OK or frame is None:
        return None
    gray = _to_gray_clahe(frame)
    edges = cv2.Canny(gray, HOOK_CANNY_LO, HOOK_CANNY_HI)
    edges = cv2.dilate(edges, np.ones((3, 3), np.uint8), iterations=2)
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    best = _best_contour(edges, min_area)
    if best is not None:
        area, center, _conf = best
        return center, area
    # Fallback Hough (lengkung-U / lubang)
    g = cv2.medianBlur(gray, 5)
    minr = max(3, int((min_area / 3.14) ** 0.5))
    circles = cv2.HoughCircles(g, cv2.HOUGH_GRADIENT, dp=1.2, minDist=40,
                               param1=100, param2=30, minRadius=minr, maxRadius=0)
    if circles is None:
        return None
    circles = np.round(circles[0, :]).astype(int)
    cx, cy, r = max(circles, key=lambda c: c[2])
    area = float(np.pi * r * r)
    if area < min_area:
        return None
    return (float(cx), float(cy)), area


class HookDetector(Node):
    def __init__(self):
        super().__init__('hook_detector')
        self.declare_parameter('image_topic', '/hydroships/camera_front/image_raw')
        self.declare_parameter('max_rate', 5.0)
        self.declare_parameter('min_area', HOOK_MIN_AREA)
        topic = self.get_parameter('image_topic').value
        self.pub = self.create_publisher(PointStamped, '/hydroships/hook_offset', 10)
        self.create_subscription(Image, topic, self._on_image, 5)
        self._last_t = 0.0
        if not CV2_OK:
            self.get_logger().warn('opencv tak tersedia — hook_detector nonaktif')
        self.get_logger().info('hook_detector siap (subscribe %s)' % topic)

    def _to_cv(self, msg: Image):
        try:
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            h, w, enc = msg.height, msg.width, msg.encoding
            if enc in ('rgb8', 'bgr8'):
                img = buf.reshape(h, w, 3)
                if enc == 'rgb8':
                    img = img[:, :, ::-1]
                return np.ascontiguousarray(img)
            if enc in ('mono8', '8UC1'):
                return buf.reshape(h, w)
            return buf.reshape(h, w, -1)[:, :, :3]
        except Exception as e:
            self.get_logger().warn('decode image gagal: %s' % e, throttle_duration_sec=5.0)
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
        det = detect_hook(img, min_area=float(self.get_parameter('min_area').value))
        if det is None:
            return
        center, area = det
        ex, ey, size = normalize_hook_offset(center, area, img.shape[1], img.shape[0])
        ps = PointStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = 'camera_front_link'
        ps.point.x = float(ex); ps.point.y = float(ey); ps.point.z = float(size)
        self.pub.publish(ps)


def main(args=None):
    rclpy.init(args=args)
    node = HookDetector()
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
