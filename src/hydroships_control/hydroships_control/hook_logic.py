"""hook_logic — helper murni deteksi/servo hook (tanpa ROS/cv2), agar testable.

Dipisah dari hook_detector.py (node, butuh rclpy) & dari detect_hook (butuh cv2)
supaya normalisasi offset & keputusan servo bisa diuji headless."""


def normalize_hook_offset(center, area, frame_w, frame_h):
    """(center px, area px^2, ukuran frame) -> (ex, ey, size) ternormalisasi.

    Konvensi sama dgn qr_offset:
      ex = (cx - W/2)/(W/2)  (+ = hook di KANAN pusat frame)
      ey = (cy - H/2)/(H/2)  (+ = hook di BAWAH pusat frame)
      size = sqrt(area)/W    (proxy jarak; besar = dekat)"""
    w = float(frame_w); h = float(frame_h)
    cx, cy = float(center[0]), float(center[1])
    ex = (cx - w / 2.0) / (w / 2.0) if w > 0 else 0.0
    ey = (cy - h / 2.0) / (h / 2.0) if h > 0 else 0.0
    size = (float(area) ** 0.5) / w if w > 0 else 0.0
    return ex, ey, size
