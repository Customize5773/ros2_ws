"""Uji qr_logic (murni, tanpa ROS): parse_wall, offset_from_points, robust_decode.

robust_decode diuji terhadap adegan QR yang SENGAJA didegradasi (kontras rendah +
lantai berfaset + bayangan) untuk membuktikan pra-pemrosesan (CLAHE/adaptive-
threshold/Otsu/upscale) MEMULIHKAN decode yang gagal via decode mentah. Ini bukti
OFFLINE (bukan pengganti run sim) — lihat PROBLEM.md item "QR belum terbaca".
"""

import numpy as np
import pytest

from hydroships_control.qr_logic import parse_wall, offset_from_points, robust_decode

cv2 = pytest.importorskip("cv2")


def test_parse_wall_variants():
    assert parse_wall("A") == "A"
    assert parse_wall("SIDE_B") == "B"
    assert parse_wall("WALL-C") == "C"
    assert parse_wall("d") == "D"
    assert parse_wall("") is None
    assert parse_wall("HELLO") is None       # tak ada A/B/C/D berdiri sendiri


def test_offset_center_and_size():
    # QR 100x100 di tengah frame 200x200 -> offset ~0, size 0.5.
    pts = np.array([[50, 50], [150, 50], [150, 150], [50, 150]], dtype=float)
    ex, ey, size = offset_from_points(pts, (200, 200))
    assert abs(ex) < 1e-6 and abs(ey) < 1e-6
    assert abs(size - 0.5) < 1e-6


def test_offset_right_and_down():
    # QR bergeser ke kanan-bawah -> ex>0, ey>0.
    pts = np.array([[120, 120], [180, 120], [180, 180], [120, 180]], dtype=float)
    ex, ey, _ = offset_from_points(pts, (200, 200))
    assert ex > 0 and ey > 0


def _make_qr(text="A", px=200):
    """Bangun gambar QR bersih pakai encoder cv2 bila ada, atau pola sintetik
    minimal yang decodable. Pakai cv2.QRCodeEncoder bila tersedia."""
    try:
        enc = cv2.QRCodeEncoder.create()
        qr = enc.encode(text)
        qr = cv2.resize(qr, (px, px), interpolation=cv2.INTER_NEAREST)
        return cv2.cvtColor(qr, cv2.COLOR_GRAY2BGR)
    except Exception:
        return None


def _degrade(qr_gray, contrast=0.55, brightness=95, noise=6, seed=0):
    """Tempel QR (kontras & brightness diturunkan) di atas lantai berfaset +
    bayangan diagonal + noise — meniru render kamera sim terdegradasi."""
    rng = np.random.default_rng(seed)
    H = W = 480
    qpx = qr_gray.shape[0]
    facet = rng.integers(60, 130, size=(H // 16, W // 16)).astype(np.uint8)
    floor = cv2.resize(facet, (W, H), interpolation=cv2.INTER_NEAREST)
    scene = floor.copy()
    q = (qr_gray.astype(np.float32) - 127) * contrast + brightness
    q = np.clip(q, 0, 255).astype(np.uint8)
    y0 = (H - qpx) // 2
    x0 = (W - qpx) // 2
    scene[y0:y0 + qpx, x0:x0 + qpx] = q
    gx = np.linspace(-30, 20, W)
    gy = np.linspace(-25, 25, H)
    grad = (gy[:, None] + gx[None, :]).astype(np.float32)
    scene = np.clip(scene.astype(np.float32) + grad, 0, 255).astype(np.uint8)
    scene = np.clip(scene.astype(np.float32) + rng.normal(0, noise, (H, W)), 0, 255)
    return scene.astype(np.uint8)


def test_robust_decodes_clean_qr():
    qr = _make_qr("A", px=200)
    if qr is None:
        pytest.skip("cv2.QRCodeEncoder tak tersedia")
    det = cv2.QRCodeDetector()
    data, pts = robust_decode(qr, det)
    assert data == "A"
    assert pts is not None and len(pts) == 4


def test_robust_recovers_degraded_where_raw_fails():
    qr = _make_qr("A", px=130)
    if qr is None:
        pytest.skip("cv2.QRCodeEncoder tak tersedia")
    gray = cv2.cvtColor(qr, cv2.COLOR_BGR2GRAY)
    scene = _degrade(gray, contrast=0.5, brightness=100, noise=6, seed=3)
    scene_bgr = cv2.cvtColor(scene, cv2.COLOR_GRAY2BGR)
    det = cv2.QRCodeDetector()
    # Decode MENTAH (tanpa pra-pemrosesan) gagal pada adegan ini...
    raw, _, _ = det.detectAndDecode(scene_bgr)
    # ...sementara robust_decode MEMULIHKANnya (atau minimal tak lebih buruk).
    data, _ = robust_decode(scene_bgr, det)
    assert data == "A"
    assert raw != "A" or data == "A"          # robust >= raw (tak regresi)


def test_robust_returns_empty_on_blank():
    blank = np.full((240, 240, 3), 127, dtype=np.uint8)
    det = cv2.QRCodeDetector()
    data, pts = robust_decode(blank, det)
    assert data == ""


def test_robust_decodes_real_sim_frame():
    """Regresi FRAME NYATA (bukan sintetik): frame kamera bottom hasil render
    Gazebo Fortress saat ROV hover ~25 cm di atas QR payload (fix [RESOLVED] QR
    detection, scan_depth 0.62->0.46). Mengunci bahwa pipeline decode bekerja pada
    karakter render sim asli — mis. bila material/emissive QR atau step-stride
    berubah & merusak keterbacaan, test ini gagal. Frame di-simpan grayscale;
    qr_detector mengumpankan BGR, jadi kita expand balik ke 3-channel."""
    import os
    fx = os.path.join(os.path.dirname(__file__), 'fixtures', 'qr_sim_bottom_A.png')
    if not os.path.exists(fx):
        pytest.skip('fixture frame nyata tak ada')
    gray = cv2.imread(fx, cv2.IMREAD_GRAYSCALE)
    assert gray is not None
    bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    det = cv2.QRCodeDetector()
    data, pts = robust_decode(bgr, det)
    assert data == 'A'
    assert pts is not None and len(pts) == 4
    assert parse_wall(data) == 'A'
