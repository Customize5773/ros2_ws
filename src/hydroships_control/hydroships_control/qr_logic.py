"""qr_logic — logika murni deteksi/decode QR (tanpa rclpy), agar testable headless.

Dipisah dari `qr_detector.py` (node ROS) mengikuti pola `hook_logic.py`/
`gripper_logic.py`. Berisi:
  * `robust_decode(img, detector)` — coba decode QR lewat beberapa pra-pemrosesan
    ringan (grayscale + CLAHE + adaptive threshold + upscale) untuk mengatasi
    render kamera sim yang terdegradasi (lantai berfaset, bayangan, QR kecil di
    frame). Tanpa dependency baru — hanya cv2 + numpy yang sudah dipakai node.
  * `parse_wall(data)` — ekstrak huruf sisi A/B/C/D dari string QR.
  * `offset_from_points(pts, shape)` — hitung offset piksel ternormalisasi +
    ukuran-tampak dari 4 sudut QR (dipakai visual servo `/hydroships/qr_offset`).

Alasan pra-pemrosesan (lihat PROBLEM.md "QR belum terbaca meski posisi tepat"):
`cv2.QRCodeDetector.detectAndDecode` gagal ketika kontras rendah / quiet-zone
QR terganggu bayangan / QR kecil di frame. CLAHE menormalkan kontras lokal,
adaptive threshold memisahkan modul QR dari lantai berfaset, upscale membantu
saat modul QR mendekati batas resolusi decoder.
"""

import re

import numpy as np

try:
    import cv2
    CV2_OK = True
except ImportError:                       # pragma: no cover
    CV2_OK = False

# Huruf sisi A-D berdiri sendiri (mis. "A", "SIDE_B", "WALL-C") — sama dgn GUI/autonomy.
_WALL_RE = re.compile(r'(?:^|[^A-Z])([ABCD])(?![A-Z])')

# Ambang preprocessing (titik-awal; boleh di-tuning bila render sim berubah).
CLAHE_CLIP = 3.0
CLAHE_TILE = 8
ADAPT_BLOCK = 31           # ganjil; ukuran window adaptive-threshold
ADAPT_C = 5                # konstanta pengurang adaptive-threshold
UPSCALE = 2.0              # faktor perbesar saat QR kecil di frame


def parse_wall(data):
    """Ekstrak huruf sisi A/B/C/D dari isi QR, atau None bila tak ada."""
    if not data:
        return None
    m = _WALL_RE.search(data.upper())
    return m.group(1) if m else None


def _to_gray(img):
    if img is None:
        return None
    if img.ndim == 3:
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return img


def _candidates(img):
    """Hasilkan (varian_gambar, skala) untuk dicoba decode, dari yang paling
    murah/tak mengubah data (mentah) ke yang lebih agresif (CLAHE, threshold,
    upscale). `skala` = faktor perbesar relatif ke frame asli (utk kembalikan
    koordinat sudut). Generator agar decode berhenti di kandidat pertama sukses."""
    yield img, 1.0                                       # 1. mentah (paling ringan)
    if not CV2_OK:
        return
    gray = _to_gray(img)
    if gray is None:
        return
    # 2. grayscale + CLAHE (normalisasi kontras lokal; lawan bayangan/redup)
    clahe = cv2.createCLAHE(clipLimit=CLAHE_CLIP,
                            tileGridSize=(CLAHE_TILE, CLAHE_TILE))
    eq = clahe.apply(gray)
    yield eq, 1.0
    # 3. adaptive threshold di atas CLAHE (pisahkan modul QR dari lantai berfaset).
    # Coba TANPA denoise (jaga modul QR kecil) & DENGAN median-blur (lawan noise).
    th = cv2.adaptiveThreshold(eq, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                               cv2.THRESH_BINARY, ADAPT_BLOCK, ADAPT_C)
    yield th, 1.0
    den = cv2.medianBlur(eq, 3)
    th_d = cv2.adaptiveThreshold(den, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                 cv2.THRESH_BINARY, ADAPT_BLOCK, ADAPT_C)
    yield th_d, 1.0
    # 4. Otsu global threshold (bila iluminasi cukup seragam di area QR)
    _, oth = cv2.threshold(den, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    yield oth, 1.0
    # 5. upscale varian ter-threshold (bantu saat QR kecil di frame)
    if UPSCALE > 1.0:
        yield cv2.resize(th, None, fx=UPSCALE, fy=UPSCALE,
                         interpolation=cv2.INTER_NEAREST), UPSCALE
        yield cv2.resize(oth, None, fx=UPSCALE, fy=UPSCALE,
                         interpolation=cv2.INTER_NEAREST), UPSCALE


def robust_decode(img, detector):
    """Coba decode QR dari `img` lewat beberapa pra-pemrosesan.

    Mengembalikan (data, pts):
      data : str isi QR ('' bila gagal decode di semua kandidat)
      pts  : ndarray 4x2 sudut QR pada FRAME ASLI (utk offset), atau None.
    Titik dikembalikan begitu QR TERDETEKSI walau decode gagal, agar visual
    servo tetap bisa memusatkan. Koordinat dari kandidat ter-upscale dibagi
    balik ke skala frame asli.
    """
    if img is None or detector is None:
        return '', None
    best_pts = None
    for cand, s in _candidates(img):
        try:
            data, pts, _ = detector.detectAndDecode(cand)
        except Exception:
            continue
        has_pts = pts is not None and len(pts) > 0
        if has_pts and best_pts is None:
            best_pts = np.asarray(pts, dtype=float).reshape(-1, 2) / s
        if data:
            p = np.asarray(pts, dtype=float).reshape(-1, 2) / s if has_pts else best_pts
            return data, p
    return '', best_pts


def offset_from_points(pts, shape):
    """Hitung (ex, ey, size) ternormalisasi dari 4 sudut QR (piksel).
      ex   : offset horizontal [-1..1] (+ = QR di kanan pusat)
      ey   : offset vertikal   [-1..1] (+ = QR di bawah pusat)
      size : ukuran-tampak QR (fraksi sisi frame; besar = dekat = proxy jarak)
    """
    p = np.asarray(pts, dtype=float).reshape(-1, 2)
    h, w = float(shape[0]), float(shape[1])
    cx, cy = p[:, 0].mean(), p[:, 1].mean()
    bw = p[:, 0].max() - p[:, 0].min()
    bh = p[:, 1].max() - p[:, 1].min()
    ex = (cx - w / 2.0) / (w / 2.0)
    ey = (cy - h / 2.0) / (h / 2.0)
    size = max(bw / w, bh / h)
    return float(ex), float(ey), float(size)
