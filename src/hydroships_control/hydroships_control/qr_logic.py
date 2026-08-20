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
import yaml

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
    """Hasilkan (varian_gambar, skala, nama) untuk dicoba decode, dari yang paling
    murah/tak mengubah data (mentah) ke yang lebih agresif (CLAHE, threshold,
    upscale). `skala` = faktor perbesar relatif ke frame asli (utk kembalikan
    koordinat sudut). `nama` dipakai HANYA utk instrumentasi/logging (P0-2.6
    kandidat #1 -- BELUM diimplementasi, ini cuma observability), tidak
    memengaruhi urutan/logika decode itu sendiri. Generator agar decode
    berhenti di kandidat pertama sukses."""
    yield img, 1.0, 'mentah'                             # 1. mentah (paling ringan)
    if not CV2_OK:
        return
    gray = _to_gray(img)
    if gray is None:
        return
    # 2. grayscale + CLAHE (normalisasi kontras lokal; lawan bayangan/redup)
    clahe = cv2.createCLAHE(clipLimit=CLAHE_CLIP,
                            tileGridSize=(CLAHE_TILE, CLAHE_TILE))
    eq = clahe.apply(gray)
    yield eq, 1.0, 'clahe'
    # 3. adaptive threshold di atas CLAHE (pisahkan modul QR dari lantai berfaset).
    # Coba TANPA denoise (jaga modul QR kecil) & DENGAN median-blur (lawan noise).
    th = cv2.adaptiveThreshold(eq, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                               cv2.THRESH_BINARY, ADAPT_BLOCK, ADAPT_C)
    yield th, 1.0, 'adaptive_thresh'
    den = cv2.medianBlur(eq, 3)
    th_d = cv2.adaptiveThreshold(den, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                 cv2.THRESH_BINARY, ADAPT_BLOCK, ADAPT_C)
    # 4. Otsu global threshold (bila iluminasi cukup seragam di area QR)
    _, oth = cv2.threshold(den, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    yield oth, 1.0, 'otsu'
    # 5. upscale varian ter-threshold (bantu saat QR kecil di frame)
    if UPSCALE > 1.0:
        yield (cv2.resize(th, None, fx=UPSCALE, fy=UPSCALE,
                          interpolation=cv2.INTER_NEAREST), UPSCALE,
               'adaptive_thresh_upscaled')
        yield (cv2.resize(oth, None, fx=UPSCALE, fy=UPSCALE,
                          interpolation=cv2.INTER_NEAREST), UPSCALE,
               'otsu_upscaled')
    # P0-2.8: demoted to last (was index 3) -- docs/P0-2-8-CANDIDATE-ORDERING-REVIEW.md.
    # 0/32 decode successes across P0-2.6/P0-2.7 batteries while pre-empting best_pts
    # on corner-only frames; demoted (not removed) since it never contributed a decode.
    yield th_d, 1.0, 'adaptive_thresh_denoised'


def robust_decode(img, detector, debug_info=None):
    """Coba decode QR dari `img` lewat beberapa pra-pemrosesan.

    Mengembalikan (data, pts):
      data : str isi QR ('' bila gagal decode di semua kandidat)
      pts  : ndarray 4x2 sudut QR pada FRAME ASLI (utk offset), atau None.
    Titik dikembalikan begitu QR TERDETEKSI walau decode gagal, agar visual
    servo tetap bisa memusatkan. Koordinat dari kandidat ter-upscale dibagi
    balik ke skala frame asli.

    `debug_info` (opsional, P0-2.6 instrumentasi -- observability only, TIDAK
    mengubah urutan/logika decode di atas): kalau diberi dict, diisi dengan
    `winning_candidate_index`/`winning_candidate_name` (kandidat yang
    berhasil decode, -1/None kalau tak ada satu pun), dan
    `first_pts_candidate_index`/`first_pts_candidate_name` (kandidat PERTAMA
    yang menghasilkan corner points, dipakai sbg `best_pts` fallback saat
    decode gagal semua -- ini mekanisme corner-only yang diaudit
    docs/P0-2-6-DIAGNOSTIC.md S2). Signature return TIDAK berubah supaya
    caller lama (qr_detector.py, test_qr_logic.py) tetap jalan tanpa ubahan.
    """
    if img is None or detector is None:
        return '', None
    best_pts = None
    first_pts_idx, first_pts_name = None, None
    idx = -1
    for idx, (cand, s, name) in enumerate(_candidates(img)):
        try:
            data, pts, _ = detector.detectAndDecode(cand)
        except Exception:
            continue
        has_pts = pts is not None and len(pts) > 0
        if has_pts and best_pts is None:
            cand_pts = np.asarray(pts, dtype=float).reshape(-1, 2) / s
            if _quiet_zone_ok(_to_gray(img), cand_pts):
                best_pts = cand_pts
                first_pts_idx, first_pts_name = idx, name
        if data:
            p = np.asarray(pts, dtype=float).reshape(-1, 2) / s if has_pts else best_pts
            if debug_info is not None:
                debug_info['winning_candidate_index'] = idx
                debug_info['winning_candidate_name'] = name
                debug_info['first_pts_candidate_index'] = first_pts_idx
                debug_info['first_pts_candidate_name'] = first_pts_name
                debug_info['n_candidates_tried'] = idx + 1
            return data, p
    if debug_info is not None:
        debug_info['winning_candidate_index'] = -1
        debug_info['winning_candidate_name'] = None
        debug_info['first_pts_candidate_index'] = first_pts_idx
        debug_info['first_pts_candidate_name'] = first_pts_name
        debug_info['n_candidates_tried'] = idx + 1
    return '', best_pts


def _quiet_zone_ok(gray, pts):
    """True bila ring sempit di luar quad QR dominan terang (quiet zone putih).

    Nyaring false positive korner-QR: siluet hook/tembok/bayangan memberi corner
    palsu tanpa quiet zone (terukur di run 2026-08-18: "QR besar" di hook 2 yg
    bukan payload). Ring dipilih [0.98, 1.15] * r0 agar TETAP berada di dalam
    bidang putih payload (plane 0.12 m > QR 0.06 m -> tepi ~1.41*r0); kalau
    ring menyentuh lantai gelap, QR asli ikut tertolak. Relatif thd kecerahan
    dalam quad (bukan ambang absolut) agar kebal variasi pencahayaan sim.
    """
    if gray is None or pts is None or len(pts) < 4:
        return False
    q = np.asarray(pts, dtype=float).reshape(-1, 2)
    cx, cy = q.mean(axis=0)
    r0 = max(1e-3, float(np.mean(np.linalg.norm(q - [cx, cy], axis=1))))
    h, w = gray.shape[:2]
    xs, ys = np.meshgrid(np.arange(w, dtype=float), np.arange(h, dtype=float))
    d = np.hypot(xs - cx, ys - cy)
    ring = gray[(d >= 0.98 * r0) & (d <= 1.15 * r0)]
    inner = gray[d < 0.55 * r0]
    if ring.size == 0 or inner.size == 0:
        return False
    return float(ring.mean()) > 1.25 * float(inner.mean())


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

def qr_ey_target(depth, cam_gripper_dx, qr_floor_z, cam_bottom_dz,
                 vfov_half_tan, ey_max):
    """Offset vertikal ternormalisasi tempat QR HARUS tampak di kamera bawah.

    Gripper berada `cam_gripper_dx` meter DI DEPAN kamera bawah (sumbu x body).
    Kalau servo memusatkan QR di kamera (ey=0), gripper meleset sejauh itu. Jadi
    QR harus dibiarkan tampak di DEPAN pusat frame supaya gripper-lah yang tepat
    di atas QR.

    Konvensi `qr_logic.offset_from_points`: ey > 0 = QR di BAWAH pusat frame =
    payload di BELAKANG ROV. Maka "QR di depan" berarti ey NEGATIF.

    Geometri: kamera berada `h_cam` di atas bidang QR, dan setengah-tinggi
    petak pandang di bidang itu = h_cam * tan(½ FOV vertikal). Offset metrik
    dinormalkan terhadap setengah-tinggi tsb.

    Dikembalikan sudah ter-clamp ke ±ey_max agar QR tak terdorong keluar frame
    (|ey| = 1.0 tepat di tepi). Clamp yang AKTIF adalah tanda bahwa `depth`
    terlalu dalam untuk offset gripper sebesar ini — lihat catatan `scan_depth`.
    """
    h_cam = max(0.05, abs(qr_floor_z) - depth - cam_bottom_dz)
    half_h = max(1e-3, h_cam * vfov_half_tan)
    ey = -cam_gripper_dx / half_h
    return max(-ey_max, min(ey_max, ey))


def load_calibration_yaml(path):
    """Baca file kalibrasi kamera fisik & kembalikan K sebagai ndarray 3x3.

    Dua format didukung (dipilih dari ekstensi file), keduanya OUTPUT tool
    kalibrasi yang sudah ada -- tidak menulis parser/solver kalibrasi sendiri:
      * `.yaml`/`.yml` -- format `camera_calibration` ROS (`ost.yaml`, key
        `camera_matrix: {rows, cols, data}`), dihasilkan
        `ros2 run camera_calibration cameracalibrator`.
      * `.npz` -- hasil `cv2.calibrateCamera` disimpan lewat `np.savez`
        (key `K`, plus `dist`/`image_size`/`rms` yang diabaikan di sini --
        lihat `dwe.npz` di root repo, kalibrasi nyata kamera DWE ExploreHD,
        RMS reprojection tercatat di file itu sendiri).

    Dipisah dari `qr_detector.py` supaya testable headless (M3: kalibrasi
    kamera fisik, lihat docs/HARDWARE.md).
    """
    if path.endswith('.npz'):
        d = np.load(path)
        return np.asarray(d['K'], dtype=float).reshape(3, 3)
    with open(path) as f:
        doc = yaml.safe_load(f)
    m = doc['camera_matrix']
    k = np.asarray(m['data'], dtype=float).reshape(m['rows'], m['cols'])
    return k
