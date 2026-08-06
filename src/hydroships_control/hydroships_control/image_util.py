"""image_util — decode sensor_msgs/Image ke array BGR (tanpa rclpy/cv_bridge).

Dipisah dari `qr_detector.py` karena `hook_detector.py` butuh logika yang SAMA:
sebelumnya hook_detector reshape polos `buf.reshape(h, w, 3)` sehingga mengabaikan
`msg.step` — padahal ros_gz bridge MEMANG memberi padding baris, jadi frame
ter-shear diagonal dan deteksi gagal diam-diam. Satu implementasi dipakai berdua.

Fungsi murni (numpy saja) supaya testable headless.
"""

import numpy as np


def channels_for_encoding(enc):
    """Jumlah channel per piksel dari string encoding sensor_msgs/Image."""
    if enc in ('mono8', '8UC1'):
        return 1
    return 3                                 # rgb8/bgr8/fallback


def reshape_with_step(buf, h, w, ch, step):
    """Reshape buffer Image menghormati msg.step (row stride).

    sensor_msgs/Image.step = byte per baris. Banyak publisher (mis. ros_gz
    bridge) MENAMBAH padding di akhir tiap baris agar align memori, jadi
    step bisa > width*channels. Kalau kita reshape polos ke (h, w, ch)
    dgn asumsi step == width*channels, byte padding ikut terbaca dan seluruh
    gambar TERGESER diagonal per baris (decode QR/hook gagal diam-diam). Di sini:
    bila step cocok tanpa padding -> reshape langsung; bila step lebih besar
    -> reshape ke (h, step) lalu potong width*channels byte pertama tiap baris
    (buang padding) sebelum reshape ke (h, w, ch)."""
    row_bytes = w * ch
    step = int(step) if step else row_bytes
    if step <= row_bytes or (h * step) > buf.size:
        # Tak ada padding (atau step tak masuk akal) -> packing rapat.
        flat = buf[:h * row_bytes]
    else:
        flat = buf[:h * step].reshape(h, step)[:, :row_bytes].reshape(-1)
    img = flat.reshape(h, w, ch)
    return img[:, :, 0] if ch == 1 else img


def image_msg_to_bgr(msg):
    """sensor_msgs/Image -> ndarray BGR (atau grayscale utk mono8).

    Melempar exception bila buffer/encoding tak masuk akal; pemanggil (node)
    yang menangani logging.
    """
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    h, w, enc = msg.height, msg.width, msg.encoding
    ch = channels_for_encoding(enc)
    img = reshape_with_step(buf, h, w, ch, msg.step)
    if enc == 'rgb8':
        img = img[:, :, ::-1]                # RGB -> BGR
    return np.ascontiguousarray(img)
