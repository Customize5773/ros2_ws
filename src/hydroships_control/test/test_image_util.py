"""Uji image_util: reshape Image yang menghormati msg.step (row stride).

ros_gz bridge MENAMBAH padding di akhir tiap baris (step > width*channels). Kalau
diabaikan, gambar tergeser diagonal per baris dan deteksi QR/hook gagal DIAM-DIAM
— itulah bug yang ada di hook_detector._to_cv sebelum modul ini dipakai bersama.
"""

import numpy as np

from hydroships_control.image_util import channels_for_encoding, reshape_with_step


def _padded_buffer(img, pad_bytes):
    """Susun buffer baris-per-baris dgn `pad_bytes` sampah di akhir tiap baris."""
    h, w, ch = img.shape
    row_bytes = w * ch
    step = row_bytes + pad_bytes
    buf = np.zeros(h * step, dtype=np.uint8)
    for r in range(h):
        buf[r * step:r * step + row_bytes] = img[r].reshape(-1)
        buf[r * step + row_bytes:(r + 1) * step] = 0xEF     # padding "sampah"
    return buf, step


def test_channels_per_encoding():
    assert channels_for_encoding('mono8') == 1
    assert channels_for_encoding('8UC1') == 1
    assert channels_for_encoding('rgb8') == 3
    assert channels_for_encoding('bgr8') == 3


def test_tanpa_padding_reshape_apa_adanya():
    img = np.arange(4 * 5 * 3, dtype=np.uint8).reshape(4, 5, 3)
    out = reshape_with_step(img.reshape(-1), 4, 5, 3, 5 * 3)
    assert np.array_equal(out, img)


def test_padding_dibuang_gambar_tidak_ter_shear():
    """Inti bug: dgn padding, reshape polos menghasilkan gambar yang salah."""
    img = np.arange(6 * 7 * 3, dtype=np.uint8).reshape(6, 7, 3)
    buf, step = _padded_buffer(img, pad_bytes=5)

    out = reshape_with_step(buf, 6, 7, 3, step)
    assert np.array_equal(out, img)
    assert not np.any(out == 0xEF)          # tak ada byte padding yang lolos

    # Kontrol: cara lama (abaikan step) memang menghasilkan gambar berbeda.
    naive = buf[:6 * 7 * 3].reshape(6, 7, 3)
    assert not np.array_equal(naive, img)


def test_mono8_dikembalikan_2d():
    img = np.arange(3 * 4, dtype=np.uint8).reshape(3, 4)
    buf, step = _padded_buffer(img.reshape(3, 4, 1), pad_bytes=2)
    out = reshape_with_step(buf, 3, 4, 1, step)
    assert out.shape == (3, 4)
    assert np.array_equal(out, img)


def test_step_nol_atau_tak_masuk_akal_jatuh_ke_packing_rapat():
    img = np.arange(2 * 3 * 3, dtype=np.uint8).reshape(2, 3, 3)
    flat = img.reshape(-1)
    assert np.array_equal(reshape_with_step(flat, 2, 3, 3, 0), img)
    # step raksasa -> buffer tak cukup; jangan crash, anggap packing rapat.
    assert np.array_equal(reshape_with_step(flat, 2, 3, 3, 9999), img)
