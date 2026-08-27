"""Uji qr_ey_target: koreksi offset kamera bawah -> gripper (APPROACH_QR).

Kamera bawah ada di x=+0.02 dan gripper_base di x=+0.18 (hydroships.urdf.xacro),
jadi gripper 0.16 m DI DEPAN kamera. Memusatkan QR di kamera (ey=0) membuat
gripper selalu melewati payload sejauh itu. `qr_ey_target` menghitung di mana QR
HARUS tampak supaya gripper-lah yang tepat di atas QR.

Uji ini juga mengunci angka yang mendasari pilihan scan_depth=0.30 (lihat komentar
param di mission_fsm.py). CATATAN 27 Agu: setelah kamera direkalibrasi ke hFOV
70° (26 Agu, sebelumnya 80°) dan lantai kolam naik ke -0.80 (kolam latihan,
sebelumnya -0.90/-0.894), geometri gripper_base_dx (0.16 m) ternyata SUDAH
memenuhi clamp ey_max di scan_depth=0.30 juga (dulu cuma 0.46 yang ter-clamp) --
bukan regresi baru, cuma konstanta lama (0.6293/-0.894) menyembunyikan
kenyataan geometrisnya sampai sekarang.
"""

import math

import pytest

from hydroships_control.mission_fsm import qr_ey_target

# Konstanta geometri = default param mission_fsm (kolam latihan default, 27 Agu:
# floor_z & TAN diperbarui mengikuti lantai kolam baru -0.80 dan kamera
# direkalibrasi ke hFOV 70 deg -- nilai lama -0.894/0.6293 basi sejak 26 Agu).
DX = 0.16          # cam_gripper_dx
FLOOR_Z = -0.794   # qr_floor_z (payload_spawner.py, kolam latihan)
DZ = 0.18          # cam_bottom_dz
TAN = 0.5252       # tan(½ FOV vertikal), hFOV 70° @ 4:3
EY_MAX = 0.8


def ey(depth, ey_max=EY_MAX):
    return qr_ey_target(depth, DX, FLOOR_Z, DZ, TAN, ey_max)


def test_vfov_konstanta_konsisten_dgn_fov_70_derajat():
    """TAN harus = tan(atan(0.75 * tan(35°))) utk sensor 640x480, hFOV 70°
    (kamera direkalibrasi 26 Agu; dulu 80°/tan(40°), lihat CHANGELOG)."""
    expected = 0.75 * math.tan(math.radians(35.0))
    assert expected == pytest.approx(TAN, abs=1e-3)


def test_scan_depth_030_terclamp_ke_ey_max():
    """Depth operasional: h_cam=0.314, ½-tinggi=0.165 m < cam_gripper_dx
    (0.16 m nyaris sama) -> target GEOMETRIS jauh di luar frame, ter-clamp
    ke ey_max. Beda dari asumsi lama (-0.61, "aman") -- itu produk konstanta
    basi (vfov 80°, floor -0.894), bukan geometri sesungguhnya."""
    assert ey(0.30) == pytest.approx(-EY_MAX)


def test_scan_depth_046_juga_terclamp_lebih_ekstrem():
    """Pada 0.46, h_cam=0.154 -> target geometris jauh lebih ekstrem (~-1.98
    tanpa clamp) drpd di 0.30 (~-0.97) -- makin dalam makin ekstrem, tapi
    KEDUANYA sekarang ter-clamp ke ey_max yang sama (lihat catatan modul)."""
    assert ey(0.46) == pytest.approx(-EY_MAX)


def test_selalu_negatif_karena_qr_harus_tampak_di_depan():
    """Konvensi offset_from_points: ey<0 = QR di ATAS pusat = payload di DEPAN."""
    for depth in (0.10, 0.20, 0.30, 0.40, 0.46):
        assert ey(depth) < 0.0


def test_makin_dangkal_makin_kecil_magnitudonya():
    """Naik = petak pandang melebar = offset 0.16 m jadi fraksi frame lebih kecil."""
    magnitudes = [abs(ey(d)) for d in (0.40, 0.30, 0.20, 0.10)]
    assert magnitudes == sorted(magnitudes, reverse=True)


def test_terclamp_ke_ey_max():
    """Sedalam apa pun, hasil tak pernah keluar frame."""
    assert ey(0.70) == pytest.approx(-EY_MAX)
    assert abs(ey(0.70)) <= EY_MAX


def test_tanpa_offset_gripper_target_nol():
    """cam_gripper_dx=0 (kamera sejajar gripper) -> perilaku lama: pusatkan di 0."""
    assert qr_ey_target(0.30, 0.0, FLOOR_Z, DZ, TAN, EY_MAX) == pytest.approx(0.0)


def test_depth_ekstrem_tak_bikin_pembagian_nol():
    """h_cam di-floor ke 0.05 supaya depth absurd tak memicu ZeroDivision/inf."""
    val = ey(10.0)
    assert math.isfinite(val)
    assert val == pytest.approx(-EY_MAX)
