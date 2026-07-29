"""Uji qr_ey_target: koreksi offset kamera bawah -> gripper (APPROACH_QR).

Kamera bawah ada di x=+0.02 dan gripper_base di x=+0.18 (rov_kki2026_new_design.urdf.xacro),
jadi gripper 0.16 m DI DEPAN kamera. Memusatkan QR di kamera (ey=0) membuat
gripper selalu melewati payload sejauh itu. `qr_ey_target` menghitung di mana QR
HARUS tampak supaya gripper-lah yang tepat di atas QR.

Uji ini juga mengunci angka yang mendasari pilihan scan_depth=0.30 (lihat komentar
param di mission_fsm.py): pada 0.46 target jatuh di tepi frame, pada 0.30 aman.
"""

import math

import pytest

from hydroships_control.mission_fsm import qr_ey_target

# Konstanta geometri = default param mission_fsm.
DX = 0.16          # cam_gripper_dx
FLOOR_Z = -0.894   # qr_floor_z (payload_spawner.py)
DZ = 0.18          # cam_bottom_dz
TAN = 0.6293       # tan(½ FOV vertikal), hFOV 80° @ 4:3
EY_MAX = 0.8


def ey(depth, ey_max=EY_MAX):
    return qr_ey_target(depth, DX, FLOOR_Z, DZ, TAN, ey_max)


def test_vfov_konstanta_konsisten_dgn_fov_80_derajat():
    """TAN harus = tan(atan(0.75 * tan(40°))) utk sensor 640x480, hFOV 80°."""
    expected = 0.75 * math.tan(math.radians(40.0))
    assert expected == pytest.approx(TAN, abs=1e-3)


def test_scan_depth_030_beri_target_di_dalam_frame():
    """Depth operasional: h_cam=0.414, ½-tinggi=0.261 -> ey=-0.61 (aman)."""
    assert ey(0.30) == pytest.approx(-0.61, abs=0.02)


def test_scan_depth_046_lama_terclamp_di_tepi_frame():
    """Bukti kenapa scan_depth harus dinaikkan.

    Pada 0.46, h_cam=0.254 -> ½-tinggi=0.160 m = tepat cam_gripper_dx, jadi
    target GEOMETRIS = -1.00 (tepi frame) dan ter-clamp. Clamp aktif = sinyal
    bahwa kedalaman itu tak bisa dipakai untuk koreksi gripper.
    """
    assert ey(0.46, ey_max=1.5) == pytest.approx(-1.0, abs=0.02)
    assert ey(0.46) == pytest.approx(-EY_MAX)      # ter-clamp dgn ey_max default


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
