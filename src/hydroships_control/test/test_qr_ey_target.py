"""Uji qr_ey_target: koreksi offset kamera bawah -> gripper (APPROACH_QR).

Kamera bawah ada di x=+0.02 dan gripper_base di x=+0.18 (hydroships.urdf.xacro),
jadi gripper 0.16 m DI DEPAN kamera. Memusatkan QR di kamera (ey=0) membuat
gripper selalu melewati payload sejauh itu. `qr_ey_target` menghitung di mana QR
HARUS tampak supaya gripper-lah yang tepat di atas QR.

Default arena 5x5 m, lantai -0.90 m, kamera pitch +90 derajat.
"""

import math

import pytest

from hydroships_control.mission_fsm import qr_ey_target

DX = 0.16          # cam_gripper_dx
FLOOR_Z = -0.894   # qr_floor_z (arena default 5x5 m)
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


def test_scan_depth_030_target_muat_di_frame():
    assert ey(0.30) == pytest.approx(-DX / ((abs(FLOOR_Z) - 0.30 - DZ) * TAN))
    assert -EY_MAX < ey(0.30) < 0


def test_scan_depth_046_terclamp():
    assert ey(0.46) == pytest.approx(-EY_MAX)


def test_target_matches_camera_joint_projection():
    # Gazebo camera +X looks forward, +Z is image-up. Pitch +90 maps
    # body +X (gripper ahead) to camera +Z, hence negative image y.
    import re
    from pathlib import Path
    xacro = (Path(__file__).resolve().parents[2] /
             'hydroships_description/urdf/hydroships.urdf.xacro').read_text()
    pitch = float(re.search(r'name="camera_bottom_joint".*?rpy="0 ([\d.]+) 0"',
                            xacro, re.S).group(1))
    height = abs(FLOOR_Z) - 0.30 - DZ
    camera_x = math.cos(pitch) * DX + math.sin(pitch) * height
    camera_z = math.sin(pitch) * DX - math.cos(pitch) * height
    # R_y(pitch)^T maps the body vector (DX, 0, -height) into camera axes.
    assert ey(0.30) == pytest.approx(-camera_z / (camera_x * TAN), abs=1e-4)


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
