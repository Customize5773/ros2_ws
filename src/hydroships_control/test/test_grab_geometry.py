"""Uji geometri fase turun-untuk-mencengkeram (M5-D) — murni aritmetika.

Blocker M5-D (docs/P1-FASE1-RESULTS.md §4): attach dari ketinggian jelajah
membuat DetachableJoint mengelas ROV ke payload yang masih tergeletak di lantai
(terukur celah 0.60 m, 3/3 run) -> ROV terjangkar, NAV_WALL ABORT. Perbaikannya
tersebar di TIGA file yang tak saling meng-import:

    hydroships.urdf.xacro       gripper_base_joint z  (seberapa rendah gripper)
    rov_params.yaml             buoyancy_collision/cob (di mana dasar hull)
    mission_fsm.py              grab_depth            (seberapa dalam turun)
    gripper_controller.py       max_alt_gap           (gerbang attach)

Test ini membaca nilai SESUNGGUHNYA dari keempatnya dan menguji ketiga syarat
yang harus berlaku bersamaan. Ubah salah satu tanpa yang lain -> test gagal.
Sengaja regex, bukan import: mission_fsm/gripper_controller butuh rclpy.
"""

import re
from pathlib import Path

import pytest
import yaml

SRC = Path(__file__).resolve().parents[2]
XACRO = SRC / 'hydroships_description/urdf/hydroships.urdf.xacro'
PARAMS = SRC / 'hydroships_description/config/rov_params.yaml'
FSM = SRC / 'hydroships_control/hydroships_control/mission_fsm.py'
GRIP = SRC / 'hydroships_control/hydroships_control/gripper_controller.py'
SPAWNER = SRC / 'hydroships_gazebo/scripts/payload_spawner.py'


def _text(path):
    if not path.exists():                      # mis. dijalankan dari install/
        pytest.skip('%s tak ada di layout ini' % path.name)
    return path.read_text()


def _param(path, name):
    """Nilai default parameter ROS `p('nama', <float>)` dari sumber node."""
    m = re.search(r"p\(\s*'%s'\s*,\s*(-?[\d.]+)" % re.escape(name), _text(path))
    assert m, 'parameter %r tak ditemukan di %s' % (name, path.name)
    return float(m.group(1))


def _geom():
    xacro = _text(XACRO)
    m = re.search(r'name="gripper_base_joint"[^>]*>\s*<origin xyz="[\d.]+ [\d.]+ '
                  r'(-?[\d.]+)"', xacro)
    assert m, 'origin gripper_base_joint tak terbaca'
    gripper_dz = float(m.group(1))                     # z joint (negatif = turun)
    box = re.search(r'name="gripper_base"[\s\S]*?<box size="[\d.]+ [\d.]+ ([\d.]+)"',
                    xacro)
    assert box, 'ukuran box gripper_base tak terbaca'
    gripper_half_h = float(box.group(1)) / 2.0
    rp = yaml.safe_load(_text(PARAMS))
    hull_half_h = float(rp['buoyancy_collision']['z']) / 2.0
    cob_z = float(rp['cob']['z'])
    qr_floor_z = _param(SPAWNER, 'payload_z')
    return dict(gripper_dz=gripper_dz, gripper_half_h=gripper_half_h,
                hull_half_h=hull_half_h, cob_z=cob_z, qr_floor_z=qr_floor_z,
                grab_depth=_param(FSM, 'grab_depth'))


def test_gripper_mencapai_payload_di_grab_depth():
    """Dasar gripper harus SANGAT dekat bidang QR — inilah inti M5-D. Kalau
    celah ini besar lagi, DetachableJoint kembali mengelas lintas ruang."""
    g = _geom()
    gripper_bottom = -g['grab_depth'] + g['gripper_dz'] - g['gripper_half_h']
    gap = gripper_bottom - g['qr_floor_z']
    assert 0.0 < gap < 0.08, 'celah gripper->QR = %.3f m (harus 0..0.08)' % gap


def test_hull_tidak_menabrak_lantai_di_grab_depth():
    """Turun secukupnya, jangan sampai setpoint kedalaman menekan hull ke lantai
    (thruster akan saturasi melawan kontak & XY hold ikut rusak)."""
    g = _geom()
    hull_bottom = -g['grab_depth'] + g['cob_z'] - g['hull_half_h']
    clearance = hull_bottom - g['qr_floor_z']
    assert clearance > 0.05, 'jarak hull->lantai = %.3f m (terlalu mepet)' % clearance


def test_gerbang_attach_konsisten_dengan_grab_depth():
    """max_alt_gap diukur dari DASAR GRIPPER; harus cukup longgar untuk celah
    rancangan + toleransi kedalaman, tapi tetap jauh di bawah 0.60 m yang
    menjangkarkan ROV di run E1/E7/E13."""
    g = _geom()
    gripper_bottom = -g['grab_depth'] + g['gripper_dz'] - g['gripper_half_h']
    gap = gripper_bottom - g['qr_floor_z']
    max_alt_gap = _param(GRIP, 'max_alt_gap')
    depth_tol = _param(FSM, 'depth_tol')
    assert gap <= max_alt_gap, (
        'gerbang attach (%.3f) lebih ketat dari celah rancangan (%.3f) '
        '-> attach tak akan pernah terpicu' % (max_alt_gap, gap))
    assert max_alt_gap < gap + depth_tol + 0.05
    # gripper_bottom_dz di gripper_controller harus cocok dgn URDF, kalau tidak
    # gerbang menilai ketinggian yang salah.
    assert _param(GRIP, 'gripper_bottom_dz') == pytest.approx(
        -g['gripper_dz'] + g['gripper_half_h'], abs=1e-6)
