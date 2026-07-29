"""Jaga agar geometri thruster di URDF dan allocation.py tetap identik.

Geometri thruster diduplikasi di tiga tempat (lihat docs/thruster_config.md):
  * hydroships_description/urdf/rov_kki2026_new_design.urdf.xacro (model mesh CAD)
  * hydroships_control/allocation.py                        (tabel THRUSTERS)

Duplikasi itu pernah menyimpang diam-diam dan bikin matriks alokasi salah.
Test ini mem-parse kedua URDF lalu membandingkannya dengan THRUSTERS, jadi
penyimpangan ketahuan di CI, bukan di kolam.

Di-skip (bukan gagal) kalau xacro atau paket description tidak tersedia —
supaya test suite murni-Python tetap bisa jalan di mana saja.
"""

import os
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from hydroships_control.allocation import THRUSTERS

_HERE = os.path.dirname(os.path.abspath(__file__))
_URDF_DIR = os.path.normpath(
    os.path.join(_HERE, '..', '..', 'hydroships_description', 'urdf'))

MODELS = ['rov_kki2026_new_design.urdf.xacro']

# Toleransi: URDF menulis posisi 4 desimal (0,1 mm), jadi 1e-4 sudah ketat.
TOL = 1e-4


def _expand(model):
    path = os.path.join(_URDF_DIR, model)
    if not os.path.exists(path):
        pytest.skip('URDF tidak ditemukan: %s' % path)
    try:
        out = subprocess.run(['xacro', path], capture_output=True, check=True)
    except (FileNotFoundError, subprocess.CalledProcessError) as exc:
        pytest.skip('xacro tidak bisa dijalankan: %s' % exc)
    return ET.fromstring(out.stdout)


def _thruster_joints(root):
    """{nama: (posisi, sumbu)} untuk joint thruster_N_joint di URDF."""
    found = {}
    for joint in root.findall('joint'):
        name = joint.get('name', '')
        if not (name.startswith('thruster_') and name.endswith('_joint')):
            continue
        origin = joint.find('origin')
        axis = joint.find('axis')
        pos = np.array([float(v) for v in origin.get('xyz').split()])
        vec = np.array([float(v) for v in axis.get('xyz').split()])
        found[name[:-len('_joint')]] = (pos, vec)
    return found


@pytest.mark.parametrize('model', MODELS)
def test_urdf_matches_allocation_table(model):
    joints = _thruster_joints(_expand(model))
    assert len(joints) == len(THRUSTERS), (
        '%s punya %d joint thruster, THRUSTERS punya %d'
        % (model, len(joints), len(THRUSTERS)))

    for i, (pos, axis) in enumerate(THRUSTERS, start=1):
        name = 'thruster_%d' % i
        assert name in joints, '%s tidak ada di %s' % (name, model)
        upos, uaxis = joints[name]
        assert np.allclose(upos, pos, atol=TOL), (
            '%s: posisi %s di %s vs %s di allocation.py'
            % (name, upos, model, pos))
        # bandingkan sebagai vektor satuan; tanda IKUT dibandingkan karena
        # tanda menentukan arah dorong dan karenanya isi TAM.
        assert np.allclose(uaxis / np.linalg.norm(uaxis),
                           axis / np.linalg.norm(axis), atol=TOL), (
            '%s: sumbu %s di %s vs %s di allocation.py'
            % (name, uaxis, model, axis))


def test_kedua_model_identik():
    """Kedua URDF harus sepakat satu sama lain, bukan cuma dengan tabel."""
    a = _thruster_joints(_expand(MODELS[0]))
    b = _thruster_joints(_expand(MODELS[1]))
    assert set(a) == set(b)
    for name in a:
        assert np.allclose(a[name][0], b[name][0], atol=TOL), name
        assert np.allclose(a[name][1], b[name][1], atol=TOL), name


def test_massa_total_kedua_model_sama():
    """vehicle_mass di rov_params.yaml harus menghasilkan massa total yang
    sama di kedua model, meski link tambahannya berbeda."""
    totals = []
    for model in MODELS:
        root = _expand(model)
        totals.append(sum(float(m.get('value'))
                          for m in root.iter('mass')))
    assert abs(totals[0] - totals[1]) < 1e-6, (
        'massa total beda: %s' % dict(zip(MODELS, totals)))
