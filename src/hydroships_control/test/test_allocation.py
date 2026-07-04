"""Uji Thrust Allocation Matrix (murni, tanpa ROS)."""

import numpy as np

from hydroships_control.allocation import (
    MAX_THRUST,
    MIN_THRUST,
    THRUSTERS,
    allocate,
    build_allocation_matrix,
)


def test_tam_full_rank():
    tam = build_allocation_matrix(THRUSTERS)
    assert tam.shape == (6, 6)
    assert np.linalg.matrix_rank(tam) == 6


def test_each_dof_reconstructs():
    tam = build_allocation_matrix(THRUSTERS)
    pinv = np.linalg.pinv(tam)
    identity = np.eye(6)
    for i in range(6):
        wrench = identity[:, i]
        forces = pinv @ wrench
        recon = tam @ forces
        assert np.allclose(recon, wrench, atol=1e-9), f'DOF {i} tidak terekonstruksi'


def test_horizontal_vertical_decoupling():
    """Surge/sway/yaw hanya pakai thruster horizontal (1-3);
    heave/roll/pitch hanya pakai vertikal (4-6)."""
    tam = build_allocation_matrix(THRUSTERS)
    pinv = np.linalg.pinv(tam)
    # surge, sway, yaw -> indeks wrench 0,1,5
    for i in (0, 1, 5):
        f = pinv @ np.eye(6)[:, i]
        assert np.allclose(f[3:6], 0.0, atol=1e-9), f'DOF {i} bocor ke thruster vertikal'
    # heave, roll, pitch -> indeks 2,3,4
    for i in (2, 3, 4):
        f = pinv @ np.eye(6)[:, i]
        assert np.allclose(f[0:3], 0.0, atol=1e-9), f'DOF {i} bocor ke thruster horizontal'


def test_allocate_clips_to_limits():
    tam = build_allocation_matrix(THRUSTERS)
    pinv = np.linalg.pinv(tam)
    # Wrench sangat besar -> pasti kena batas.
    forces = allocate([1e6, 0, 0, 0, 0, 0], pinv)
    assert np.all(forces <= MAX_THRUST + 1e-9)
    assert np.all(forces >= MIN_THRUST - 1e-9)
