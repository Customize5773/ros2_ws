#!/usr/bin/env python3
"""Geometri thruster & Thrust Allocation Matrix HYDROships (modul murni).

Dipisah dari node ROS agar bisa diuji tanpa rclpy. Konstanta di sini HARUS
konsisten dengan urdf/hydroships.urdf.xacro dan docs/thruster_config.md.
"""

import numpy as np

# (posisi [m], arah dorong unit) tiap thruster di frame body. Urutan = thruster_1..6.
THRUSTERS = [
    # 3 horizontal (tangensial 120 deg, rh = 0.16 m) -> surge/sway/yaw
    (np.array([0.0,     0.16,  0.0]), np.array([-1.0,  0.0,   0.0])),
    (np.array([-0.1386, -0.08, 0.0]), np.array([0.5,  -0.866, 0.0])),
    (np.array([0.1386,  -0.08, 0.0]), np.array([0.5,   0.866, 0.0])),
    # 3 vertikal -> heave/roll/pitch
    (np.array([0.12,  0.0,  0.0]),    np.array([0.0,  0.0,  1.0])),
    (np.array([-0.10, 0.12, 0.0]),    np.array([0.0,  0.0,  1.0])),
    (np.array([-0.10, -0.12, 0.0]),   np.array([0.0,  0.0,  1.0])),
]

# Batas gaya per thruster (N) - konsisten dengan max/min_thrust_cmd di URDF.
MAX_THRUST = 50.0
MIN_THRUST = -40.0


def build_allocation_matrix(thrusters=THRUSTERS):
    """Kembalikan TAM 6xN: kolom i = [axis_i ; pos_i x axis_i]."""
    n = len(thrusters)
    tam = np.zeros((6, n))
    for i, (pos, axis) in enumerate(thrusters):
        axis = axis / np.linalg.norm(axis)
        tam[0:3, i] = axis
        tam[3:6, i] = np.cross(pos, axis)
    return tam


def allocate(wrench, tam_pinv, lo=MIN_THRUST, hi=MAX_THRUST):
    """Peta wrench body 6-DOF -> gaya per thruster (N), sudah di-clip."""
    forces = tam_pinv @ np.asarray(wrench, dtype=float)
    return np.clip(forces, lo, hi)
