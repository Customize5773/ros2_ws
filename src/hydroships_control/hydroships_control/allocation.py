#!/usr/bin/env python3
"""Geometri thruster & Thrust Allocation Matrix HYDROships (modul murni).

Dipisah dari node ROS agar bisa diuji tanpa rclpy. Konstanta di sini HARUS
konsisten dengan urdf/hydroships.urdf.xacro dan docs/thruster_config.md.
"""

import numpy as np

# (posisi [m], arah dorong unit) tiap thruster di frame body. Urutan = thruster_1..6.
THRUSTERS = [
    # thruster_1 = T200-E (vertikal)
    (np.array([0.1234, 0.0275, 0.0142]),    np.array([0.0, 0.0, 1.0])),
    # thruster_2 = T200-F (vertikal)
    (np.array([-0.1228, 0.0290, 0.0148]),   np.array([0.0, 0.0, 1.0])),
    # thruster_3 = T100-C (depan horizontal)
    (np.array([0.1371, -0.1298, 0.0336]),   np.array([1.0, 0.0, 0.0])),
    # thruster_4 = T100-A (depan horizontal)
    (np.array([-0.1371, -0.1296, 0.0374]), np.array([1.0, 0.0, 0.0])),
    # thruster_5 = T100-B (lateral bawah horizontal)
    (np.array([0.0003, 0.0455, -0.0994]),   np.array([0.0, 1.0, 0.0])),
    # thruster_6 = T100-D (belakang sentral vertikal)
    (np.array([-0.0003, 0.1364, 0.0403]),   np.array([0.0, 0.0, 1.0])),
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
