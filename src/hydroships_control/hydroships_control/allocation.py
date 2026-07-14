#!/usr/bin/env python3
"""Geometri thruster & Thrust Allocation Matrix HYDROships (modul murni).

Dipisah dari node ROS agar bisa diuji tanpa rclpy. Konstanta di sini HARUS
konsisten dengan urdf/hydroships.urdf.xacro dan docs/thruster_config.md.
"""

import numpy as np

# (posisi [m], arah dorong unit) tiap thruster di FRAME BODY ROS (x=maju, y=kiri,
# z=atas). Urutan = thruster_1..6. HARUS konsisten dgn urdf/hydroships.urdf.xacro.
#
# Posisi diturunkan dari thruster_positions.csv, yang berkonvensi berbeda:
#   CSV: X=lateral (kanan +), Y=fore/aft (DEPAN negatif), Z=atas.
# Konversi ke body ROS:  x_body = -Y_csv,  y_body = -X_csv,  z_body = Z_csv  (mm→m).
# (Bug lama: kolom CSV disalin MENTAH ke (x,y,z) tanpa rotasi frame → posisi
#  terputar 90°, momen yaw dari T100-A/C saling meniadakan → yaw near-singular
#  cond~1.2e4. Setelah dikoreksi: cond~20, yaw pulih. Lihat PROBLEM.md.)
#
# Peran (axis) — T100-B kini dilabeli T200-B tapi TETAP thruster sway horizontal:
THRUSTERS = [
    # thruster_1 = T200-E (vertikal, kanan)      CSV(123.4, 27.5, 14.2)
    (np.array([-0.0275, -0.1234, 0.0142]),  np.array([0.0, 0.0, 1.0])),
    # thruster_2 = T200-F (vertikal, kiri)       CSV(-122.8, 29.0, 14.8)
    (np.array([-0.0290, 0.1228, 0.0148]),   np.array([0.0, 0.0, 1.0])),
    # thruster_3 = T100-C (surge, depan-kanan)   CSV(137.1, -129.8, 33.6)
    (np.array([0.1298, -0.1371, 0.0336]),   np.array([1.0, 0.0, 0.0])),
    # thruster_4 = T100-A (surge, depan-kiri)    CSV(-137.1, -129.6, 37.4)
    (np.array([0.1296, 0.1371, 0.0374]),    np.array([1.0, 0.0, 0.0])),
    # thruster_5 = T200-B (sway, tengah-bawah)   CSV(0.3, 45.5, -99.4)
    (np.array([-0.0455, -0.0003, -0.0994]), np.array([0.0, 1.0, 0.0])),
    # thruster_6 = T100-D (vertikal, belakang)   CSV(-0.3, 136.4, 40.3)
    (np.array([-0.1364, 0.0003, 0.0403]),   np.array([0.0, 0.0, 1.0])),
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


def build_damped_pinv(tam, damping=0.1):
    """Pseudo-inverse teredam (damped least-squares / Tikhonov).

        pinv_damped = TAM^T (TAM TAM^T + damping^2 I)^-1

    Geometri thruster HYDROships saat ini near-singular pada sumbu YAW
    (cond(TAM) ~ 1.2e4, singular value terkecil ~1e-4; lihat PROBLEM.md).
    Dengan pseudo-inverse polos (`np.linalg.pinv`), perintah pada arah lemah
    itu menuntut gaya thruster raksasa (ribuan N) yang menjenuhkan batas lalu
    MERUSAK DOF lain setelah di-clip. Redaman membatasi penguatan gaya pada
    arah kurang-terkendali: perintah yang tak tercapai "menyerah anggun"
    (mendekati nol) alih-alih meledak, sementara arah yang sehat (heave, sway,
    surge) tetap terlayani hampir penuh. damping -> 0 kembali ke pinv biasa.
    """
    tam = np.asarray(tam, dtype=float)
    m = tam.shape[0]
    if damping <= 0.0:
        return np.linalg.pinv(tam)
    return tam.T @ np.linalg.inv(tam @ tam.T + (damping ** 2) * np.eye(m))


def allocate(wrench, tam_pinv, lo=MIN_THRUST, hi=MAX_THRUST):
    """Peta wrench body 6-DOF -> gaya per thruster (N), sudah di-clip."""
    forces = tam_pinv @ np.asarray(wrench, dtype=float)
    return np.clip(forces, lo, hi)
