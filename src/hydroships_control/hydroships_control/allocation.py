#!/usr/bin/env python3
"""Geometri thruster & Thrust Allocation Matrix HYDROships (modul murni).

Dipisah dari node ROS agar bisa diuji tanpa rclpy. Konstanta di sini HARUS
konsisten dengan urdf/rov_kki2026_new_design.urdf.xacro dan docs/thruster_config.md.
"""

import numpy as np

# (posisi [m], arah dorong unit) tiap thruster di FRAME BODY ROS (x=maju, y=kiri,
# z=atas). Urutan = thruster_1..6. HARUS konsisten dgn urdf/rov_kki2026_new_design.urdf.xacro
# dan urdf/rov_kki2026_new_design.urdf.xacro.
#
# SUMBER (2026-07-28): posisi & sumbu DIUKUR dari CAD
# "DOKUMENTASI ROV/@ROV KKI 2026 NEW DESIGN.stl" memakai
# hydroships_description/scripts/measure_cad_frames.py — bukan lagi dari
# thruster_positions.csv. Tiap unit dikenali lewat connected-component analysis:
# rumah thruster = bongkah kompak 45-110 mm, arah dorong = normal cakram
# propeller/duct terdekat. Setelah transform, keenam titik berjarak 3,4-8,5 mm
# dari permukaan mesh (dgn angka CSV lama: 8-108 mm, dua di antaranya
# menggantung di ruang kosong).
#
# PENOMORAN mengikuti Gambar 2.9/2.10 dokumen desain (penulis, 2026):
#   1,2 = pasangan tengah-samping (surge)   3,4 = pasangan haluan (heave)
#   5   = buritan tengah (heave)            6   = tengah bawah (sway)
# Ini MENGOREKSI mapping lama yang menukar peran pasangan haluan <-> tengah
# (dulu 1,2 dianggap vertikal dan 3,4 surge; CAD & Gambar 2.9 menunjukkan
# sebaliknya) serta menukar 5 <-> 6.
#
# Dampak ke kondisi TAM: cond turun 19.7 -> 10.0, singular value terkecil naik
# 0.088 -> 0.174, gaya thruster utk wrench satuan pada DOF terlemah turun
# 9.25 N -> 4.06 N. Jadi geometri terukur ini LEBIH mudah dikendalikan.
#
# ARAH PUTAR PROPELLER (Gambar 2.9): 1=CCW 2=CW 3=CCW 4=CW 5=CW 6=CCW.
# Tiap pasangan berlawanan arah supaya torsi reaksinya saling meniadakan.
# Nilai ini TIDAK dipakai TAM (TAM hanya butuh gaya & lengan momen); disimpan
# di SPIN sebagai acuan perakitan/verifikasi.
#
# TANDA SUMBU — perlu verifikasi bench. Gambar 2.10 menggambar F1/F2 (surge)
# menunjuk ke BURITAN dan F6 (sway) ke KIRI. Arah F6 dipakai apa adanya
# (+y). Untuk surge dipakai +x (maju) mengikuti REP-103, BUKAN arah panah
# gambar, karena panah untuk thruster vertikal (F3/F4/F5) jelas artefak
# gambar (thruster vertikal tak bisa digambar arah gayanya di tampak atas)
# sehingga gambar itu tidak konsisten sendiri. Kalau uji bench menunjukkan
# perintah positif justru mendorong mundur, balik tanda di sini DAN di kedua
# URDF — TAM otomatis ikut.
THRUSTERS = [
    # thruster_1 = surge kanan (T200, duct 90.1 mm)
    (np.array([-0.0291, -0.1188, 0.0186]),  np.array([1.0, 0.0, 0.0])),
    # thruster_2 = surge kiri (T200, duct 90.1 mm)
    (np.array([-0.0287, 0.1274, 0.0190]),   np.array([1.0, 0.0, 0.0])),
    # thruster_3 = heave haluan kanan (T100, prop 66.6 mm)
    (np.array([0.1050, -0.1332, 0.0592]),   np.array([0.0, 0.0, 1.0])),
    # thruster_4 = heave haluan kiri (T100, prop 66.6 mm)
    (np.array([0.1053, 0.1411, 0.0550]),    np.array([0.0, 0.0, 1.0])),
    # thruster_5 = heave buritan tengah (T100, prop 66.6 mm)
    (np.array([-0.1610, 0.0037, 0.0621]),   np.array([0.0, 0.0, 1.0])),
    # thruster_6 = sway tengah-bawah (T100, prop 59.0 mm)
    (np.array([-0.0700, 0.0200, -0.0939]),  np.array([0.0, 1.0, 0.0])),
]

# Arah putar propeller per thruster (Gambar 2.9 dokumen desain, penulis 2026).
# Dokumentatif: dipakai saat perakitan & pemeriksaan, tidak masuk TAM.
SPIN = ('CCW', 'CW', 'CCW', 'CW', 'CW', 'CCW')

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
