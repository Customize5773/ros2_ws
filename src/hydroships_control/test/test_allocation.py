"""Uji Thrust Allocation Matrix (murni, tanpa ROS)."""

import numpy as np

from hydroships_control.allocation import (
    MAX_THRUST,
    MIN_THRUST,
    THRUSTERS,
    allocate,
    build_allocation_matrix,
    build_damped_pinv,
)


def test_tam_full_rank():
    tam = build_allocation_matrix(THRUSTERS)
    assert tam.shape == (6, 6)
    assert np.linalg.matrix_rank(tam) == 6


def test_each_dof_reconstructs():
    """Invers polos (undamped) merekonstruksi tiap DOF secara eksak — sifat
    matematis TAM full-rank. Catatan: pada arah near-singular (yaw) gaya yang
    dibutuhkan bisa raksasa; itu justru alasan node memakai damped pinv."""
    tam = build_allocation_matrix(THRUSTERS)
    pinv = np.linalg.pinv(tam)
    identity = np.eye(6)
    for i in range(6):
        wrench = identity[:, i]
        forces = pinv @ wrench
        recon = tam @ forces
        assert np.allclose(recon, wrench, atol=1e-6), f'DOF {i} tidak terekonstruksi'


def _axis_groups():
    """Indeks thruster berdasarkan sumbu dorong: vertikal (z) vs horizontal (x/y).
    Digeneralisasi dari THRUSTERS supaya test tak bergantung urutan/mapping."""
    vertical, horizontal = [], []
    for i, (_pos, axis) in enumerate(THRUSTERS):
        (vertical if abs(axis[2]) > 0.9 else horizontal).append(i)
    return vertical, horizontal


def test_axis_force_decoupling():
    """Sifat STRUKTURAL yang dijamin konstruksi TAM (baris gaya = sumbu dorong):
    thruster vertikal (sumbu z) tak menghasilkan gaya horizontal (Fx,Fy=0), dan
    thruster horizontal (sumbu x/y) tak menghasilkan gaya vertikal (Fz=0).
    Menjaga definisi `axis` di THrusterS tetap konsisten dgn URDF.
    (Menggantikan test lama yang mengasumsikan thruster 1-3 horizontal / 4-6
    vertikal — asumsi itu tak lagi benar setelah mapping thruster diperbarui;
    lihat PROBLEM.md.)"""
    tam = build_allocation_matrix(THRUSTERS)
    vertical, horizontal = _axis_groups()
    for i in vertical:
        assert np.allclose(tam[0:2, i], 0.0, atol=1e-9), f'thruster {i} vertikal bocor ke Fx/Fy'
    for i in horizontal:
        assert np.allclose(tam[2, i], 0.0, atol=1e-9), f'thruster {i} horizontal bocor ke Fz'


def test_allocate_clips_to_limits():
    tam = build_allocation_matrix(THRUSTERS)
    pinv = np.linalg.pinv(tam)
    # Wrench sangat besar -> pasti kena batas.
    forces = allocate([1e6, 0, 0, 0, 0, 0], pinv)
    assert np.all(forces <= MAX_THRUST + 1e-9)
    assert np.all(forces >= MIN_THRUST - 1e-9)


def test_geometry_well_conditioned():
    """Geometri thruster (frame body benar) harus well-conditioned di 6 DOF —
    khususnya YAW punya otoritas nyata. Dulu posisi tersalin dgn frame salah →
    yaw near-singular (cond~1.2e4, butuh ribuan N); setelah dikoreksi cond~20,
    yaw 5 N·m < 100 N. Test ini menjaga agar frame posisi tak ter-regresi
    (lihat PROBLEM.md)."""
    tam = build_allocation_matrix(THRUSTERS)
    assert np.linalg.cond(tam) < 100.0, 'TAM ill-conditioned — cek frame posisi thruster'
    f_yaw = np.linalg.pinv(tam) @ np.array([0.0, 0.0, 0.0, 0.0, 0.0, 5.0])
    assert np.max(np.abs(f_yaw)) < 100.0, 'otoritas yaw lemah — cek posisi T100-A/C'


def test_damped_pinv_preserves_strong_dofs():
    """Redaman tak boleh melumpuhkan DOF sehat (heave/sway/surge): wrench yang
    tercapai setelah alokasi+clip harus mendekati perintah."""
    tam = build_allocation_matrix(THRUSTERS)
    pinv_d = build_damped_pinv(tam, damping=0.1)
    for idx, val, name in [(2, 25.0, 'heave'), (1, 10.0, 'sway'), (0, 25.0, 'surge')]:
        w = np.zeros(6); w[idx] = val
        forces = allocate(w, pinv_d)          # pinv_d @ w, lalu clip ke batas thruster
        delivered = (tam @ forces)[idx]
        assert delivered >= 0.9 * val, f'{name} hanya tercapai {delivered:.1f}/{val}'


def test_damping_zero_equals_plain_pinv():
    tam = build_allocation_matrix(THRUSTERS)
    assert np.allclose(build_damped_pinv(tam, 0.0), np.linalg.pinv(tam), atol=1e-9)
