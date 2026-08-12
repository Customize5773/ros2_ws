"""Uji ComplementaryFilter (murni, tanpa ROS) -- P1.2A."""

import math

from hydroships_control.attitude_filter_logic import (
    ComplementaryFilter,
    quat_to_euler,
    roll_pitch_from_accel,
)

G = 9.81


def _rpy(cf):
    w, x, y, z = cf.quaternion()
    return quat_to_euler(w, x, y, z)


def test_static_orientation_converges_and_stays_stable():
    """Accel gravity-only + gyro nol -> roll/pitch ~ 0, stabil, tak drift."""
    cf = ComplementaryFilter(alpha=0.98)
    for _ in range(50):
        cf.update(0.0, 0.0, G, 0.0, 0.0, 0.0, dt=0.02)
    roll, pitch, _ = _rpy(cf)
    assert math.isclose(roll, 0.0, abs_tol=1e-3)
    assert math.isclose(pitch, 0.0, abs_tol=1e-3)


def test_known_angular_rate_yaw_integration():
    """Yaw murni gyro: yaw ~= rate * waktu (dt aktual per-langkah, bukan tetap)."""
    cf = ComplementaryFilter(alpha=0.98)
    cf.update(0.0, 0.0, G, 0.0, 0.0, 0.0, dt=0.02)  # bootstrap
    rate = 0.3  # rad/s
    dt = 0.02
    steps = 100
    for _ in range(steps):
        cf.update(0.0, 0.0, G, 0.0, 0.0, rate, dt=dt)
    _, _, yaw = _rpy(cf)
    expected = rate * dt * steps
    assert math.isclose(yaw, expected, abs_tol=1e-2)


def test_roll_pitch_convergence_from_accel():
    """State roll salah -> koreksi accel menariknya kembali ke 0 (bukan instan)."""
    cf = ComplementaryFilter(alpha=0.9)  # alpha lebih rendah -> koreksi lebih cepat, uji lebih pendek
    cf.update(0.0, 0.0, G, 0.0, 0.0, 0.0, dt=0.02)  # bootstrap roll=0
    cf._roll = math.radians(45.0)  # paksa state salah (akses internal, uji whitebox)
    for _ in range(200):
        cf.update(0.0, 0.0, G, 0.0, 0.0, 0.0, dt=0.02)
    roll, _, _ = _rpy(cf)
    assert abs(roll) < math.radians(1.0)


def test_yaw_not_affected_by_accel():
    """Accel non-gravity apa pun tidak boleh mengoreksi yaw -- murni integrasi gyro."""
    cf = ComplementaryFilter(alpha=0.98)
    cf.update(0.0, 0.0, G, 0.0, 0.0, 0.0, dt=0.02)  # bootstrap
    dt = 0.02
    gz = 0.1
    for _ in range(20):
        cf.update(5.0, -3.0, 2.0, 0.0, 0.0, gz, dt=dt)  # accel non-gravity sembarang
    _, _, yaw = _rpy(cf)
    assert math.isclose(yaw, gz * dt * 20, abs_tol=1e-2)


def test_dt_clamp_prevents_large_jump():
    cf = ComplementaryFilter(alpha=0.98, dt_max=0.25)
    cf.update(0.0, 0.0, G, 0.0, 0.0, 0.0, dt=0.02)  # bootstrap
    ok = cf.update(0.0, 0.0, G, 0.0, 0.0, 1.0, dt=100.0)  # gap besar
    assert ok
    _, _, yaw = _rpy(cf)
    # delta yaw tak boleh melebihi rate * dt_max (bukan rate * dt mentah 100s).
    assert abs(yaw) <= 1.0 * 0.25 + 1e-6


def test_yaw_wrap_stays_in_range():
    cf = ComplementaryFilter(alpha=0.98)
    cf.update(0.0, 0.0, G, 0.0, 0.0, 0.0, dt=0.02)  # bootstrap
    rate = 4.0  # rad/s, cukup cepat untuk melewati +-pi berulang
    for _ in range(500):
        cf.update(0.0, 0.0, G, 0.0, 0.0, rate, dt=0.02)
    _, _, yaw = _rpy(cf)
    assert -math.pi < yaw <= math.pi


def test_invalid_input_preserves_previous_state():
    cf = ComplementaryFilter(alpha=0.98)
    cf.update(0.0, 0.0, G, 0.0, 0.0, 0.3, dt=0.02)
    cf.update(0.0, 0.0, G, 0.0, 0.0, 0.3, dt=0.02)
    before = cf.quaternion()

    ok = cf.update(float('nan'), 0.0, G, 0.0, 0.0, 0.3, dt=0.02)
    assert ok is False
    assert cf.quaternion() == before  # state tak berubah dari sample invalid

    ok2 = cf.update(0.0, 0.0, G, 0.0, 0.0, 0.3, dt=0.02)  # sample valid berikutnya
    assert ok2 is True
    assert cf.quaternion() != before  # lanjut dari state sebelum-NaN, bukan corrupt


def test_negative_dt_rejected():
    cf = ComplementaryFilter(alpha=0.98)
    cf.update(0.0, 0.0, G, 0.0, 0.0, 0.0, dt=0.02)
    before = cf.quaternion()
    ok = cf.update(0.0, 0.0, G, 0.0, 0.0, 0.3, dt=-0.01)
    assert ok is False
    assert cf.quaternion() == before


def test_zero_dt_rejected():
    cf = ComplementaryFilter(alpha=0.98)
    cf.update(0.0, 0.0, G, 0.0, 0.0, 0.0, dt=0.02)
    before = cf.quaternion()
    ok = cf.update(0.0, 0.0, G, 0.0, 0.0, 0.3, dt=0.0)
    assert ok is False
    assert cf.quaternion() == before


def test_quaternion_stays_normalized():
    cf = ComplementaryFilter(alpha=0.95)
    cf.update(0.0, 0.0, G, 0.0, 0.0, 0.0, dt=0.02)
    for i in range(300):
        cf.update(0.1, -0.2, G + 0.05, 0.05, -0.03, 0.4, dt=0.017)
    w, x, y, z = cf.quaternion()
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    assert math.isclose(norm, 1.0, abs_tol=1e-9)


def test_roll_pitch_from_accel_level():
    roll, pitch = roll_pitch_from_accel(0.0, 0.0, G)
    assert math.isclose(roll, 0.0, abs_tol=1e-9)
    assert math.isclose(pitch, 0.0, abs_tol=1e-9)


def test_accel_out_of_range_falls_back_to_gyro_only():
    """Accel magnitudo di luar rentang wajar (akselerasi keras) -> tidak dipakai koreksi."""
    cf = ComplementaryFilter(alpha=0.5)  # alpha rendah supaya koreksi accel (jika dipakai) besar
    cf.update(0.0, 0.0, G, 0.0, 0.0, 0.0, dt=0.02)  # bootstrap, roll=0
    # accel magnitudo ~50 m/s^2 (di luar ACCEL_VALID_MAX) menuding roll besar kalau dipakai.
    ok = cf.update(40.0, 0.0, 30.0, 0.0, 0.0, 0.0, dt=0.02)
    assert ok
    roll, pitch, _ = _rpy(cf)
    # Tanpa koreksi accel dan gyro roll/pitch rate = 0, roll/pitch harus tetap ~0.
    assert math.isclose(roll, 0.0, abs_tol=1e-6)
    assert math.isclose(pitch, 0.0, abs_tol=1e-6)
