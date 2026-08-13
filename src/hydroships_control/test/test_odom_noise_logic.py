"""Uji odom_noise_logic (murni, tanpa ROS): white noise posisi/kecepatan,
random walk heading, dan rotasi quaternion di sumbu Z (P2-B).
"""

import math
import random

from hydroships_control.odom_noise_logic import (
    add_white_noise, perturb_heading, quat_mul, step_heading_bias,
    yaw_delta_quat)

IDENTITY_Q = (1.0, 0.0, 0.0, 0.0)


def test_add_white_noise_zero_std_passthrough():
    rng = random.Random(1)
    assert add_white_noise(5.0, 0.0, rng) == 5.0


def test_add_white_noise_matches_distribution():
    rng = random.Random(42)
    n = 20000
    samples = [add_white_noise(0.0, 0.03, rng) for _ in range(n)]
    mean = sum(samples) / n
    std = math.sqrt(sum((s - mean) ** 2 for s in samples) / n)
    assert abs(mean) < 0.01
    assert abs(std - 0.03) < 0.005


def test_step_heading_bias_zero_std_passthrough():
    rng = random.Random(1)
    assert step_heading_bias(0.3, 1.0, 0.0, rng) == 0.3


def test_step_heading_bias_accumulates_over_time():
    """Random walk: bias TIDAK kembali ke 0 antar tick (beda dari white noise)."""
    rng = random.Random(7)
    bias = 0.0
    for _ in range(50):
        bias = step_heading_bias(bias, 0.1, math.radians(1.0), rng)
    assert bias != 0.0


def test_step_heading_bias_scales_with_sqrt_dt():
    """Laju drift efektif per-detik konstan -> varians step sebanding dt (Wiener)."""
    rng_a = random.Random(3)
    rng_b = random.Random(3)
    step_small = step_heading_bias(0.0, 0.01, 1.0, rng_a)
    step_large = step_heading_bias(0.0, 1.0, 1.0, rng_b)
    # sqrt(1.0/0.01) = 10x -> gaussian sample sama (seed sama) discale identik
    assert abs(step_large / step_small - 10.0) < 1e-9


def test_quat_mul_identity():
    q = (0.7071, 0.0, 0.0, 0.7071)
    assert quat_mul(IDENTITY_Q, q) == q


def test_yaw_delta_quat_90deg():
    dq = yaw_delta_quat(math.pi / 2)
    assert math.isclose(dq[0], math.cos(math.pi / 4), abs_tol=1e-9)
    assert math.isclose(dq[3], math.sin(math.pi / 4), abs_tol=1e-9)
    assert dq[1] == 0.0 and dq[2] == 0.0


def test_perturb_heading_preserves_roll_pitch_when_zero():
    """Heading bias 0 -> quaternion tak berubah."""
    q = (0.9239, 0.3827, 0.0, 0.0)  # roll ~45deg, tak ada yaw
    nq = perturb_heading(q, 0.0)
    for a, b in zip(q, nq):
        assert math.isclose(a, b, abs_tol=1e-9)


def test_perturb_heading_rotates_yaw_only():
    """Quaternion murni heading (identity roll/pitch) diputar dyaw -> yaw baru
    = yaw lama + dyaw (dicek via ekstraksi yaw standar dari quaternion)."""
    def yaw_of(q):
        w, x, y, z = q
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    q0 = yaw_delta_quat(math.radians(30))
    nq = perturb_heading(q0, math.radians(20))
    assert math.isclose(math.degrees(yaw_of(nq)), 50.0, abs_tol=1e-6)
