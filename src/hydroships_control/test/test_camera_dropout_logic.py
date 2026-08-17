"""Uji camera_dropout_logic (murni, tanpa ROS): bernoulli drop frame (R-8)."""

import random

from hydroships_control.camera_dropout_logic import should_drop


def test_zero_prob_never_drops():
    rng = random.Random(1)
    assert not any(should_drop(rng, 0.0) for _ in range(1000))


def test_full_prob_always_drops():
    rng = random.Random(1)
    assert all(should_drop(rng, 1.0) for _ in range(1000))


def test_rate_matches_prob():
    rng = random.Random(42)
    n = 20000
    p = 0.05
    dropped = sum(should_drop(rng, p) for _ in range(n))
    rate = dropped / n
    assert abs(rate - p) < 0.01
