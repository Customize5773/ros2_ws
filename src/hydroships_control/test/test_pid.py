"""Uji PID controller (murni, tanpa ROS)."""

import math

from hydroships_control.pid import PID, wrap_to_pi


def test_proportional_only():
    pid = PID(kp=3.0, ki=0.0, kd=0.0)
    # d=0 pada langkah pertama, i=0 -> keluar = kp*error.
    assert pid.update(error=2.0, measurement=0.0, dt=0.1) == 6.0


def test_output_clamped():
    pid = PID(kp=100.0, out_min=-5.0, out_max=5.0)
    assert pid.update(error=10.0, measurement=0.0, dt=0.1) == 5.0
    assert pid.update(error=-10.0, measurement=0.0, dt=0.1) == -5.0


def test_integral_accumulates():
    pid = PID(kp=0.0, ki=1.0, kd=0.0)
    assert math.isclose(pid.update(1.0, 0.0, 1.0), 1.0)
    assert math.isclose(pid.update(1.0, 0.0, 1.0), 2.0)
    assert math.isclose(pid.update(1.0, 0.0, 1.0), 3.0)


def test_integral_limited_anti_windup():
    pid = PID(kp=0.0, ki=1.0, kd=0.0, integral_limit=2.0)
    for _ in range(10):
        out = pid.update(1.0, 0.0, 1.0)
    assert out <= 2.0 + 1e-9


def test_derivative_on_measurement():
    # kp=ki=0, kd=1: output = -(delta measurement)/dt.
    pid = PID(kp=0.0, ki=0.0, kd=1.0)
    pid.update(0.0, measurement=0.0, dt=1.0)      # inisialisasi prev
    out = pid.update(0.0, measurement=2.0, dt=1.0)
    assert math.isclose(out, -2.0)


def test_dt_zero_safe():
    pid = PID(kp=2.0, ki=1.0, kd=1.0)
    # dt<=0 tidak boleh error / tidak update state, hanya proporsional.
    assert pid.update(3.0, 0.0, 0.0) == 6.0


def test_wrap_to_pi():
    assert math.isclose(wrap_to_pi(0.0), 0.0)
    assert math.isclose(wrap_to_pi(math.pi + 0.1), -(math.pi - 0.1), abs_tol=1e-9)
    assert math.isclose(wrap_to_pi(-math.pi - 0.1), math.pi - 0.1, abs_tol=1e-9)


def test_closed_loop_converges():
    """Plant integrator sederhana: pengukuran mendekati setpoint."""
    pid = PID(kp=2.0, ki=0.5, kd=0.5, out_min=-10, out_max=10)
    x = 0.0            # pengukuran
    target = 1.0
    dt = 0.05
    for _ in range(400):
        u = pid.update(target - x, x, dt)
        x += u * dt * 0.3     # dinamika teredam
    assert abs(target - x) < 0.05
