"""odom_noise_logic — suntik noise sensor realistis ke odom ground truth (P2-B).

/hydroships/odom dari Gazebo adalah ground truth noiseless (posisi/kecepatan/
heading persis nilai fisika sim) — di kontes nyata tak ada sensor sepresisi itu
tanpa DVL/GPS bawah air. Modul ini menambah:
  * white noise iid ke posisi & kecepatan linear (galat sensor acak per-sampel)
  * random walk ke heading (drift kompas/IMU tanpa referensi absolut — bias
    terakumulasi antar tick, bukan iid, supaya perilakunya mirip sensor nyata)

Fungsi murni (tanpa rclpy/ROS msg) supaya testable headless; node ROS
(odom_injector.py) cuma jembatan Odometry<->primitif di sekitar fungsi ini.
"""

import math


def quat_mul(q1, q2):
    """Hasil kali quaternion (w, x, y, z) x (w, x, y, z)."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


def yaw_delta_quat(dyaw):
    """Quaternion rotasi murni `dyaw` (rad) di sumbu Z (heading dunia)."""
    h = dyaw / 2.0
    return (math.cos(h), 0.0, 0.0, math.sin(h))


def perturb_heading(q, heading_bias):
    """Putar quaternion (w,x,y,z) `heading_bias` rad di sumbu Z, tanpa mengubah
    roll/pitch — rotasi-kiri drpd dekomposisi euler (hindari gimbal-lock)."""
    return quat_mul(yaw_delta_quat(heading_bias), q)


def step_heading_bias(heading_bias, dt, heading_std, rng):
    """Random walk: bias heading (rad) terakumulasi tiap tick. Skala sqrt(dt)
    (konvensi proses Wiener) supaya laju drift efektif tak tergantung rate
    publish odom."""
    if heading_std <= 0.0:
        return heading_bias
    return heading_bias + rng.gauss(0.0, heading_std) * math.sqrt(max(dt, 1e-3))


def add_white_noise(value, std, rng):
    """value + N(0, std). std<=0 -> passthrough (tak ada noise)."""
    return value + rng.gauss(0.0, std) if std > 0.0 else value
