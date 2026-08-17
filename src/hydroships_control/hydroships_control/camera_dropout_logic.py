"""camera_dropout_logic — bernoulli drop utk simulasi frame loss kamera (R-8).

Fungsi murni (tanpa rclpy/sensor_msgs) supaya testable headless; node ROS
(camera_dropout_injector.py) cuma jembatan Image<->primitif di sekitar fungsi ini.
"""


def should_drop(rng, drop_prob):
    """True kalau frame ini harus di-drop (tak diteruskan). drop_prob<=0 -> tak
    pernah drop, drop_prob>=1 -> selalu drop."""
    if drop_prob <= 0.0:
        return False
    if drop_prob >= 1.0:
        return True
    return rng.random() < drop_prob
