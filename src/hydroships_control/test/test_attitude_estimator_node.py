"""Smoke test node attitude_estimator (dengan ROS, tanpa sim) -- P1.2A.

Publish beberapa Imu sintetis ke /hydroships/imu, verifikasi
/hydroships/imu/filtered menerima orientation terisi & bukan sekadar
copy dari input, field lain diteruskan, timestamp diambil dari pesan input.
"""

import math
import time

import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from hydroships_control.attitude_estimator import INPUT_TOPIC, OUTPUT_TOPIC, AttitudeEstimator


def _make_imu(stamp_sec, stamp_nanosec, ax=0.0, ay=0.0, az=9.81,
              gx=0.0, gy=0.0, gz=0.0, frame_id='hydroships/base_link/imu'):
    msg = Imu()
    msg.header.stamp.sec = stamp_sec
    msg.header.stamp.nanosec = stamp_nanosec
    msg.header.frame_id = frame_id
    # orientation input SENGAJA diisi nilai non-trivial berbeda dari output yang
    # diharapkan (identity-ish), supaya uji "bukan copy input" berarti.
    msg.orientation.w = 0.0
    msg.orientation.x = 1.0
    msg.orientation.y = 0.0
    msg.orientation.z = 0.0
    msg.orientation_covariance = [0.0] * 9
    msg.angular_velocity.x = gx
    msg.angular_velocity.y = gy
    msg.angular_velocity.z = gz
    msg.angular_velocity_covariance = [0.0] * 9
    msg.linear_acceleration.x = ax
    msg.linear_acceleration.y = ay
    msg.linear_acceleration.z = az
    msg.linear_acceleration_covariance = [0.0] * 9
    return msg


@pytest.fixture
def ros_context():
    # Remap ke topic khusus test: environment CI/dev bisa punya sim
    # hydroships_bringup lain berjalan paralel di domain ROS yang sama dan
    # mempublish /hydroships/imu sungguhan -- remap ini mengisolasi smoke
    # test ini dari trafik topic produksi tersebut.
    rclpy.init(args=[
        '--ros-args',
        '-r', f'{INPUT_TOPIC}:=/test_attitude_estimator{INPUT_TOPIC}',
        '-r', f'{OUTPUT_TOPIC}:=/test_attitude_estimator{OUTPUT_TOPIC}',
    ])
    yield
    rclpy.shutdown()


def test_node_publishes_fused_orientation(ros_context):
    estimator = AttitudeEstimator()

    harness = Node('test_harness')
    pub = harness.create_publisher(Imu, INPUT_TOPIC, 10)
    received = []
    harness.create_subscription(Imu, OUTPUT_TOPIC, received.append, 10)

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(estimator)
    executor.add_node(harness)

    try:
        # Beberapa sample IMU sintetis: diam, lalu sedikit yaw rate.
        t0_sec, t0_ns = 10, 0
        dt_ns = 20_000_000  # 20ms
        for i in range(10):
            total_ns = t0_ns + i * dt_ns
            sec = t0_sec + total_ns // 1_000_000_000
            ns = total_ns % 1_000_000_000
            gz = 0.0 if i < 3 else 0.2
            msg = _make_imu(sec, ns, gz=gz)
            pub.publish(msg)
            for _ in range(5):
                executor.spin_once(timeout_sec=0.05)
            time.sleep(0.01)

        assert len(received) > 0, 'node tidak publish apa pun ke topik output'

        last = received[-1]

        # orientation terisi (quaternion valid, norm ~ 1) dan BUKAN copy input
        # (input diisi (0,1,0,0), filter mestinya menghasilkan sesuatu yang lain
        # karena bootstrap dari accel gravity-only -> mendekati identity).
        norm = math.sqrt(last.orientation.w ** 2 + last.orientation.x ** 2
                          + last.orientation.y ** 2 + last.orientation.z ** 2)
        assert math.isclose(norm, 1.0, abs_tol=1e-6)
        assert not (math.isclose(last.orientation.w, 0.0, abs_tol=1e-6)
                    and math.isclose(last.orientation.x, 1.0, abs_tol=1e-6)), (
            'output orientation sama persis dengan input -- diduga sekadar di-copy')

        # frame_id diteruskan dari input.
        assert last.header.frame_id == 'hydroships/base_link/imu'

        # field non-orientation diteruskan (pass-through) dari sample terakhir.
        assert math.isclose(last.linear_acceleration.z, 9.81, abs_tol=1e-9)
        assert math.isclose(last.angular_velocity.z, 0.2, abs_tol=1e-9)

        # covariance orientation output non-zero (estimator sendiri yang mengisi,
        # bukan mewarisi all-zero dari input).
        assert last.orientation_covariance[0] > 0.0

        # timestamp output = timestamp pesan input terakhir yang dipakai (bukan wall-clock).
        expected_total_ns = t0_ns + 9 * dt_ns
        expected_sec = t0_sec + expected_total_ns // 1_000_000_000
        expected_ns = expected_total_ns % 1_000_000_000
        assert last.header.stamp.sec == expected_sec
        assert last.header.stamp.nanosec == expected_ns

    finally:
        executor.remove_node(estimator)
        executor.remove_node(harness)
        estimator.destroy_node()
        harness.destroy_node()
