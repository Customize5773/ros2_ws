"""attitude_estimator — node ROS2 tipis di sekitar ComplementaryFilter (P1.2A).

Estimator ORIENTASI standalone, BELUM dipakai konsumen mana pun. Tidak
mengubah stabilizer.py/mission_fsm.py/thruster_allocator.py/gui_bridge.py --
integrasi consumer adalah tugas terpisah dengan gate validasinya sendiri
(lihat docs/P1-2A-ORIENTATION-ESTIMATION-DESIGN.md §11).

Kontrak topic:
    /hydroships/imu           (sensor_msgs/Imu) -> masuk : IMU mentah (ros_gz_bridge)
    /hydroships/imu/filtered  (sensor_msgs/Imu) -> keluar: orientation hasil fusi

`orientation` output dihitung sendiri oleh ComplementaryFilter (accel+gyro);
`msg.orientation` dari input SENGAJA diabaikan (lihat attitude_filter_logic.py
docstring dan docs/P1-2A-RUNTIME-VERIFICATION.md §3 -- field itu terisi di
runtime tapi covariance-nya all-zero, tak auditable).

`angular_velocity`/`linear_acceleration` (dan covariance masing-masing)
diteruskan apa adanya dari input (pass-through) -- field itu terbukti valid
secara fisik di runtime (docs/P1-2A-RUNTIME-VERIFICATION.md §1).

Kesehatan/kegagalan (minimal, sesuai desain §7, bukan diagnostics framework):
  - Selama inisialisasi (sample pertama): tidak publish output valid --
    filter butuh 1 sample accel untuk bootstrap roll/pitch awal.
  - `dt <= 0` (timestamp mundur/kembar) atau NaN/Inf pada field IMU manapun:
    sample DIBUANG (filter.update mengembalikan False), tidak publish untuk
    sample itu, state filter sebelumnya dipertahankan.
  - `dt` berlebihan (gap/drop pesan): diklem ke `dt_max` (parameter node),
    bukan dibuang -- filter tetap lanjut dengan integrasi dt terklem.
  - Saat sehat: `orientation_covariance` diisi nilai diagonal tetap kecil
    (estimator sendiri yang menentukan, TIDAK mewarisi covariance all-zero
    dari IMU mentah). Saat belum terinisialisasi, tidak ada pesan yang
    dipublish sama sekali (bukan publish dengan flag -1) -- lebih sederhana
    dan konsisten dengan "tidak ada instruksi diagnostics framework".

Keterbatasan yang diketahui dan disengaja: yaw murni integrasi gyro (tidak
ada magnetometer di ROV ini) -- akan DRIFT tanpa batas seiring waktu. Ini
bukan bug, ini properti struktural yang harus ditangani di P1.3 kalau
dibutuhkan heading akurat jangka panjang.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from hydroships_control.attitude_filter_logic import ComplementaryFilter

INPUT_TOPIC = '/hydroships/imu'
OUTPUT_TOPIC = '/hydroships/imu/filtered'

# Diagonal covariance tetap saat sehat -- estimasi kasar, bukan hasil tracking
# uncertainty riil (di luar scope P1.2A). ~0.05 rad^2 std ~ 13 derajat, cukup
# konservatif untuk consumer masa depan yang membaca field ini secara sadar.
HEALTHY_ORIENTATION_VARIANCE = 0.05


class AttitudeEstimator(Node):
    def __init__(self):
        super().__init__('attitude_estimator')

        self.declare_parameter('alpha', 0.98)
        self.declare_parameter('dt_max', 0.25)

        alpha = self.get_parameter('alpha').value
        dt_max = self.get_parameter('dt_max').value
        self._filter = ComplementaryFilter(alpha=alpha, dt_max=dt_max)

        self._last_stamp_sec = None  # float, detik, dari header.stamp pesan terakhir yang DIPAKAI

        self.pub = self.create_publisher(Imu, OUTPUT_TOPIC, 10)
        self.sub = self.create_subscription(Imu, INPUT_TOPIC, self._on_imu, 10)

        self.get_logger().info(
            f'attitude_estimator: {INPUT_TOPIC} -> {OUTPUT_TOPIC} '
            f'(alpha={alpha}, dt_max={dt_max}s) -- BELUM dipakai consumer manapun.')

    def _on_imu(self, msg: Imu):
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self._last_stamp_sec is None:
            dt = 0.0  # sample pertama: filter.update() melakukan bootstrap, dt tak dipakai.
        else:
            dt = stamp_sec - self._last_stamp_sec

        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z

        was_initialized = self._filter.initialized
        # Sample pertama: paksa dt positif kecil supaya tidak ditolak oleh guard dt<=0
        # (filter.update mengabaikan dt saat bootstrap, tapi guard dt<=0 tetap generik).
        update_dt = dt if was_initialized else 1e-3
        ok = self._filter.update(ax, ay, az, gx, gy, gz, update_dt)

        if not ok:
            if was_initialized and dt <= 0.0:
                self.get_logger().warn(
                    f'attitude_estimator: sample dibuang (dt={dt:.6f}s <= 0 atau '
                    'field non-finite) -- state sebelumnya dipertahankan.')
            return

        self._last_stamp_sec = stamp_sec

        if not self._filter.initialized:
            return  # belum ada estimasi valid untuk dipublish (seharusnya tak tercapai: update() True => initialized True).

        w, x, y, z = self._filter.quaternion()

        out = Imu()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = msg.header.frame_id

        out.orientation.w = w
        out.orientation.x = x
        out.orientation.y = y
        out.orientation.z = z
        out.orientation_covariance = [
            HEALTHY_ORIENTATION_VARIANCE, 0.0, 0.0,
            0.0, HEALTHY_ORIENTATION_VARIANCE, 0.0,
            0.0, 0.0, HEALTHY_ORIENTATION_VARIANCE,
        ]

        out.angular_velocity = msg.angular_velocity
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance

        self.pub.publish(out)


def main():
    rclpy.init()
    node = AttitudeEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
