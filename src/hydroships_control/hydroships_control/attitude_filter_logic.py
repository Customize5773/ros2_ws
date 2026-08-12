"""attitude_filter_logic — complementary filter orientasi, murni tanpa ROS.

Estimasi roll/pitch dari gravitasi (accelerometer) dikombinasikan dengan
integrasi gyroscope untuk roll/pitch/yaw. Representasi internal orientasi
adalah quaternion (w, x, y, z); blending accel/gyro dihitung lewat dekomposisi
Euler sesaat lalu direkonstruksi kembali ke quaternion tiap update.

Yaw TIDAK dikoreksi oleh apa pun (tidak ada magnetometer di ROV ini, lihat
docs/P1-2A-ORIENTATION-ESTIMATION-DESIGN.md §3) -- murni integrasi gyro,
sehingga akan drift tanpa batas seiring waktu. Ini keterbatasan yang
disengaja, bukan bug.

`msg.orientation` dari `/hydroships/imu` mentah SENGAJA tidak pernah dibaca
modul ini (lihat docs/P1-2A-RUNTIME-VERIFICATION.md §3): field itu terisi di
runtime tapi covariance-nya all-zero (tak auditable) dan tampaknya cuma
integrasi gyro plugin sendiri -- tidak lebih baik dari yang dihasilkan filter
ini sendiri.
"""

import math

DT_MAX_DEFAULT = 0.25          # klem dt (detik); rate runtime terukur ~31-36 Hz, nominal 50 Hz.
ACCEL_GRAVITY = 9.81            # m/s^2, untuk normalisasi vektor accel.
ACCEL_VALID_MIN = 2.0           # magnitudo accel di luar rentang ini dianggap tidak andal
ACCEL_VALID_MAX = 20.0          # untuk koreksi roll/pitch (mis. saat akselerasi keras).


def _is_finite(*values):
    return all(math.isfinite(v) for v in values)


def quat_normalize(w, x, y, z):
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-12:
        return 1.0, 0.0, 0.0, 0.0
    return w / n, x / n, y / n, z / n


def quat_from_euler(roll, pitch, yaw):
    """ZYX (yaw-pitch-roll) -> quaternion (w, x, y, z)."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return quat_normalize(w, x, y, z)


def quat_to_euler(w, x, y, z):
    """quaternion (w, x, y, z) -> (roll, pitch, yaw), rad, konvensi ZYX."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def wrap_to_pi(angle):
    """Sama seperti hydroships_control.pid.wrap_to_pi -- direplikasi di sini
    supaya modul ini tetap ROS-independent murni (pid.py juga pure, tapi
    menjaga modul ini tanpa dependency lintas-file selain math)."""
    return math.atan2(math.sin(angle), math.cos(angle))


def roll_pitch_from_accel(ax, ay, az):
    """Roll/pitch (rad) dari vektor gravitasi accelerometer body-frame.

    Konvensi: roll di sekitar sumbu x, pitch di sekitar sumbu y, az positif
    saat ROV level (gravitasi terbaca di sumbu z saat diam).
    """
    roll = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
    return roll, pitch


class ComplementaryFilter:
    """Complementary filter orientasi: accel (roll/pitch, low-freq) + gyro
    (roll/pitch/yaw, high-freq, terintegrasi). Yaw murni gyro -- drift.
    """

    def __init__(self, alpha=0.98, dt_max=DT_MAX_DEFAULT):
        if not (0.0 < alpha < 1.0):
            raise ValueError('alpha harus di (0, 1)')
        if dt_max <= 0.0:
            raise ValueError('dt_max harus > 0')
        self.alpha = alpha
        self.dt_max = dt_max
        self._initialized = False
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0

    @property
    def initialized(self):
        return self._initialized

    def reset(self):
        self._initialized = False
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0

    def quaternion(self):
        return quat_from_euler(self._roll, self._pitch, self._yaw)

    def update(self, ax, ay, az, gx, gy, gz, dt):
        """Satu langkah filter. Mengembalikan True jika state diperbarui
        (dan quaternion output valid), False jika sample ditolak (input
        invalid atau dt tidak valid) -- pada False, state SEBELUMNYA
        dipertahankan apa adanya (tidak di-corrupt oleh sample buruk)."""
        if not _is_finite(ax, ay, az, gx, gy, gz, dt):
            return False

        # dt<=0 (timestamp mundur/kembar): tolak sample, jangan integrasi terbalik/nan.
        if dt <= 0.0:
            return False

        dt = min(dt, self.dt_max)

        accel_mag = math.sqrt(ax * ax + ay * ay + az * az)
        accel_usable = ACCEL_VALID_MIN <= accel_mag <= ACCEL_VALID_MAX

        if not self._initialized:
            if accel_usable:
                self._roll, self._pitch = roll_pitch_from_accel(ax, ay, az)
            self._yaw = 0.0
            self._initialized = True
            return True

        # Integrasi gyro murni (dead-reckoning) untuk ketiga sudut.
        gyro_roll = self._roll + gx * dt
        gyro_pitch = self._pitch + gy * dt
        gyro_yaw = wrap_to_pi(self._yaw + gz * dt)

        if accel_usable:
            accel_roll, accel_pitch = roll_pitch_from_accel(ax, ay, az)
            self._roll = self.alpha * gyro_roll + (1.0 - self.alpha) * accel_roll
            self._pitch = self.alpha * gyro_pitch + (1.0 - self.alpha) * accel_pitch
        else:
            # Accel tak andal (mis. akselerasi keras) -- percaya gyro saja untuk langkah ini.
            self._roll = gyro_roll
            self._pitch = gyro_pitch

        self._yaw = gyro_yaw  # tidak pernah dikoreksi accel -- tidak ada magnetometer.
        return True
