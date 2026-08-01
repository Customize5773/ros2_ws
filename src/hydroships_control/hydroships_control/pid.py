#!/usr/bin/env python3
"""PID controller modul murni (tanpa dependensi ROS) untuk HYDROships M2.

Fitur & Keamanan:
  * Turunan pada PENGUKURAN (bukan error) untuk mencegah "derivative kick".
  * `angular=True`: delta pengukuran di-wrap ke [-pi, pi] -> aman untuk yaw yang
    melintasi batas +-pi.
  * `d_filter_alpha`: Low-pass filter (EMA) pada turunan dengan flag warm-start
    `_d_warm` (sampel turunan pertama langsung dipakai penuh, tanpa lag palsu
    dari ramp-up 0; TIDAK memakai `nilai == 0.0` sebagai sentinel, supaya 0.0
    yang absah -- mis. saat ROV diam stabil di target -- tidak disalahartikan
    sebagai "belum diinisialisasi" dan membuat filter bocor).
  * Anti-windup back-calculation + re-clamp ke integral_limit.
  * Robust setter property untuk runtime update d_filter_alpha (di-clamp ulang
    setiap kali di-set, bukan cuma sekali saat __init__).
  * dt divalidasi finite di dalam update() sendiri (pertahanan berlapis,
    tidak hanya mengandalkan validasi dt di sisi caller/node).
  * abs(ki) > 1e-6 sebagai syarat anti-windup, mencegah pembagian dengan ki
    yang mendekati nol membuat koreksi integral meledak.
"""

import math


def wrap_to_pi(angle: float) -> float:
    """Bungkus sudut (rad) ke rentang [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class PID:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0,
                 out_min=-math.inf, out_max=math.inf,
                 integral_limit=math.inf,
                 angular=False,
                 d_filter_alpha=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self.integral_limit = integral_limit
        self.angular = angular

        self._d_filter_alpha = 1.0
        self.d_filter_alpha = d_filter_alpha  # Memanggil property setter

        self.reset()

    @property
    def d_filter_alpha(self) -> float:
        return self._d_filter_alpha

    @d_filter_alpha.setter
    def d_filter_alpha(self, val: float):
        """Pastikan alpha selalu di-clamp ke rentang aman (1e-6, 1.0]."""
        try:
            v = float(val)
            self._d_filter_alpha = min(1.0, max(1e-6, v))
        except (ValueError, TypeError):
            self._d_filter_alpha = 1.0

    def reset(self):
        """Reset internal state controller."""
        self._integral = 0.0
        self._prev_measurement = None
        self._d_filtered = 0.0
        self._d_warm = False

    def set_gains(self, kp: float, ki: float, kd: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def update(self, error: float, measurement: float, dt: float) -> float:
        """Hitung output kendali dari error & pengukuran saat ini."""
        if dt <= 0.0 or not math.isfinite(dt):
            return self._clamp(self.kp * error)

        # 1. Proporsional
        p = self.kp * error

        # 2. Integral dengan Clamping Awal
        self._integral += error * dt
        if math.isfinite(self.integral_limit):
            self._integral = max(-self.integral_limit,
                                 min(self.integral_limit, self._integral))
        i = self.ki * self._integral

        # 3. Derivatif pada Pengukuran + Low-Pass Filter
        if self._prev_measurement is None:
            d_raw = 0.0
        else:
            delta = measurement - self._prev_measurement
            if self.angular:
                delta = wrap_to_pi(delta)
            d_raw = -delta / dt

        if not self._d_warm:
            self._d_filtered = d_raw
            if self._prev_measurement is not None:
                self._d_warm = True
        else:
            a = self._d_filter_alpha
            self._d_filtered = a * d_raw + (1.0 - a) * self._d_filtered

        self._prev_measurement = measurement
        d = self.kd * self._d_filtered

        # 4. Total Output & Clamping
        raw = p + i + d
        out = self._clamp(raw)

        # 5. Anti-Windup Back-Calculation
        if raw != out and abs(self.ki) > 1e-6:
            self._integral -= (raw - out) / self.ki
            if math.isfinite(self.integral_limit):
                self._integral = max(-self.integral_limit,
                                     min(self.integral_limit, self._integral))

        return out

    def _clamp(self, value: float) -> float:
        return max(self.out_min, min(self.out_max, value))