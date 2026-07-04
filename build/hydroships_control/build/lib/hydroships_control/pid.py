#!/usr/bin/env python3
"""PID controller sederhana (modul murni, tanpa ROS) untuk HYDROships M2.

Fitur:
  * Turunan pada PENGUKURAN (bukan error) -> hindari "derivative kick" saat
    setpoint berubah mendadak.
  * Anti-windup: integral dibatasi + back-calculation saat output ter-clamp.
  * Aman terhadap dt <= 0 (langkah dilewati).
"""

import math


def wrap_to_pi(angle):
    """Bungkus sudut (rad) ke rentang [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class PID:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0,
                 out_min=-math.inf, out_max=math.inf,
                 integral_limit=math.inf):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self.integral_limit = integral_limit
        self.reset()

    def reset(self):
        self._integral = 0.0
        self._prev_measurement = None

    def set_gains(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd

    def update(self, error, measurement, dt):
        """Hitung output kendali dari error & pengukuran saat ini."""
        if dt <= 0.0:
            # Tanpa waktu berlalu: hanya proporsional, tanpa update state.
            return self._clamp(self.kp * error)

        # Proporsional.
        p = self.kp * error

        # Integral dengan pembatasan.
        self._integral += error * dt
        self._integral = max(-self.integral_limit,
                             min(self.integral_limit, self._integral))
        i = self.ki * self._integral

        # Derivatif pada pengukuran (negatif turunan measurement).
        if self._prev_measurement is None:
            d = 0.0
        else:
            d = -self.kd * (measurement - self._prev_measurement) / dt
        self._prev_measurement = measurement

        raw = p + i + d
        out = self._clamp(raw)

        # Anti-windup back-calculation: kalau ter-clamp, tarik balik integral
        # agar tidak menumpuk.
        if raw != out and self.ki != 0.0:
            self._integral -= (raw - out) / self.ki

        return out

    def _clamp(self, value):
        return max(self.out_min, min(self.out_max, value))
