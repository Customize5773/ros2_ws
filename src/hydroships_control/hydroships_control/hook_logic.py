"""hook_logic — helper murni deteksi/servo hook (tanpa ROS/cv2), agar testable.

Dipisah dari hook_detector.py (node, butuh rclpy) & dari detect_hook (butuh cv2)
supaya normalisasi offset & keputusan servo bisa diuji headless."""

from collections import namedtuple
from dataclasses import dataclass


def _clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def normalize_hook_offset(center, area, frame_w, frame_h):
    """(center px, area px^2, ukuran frame) -> (ex, ey, size) ternormalisasi.

    Konvensi sama dgn qr_offset:
      ex = (cx - W/2)/(W/2)  (+ = hook di KANAN pusat frame)
      ey = (cy - H/2)/(H/2)  (+ = hook di BAWAH pusat frame)
      size = sqrt(area)/W    (proxy jarak; besar = dekat)"""
    w = float(frame_w); h = float(frame_h)
    cx, cy = float(center[0]), float(center[1])
    ex = (cx - w / 2.0) / (w / 2.0) if w > 0 else 0.0
    ey = (cy - h / 2.0) / (h / 2.0) if h > 0 else 0.0
    size = (float(area) ** 0.5) / w if w > 0 else 0.0
    return ex, ey, size


@dataclass
class HookServoGains:
    """Gain PD visual-servo APPROACH_HOOK (holonomik: sway+surge+depth-setpoint).

    Mengganti servo proporsional-heading lama dgn PD penuh (kp*error - kd*velocity),
    mengikuti pola `mission_fsm._goto_xy`. Damping pakai kecepatan body-frame (vx/vy
    dari odom) untuk cegah overshoot; koreksi vertikal lewat SETPOINT kedalaman
    (stabilizer yg meng-hold depth, jadi tak butuh D di sini)."""
    kp_surge: float = 40.0     # N per unit error ukuran-tampak (maju bila terlalu jauh)
    kd_surge: float = 30.0     # N per (m/s) redaman kecepatan surge (body vx)
    kp_sway: float = 45.0      # N per unit offset-x (koreksi lateral ke tengah)
    kd_sway: float = 30.0      # N per (m/s) redaman kecepatan sway (body vy)
    kp_depth: float = 0.25     # m per unit offset-y (geser setpoint kedalaman)
    size_stop: float = 0.35    # ukuran-tampak hook -> dianggap cukup dekat
    center_tol: float = 0.15   # |offset| dianggap "sejajar"
    fmax: float = 16.0         # N batas gaya horizontal (sway/surge)
    depth_range: float = 0.20  # m simpangan maks setpoint kedalaman dari hook_depth


HookServoCmd = namedtuple('HookServoCmd', 'surge sway target_depth aligned near')


def hook_servo(off, vx, vy, hook_depth, gains):
    """PD visual servo hook -> perintah gerak (fungsi MURNI, testable).

    Args:
        off        : (ex, ey, size) offset ternormalisasi dari hook_offset.
                     ex>0 = hook di KANAN, ey>0 = hook di BAWAH, size = proxy jarak.
        vx, vy     : kecepatan body-frame (surge/sway) dari odom, utk redaman.
        hook_depth : kedalaman dasar hook (m, positif) — pusat rentang setpoint.
        gains      : HookServoGains.
    Returns HookServoCmd(surge, sway, target_depth, aligned, near):
        surge       : gaya maju body +x (N). + = maju (hook terlalu jauh).
        sway        : gaya lateral body +y (N). Body +y = KIRI, jadi hook di kanan
                      (ex>0) -> sway negatif (geser kanan).
        target_depth: setpoint kedalaman (m, positif) = hook_depth + koreksi(ey),
                      di-clamp ke ±depth_range.
        aligned     : True bila |ex| & |ey| < center_tol (terpusat).
        near        : True bila size >= size_stop (cukup dekat utk transisi).
    """
    ex, ey, size = float(off[0]), float(off[1]), float(off[2])
    g = gains
    e_size = g.size_stop - size                         # + = terlalu jauh -> maju
    surge = _clamp(g.kp_surge * e_size - g.kd_surge * vx, -g.fmax, g.fmax)
    sway = _clamp(-g.kp_sway * ex - g.kd_sway * vy, -g.fmax, g.fmax)
    d_depth = _clamp(g.kp_depth * ey, -g.depth_range, g.depth_range)
    target_depth = hook_depth + d_depth
    aligned = abs(ex) < g.center_tol and abs(ey) < g.center_tol
    near = size >= g.size_stop
    return HookServoCmd(surge, sway, target_depth, aligned, near)
