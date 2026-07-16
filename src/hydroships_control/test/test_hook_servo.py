"""Uji hook_servo (PD visual servo APPROACH_HOOK) — fungsi murni, tanpa rclpy.

Verifikasi arah tanda, redaman kecepatan, konvergensi setpoint depth, clamp gaya,
dan flag aligned/near. Ikuti pola test_pid.py / test_allocation.py.
"""

import math

from hydroships_control.hook_logic import (
    HookServoGains, hook_servo, normalize_hook_offset)


G = HookServoGains()   # gain default


def test_centered_far_moves_forward_only():
    # Hook terpusat (ex=ey=0) tapi kecil (jauh) -> maju, tanpa sway, belum near.
    cmd = hook_servo((0.0, 0.0, 0.05), vx=0.0, vy=0.0, hook_depth=0.45, gains=G)
    assert cmd.surge > 0.0
    assert abs(cmd.sway) < 1e-9
    assert math.isclose(cmd.target_depth, 0.45)   # ey=0 -> tak geser depth
    assert cmd.near is False
    assert cmd.aligned is True                     # terpusat


def test_hook_right_sways_right():
    # Hook di KANAN (ex>0) -> body +y = kiri, jadi sway NEGATIF (geser kanan).
    cmd = hook_servo((0.5, 0.0, 0.1), vx=0.0, vy=0.0, hook_depth=0.45, gains=G)
    assert cmd.sway < 0.0
    assert cmd.aligned is False


def test_hook_left_sways_left():
    cmd = hook_servo((-0.5, 0.0, 0.1), vx=0.0, vy=0.0, hook_depth=0.45, gains=G)
    assert cmd.sway > 0.0


def test_hook_below_goes_deeper():
    # Hook di BAWAH frame (ey>0) -> setpoint kedalaman bertambah (turun).
    cmd = hook_servo((0.0, 0.6, 0.1), vx=0.0, vy=0.0, hook_depth=0.45, gains=G)
    assert cmd.target_depth > 0.45


def test_hook_above_goes_shallower():
    cmd = hook_servo((0.0, -0.6, 0.1), vx=0.0, vy=0.0, hook_depth=0.45, gains=G)
    assert cmd.target_depth < 0.45


def test_depth_setpoint_clamped():
    # ey ekstrem tak boleh menggeser depth > depth_range.
    cmd = hook_servo((0.0, 100.0, 0.1), vx=0.0, vy=0.0, hook_depth=0.45, gains=G)
    assert math.isclose(cmd.target_depth, 0.45 + G.depth_range)


def test_velocity_damping_reduces_surge():
    # Kecepatan surge maju (vx>0) mengurangi gaya surge (redaman).
    still = hook_servo((0.0, 0.0, 0.05), vx=0.0, vy=0.0, hook_depth=0.45, gains=G)
    moving = hook_servo((0.0, 0.0, 0.05), vx=0.3, vy=0.0, hook_depth=0.45, gains=G)
    assert moving.surge < still.surge


def test_force_clamped_to_fmax():
    cmd = hook_servo((1.0, 0.0, -5.0), vx=-5.0, vy=-5.0, hook_depth=0.45, gains=G)
    assert -G.fmax <= cmd.surge <= G.fmax
    assert -G.fmax <= cmd.sway <= G.fmax


def test_near_and_aligned_when_close_and_centered():
    cmd = hook_servo((0.02, 0.02, 0.40), vx=0.0, vy=0.0, hook_depth=0.45, gains=G)
    assert cmd.near is True          # size 0.40 >= size_stop 0.35
    assert cmd.aligned is True       # |ex|,|ey| < center_tol


def test_convergence_reduces_error_over_iterations():
    # Simulasi loop tertutup sederhana (tanpa dinamika penuh): offset lateral
    # mengecil saat sway didorong -> servo stabil (tanda benar, tak divergen).
    ex = 0.6
    pos = ex
    for _ in range(40):
        cmd = hook_servo((pos, 0.0, 0.1), vx=0.0, vy=0.0, hook_depth=0.45, gains=G)
        # model kasar: gaya sway -> perpindahan lateral (body +y = kiri = ex mengecil
        # bila sway>0). sway = -kp*ex, jadi pos berkurang menuju 0.
        pos += 0.002 * cmd.sway
    assert abs(pos) < abs(ex)         # error mengecil (konvergen)


def test_offset_helper_consistent_with_servo_input():
    # normalize_hook_offset -> (ex,ey,size) langsung dipakai hook_servo.
    ex, ey, size = normalize_hook_offset((480, 240), area=900.0,
                                         frame_w=640, frame_h=480)
    cmd = hook_servo((ex, ey, size), vx=0.0, vy=0.0, hook_depth=0.45, gains=G)
    assert ex > 0 and cmd.sway < 0    # hook di kanan -> geser kanan
