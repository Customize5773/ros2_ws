"""Uji logika adapter GUI-ROV <-> ROS (murni, tanpa ROS/UDP)."""

import math

from hydroships_control.gui_bridge_logic import GuiBridgeLogic
from hydroships_control.hook_logic import normalize_hook_offset


def test_disarmed_wrench_zero():
    g = GuiBridgeLogic()
    g.on_command('surge', 100)
    # belum armed -> gerak nol (failsafe)
    assert g.wrench() == (0.0, 0.0, 0.0, 0.0)


def test_armed_axis_to_wrench():
    g = GuiBridgeLogic(surge_gain=0.4, sway_gain=0.4, heave_gain=0.3, yaw_gain=0.12)
    g.on_command('arm', True)
    g.on_command('surge', 100)
    g.on_command('sway', -50)
    g.on_command('heave', 100)
    g.on_command('yaw', 50)
    fx, fy, fz, mz = g.wrench()
    assert math.isclose(fx, 40.0)
    assert math.isclose(fy, -20.0)
    assert math.isclose(fz, 30.0)
    assert math.isclose(mz, 6.0)


def test_axis_clamped_to_percent():
    g = GuiBridgeLogic(surge_gain=0.4)
    g.on_command('arm', True)
    g.on_command('surge', 999)      # di-clamp ke 100
    assert math.isclose(g.wrench()[0], 40.0)


def test_stop_neutralizes_and_disarms():
    g = GuiBridgeLogic()
    g.on_command('arm', True)
    g.on_command('surge', 100)
    act = g.on_command('stop', None)
    assert act['wrench'] == (0.0, 0.0, 0.0, 0.0)
    assert g.armed is False


def test_disarm_neutralizes():
    g = GuiBridgeLogic()
    g.on_command('arm', True)
    g.on_command('surge', 80)
    act = g.on_command('arm', False)
    assert act['wrench'] == (0.0, 0.0, 0.0, 0.0)
    assert g.armed is False


def test_gripper_command_passthrough():
    g = GuiBridgeLogic()
    assert g.on_command('gripper', 'close') == {'gripper': 'close'}
    assert g.on_command('gripper', 'OPEN') == {'gripper': 'open'}
    assert g.on_command('gripper', 'wat') == {}


def test_light_toggle():
    g = GuiBridgeLogic()
    assert g.on_command('light', True) == {'light': True}
    assert g.light is True


def test_heading_wrap():
    assert math.isclose(GuiBridgeLogic.yaw_to_heading_deg(0.0), 0.0)
    assert math.isclose(GuiBridgeLogic.yaw_to_heading_deg(math.pi), 180.0)
    # -90 deg -> 270
    assert math.isclose(GuiBridgeLogic.yaw_to_heading_deg(-math.pi / 2), 270.0)


def test_build_telemetry_shape():
    g = GuiBridgeLogic(mode='manual')
    g.on_command('arm', True)
    t = g.build_telemetry(yaw_rad=math.pi / 2, depth_m=0.6, roll=0.0, pitch=0.0)
    assert math.isclose(t['heading'], 90.0)
    assert math.isclose(t['depth'], 0.6)
    assert t['armed'] is True
    assert set(t) >= {'heading', 'depth', 'roll', 'pitch', 'temp',
                      'voltage', 'armed', 'light', 'mode'}


def test_telemetry_none_safe():
    g = GuiBridgeLogic()
    t = g.build_telemetry()      # semua None
    assert t['heading'] == 0.0 and t['depth'] == 0.0


# ---- hook offset normalization (dipakai APPROACH_HOOK visual servo) ----
def test_hook_offset_centered():
    ex, ey, size = normalize_hook_offset((320, 240), area=1600, frame_w=640, frame_h=480)
    assert math.isclose(ex, 0.0) and math.isclose(ey, 0.0)
    assert math.isclose(size, 40.0 / 640.0)


def test_hook_offset_right_and_down():
    ex, ey, _ = normalize_hook_offset((640, 480), area=100, frame_w=640, frame_h=480)
    assert math.isclose(ex, 1.0) and math.isclose(ey, 1.0)


def test_hook_offset_left_up():
    ex, ey, _ = normalize_hook_offset((0, 0), area=100, frame_w=640, frame_h=480)
    assert math.isclose(ex, -1.0) and math.isclose(ey, -1.0)
