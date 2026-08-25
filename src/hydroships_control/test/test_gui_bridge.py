"""Uji logika adapter GUI-ROV <-> ROS (murni, tanpa ROS/UDP)."""

import math

from hydroships_control.gui_bridge_logic import DelayLine, GuiBridgeLogic, parse_extra_dests
from hydroships_control.hook_logic import normalize_hook_offset


def test_parse_extra_dests_empty():
    assert parse_extra_dests('') == []
    assert parse_extra_dests(None) == []


def test_parse_extra_dests_single():
    assert parse_extra_dests('127.0.0.1:14552') == [('127.0.0.1', 14552)]


def test_parse_extra_dests_multiple_and_whitespace():
    assert parse_extra_dests(' 127.0.0.1:14552 , 10.0.0.5:9999 ') == [
        ('127.0.0.1', 14552), ('10.0.0.5', 9999)]


def test_delay_line_zero_delay_passthrough():
    d = DelayLine(0.0)
    d.push('a', now=10.0)
    assert d.pop_ready(now=10.0) == ['a']


def test_delay_line_holds_until_release():
    d = DelayLine(1.0)
    d.push('a', now=10.0)
    assert d.pop_ready(now=10.5) == []
    assert d.pop_ready(now=10.9) == []
    assert d.pop_ready(now=11.0) == ['a']


def test_delay_line_fifo_order():
    d = DelayLine(1.0)
    d.push('a', now=10.0)
    d.push('b', now=10.2)
    d.push('c', now=10.4)
    assert d.pop_ready(now=11.5) == ['a', 'b', 'c']


def test_disarmed_wrench_zero():
    g = GuiBridgeLogic()
    g.on_command('surge', 1000)
    # belum armed -> gerak nol (failsafe)
    assert g.wrench() == (0.0, 0.0, 0.0, 0.0)


def test_armed_axis_to_wrench():
    # value dalam skala kawat GUI-ROV (-1000..1000, lihat server.js clampAxis).
    g = GuiBridgeLogic(surge_gain=0.4, sway_gain=0.4, heave_gain=0.3, yaw_gain=0.12)
    g.on_command('arm', True)
    g.on_command('surge', 1000)
    g.on_command('sway', -500)
    g.on_command('heave', 1000)
    g.on_command('yaw', 500)
    fx, fy, fz, mz = g.wrench()
    assert math.isclose(fx, 40.0)
    assert math.isclose(fy, -20.0)
    assert math.isclose(fz, 30.0)
    assert math.isclose(mz, 6.0)


def test_axis_clamped_to_percent():
    g = GuiBridgeLogic(surge_gain=0.4)
    g.on_command('arm', True)
    g.on_command('surge', 9999)      # di luar rentang kawat -> tetap clamp ke 100%
    assert math.isclose(g.wrench()[0], 40.0)


def test_stop_neutralizes_and_disarms():
    g = GuiBridgeLogic()
    g.on_command('arm', True)
    g.on_command('surge', 1000)
    act = g.on_command('stop', None)
    assert act['wrench'] == (0.0, 0.0, 0.0, 0.0)
    assert g.armed is False


def test_disarm_neutralizes():
    g = GuiBridgeLogic()
    g.on_command('arm', True)
    g.on_command('surge', 800)
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
    assert t['mission_state'] == 'IDLE'
    assert t['qr_result'] == '' and t['gripper_status'] == '' and t['gripper_state'] == ''
    assert t['hook_ex'] == 0.0 and t['hook_fresh'] is False


def test_start_abort_mission_commands():
    g = GuiBridgeLogic()
    assert g.on_command('start_mission', None) == {'mission_start': True}
    assert g.on_command('abort_mission', None) == {'mission_abort': True}


def test_mission_state_drives_mode():
    g = GuiBridgeLogic()
    assert g.build_telemetry(mission_state='DIVE')['mode'] == 'auto'
    for s in (None, 'IDLE', 'DONE', 'ABORT'):
        assert g.build_telemetry(mission_state=s)['mode'] == 'manual'


def test_build_telemetry_new_fields_passthrough():
    g = GuiBridgeLogic()
    t = g.build_telemetry(mission_state='GRAB', qr_result='B',
                          hook_offset=(0.1, -0.2, 0.3), hook_age=0.5,
                          gripper_status='attached', gripper_state='closed')
    assert t['mission_state'] == 'GRAB'
    assert t['qr_result'] == 'B'
    assert math.isclose(t['hook_ex'], 0.1)
    assert math.isclose(t['hook_ey'], -0.2)
    assert math.isclose(t['hook_size'], 0.3)
    assert t['hook_fresh'] is True
    assert t['gripper_status'] == 'attached'
    assert t['gripper_state'] == 'closed'


def test_hook_freshness_threshold():
    g = GuiBridgeLogic()
    fresh = g.build_telemetry(hook_offset=(0.1, 0.2, 0.3), hook_age=0.5)
    stale = g.build_telemetry(hook_offset=(0.1, 0.2, 0.3), hook_age=5.0)
    assert fresh['hook_fresh'] is True
    assert stale['hook_fresh'] is False


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
