"""Uji _st_grab (R-9 ack/retry/timeout) -- node rclpy, pola ikut
test_attitude_estimator_node.py.

Menguji langsung metode _st_grab() (bukan lewat _tick()/timer 10Hz):
publish "close" pertama kali, ack 'attached' -> skor+transisi, ack
'rejected' -> retry, dan timeout T['grab'] tanpa ack -> ABORT.
"""

import math

import pytest
import rclpy

from hydroships_control.mission_fsm import MissionFSM, St


@pytest.fixture
def node():
    rclpy.init()
    n = MissionFSM()
    n._to(St.GRAB)   # reset t_state/_hold_since/gripper_status seperti masuk state nyata
    yield n
    n.destroy_node()
    rclpy.shutdown()


def test_first_tick_publishes_close_no_transition(node, monkeypatch):
    sent = []
    monkeypatch.setattr(node.pub_grip, 'publish', lambda msg: sent.append(msg.data))

    node._st_grab()

    assert sent == ['close']
    assert node.state is St.GRAB
    assert node.score['m2'] == 0


def test_ack_attached_scores_and_transitions(node, monkeypatch):
    monkeypatch.setattr(node.pub_grip, 'publish', lambda msg: None)
    node._st_grab()   # tick pertama: kirim close

    node.gripper_status = 'attached'
    node._st_grab()

    assert node.score['m2'] == 15
    assert node.state is St.NAV_WALL


def test_ack_rejected_retries_close(node, monkeypatch):
    sent = []
    monkeypatch.setattr(node.pub_grip, 'publish', lambda msg: sent.append(msg.data))
    node._st_grab()   # tick pertama: kirim close
    assert sent == ['close']

    node.gripper_status = 'rejected'
    node._st_grab()

    assert sent == ['close', 'close']   # retry terkirim
    assert node.gripper_status is None  # status lama dibuang, tak dianggap ack tersisa
    assert node.state is St.GRAB


def test_timeout_without_ack_aborts(node, monkeypatch):
    monkeypatch.setattr(node.pub_grip, 'publish', lambda msg: None)
    node._st_grab()   # tick pertama: kirim close, set t_state

    node.t_state -= (node.T['grab'] + 1.0)   # simulasikan T['grab'] terlewati
    node._st_grab()

    assert node.state is St.ABORT


# ---- NAV_WALL -> HANG handoff (gate kecepatan; overshoot hook) ----

@pytest.fixture
def nav_node():
    rclpy.init()
    n = MissionFSM()
    n._to(St.NAV_WALL)
    n.wall = 'A'
    n.x, n.y = n._wall_xy(n.wall)
    n.yaw = -math.pi / 2    # menghadap wall A
    yield n
    n.destroy_node()
    rclpy.shutdown()


def test_nav_wall_fast_arrival_brakes_not_hang(nav_node, monkeypatch):
    cmds = []
    monkeypatch.setattr(nav_node.pub_manual, 'publish', lambda m: cmds.append(m))
    nav_node.vx = 0.30   # terukur runtime: 'dist 0.20m, v 0.30m/s' (STATUS.md M5)
    nav_node._vx_filt = nav_node.vx

    nav_node._st_nav_wall()

    assert nav_node.state is St.NAV_WALL   # TIDAK masuk HANG saat masih kencang
    assert cmds[-1].linear.x < 0.0         # rem aktif diterbitkan


def test_nav_wall_slow_arrival_transitions_hang(nav_node):
    nav_node.vx = 0.01

    nav_node._st_nav_wall()

    assert nav_node.state is St.HANG


def test_nav_wall_misaligned_yaw_stays_and_rotates(nav_node):
    # Run 2026-08-26: masuk HANG dgn yaw meleset 25° memicu retreat yg
    # menggeser plat ke struktur hook -> HANG timeout. Rotasi harus
    # selesai di NAV_WALL dulu.
    nav_node.vx = 0.01
    nav_node.yaw = -math.pi / 2 + math.radians(25.0)

    nav_node._st_nav_wall()

    assert nav_node.state is St.NAV_WALL


# ---- HANG fase-2 turun: penerimaan seat via depth-stall ----

def _hang_descent_node():
    rclpy.init()
    n = MissionFSM()
    n._to(St.HANG)
    n.wall = 'A'
    n._hang_pos_done = True          # langsung fase turun
    n.x, n.y = n._hang_xy(n.wall)
    n.yaw = -math.pi / 2
    n.depth = 0.28                   # rentang duduk terdoktrin 0.28..0.32
    n.hold_settle_s = 0.0            # dwell instan di test
    return n


def test_hang_seated_strict_accepts(nav_node=None):
    n = _hang_descent_node()
    try:
        n._st_hang()                 # tick-1: set _hang_depth_max, reset hold
        n._st_hang()                 # tick-2: stall -> strict accept
        assert n.state is St.SURFACE
        assert n.score['m3'] == 15
    finally:
        n.destroy_node(); rclpy.shutdown()


def test_hang_seated_outside_slot_not_scored():
    n = _hang_descent_node()
    n.x -= 0.032                     # dist 32mm > hang_tol 25mm (run 2026-08-26)
    try:
        n._st_hang()
        n._st_hang()
        assert n.state is St.HANG
        assert n.score['m3'] == 0
    finally:
        n.destroy_node(); rclpy.shutdown()


def test_hang_stalled_far_from_target_not_scored():
    n = _hang_descent_node()
    n.x -= 0.10                      # 100mm: nyangkut struktur, bukan threading
    try:
        n._st_hang()
        n._st_hang()
        assert n.state is St.HANG    # jalan terus, tunggu timeout ABORT
        assert n.score['m3'] == 0
    finally:
        n.destroy_node(); rclpy.shutdown()


def test_hang_descent_keeps_position_force_near_hook():
    n = _hang_descent_node()
    n.y += 0.054  # observed drift after lowering the position-force limit
    cmds = []
    n.pub_manual.publish = cmds.append
    try:
        n._st_hang()
        assert cmds[-1].linear.x == pytest.approx(180.0 * 0.054)
        assert n.state is St.HANG
    finally:
        n.destroy_node(); rclpy.shutdown()


# ---- Fase-1 longgar (anti-JAM tip-slot, run 2026-08-26 dist 46mm) ----

def test_hang_phase1_misaligned_stays_above_hook():
    rclpy.init()
    n = MissionFSM()
    try:
        n._to(St.HANG)
        n.wall = 'B'
        tx, ty = n._hang_xy(n.wall)
        n.x, n.y = tx - 0.03, ty + 0.02
        n.yaw = math.pi / 2
        n.depth = 0.13
        n.hold_settle_s = 0.0
        n._st_hang()
        assert n._hang_pos_done is False
    finally:
        n.destroy_node(); rclpy.shutdown()


def test_ar_phase1_misaligned_stays_above_hook():
    rclpy.init()
    n = MissionFSM()
    try:
        n._to(St.AUTO_RELEASE)
        n.wall = 'B'
        tx, ty = n._hang_xy(n.wall)
        n.x, n.y = tx - 0.03, ty + 0.0333
        n.yaw = math.pi / 2
        n.depth = 0.12
        n.hold_settle_s = 0.0
        n._st_auto_release()
        assert n._hang_pos_done is False
        assert n.state is St.AUTO_RELEASE   # lanjut fase turun, bukan timeout
    finally:
        n.destroy_node(); rclpy.shutdown()


def test_ar_phase2_misaligned_does_not_detach():
    rclpy.init()
    n = MissionFSM()
    try:
        n._to(St.AUTO_RELEASE)
        n.wall = 'B'
        n._hang_pos_done = True      # preset SETELAH _to (yang me-reset flag)
        tx, ty = n._hang_xy(n.wall)
        n.x, n.y = tx - 0.03, ty + 0.0333
        n.yaw = math.pi / 2
        n.depth = 0.31               # rentang duduk 0.28..0.32
        n.hold_settle_s = 0.0
        detaches, grips = [], []
        n.pub_detach.publish = lambda m: detaches.append(m)
        n.pub_grip.publish = lambda m: grips.append(m.data)
        n._st_auto_release()         # tick-1: set _hang_depth_max, reset hold
        n._st_auto_release()         # tick-2: stall -> LONGGOR detach
        assert detaches == []
        assert n._detach_sent is False
        assert grips == []
    finally:
        n.destroy_node(); rclpy.shutdown()


def test_ar_phase1_stall_triggers_backoff():
    # Run R5 2026-08-26: fase-1 beku dist 0.155 menekan struktur sampai
    # timeout -- kini 15s tanpa progres harus memicu mundur 2s.
    rclpy.init()
    n = MissionFSM()
    try:
        n._to(St.AUTO_RELEASE)
        n.wall = 'C'
        n.x, n.y = 2.23, 0.05
        n.yaw = 0.0
        n.depth = 0.10
        n.hold_settle_s = 0.0
        cmds = []
        n.pub_manual.publish = lambda m: cmds.append(m)
        n._st_auto_release()             # tick-1: catat dist best
        n._ar_stall_since = n._now() - 16.0   # simulasi 16s tanpa progres
        n._st_auto_release()             # tick-2: stall -> backoff
        assert n._ar_backoff_until is not None
        assert cmds[-1].linear.x < 0     # dorong mundur dari wall
        assert n._hang_pos_done is False
    finally:
        n.destroy_node(); rclpy.shutdown()


@pytest.mark.parametrize('state', [St.HANG, St.AUTO_RELEASE])
@pytest.mark.parametrize('forward_error, accepted', [(0.0005, True), (0.04, False)])
def test_hook_alignment_required_even_after_15_seconds(state, forward_error, accepted):
    rclpy.init()
    n = MissionFSM()
    try:
        n._to(state)
        n.wall = 'B'
        tx, ty = n._hang_xy(n.wall)
        n.x, n.y = tx, ty - forward_error
        n.yaw = math.pi / 2
        n.depth = n.hang_approach_depth
        n.hold_settle_s = 0.0
        n._elapsed = lambda: 20.0
        if state is St.HANG:
            n._st_hang()
        else:
            n._st_auto_release()
        assert n._hang_pos_done is accepted
    finally:
        n.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('state', [St.HANG, St.AUTO_RELEASE])
def test_hook_dwell_resets_when_alignment_is_lost(state):
    rclpy.init()
    n = MissionFSM()
    try:
        n._to(state)
        n.wall = 'B'
        tx, ty = n._hang_xy(n.wall)
        n.x, n.y = tx, ty - 0.04
        n.yaw = math.pi / 2
        n.depth = n.hang_approach_depth
        n._hold_since = n._now() - n.hold_settle_s - 1.0
        step = n._st_hang if state is St.HANG else n._st_auto_release
        step()
        assert n._hang_pos_done is False
        assert n._hold_since is None
        n.y = ty
        step()
        assert n._hang_pos_done is False
        assert n._hold_since is not None
    finally:
        n.destroy_node()
        rclpy.shutdown()


def test_hang_target_uses_fresh_attached_payload_pose(node, monkeypatch):
    monkeypatch.setattr(node, '_now', lambda: 10.0)
    node.x, node.y = 1.0, 2.0
    node._grip_state = 'closed'
    node._hole_world = (1.2, 2.1, -0.3)
    node._hole_received = 10.0
    tx, ty = node._tip_xy('B')
    assert node._hang_xy('B') == pytest.approx((tx - 0.2, ty - 0.1))
    node._hole_received = 9.0
    assert node._hang_xy('B') == pytest.approx((tx, ty - node.hang_hole_dx))


def test_qr_servo_corrects_from_current_pose(node, monkeypatch):
    # A spawn-relative correction can settle at its own biased target while
    # the observed QR remains off-centre forever (run hook-fix/run3).
    node._to(St.APPROACH_QR)
    node.x, node.y, node.yaw = 0.30, 0.04, 0.0
    node.depth = node.scan_depth
    node.payload_x, node.payload_y = 0.4, 0.04
    node.qr_off = (0.2, 0.0, 0.15)
    node.qr_off_time = node._now()
    targets = []
    monkeypatch.setattr(node, '_goto_xy', lambda x, y, **kw: targets.append((x, y)) or 0.1)
    node._st_approach_qr()
    from hydroships_control.qr_logic import qr_ey_target
    ey = qr_ey_target(node.depth, node.cam_gripper_dx, node.qr_floor_z,
                      node.cam_bottom_dz, node.cam_vfov_half_tan, node.ey_target_max)
    assert targets[-1] == pytest.approx((node.x + ey * node.qr_servo_gain,
                                        node.y - 0.2 * node.qr_servo_gain))
    assert node.state is St.APPROACH_QR


def test_no_decoded_qr_never_selects_a_random_wall(node, monkeypatch):
    node._to(St.APPROACH_QR)
    node.x, node.y, node.yaw = node.payload_x - node.gripper_base_dx, node.payload_y, 0.
    node.depth = node.scan_depth
    node._approach_recovered = True
    node._recovery_started_at = node._now() - 10.
    monkeypatch.setattr(node, '_elapsed', lambda: node.t_nav_qr + 5.)
    node._st_approach_qr()
    assert node.state is St.APPROACH_QR
    assert node.wall is None
    assert node.score['m1'] == 0
    monkeypatch.setattr(node, '_elapsed', lambda: node.T['scan'] + 1.)
    node._st_approach_qr()
    assert node.state is St.ABORT


def test_new_cycle_discards_old_qr_and_heading(node):
    node.qr_wall, node.qr_off, node._locked_yaw = 'A', (0., 0., 0.2), 1.57
    node._to(St.DIVE)
    assert node.qr_wall is None
    assert node.qr_off is None
    assert node._locked_yaw is None


@pytest.mark.parametrize('retries_left', [True, False])
def test_release_retry_has_a_new_time_budget(node, monkeypatch, retries_left):
    monkeypatch.setattr(node, '_now', lambda: 1000.)
    node._to(St.AUTO_RELEASE)
    node.wall = 'B'
    node._hang_pos_done = True
    node.x, node.y = node._hang_xy('B')
    node.x += 0.10
    node.yaw = math.pi / 2
    node.depth = node.hook_depth
    node.t_state -= node.T['release'] + 1.
    if not retries_left:
        node._release_retries = node.release_max_retries
    node._st_auto_release()
    if retries_left:
        assert node.state is St.AUTO_RELEASE
        assert not node._hang_pos_done
        assert node._release_retries == 1
        assert node._elapsed() == 0.
        assert node._hold_since is None
    else:
        assert node.state is St.ABORT


@pytest.mark.parametrize('shutdown, message, raises', [
    (True, 'Unable to convert call argument to Python object', False),
    (False, 'Unable to convert call argument to Python object', True),
    (True, 'unrelated error', True),
])
def test_main_only_ignores_known_dds_error_after_shutdown(monkeypatch, shutdown, message, raises):
    from hydroships_control import mission_fsm
    def spin(_node):
        if shutdown:
            rclpy.shutdown()
        raise RuntimeError(message)
    monkeypatch.setattr(rclpy, 'spin', spin)
    if raises:
        with pytest.raises(RuntimeError, match=message):
            mission_fsm.main()
    else:
        mission_fsm.main()
    assert not rclpy.ok()
