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
    n.x, n.y = 0.0, -2.15   # persis standoff wall A -> dist 0 < nav_tol
    n.yaw = -math.pi / 2    # menghadap wall A
    yield n
    n.destroy_node()
    rclpy.shutdown()


def test_nav_wall_fast_arrival_brakes_not_hang(nav_node, monkeypatch):
    cmds = []
    monkeypatch.setattr(nav_node.pub_manual, 'publish', lambda m: cmds.append(m))
    nav_node.vx = 0.30   # terukur runtime: 'dist 0.20m, v 0.30m/s' (STATUS.md M5)

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
    n.x, n.y = 0.0, -2.0967          # target _hang_xy wall A
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


def test_hang_seated_loose_accepts_with_warning(caplog):
    n = _hang_descent_node()
    n.x -= 0.032                     # dist 32mm > hang_tol 25mm (run 2026-08-26)
    try:
        n._st_hang()
        n._st_hang()
        assert n.state is St.SURFACE
        assert n.score['m3'] == 15
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


# ---- Fase-1 longgar (anti-JAM tip-slot, run 2026-08-26 dist 46mm) ----

def test_hang_phase1_loose_accepts():
    rclpy.init()
    n = MissionFSM()
    try:
        n._to(St.HANG)
        n.wall = 'B'
        n.x, n.y = -0.03, 2.13       # 40mm dr target (0,2.0967); dulu ABORT
        n.yaw = math.pi / 2
        n.depth = 0.13
        n.hold_settle_s = 0.0
        n._st_hang()
        assert n._hang_pos_done is True
    finally:
        n.destroy_node(); rclpy.shutdown()


def test_ar_phase1_loose_accepts():
    rclpy.init()
    n = MissionFSM()
    try:
        n._to(St.AUTO_RELEASE)
        n.wall = 'B'
        n.x, n.y = -0.03, 2.13       # log user: macet dist 46mm -> timeout posisi
        n.yaw = math.pi / 2
        n.depth = 0.12
        n.hold_settle_s = 0.0
        n._st_auto_release()
        assert n._hang_pos_done is True
        assert n.state is St.AUTO_RELEASE   # lanjut fase turun, bukan timeout
    finally:
        n.destroy_node(); rclpy.shutdown()


def test_ar_phase2_loose_seated_detaches():
    rclpy.init()
    n = MissionFSM()
    try:
        n._to(St.AUTO_RELEASE)
        n.wall = 'B'
        n._hang_pos_done = True      # preset SETELAH _to (yang me-reset flag)
        n.x, n.y = -0.03, 2.13       # duduk geser 40mm -- tetap threading valid
        n.yaw = math.pi / 2
        n.depth = 0.31               # rentang duduk 0.28..0.32
        n.hold_settle_s = 0.0
        detaches, grips = [], []
        n.pub_detach.publish = lambda m: detaches.append(m)
        n.pub_grip.publish = lambda m: grips.append(m.data)
        n._st_auto_release()         # tick-1: set _hang_depth_max, reset hold
        n._st_auto_release()         # tick-2: stall -> LONGGOR detach
        assert len(detaches) == 1
        assert n._detach_sent is True
        assert grips == ['open']
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
