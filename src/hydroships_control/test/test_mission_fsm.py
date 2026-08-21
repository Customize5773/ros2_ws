"""Uji _st_grab (R-9 ack/retry/timeout) -- node rclpy, pola ikut
test_attitude_estimator_node.py.

Menguji langsung metode _st_grab() (bukan lewat _tick()/timer 10Hz):
publish "close" pertama kali, ack 'attached' -> skor+transisi, ack
'rejected' -> retry, dan timeout T['grab'] tanpa ack -> ABORT.
"""

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
