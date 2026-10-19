"""A rejected physical grasp must never acknowledge an attachment."""
import pytest
import rclpy
from std_msgs.msg import String
from hydroships_control.gripper_controller import GripperController


@pytest.fixture
def gripper():
    rclpy.init()
    n = GripperController()
    n._rov_pose = (0., 0., -0.70, 0., 0., 0., 1.)
    n.logic.update_offset(0., 0., 0.2, n._now())
    n.logic.update_altitude(0.03)
    yield n
    n.destroy_node()
    rclpy.shutdown()


def test_xy_rejection_does_not_latch_attached(gripper, monkeypatch):
    gripper._payload_pose = (2., 2., -0.9)
    status, attach = [], []
    monkeypatch.setattr(gripper.pub_status, 'publish', lambda m: status.append(m.data))
    monkeypatch.setattr(gripper.pub_attach, 'publish', attach.append)
    gripper._on_cmd(String(data='close'))
    gripper._on_cmd(String(data='close'))
    assert status == ['rejected', 'rejected']
    assert not gripper.logic.attached
    assert not attach


@pytest.mark.parametrize('success', [True, False])
def test_pose_service_failure_rejects_attach(gripper, monkeypatch, success):
    status, attach = [], []
    gripper._payload_pose = (0.18, 0., -0.9)
    monkeypatch.setattr(gripper, '_teleport_payload_to_gripper', lambda: success)
    monkeypatch.setattr(gripper.pub_status, 'publish', lambda m: status.append(m.data))
    monkeypatch.setattr(gripper.pub_attach, 'publish', attach.append)
    gripper._on_cmd(String(data='close'))
    assert status == ['attached' if success else 'rejected']
    assert gripper.logic.attached is success
    assert len(attach) == int(success)


def test_gripper_arms_from_camera_not_synthetic_offset(gripper):
    from geometry_msgs.msg import PointStamped
    gripper.logic._armed_t = None
    gripper.logic._offset = None
    m = PointStamped()
    m.point.z = 0.2
    m.header.frame_id = 'synthetic'
    gripper._on_offset(m)
    assert gripper.logic._armed_t is None
    m.header.frame_id = 'camera_bottom_link'
    gripper._on_offset(m)
    assert gripper.logic._armed_t is not None
