"""Spawn coordinates must match the pose advertised to the mission."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import pytest
import rclpy
from rclpy.parameter import Parameter

path = Path(__file__).resolve().parents[2] / 'hydroships_gazebo/scripts/payload_spawner.py'
spec = importlib.util.spec_from_file_location('payload_spawner', path)
spawner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(spawner)


@pytest.mark.parametrize('success', [True, False])
def test_spawn_passes_pose_to_create_before_advertising(monkeypatch, success):
    rclpy.init()
    node = spawner.PayloadSpawner()
    try:
        node.set_parameters([Parameter('payload_x', value=0.5),
                             Parameter('payload_y', value=-1.2)])
        commands, advertised = [], []
        def run(cmd, **kwargs):
            commands.append(cmd)
            assert node._pose is None
            return SimpleNamespace(returncode=0 if success else 1, stderr='test failure')
        monkeypatch.setattr(spawner.subprocess, 'run', run)
        monkeypatch.setattr(node.pub_pose, 'publish', advertised.append)
        node._spawn('C')
        cmd = commands[0]
        for flag, value in [('-x', 0.5), ('-y', -1.2), ('-z', -0.9), ('-R', 1.5708)]:
            assert float(cmd[cmd.index(flag) + 1]) == pytest.approx(value)
        assert bool(advertised) is success
        if success:
            p = advertised[0].point
            assert (p.x, p.y, p.z) == pytest.approx((0.5, -1.2, -0.9))
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('shutdown', [True, False])
def test_spawn_main_propagates_runtime_errors_except_shutdown(monkeypatch, shutdown):
    def spin(_node):
        if shutdown:
            rclpy.shutdown()
        raise RuntimeError('Unable to convert call argument to Python object')
    monkeypatch.setattr(rclpy, 'spin', spin)
    if shutdown:
        spawner.main()
    else:
        with pytest.raises(RuntimeError):
            spawner.main()
    assert not rclpy.ok()
