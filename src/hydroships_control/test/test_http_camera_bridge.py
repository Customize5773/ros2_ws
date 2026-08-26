"""Uji http_camera_bridge_logic (murni, tanpa rclpy)."""

import pytest

from hydroships_control.http_camera_bridge_logic import FrameStore, to_bgr


def test_rgb8_swaps_to_bgr():
    # 1 piksel: R=10,G=20,B=30 (rgb8) -> BGR harus (30,20,10)
    out = to_bgr(1, 1, 'rgb8', bytes([10, 20, 30]))
    assert out.shape == (1, 1, 3)
    assert list(out[0, 0]) == [30, 20, 10]


def test_bgr8_passthrough():
    out = to_bgr(1, 1, 'bgr8', bytes([1, 2, 3]))
    assert list(out[0, 0]) == [1, 2, 3]


def test_mono8_replicated_to_3_channels():
    out = to_bgr(1, 1, 'mono8', bytes([42]))
    assert out.shape == (1, 1, 3)
    assert list(out[0, 0]) == [42, 42, 42]


def test_unsupported_encoding_raises():
    with pytest.raises(ValueError):
        to_bgr(1, 1, 'yuv422', bytes([0, 0]))


def test_frame_store_get_missing_returns_none():
    s = FrameStore()
    assert s.get('cam_bottom') is None


def test_frame_store_update_then_get():
    s = FrameStore()
    s.update('cam_bottom', b'\xff\xd8fake')
    assert s.get('cam_bottom') == b'\xff\xd8fake'
