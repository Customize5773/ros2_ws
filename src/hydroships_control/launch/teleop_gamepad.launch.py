"""Jalankan joy_node + teleop_gamepad (Logitech F310)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('hydroships_control'), 'config', 'gamepad.yaml')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        output='screen',
        # deadzone diterapkan di teleop_gamepad (lihat gamepad.yaml), bukan di sini,
        # supaya tidak dobel deadzone.
        parameters=[{'deadzone': 0.0}],
    )
    teleop_gamepad = Node(
        package='hydroships_control',
        executable='teleop_gamepad',
        output='screen',
        parameters=[config],
    )
    return LaunchDescription([joy_node, teleop_gamepad])
