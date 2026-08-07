"""Jalankan joy_node + teleop_gamepad (Logitech F310).

Contoh:
    # bersama hydroships_stabilized.launch.py (stabilizer aktif) - default
    ros2 launch hydroships_control teleop_gamepad.launch.py

    # bersama hydroships_sim.launch.py (tanpa stabilizer)
    ros2 launch hydroships_control teleop_gamepad.launch.py stabilized:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('hydroships_control'), 'config', 'gamepad.yaml')

    stabilized = LaunchConfiguration('stabilized')
    declare_stabilized = DeclareLaunchArgument(
        'stabilized', default_value='true',
        description='true: perintah lewat stabilizer (/hydroships/manual/cmd). '
                    'false: langsung ke /hydroships/cmd_vel (bringup tanpa stabilizer).')

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
        parameters=[config, {'route_through_stabilizer': stabilized,
                             'use_sim_time': True}],
    )
    return LaunchDescription([declare_stabilized, joy_node, teleop_gamepad])
