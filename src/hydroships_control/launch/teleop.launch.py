"""Jalankan allocator + teleop keyboard (tanpa Gazebo)."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    allocator = Node(
        package='hydroships_control',
        executable='thruster_allocator',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    # teleop butuh terminal interaktif -> jalankan manual dengan `ros2 run`.
    # Node di sini hanya allocator; teleop dijalankan terpisah oleh operator.
    return LaunchDescription([allocator])
