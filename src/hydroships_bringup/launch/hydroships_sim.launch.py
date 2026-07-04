"""Launch top-level ROV HYDROships (Milestone 1).

Menyalakan: Gazebo Fortress + world kolam + spawn ROV + ros_gz_bridge
(via sim.launch.py) lalu node thruster_allocator.

Teleop keyboard dijalankan terpisah karena butuh terminal interaktif:
    ros2 run hydroships_control teleop_keyboard

Contoh:
    ros2 launch hydroships_bringup hydroships_sim.launch.py
    ros2 launch hydroships_bringup hydroships_sim.launch.py headless:=true
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('hydroships_gazebo')

    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo, 'launch', 'sim.launch.py'])),
        launch_arguments={'headless': headless, 'world': world}.items(),
    )

    allocator = Node(
        package='hydroships_control',
        executable='thruster_allocator',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('world', default_value='kki_arena.sdf'),
        sim,
        allocator,
    ])
