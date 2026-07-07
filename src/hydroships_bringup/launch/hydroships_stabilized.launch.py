"""Launch top-level HYDROships dengan STABILISASI (Milestone 2).

Menyalakan: Gazebo + spawn + bridge + thruster_allocator + stabilizer
(depth-hold & heading-hold). Pilot mengemudi horizontal via teleop_stabilized
di terminal terpisah:

    ros2 run hydroships_control teleop_stabilized

Contoh:
    ros2 launch hydroships_bringup hydroships_stabilized.launch.py
    ros2 launch hydroships_bringup hydroships_stabilized.launch.py headless:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('hydroships_gazebo')
    pkg_control = get_package_share_directory('hydroships_control')

    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')

    gains = os.path.join(pkg_control, 'config', 'gains.yaml')

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

    # use_sim_time WAJIB True: stabilizer memakai turunan waktu (PID d-term &
    # laju setpoint). Tanpa ini node jalan di wall-clock sementara sim di sim-time
    # -> timing kacau. Node lain semua sudah sim-time; ini sebelumnya terlewat.
    stabilizer = Node(
        package='hydroships_control',
        executable='stabilizer',
        output='screen',
        parameters=[gains, {'use_sim_time': True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('world', default_value='kki_arena.sdf'),
        sim,
        allocator,
        stabilizer,
    ])
