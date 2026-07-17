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
    qr_letter = LaunchConfiguration('qr_letter')
    payload_x = LaunchConfiguration('payload_x')
    payload_y = LaunchConfiguration('payload_y')
    # Diteruskan apa adanya ke sim.launch.py (spawn ROV acak dekat dinding / manual).
    rov_args = ('rov_random_spawn', 'rov_x', 'rov_y', 'rov_z',
                'rov_wall_margin', 'rov_arena_half')

    gains = os.path.join(pkg_control, 'config', 'gains.yaml')

    sim_args = {'headless': headless, 'world': world, 'qr_letter': qr_letter,
                'payload_x': payload_x, 'payload_y': payload_y}
    sim_args.update({a: LaunchConfiguration(a) for a in rov_args})
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo, 'launch', 'sim.launch.py'])),
        launch_arguments=sim_args.items(),
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
        DeclareLaunchArgument('qr_letter', default_value='',
                              description='Huruf QR payload (A/B/C/D). Kosong = random.'),
        DeclareLaunchArgument('payload_x', default_value='0.4',
                              description='Posisi X payload (m); dipakai bila qr_letter di-set.'),
        DeclareLaunchArgument('payload_y', default_value='0.04',
                              description='Posisi Y payload (m); dipakai bila qr_letter di-set.'),
        DeclareLaunchArgument('rov_random_spawn', default_value='true',
                              description='true = spawn ROV acak dekat dinding kolam (kontes).'),
        DeclareLaunchArgument('rov_x', default_value='0.0'),
        DeclareLaunchArgument('rov_y', default_value='0.0'),
        DeclareLaunchArgument('rov_z', default_value='-0.5'),
        DeclareLaunchArgument('rov_wall_margin', default_value='0.5'),
        DeclareLaunchArgument('rov_arena_half', default_value='2.55'),
        sim,
        allocator,
        stabilizer,
    ])
