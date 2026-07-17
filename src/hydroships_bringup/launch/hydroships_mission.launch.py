"""Launch misi autonomous HYDROships (Milestone 6).

Menyalakan: Gazebo + spawn ROV + bridge + thruster_allocator + STABILIZER (M2)
+ mission_fsm (M6). FSM mengendalikan lewat setpoint stabilizer sehingga
kedalaman & heading tertahan otomatis selama manuver.

Contoh:
    ros2 launch hydroships_bringup hydroships_mission.launch.py
    ros2 launch hydroships_bringup hydroships_mission.launch.py start_state:=AUTO_RELEASE
    ros2 launch hydroships_bringup hydroships_mission.launch.py headless:=true

Catatan: SCAN_QR menunggu /hydroships/qr_result (node QR belum ada — lihat PROBLEM.md);
uji tanpa QR bisa dgn start_state:=GRAB/NAV_WALL/... atau publish qr_result manual.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('hydroships_bringup')

    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    start_state = LaunchConfiguration('start_state')
    start_wall = LaunchConfiguration('start_wall')
    qr_letter = LaunchConfiguration('qr_letter')
    payload_x = LaunchConfiguration('payload_x')
    payload_y = LaunchConfiguration('payload_y')
    # Diteruskan ke stabilized -> sim.launch.py (spawn ROV acak dekat dinding / manual).
    rov_args = ('rov_random_spawn', 'rov_x', 'rov_y', 'rov_z',
                'rov_wall_margin', 'rov_arena_half')

    # sim + allocator + stabilizer (M2). Teruskan qr_letter/payload_x/y (payload
    # spawner) + rov_* (spawn ROV) ke sim.launch.py lewat stabilized.
    stab_args = {'headless': headless, 'world': world, 'qr_letter': qr_letter,
                 'payload_x': payload_x, 'payload_y': payload_y}
    stab_args.update({a: LaunchConfiguration(a) for a in rov_args})
    stabilized = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_bringup, 'launch', 'hydroships_stabilized.launch.py'])),
        launch_arguments=stab_args.items(),
    )

    mission = Node(
        package='hydroships_control',
        executable='mission_fsm',
        output='screen',
        parameters=[{'use_sim_time': True, 'start_state': start_state,
                      'start_wall': start_wall}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('world', default_value='kki_arena.sdf'),
        DeclareLaunchArgument('start_state', default_value='DIVE',
                              description='State awal FSM (DIVE/GRAB/NAV_WALL/.../AUTO_RELEASE).'),
        DeclareLaunchArgument('start_wall', default_value='',
                              description='Seed manual wall A/B/C/D utk testing start_state '
                                          'mid-FSM (NAV_WALL/HANG/SURFACE/APPROACH_HOOK/AUTO_RELEASE).'),
        DeclareLaunchArgument('qr_letter', default_value='',
                              description='Huruf QR payload (A/B/C/D). Kosong = random.'),
        DeclareLaunchArgument('payload_x', default_value='0.4',
                              description='Posisi X payload (m); dipakai bila qr_letter di-set.'),
        DeclareLaunchArgument('payload_y', default_value='0.04',
                              description='Posisi Y payload (m); dipakai bila qr_letter di-set.'),
        DeclareLaunchArgument('rov_random_spawn', default_value='true',
                              description='true = spawn ROV acak dekat dinding kolam (kontes); '
                                          'false = pakai rov_x/rov_y/rov_z.'),
        DeclareLaunchArgument('rov_x', default_value='0.0'),
        DeclareLaunchArgument('rov_y', default_value='0.0'),
        DeclareLaunchArgument('rov_z', default_value='-0.5'),
        DeclareLaunchArgument('rov_wall_margin', default_value='0.5'),
        DeclareLaunchArgument('rov_arena_half', default_value='2.55'),
        stabilized,
        mission,
    ])
