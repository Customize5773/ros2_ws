"""Launch simulasi Gazebo Fortress + spawn ROV HYDROships + ros_gz_bridge.

Argumen:
  headless (default: false)  -> jalankan gz sim tanpa GUI (server saja) untuk CI/cloud.
  world    (default: pool_empty.sdf)
  x,y,z    (default: 0 0 -0.5) -> posisi spawn ROV (dipakai kalau randomize_spawn:=false).
  randomize_spawn (default: true) -> pose spawn acak (x,y,yaw) tiap launch.
  spawn_radius    (default: 0.8)  -> radius acak (m) dari pusat kolam.
  spawn_seed      (default: '')   -> isi utk fix seed (replay/debug), kosong = acak penuh.
"""

import math
import os
import random

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro


def _launch_setup(context, *args, **kwargs):
    pkg_gazebo = get_package_share_directory('hydroships_gazebo')
    pkg_description = get_package_share_directory('hydroships_description')

    world = LaunchConfiguration('world').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    z = LaunchConfiguration('z').perform(context)
    try:
        spawn_delay = float(LaunchConfiguration('spawn_delay').perform(context))
    except ValueError:
        spawn_delay = 3.0

    # Pose spawn ACAK (x,y,yaw) tiap launch, biar pilot latihan dari titik
    # berbeda-beda. Dibatasi radius aman dari tengah kolam (jauh dari 4
    # dinding/hook). seed bisa di-fix via arg spawn_seed utk replay run
    # tertentu (debug/testing); default kosong = benar-benar acak.
    randomize = LaunchConfiguration('randomize_spawn').perform(context).lower() == 'true'
    if randomize:
        seed_str = LaunchConfiguration('spawn_seed').perform(context)
        rng = random.Random(int(seed_str)) if seed_str else random.Random()
        half = float(LaunchConfiguration('spawn_radius').perform(context))
        x = str(rng.uniform(-half, half))
        y = str(rng.uniform(-half, half))
        yaw = rng.uniform(-math.pi, math.pi)
        print('[sim.launch.py] spawn acak: x=%s y=%s z=%s yaw=%.2f rad'
              % (x, y, z, yaw))
    else:
        yaw = 0.0

    world_path = os.path.join(pkg_gazebo, 'worlds', world)

    # Agar mesh 'package://hydroships_description/...' (di-resolve gz jadi
    # 'model://hydroships_description/...') ketemu: tambah folder share ke
    # resource path Gazebo (Fortress: IGN_GAZEBO_RESOURCE_PATH).
    for pkg in (pkg_description, pkg_gazebo):
        share_dir = os.path.dirname(pkg)  # .../install/<pkg>/share
        for var in ('IGN_GAZEBO_RESOURCE_PATH', 'GZ_SIM_RESOURCE_PATH'):
            cur = os.environ.get(var, '')
            if share_dir not in cur.split(os.pathsep):
                os.environ[var] = share_dir + (os.pathsep + cur if cur else '')

    # -r: mulai berjalan; -s: server saja (tanpa GUI) untuk mode headless.
    gz_args = '-r -s ' + world_path if headless else '-r ' + world_path
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    # Proses xacro -> URDF string. (hydroships.urdf.xacro)
    xacro_file = os.path.join(pkg_description, 'urdf', 'hydroships.urdf.xacro')
    robot_desc = xacro.process_file(xacro_file).toxml()

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
    )

    # Spawn ROV via ros_gz_sim 'create'. DITUNDA {spawn_delay}s dgn TimerAction
    # supaya server gz (dari gz_sim di atas) sudah menyediakan service
    # /world/<world>/create; kalau spawn jalan sebelum server siap, model gagal
    # muncul (race condition). Atur lewat arg spawn_delay (naikkan bila mesin lambat).
    spawn = TimerAction(
        period=spawn_delay,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                output='screen',
                arguments=[
                    '-name', 'hydroships',
                    '-string', robot_desc,
                    '-x', x, '-y', y, '-z', z,
                    '-Y', str(yaw),
                ],
            ),
        ],
    )

    payload_model_path = os.path.join(
        get_package_share_directory('hydroships_gazebo'),
        'models', 'payload', 'model.sdf')

    bottom_offset = 0.3   # m, turun dari center ROV (sesuaikan sampai pas)
    px = float(x)
    py = float(y)
    pz = float(z) - bottom_offset

    payload_spawn = TimerAction(
        period=spawn_delay + 2.0,   # setelah ROV ada, biar DetachableJoint nemu gripper_link
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                output='screen',
                arguments=[
                    '-name', 'payload',
                    '-file', payload_model_path,
                    '-x', str(px), '-y', str(py), '-z', str(pz),
                    '-R', '1.5708', '-P', '0.0', '-Y', str(yaw),
                ],
            ),
        ],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': os.path.join(pkg_gazebo, 'config', 'bridge.yaml'),
            'use_sim_time': True,
        }],
    )

    # Kedalaman (M3) diturunkan dari odom -> /hydroships/depth (Float64).
    depth_pub = Node(
        package='hydroships_control',
        executable='depth_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Deteksi QR dari kamera bawah -> /hydroships/qr_result (A/B/C/D) (M3 persepsi).
    qr = Node(
        package='hydroships_control',
        executable='qr_detector',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return [gz_sim, bridge, rsp, spawn, payload_spawn, depth_pub, qr]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='kki_arena.sdf',
                              description='Nama file world di folder worlds/ '
                                          '(kki_arena.sdf = arena lomba; pool_empty.sdf = kolam kosong).'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='true = server saja tanpa GUI (cloud/CI).'),
        DeclareLaunchArgument('x', default_value='0.0',
                              description='Dipakai kalau randomize_spawn:=false.'),
        DeclareLaunchArgument('y', default_value='0.0',
                              description='Dipakai kalau randomize_spawn:=false.'),
        DeclareLaunchArgument('z', default_value='-0.5'),
        DeclareLaunchArgument('spawn_delay', default_value='3.0',
                              description='Detik menunda spawn ROV agar server gz '
                                          'siap dulu (naikkan bila mesin lambat).'),
        DeclareLaunchArgument('randomize_spawn', default_value='true',
                              description='true = pose spawn (x,y,yaw) acak tiap launch.'),
        DeclareLaunchArgument('spawn_radius', default_value='2.0',
                              description='Radius acak (m) dari pusat kolam.'),
        DeclareLaunchArgument('spawn_seed', default_value='',
                              description='Isi utk fix seed random (replay/debug); '
                                          'kosong = acak penuh tiap launch.'),
        OpaqueFunction(function=_launch_setup),
    ])