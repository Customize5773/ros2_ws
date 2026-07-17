"""Launch simulasi Gazebo Fortress + spawn ROV HYDROships + ros_gz_bridge.

Argumen:
  headless (default: false)  -> jalankan gz sim tanpa GUI (server saja) untuk CI/cloud.
  world    (default: pool_empty.sdf)
  x,y,z    (default: 0 0 -0.5) -> posisi spawn ROV (sedikit di bawah permukaan).
"""

import os

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

    qr_letter = LaunchConfiguration('qr_letter').perform(context)
    payload_x = LaunchConfiguration('payload_x').perform(context)
    payload_y = LaunchConfiguration('payload_y').perform(context)

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

    # Manipulator (M5 rancang ulang): open/close -> gz DetachableJoint attach/detach.
    gripper = Node(
        package='hydroships_control',
        executable='gripper_controller',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Deteksi hook (port GUI-ROV) -> /hydroships/hook_offset (visual servo APPROACH_HOOK).
    hook = Node(
        package='hydroships_control',
        executable='hook_detector',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Spawner payload QR random (A/B/C/D): spawn model payload lewat ros_gz_sim
    # create + publikasi posisi ke /hydroships/payload_pose. Delay > spawn ROV agar
    # server gz & model sudah siap. Bila qr_letter/payload_x/y kosong → random.
    spawner = Node(
        package='hydroships_gazebo',
        executable='payload_spawner',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'qr_letter': qr_letter,
            'payload_x': float(payload_x) if payload_x else 0.4,
            'payload_y': float(payload_y) if payload_y else 0.04,
            'spawn_delay': spawn_delay + 1.0,
        }],
    )

    return [gz_sim, bridge, rsp, spawn, depth_pub, qr, gripper, hook, spawner]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='kki_arena.sdf',
                              description='Nama file world di folder worlds/ '
                                          '(kki_arena.sdf = arena lomba; pool_empty.sdf = kolam kosong).'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='true = server saja tanpa GUI (cloud/CI).'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='-0.5'),
        DeclareLaunchArgument('spawn_delay', default_value='3.0',
                              description='Detik menunda spawn ROV agar server gz '
                                          'siap dulu (naikkan bila mesin lambat).'),
        DeclareLaunchArgument('qr_letter', default_value='',
                              description='Huruf QR payload (A/B/C/D). Kosong = random.'),
        DeclareLaunchArgument('payload_x', default_value='0.4',
                              description='Posisi X payload (m); dipakai bila qr_letter di-set.'),
        DeclareLaunchArgument('payload_y', default_value='0.04',
                              description='Posisi Y payload (m); dipakai bila qr_letter di-set.'),
        OpaqueFunction(function=_launch_setup),
    ])
