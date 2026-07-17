"""Launch simulasi Gazebo Fortress + spawn ROV HYDROships + ros_gz_bridge.

Argumen:
  headless (default: false)  -> jalankan gz sim tanpa GUI (server saja) untuk CI/cloud.
  world    (default: pool_empty.sdf)
  rov_random_spawn (default: true) -> spawn ROV acak DEKAT salah satu dinding kolam
      (posisi kontes realistis & bervariasi tiap run). false = pakai rov_x/y/z.
  rov_x,rov_y,rov_z (default: 0 0 -0.5) -> posisi manual bila rov_random_spawn=false.
  rov_wall_margin (default: 0.5) -> jarak aman ROV dari dinding fisik (+-rov_arena_half).
  rov_arena_half  (default: 2.55) -> setengah lebar kolam (dinding di +-nilai ini).
"""

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


def _f(context, name, default):
    """Ambil LaunchConfiguration sbg float; fallback ke default bila kosong/invalid."""
    v = LaunchConfiguration(name).perform(context).strip()
    try:
        return float(v)
    except (ValueError, AttributeError):
        return default


def _rov_spawn_xyz(context):
    """Kembalikan (x, y, z) string utk spawn ROV.

    rov_random_spawn=true -> acak DEKAT salah satu dari 4 dinding (A/B/C/D): axis
    yg menempel dinding di +-(arena_half - margin), koordinat lain tersebar acak
    sepanjang dinding dalam rentang aman yg sama. false -> pakai rov_x/rov_y/rov_z.
    z selalu dari rov_z (kedalaman aman, default -0.5, di bawah permukaan)."""
    z = _f(context, 'rov_z', -0.5)
    random_spawn = LaunchConfiguration('rov_random_spawn').perform(context).strip().lower() == 'true'
    if not random_spawn:
        return (str(_f(context, 'rov_x', 0.0)), str(_f(context, 'rov_y', 0.0)), str(z))

    arena_half = _f(context, 'rov_arena_half', 2.55)
    margin = _f(context, 'rov_wall_margin', 0.5)
    lim = max(0.0, arena_half - margin)          # koordinat aman maks (mepet dinding)
    along = random.uniform(-lim, lim)            # sebaran sepanjang dinding
    wall = random.choice(('A', 'B', 'C', 'D'))
    # Konvensi sama dgn mission_fsm._wall_inward: A=-Y, B=+Y, C=+X, D=-X.
    if wall == 'A':      x, y = along, -lim
    elif wall == 'B':    x, y = along, lim
    elif wall == 'C':    x, y = lim, along
    else:                x, y = -lim, along      # D
    return (str(round(x, 3)), str(round(y, 3)), str(z))


def _launch_setup(context, *args, **kwargs):
    pkg_gazebo = get_package_share_directory('hydroships_gazebo')
    pkg_description = get_package_share_directory('hydroships_description')

    world = LaunchConfiguration('world').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    try:
        spawn_delay = float(LaunchConfiguration('spawn_delay').perform(context))
    except ValueError:
        spawn_delay = 3.0

    # Posisi spawn ROV: acak DEKAT dinding (kontes) atau manual via rov_x/y/z.
    x, y, z = _rov_spawn_xyz(context)
    _random = LaunchConfiguration('rov_random_spawn').perform(context).strip().lower() == 'true'
    print('[sim.launch] ROV spawn (random=%s) di (%s, %s, %s)' % (_random, x, y, z))

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
    # create + publikasi posisi ke /hydroships/payload_pose + sinyal
    # /hydroships/payload/spawned (memicu gripper detach SETELAH payload ada).
    # Delay > spawn ROV (server gz & model ROV siap) tapi kecil agar payload muncul
    # lebih awal (pose dipublish segera; urutan attach/detach dijaga oleh topik
    # spawned, bukan timing). Bila qr_letter/payload_x/y kosong → random.
    spawner = Node(
        package='hydroships_gazebo',
        executable='payload_spawner',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'qr_letter': qr_letter,
            'payload_x': float(payload_x) if payload_x else 0.4,
            'payload_y': float(payload_y) if payload_y else 0.04,
            'spawn_delay': spawn_delay + 0.5,
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
        DeclareLaunchArgument('rov_random_spawn', default_value='true',
                              description='true = spawn ROV acak dekat salah satu '
                                          'dinding kolam (kontes); false = pakai rov_x/y/z.'),
        DeclareLaunchArgument('rov_x', default_value='0.0',
                              description='Posisi X spawn ROV (m) bila rov_random_spawn=false.'),
        DeclareLaunchArgument('rov_y', default_value='0.0',
                              description='Posisi Y spawn ROV (m) bila rov_random_spawn=false.'),
        DeclareLaunchArgument('rov_z', default_value='-0.5',
                              description='Kedalaman spawn ROV (m, negatif = di bawah permukaan).'),
        DeclareLaunchArgument('rov_wall_margin', default_value='0.5',
                              description='Jarak aman ROV dari dinding fisik (+-rov_arena_half).'),
        DeclareLaunchArgument('rov_arena_half', default_value='2.55',
                              description='Setengah lebar kolam (dinding di +-nilai ini).'),
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
