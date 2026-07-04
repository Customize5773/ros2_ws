"""Launch simulasi Gazebo Fortress + spawn ROV HYDROships + ros_gz_bridge.

Argumen:
  headless (default: false)  -> jalankan gz sim tanpa GUI (server saja) untuk CI/cloud.
  world    (default: pool_empty.sdf)
  x,y,z    (default: 0 0 -0.5) -> posisi spawn ROV (sedikit di bawah permukaan).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
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

    world_path = os.path.join(pkg_gazebo, 'worlds', world)

    # -r: mulai berjalan; -s: server saja (tanpa GUI) untuk mode headless.
    gz_args = '-r -s ' + world_path if headless else '-r ' + world_path
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    # Proses xacro -> URDF string.
    xacro_file = os.path.join(pkg_description, 'urdf', 'hydroships.urdf.xacro')
    robot_desc = xacro.process_file(xacro_file).toxml()

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'hydroships',
            '-string', robot_desc,
            '-x', x, '-y', y, '-z', z,
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

    return [gz_sim, bridge, rsp, spawn]


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
        OpaqueFunction(function=_launch_setup),
    ])
