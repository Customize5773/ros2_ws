"""Launch top-level ROV HYDROships + jembatan GUI tim (Milestone 7).

Menyalakan: Gazebo Fortress + world + spawn ROV + ros_gz_bridge (via
sim.launch.py) + thruster_allocator + node adapter `gui_bridge`.

`gui_bridge` membuat sim ROS 2 tampak seperti ROV ArduSub bagi repo GUI tim
(Customize5773/GUI-ROV), yang memakai UDP-JSON/MAVLink (BUKAN ROS 2 — lihat
docs/GUI-INTEGRATION.md). Alurnya:

    GUI dashboard  ──UDP JSON {name,value}──►  gui_bridge  ──► /hydroships/cmd_vel
                                                              ──► /hydroships/gripper/command
    GUI dashboard  ◄──UDP JSON telemetri────  gui_bridge  ◄── /hydroships/odom, /depth

Joystick GUI (persen surge/sway/yaw/heave) diterjemahkan jadi wrench body di
/hydroships/cmd_vel -> thruster_allocator (jalur sama teleop_keyboard). Node inti
(stabilizer, mission_fsm, thruster_allocator) TIDAK diubah.

CATATAN [VERIFY]: integrasi GUI live belum diuji end-to-end; gain/tanda/port
masih estimasi (lihat PROBLEM.md seksi "Integrasi GUI tim (M7)").

Contoh:
    ros2 launch hydroships_bringup hydroships_gui.launch.py
    # arahkan telemetri ke laptop GUI (server.js) di IP tertentu:
    ros2 launch hydroships_bringup hydroships_gui.launch.py \
        gui_host:=192.168.2.1 cmd_port:=14550 telem_port:=14551
    ros2 launch hydroships_bringup hydroships_gui.launch.py headless:=true
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('hydroships_gazebo')

    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    gui_host = LaunchConfiguration('gui_host')
    cmd_port = LaunchConfiguration('cmd_port')
    telem_port = LaunchConfiguration('telem_port')

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

    # Adapter GUI-ROV (UDP-JSON) <-> ROS. Port default cocok dgn GUI
    # (UDP_CMD_PORT 14550, telemetri 14551). gui_host = tujuan telemetri (server.js).
    gui_bridge = Node(
        package='hydroships_control',
        executable='gui_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            # LaunchConfiguration -> string; koersi ke int agar cocok tipe param node.
            'cmd_port': ParameterValue(cmd_port, value_type=int),
            'telem_host': ParameterValue(gui_host, value_type=str),
            'telem_port': ParameterValue(telem_port, value_type=int),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('world', default_value='kki_arena.sdf'),
        DeclareLaunchArgument('gui_host', default_value='127.0.0.1',
                              description='IP tujuan telemetri (laptop GUI/server.js).'),
        DeclareLaunchArgument('cmd_port', default_value='14550',
                              description='UDP port dengar perintah GUI (UDP_CMD_PORT).'),
        DeclareLaunchArgument('telem_port', default_value='14551',
                              description='UDP port tujuan telemetri (server.js).'),
        sim,
        allocator,
        gui_bridge,
    ])
