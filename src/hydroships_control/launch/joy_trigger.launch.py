"""Jalankan joy_node + joy_mission_trigger (tombol joystick utk WAIT_TRIGGER).

Sudah otomatis ter-include oleh hydroships_mission.launch.py (arg
joy_trigger:=true). Dapat juga dijalankan manual:

    # default tombol A (index 0)
    ros2 launch hydroships_control joy_trigger.launch.py

    # ganti tombol (mis. B = index 1)
    ros2 launch hydroships_control joy_trigger.launch.py button_index:=1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    button_index = LaunchConfiguration('button_index')
    declare_button = DeclareLaunchArgument(
        'button_index', default_value='0',
        description='Index tombol joystick (0 = A/Cross, 1 = B/Circle pada '
                    'XInput/F310).')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        output='screen',
        parameters=[{'deadzone': 0.0}],
    )
    joy_mission_trigger = Node(
        package='hydroships_control',
        executable='joy_mission_trigger',
        output='screen',
        parameters=[{'use_sim_time': True, 'button_index': button_index}],
    )
    return LaunchDescription([declare_button, joy_node, joy_mission_trigger])
