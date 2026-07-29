#!/usr/bin/env python3
"""Tampilkan model CAD ROV KKI 2026 di RViz (tanpa Gazebo).

Dipakai untuk memeriksa hasil konversi CAD: bentuk mesh, posisi frame
thruster/sensor/gripper, dan TF tree. Tidak menjalankan fisika apa pun.

  ros2 launch hydroships_description display_kki2026.launch.py

Argumen:
model            urdf/rov_kki2026_new_design.urdf.xacro (default, CAD mesh)
  markers          true -> tampilkan bola penanda di tiap frame referensi.
  collision_mode   hull (default) | box — lihat catatan APUNG di xacro.
  gui              true (default) -> joint_state_publisher_gui utk gripper.
  rviz             true (default) -> jalankan RViz.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (Command, LaunchConfiguration, PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

PKG = 'hydroships_description'


def generate_launch_description():
    model = LaunchConfiguration('model')
    markers = LaunchConfiguration('markers')
    collision_mode = LaunchConfiguration('collision_mode')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')

    # xacro dipanggil dgn argumen supaya markers/collision bisa diubah dari CLI
    # tanpa menyunting file. Properti di xacro dibaca lewat mekanisme yg sama
    # (xacro:arg) bila kelak diperlukan; saat ini nilai default xacro dipakai.
    robot_description = Command([
        'xacro ', PathJoinSubstitution([FindPackageShare(PKG), model]),
        ' show_frame_markers:=', markers,
        ' collision_mode:=', collision_mode,
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'model', default_value='urdf/rov_kki2026_new_design.urdf.xacro',
            description='path xacro relatif thd share/hydroships_description'),
        DeclareLaunchArgument('markers', default_value='true',
                              description='tampilkan penanda frame referensi'),
        DeclareLaunchArgument('collision_mode', default_value='hull',
                              description='hull | box'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen',
             parameters=[{'robot_description': robot_description}]),

        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             condition=IfCondition(gui), output='screen'),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             condition=IfCondition(['not ', gui]), output='screen'),

        Node(package='rviz2', executable='rviz2', name='rviz2',
             condition=IfCondition(rviz), output='screen'),
    ])
