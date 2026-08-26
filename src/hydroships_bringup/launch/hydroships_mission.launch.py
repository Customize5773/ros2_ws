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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_bringup = get_package_share_directory('hydroships_bringup')
    pkg_control = get_package_share_directory('hydroships_control')

    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    start_state = LaunchConfiguration('start_state')
    start_wall = LaunchConfiguration('start_wall')
    qr_letter = LaunchConfiguration('qr_letter')
    payload_x = LaunchConfiguration('payload_x')
    payload_y = LaunchConfiguration('payload_y')
    payload_z = LaunchConfiguration('payload_z')
    # Diteruskan ke stabilized -> sim.launch.py (spawn ROV acak dekat dinding / manual).
    rov_args = ('rov_random_spawn', 'rov_x', 'rov_y', 'rov_z',
                'rov_wall_margin', 'rov_arena_half', 'spawn_seed',
                'odom_noise', 'odom_pos_noise_std', 'odom_vel_noise_std',
                'odom_heading_noise_std_deg', 'odom_noise_seed')

    # sim + allocator + stabilizer (M2). Teruskan qr_letter/payload_x/y/z (payload
    # spawner) + rov_* (spawn ROV) ke sim.launch.py lewat stabilized.
    stab_args = {'headless': headless, 'world': world, 'qr_letter': qr_letter,
                 'payload_x': payload_x, 'payload_y': payload_y, 'payload_z': payload_z}
    stab_args.update({a: LaunchConfiguration(a) for a in rov_args})
    stabilized = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_bringup, 'launch', 'hydroships_stabilized.launch.py'])),
        launch_arguments=stab_args.items(),
    )

    # Joy trigger utk melewati WAIT_TRIGGER: tombol joystick (default A =
    # index 0) mempublish Empty ke /hydroships/mission/start_autonomous,
    # yang dibaca mission_fsm di state WAIT_TRIGGER. Matikan dgn
    # joy_trigger:=false utk run battery/headless tanpa joystick.
    joy_trigger = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_control, 'launch', 'joy_trigger.launch.py'])),
        launch_arguments={'button_index': LaunchConfiguration('joy_button_index')}.items(),
        condition=IfCondition(LaunchConfiguration('joy_trigger')),
    )

    mission = Node(
        package='hydroships_control',
        executable='mission_fsm',
        output='screen',
        parameters=[{'use_sim_time': True, 'start_state': start_state,
                      'start_wall': start_wall,
                      'scan_depth': ParameterValue(LaunchConfiguration('scan_depth'),
                                                   value_type=float),
                      'descend_depth_tol': ParameterValue(
                          LaunchConfiguration('descend_depth_tol'), value_type=float),
                      'cam_gripper_dx': ParameterValue(
                          LaunchConfiguration('cam_gripper_dx'), value_type=float),
                      'hook_size_stop': ParameterValue(
                          LaunchConfiguration('hook_size_stop'), value_type=float),
                      'hook_center_tol': ParameterValue(
                          LaunchConfiguration('hook_center_tol'), value_type=float),
                      'hook_max_age': ParameterValue(
                          LaunchConfiguration('hook_max_age'), value_type=float),
                      't_approach': ParameterValue(
                          LaunchConfiguration('t_approach'), value_type=float),
                      'qr_offset_ema_alpha': ParameterValue(
                          LaunchConfiguration('qr_offset_ema_alpha'), value_type=float),
                      'qr_servo_range': ParameterValue(
                          LaunchConfiguration('qr_servo_range'), value_type=float),
                      'approach_min_fmax_frac': ParameterValue(
                          LaunchConfiguration('approach_min_fmax_frac'), value_type=float),
                      'approach_dwell_ticks': ParameterValue(
                          LaunchConfiguration('approach_dwell_ticks'), value_type=int),
                      'hang_approach_depth': ParameterValue(
                          LaunchConfiguration('hang_approach_depth'), value_type=float)}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('joy_trigger', default_value='true',
                              description='true: jalankan joy_node + joy_mission_trigger '
                                          '(tombol joystick utk lewati WAIT_TRIGGER); '
                                          'false: tanpa joystick (battery/headless).'),
        DeclareLaunchArgument('joy_button_index', default_value='0',
                              description='Index tombol joystick utk trigger '
                                          'WAIT_TRIGGER (0 = A/Cross pada XInput/F310).'),
        DeclareLaunchArgument('world', default_value='pool_practice_arena.sdf'),
        DeclareLaunchArgument('start_state', default_value='DIVE',
                              description='State awal FSM (DIVE/GRAB/NAV_WALL/HANG/SURFACE/'
                                          'WAIT_TRIGGER/APPROACH_HOOK/AUTO_RELEASE).'),
        DeclareLaunchArgument('start_wall', default_value='',
                              description='Seed manual wall A/B/C/D utk testing start_state '
                                          'mid-FSM (NAV_WALL/HANG/SURFACE/APPROACH_HOOK/AUTO_RELEASE).'),
        DeclareLaunchArgument('qr_letter', default_value='',
                              description='Huruf QR payload (A/B/C/D). Kosong = random.'),
        DeclareLaunchArgument('payload_x', default_value='0.4',
                              description='Posisi X payload (m); dipakai bila qr_letter di-set.'),
        DeclareLaunchArgument('payload_y', default_value='0.04',
                              description='Posisi Y payload (m); dipakai bila qr_letter di-set.'),
        DeclareLaunchArgument('payload_z', default_value='-0.80',
                              description='Posisi Z payload (m), harus = lantai kolam. '
                                          'Naikkan ke -0.90 bila world:=kki_arena.sdf (arena lomba).'),
        DeclareLaunchArgument('rov_random_spawn', default_value='true',
                              description='true = spawn ROV acak dekat dinding kolam (kontes); '
                                          'false = pakai rov_x/rov_y/rov_z.'),
        DeclareLaunchArgument('rov_x', default_value='0.0'),
        DeclareLaunchArgument('rov_y', default_value='0.0'),
        DeclareLaunchArgument('rov_z', default_value='-0.5'),
        DeclareLaunchArgument('rov_wall_margin', default_value='0.5'),
        DeclareLaunchArgument('rov_arena_half', default_value='1.1'),
        DeclareLaunchArgument('spawn_seed', default_value='',
                              description='Isi utk fix seed pose spawn acak '
                                          '(replay/debug); kosong = acak penuh tiap launch.'),
        DeclareLaunchArgument('odom_noise', default_value='false',
                              description='P2-B: true -> /hydroships/odom disuntik noise.'),
        DeclareLaunchArgument('odom_pos_noise_std', default_value='0.03'),
        DeclareLaunchArgument('odom_vel_noise_std', default_value='0.02'),
        DeclareLaunchArgument('odom_heading_noise_std_deg', default_value='1.0'),
        DeclareLaunchArgument('odom_noise_seed', default_value='0'),
        # Tuning kamera bawah: kedalaman scan menentukan lebar petak pandang
        # (h_cam = 0.714 - scan_depth), jadi menentukan pula seberapa besar QR di
        # frame DAN apakah offset gripper masih muat. Lihat komentar scan_depth
        # di mission_fsm.py. cam_gripper_dx=0.0 mengembalikan perilaku lama
        # (QR dipusatkan di kamera, bukan di gripper) untuk pembanding A/B.
        DeclareLaunchArgument('scan_depth', default_value='0.30',
                              description='Kedalaman scan QR (m). Naikkan angka = '
                                          'lebih dalam = QR lebih besar tapi petak '
                                          'pandang menyempit.'),
        DeclareLaunchArgument('cam_gripper_dx', default_value='0.16',
                              description='Jarak gripper di depan kamera bawah (m). '
                                          '0.0 = perilaku lama (tanpa koreksi).'),
        # R-10 (P1-OWNER-DECISIONS-AND-ROADMAP.md): toleransi exit DESCEND. Default
        # 0.02 = perilaku BARU (celah alt_gap lebih besar). 0.06 = perilaku LAMA
        # (dulu exit DESCEND pakai depth_tol yang sama dgn APPROACH_QR) -- dipakai
        # utk battery pembanding sebelum/sesudah.
        DeclareLaunchArgument('descend_depth_tol', default_value='0.02',
                              description='Toleransi kedalaman (m) exit DESCEND->GRAB. '
                                          '0.06 = replikasi perilaku lama (R-10 pembanding).'),
        # Tuning APPROACH_HOOK (visual servo hook lewat kamera depan). Naikkan
        # hook_size_stop = berhenti lebih dekat ke hook; turunkan hook_center_tol
        # = tuntut pemusatan lebih ketat (butuh deteksi lebih stabil).
        DeclareLaunchArgument('hook_size_stop', default_value='0.35',
                              description='Ukuran-tampak hook (sqrt(area)/lebar frame) '
                                          'yg dianggap "cukup dekat".'),
        DeclareLaunchArgument('hook_center_tol', default_value='0.15',
                              description='Toleransi |ex|,|ey| ternormalisasi utk '
                                          '"hook terpusat".'),
        DeclareLaunchArgument('hook_max_age', default_value='1.0',
                              description='Umur maks deteksi hook (s) sebelum '
                                          'APPROACH_HOOK jatuh ke target odometri.'),
        DeclareLaunchArgument('t_approach', default_value='25.0',
                              description='Timeout APPROACH_HOOK (s); habis waktu = '
                                          'lanjut AUTO_RELEASE, bukan abort.'),
        # P0-2.5 Candidate #2 (docs/P0-2-5-ENGINEERING-ANALYSIS.md): EMA pada
        # qr_ex/qr_ey sebelum dipakai target servo. 1.0 = filter nonaktif
        # (default, sama seperti sebelum kandidat ini ada).
        DeclareLaunchArgument('qr_offset_ema_alpha', default_value='1.0',
                              description='EMA alpha utk qr_ex/qr_ey sebelum servo '
                                          '(1.0=nonaktif/mentah, lebih kecil=lebih halus).'),
        # P0-2.5 Kandidat #1: lebar gerbang aktivasi visual servo (dist_raw <
        # qr_servo_range). 0.3 = nilai lama/default.
        DeclareLaunchArgument('qr_servo_range', default_value='0.3',
                              description='Jarak (m) di bawah mana visual servo QR mulai '
                                          'aktif (dist_raw < qr_servo_range).'),
        # P0-2.5 Kandidat #3: lantai fraksi gaya taper _goto_xy, KHUSUS
        # APPROACH_QR (_st_hang/_st_nav_wall tetap 0.05 hardcoded).
        DeclareLaunchArgument('approach_min_fmax_frac', default_value='0.05',
                              description='Lantai fraksi approach_fmax dalam radius '
                                          'slow-down 1.0m, khusus APPROACH_QR.'),
        # P0-2.5 Kandidat #4: jumlah tick berturut-turut kondisi convergen
        # harus bertahan sebelum GRAB dipicu. 1 = nilai lama/default (tanpa dwell).
        DeclareLaunchArgument('approach_dwell_ticks', default_value='1',
                              description='Tick dwell (10Hz) sebelum transisi GRAB '
                                          'benar2 dipicu di APPROACH_QR.'),
        DeclareLaunchArgument('hang_approach_depth', default_value='0.14',
                              description='m kedalaman posisi lubang di atas tip '
                                          'saat HANG/APPROACH_HOOK/AUTO_RELEASE fase 1 '
                                          '(dieksos utk kalibrasi per-wall).'),
        stabilized,
        mission,
        joy_trigger,
    ])
