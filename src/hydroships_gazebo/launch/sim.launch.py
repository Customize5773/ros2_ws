"""Launch simulasi Gazebo Fortress + spawn ROV HYDROships + ros_gz_bridge.

Argumen:
  headless (default: false)  -> jalankan gz sim tanpa GUI (server saja) untuk CI/cloud.
  world    (default: pool_practice_arena.sdf) -> kolam latihan 2,2x4,4x0,8 m.
      Ganti ke kki_arena.sdf (5x5 m) atau pool_empty.sdf (kolam kosong) bila perlu.
  rov_random_spawn (default: true) -> spawn ROV acak DEKAT salah satu dinding kolam
      (posisi kontes realistis & bervariasi tiap run). false = pakai rov_x/y/z.
  rov_x,rov_y,rov_z (default: 0 0 -0.5) -> posisi manual bila rov_random_spawn=false.
  rov_wall_margin (default: 0.5) -> jarak aman ROV dari dinding fisik (+-rov_arena_half).
  rov_arena_half  (default: 1.1) -> setengah lebar kolam (dinding di +-nilai ini).
      1.1 = setengah SISI PENDEK kolam latihan (2,2 m); dipakai utk kedua sumbu
      (asumsi arena persegi) jadi sengaja dipilih sisi pendek supaya ROV tak
      pernah spawn tembus dinding meski kolam defaultnya persegi panjang.
      Naikkan ke 2.55 bila world diganti ke kki_arena.sdf (5x5 m).
  spawn_seed      (default: '')   -> isi utk fix seed (replay/debug), kosong = acak penuh.
"""

import math
import os
import random
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro


def _world_name(world_path):
    """Nama world dari ISI file SDF (<world name="...">), bukan dari nama file.

    Keduanya TIDAK selalu sama, dan `create -world <nama>` memakai nama yang
    dideklarasikan di SDF: kalau salah, model gagal spawn TANPA error yang
    kelihatan. Di repo ini pool_empty.sdf memakai <world name="pool"> dan
    kki_arena_test.sdf memakai <world name="kki_arena">, jadi hanya kki_arena.sdf
    yang kebetulan cocok dgn nama file-nya.

    Fallback ke nama file (perilaku lama) bila file tak terbaca / tanpa <world>.
    """
    stem = os.path.splitext(os.path.basename(world_path))[0]
    try:
        world = ET.parse(world_path).getroot().find('world')
        name = world.get('name') if world is not None else None
    except (ET.ParseError, OSError) as exc:
        print('[sim.launch] WARNING: gagal membaca world name dari %s (%s); '
              'pakai nama file "%s".' % (world_path, exc, stem))
        return stem
    if not name:
        print('[sim.launch] WARNING: %s tidak punya <world name=...>; '
              'pakai nama file "%s".' % (world_path, stem))
        return stem
    return name


def _f(context, name, default):
    """Ambil LaunchConfiguration sbg float; fallback ke default bila kosong/invalid."""
    v = LaunchConfiguration(name).perform(context).strip()
    try:
        return float(v)
    except (ValueError, AttributeError):
        return default


# Heading (rad) yg menghadap KE DALAM kolam dari tiap dinding. Konvensi dinding
# sama dgn mission_fsm._wall_inward: A=-Y, B=+Y, C=+X, D=-X, jadi arah masuk
# kolamnya berlawanan: dari A menghadap +Y, dari B menghadap -Y, dst.
_WALL_INWARD_YAW = {
    'A': 0.5 * math.pi,     # nempel dinding -Y -> hadap +Y
    'B': -0.5 * math.pi,    # nempel dinding +Y -> hadap -Y
    'C': math.pi,           # nempel dinding +X -> hadap -X
    'D': 0.0,               # nempel dinding -X -> hadap +X
}

# Sebaran acak heading (rad) di sekitar arah masuk kolam, biar tiap run tidak
# persis tegak lurus dinding tapi tetap tidak membelakangi arena.
_YAW_JITTER = 0.35


def _rov_spawn_pose(context, rng):
    """Kembalikan (x, y, z, yaw) string utk spawn ROV.

    rov_random_spawn=true -> acak DEKAT salah satu dari 4 dinding (A/B/C/D): axis
    yg menempel dinding di +-(arena_half - margin), koordinat lain tersebar acak
    sepanjang dinding dalam rentang aman yg sama, heading menghadap ke dalam kolam
    (+- _YAW_JITTER). false -> pakai rov_x/rov_y/rov_z dgn yaw 0.
    z selalu dari rov_z (kedalaman aman, default -0.5, di bawah permukaan)."""
    z = _f(context, 'rov_z', -0.5)
    random_spawn = LaunchConfiguration('rov_random_spawn').perform(context).strip().lower() == 'true'
    if not random_spawn:
        return (str(_f(context, 'rov_x', 0.0)), str(_f(context, 'rov_y', 0.0)), str(z), '0.0')

    arena_half = _f(context, 'rov_arena_half', 1.1)
    margin = _f(context, 'rov_wall_margin', 0.5)
    lim = max(0.0, arena_half - margin)          # koordinat aman maks (mepet dinding)
    along = rng.uniform(-lim, lim)               # sebaran sepanjang dinding
    wall = rng.choice(('A', 'B', 'C', 'D'))
    # Konvensi sama dgn mission_fsm._wall_inward: A=-Y, B=+Y, C=+X, D=-X.
    if wall == 'A':      x, y = along, -lim
    elif wall == 'B':    x, y = along, lim
    elif wall == 'C':    x, y = lim, along
    else:                x, y = -lim, along      # D
    yaw = _WALL_INWARD_YAW[wall] + rng.uniform(-_YAW_JITTER, _YAW_JITTER)
    return (str(round(x, 3)), str(round(y, 3)), str(z), str(round(yaw, 4)))


def _spawn_rng(context):
    """RNG utk pose spawn. spawn_seed diisi -> reproducible (replay/debug);
    kosong (default) -> acak penuh tiap launch."""
    seed = LaunchConfiguration('spawn_seed').perform(context).strip()
    if not seed:
        return random.Random()
    try:
        return random.Random(int(seed))
    except ValueError:
        return random.Random(seed)          # terima seed non-numerik jg


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
    x, y, z, yaw = _rov_spawn_pose(context, _spawn_rng(context))
    _random = LaunchConfiguration('rov_random_spawn').perform(context).strip().lower() == 'true'
    print('[sim.launch] ROV spawn (random=%s) di (%s, %s, %s) yaw=%s rad'
          % (_random, x, y, z, yaw))

    qr_letter = LaunchConfiguration('qr_letter').perform(context)
    payload_x = LaunchConfiguration('payload_x').perform(context)
    payload_y = LaunchConfiguration('payload_y').perform(context)
    payload_z = LaunchConfiguration('payload_z').perform(context)

    # P2-B: opsional suntik noise ke odom (docs/ARCHITECTURE.md).
    odom_noise = LaunchConfiguration('odom_noise').perform(context).strip().lower() == 'true'

    # R-8: opsional suntik dropout frame kamera (docs/P1-OWNER-DECISIONS-AND-ROADMAP.md).
    camera_dropout = LaunchConfiguration('camera_dropout').perform(context).strip().lower() == 'true'

    world_path = os.path.join(pkg_gazebo, 'worlds', world)
    world_name = _world_name(world_path)
    print('[sim.launch] world file "%s" -> world name "%s"' % (world, world_name))

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
    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', world_name,
            '-name', 'hydroships',
            '-string', robot_desc,
            '-x', x, '-y', y, '-z', z,
            '-Y', yaw,
        ],
    )

    # Spawn yang gagal TIDAK menghentikan launch: gz tetap jalan, bridge tetap
    # hidup, hanya ROV-nya yang tak ada dan /hydroships/odom diam. Kegagalan
    # sunyi seperti itu pernah menghabiskan satu eksperimen penuh, jadi
    # kode-keluar 'create' dilaporkan keras-keras.
    spawn_check = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_node,
            on_exit=lambda event, ctx: (
                [] if event.returncode == 0 else
                [LogInfo(msg='[sim.launch] ERROR: spawn ROV GAGAL (create keluar '
                             'dgn kode %s). World "%s" mungkin tak ada di %s — '
                             'cek <world name=...> di file world tsb. Gazebo '
                             'tetap jalan TANPA ROV; /hydroships/odom tidak akan '
                             'terbit.' % (event.returncode, world_name, world))]
            ),
        )
    )

    spawn = TimerAction(
        period=spawn_delay,
        actions=[
            spawn_node,
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

    # P2-B: relay /hydroships/odom_raw (ground truth) -> /hydroships/odom,
    # opsional +noise (odom_noise:=true). Selalu jalan (satu jalur kode, bukan
    # cabang launch terpisah) -- default noise mati = passthrough identik.
    odom_inj = Node(
        package='hydroships_control',
        executable='odom_injector',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'odom_noise': odom_noise,
            'pos_noise_std': _f(context, 'odom_pos_noise_std', 0.03),
            'vel_noise_std': _f(context, 'odom_vel_noise_std', 0.02),
            'heading_noise_std_deg': _f(context, 'odom_heading_noise_std_deg', 1.0),
            'noise_seed': int(_f(context, 'odom_noise_seed', 0.0)),
        }],
    )

    # R-8: relay /hydroships/camera_{front,bottom}/image_raw_gt (ground truth) ->
    # image_raw, opsional drop frame (camera_dropout:=true). Sama pola dgn odom_inj:
    # selalu jalan, default mati = passthrough identik.
    camera_dropout_inj = Node(
        package='hydroships_control',
        executable='camera_dropout_injector',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'camera_dropout': camera_dropout,
            'drop_prob': _f(context, 'camera_drop_prob', 0.05),
            'dropout_seed': int(_f(context, 'camera_dropout_seed', 0.0)),
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
            'payload_z': float(payload_z) if payload_z else -0.80,
            'spawn_delay': spawn_delay + 0.5,
        }],
    )

    return [gz_sim, bridge, rsp, spawn_check, spawn, odom_inj, camera_dropout_inj,
            depth_pub, qr, gripper, hook, spawner]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='pool_practice_arena.sdf',
                              description='Nama file world di folder worlds/ '
                                          '(pool_practice_arena.sdf = kolam latihan 2,2x4,4x0,8 m default; '
                                          'kki_arena.sdf = arena lomba 5x5 m; pool_empty.sdf = kolam kosong).'),
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
        DeclareLaunchArgument('spawn_seed', default_value='',
                              description='Isi utk fix seed pose spawn acak '
                                          '(replay/debug); kosong = acak penuh tiap launch.'),
        DeclareLaunchArgument('rov_arena_half', default_value='1.1',
                              description='Setengah lebar kolam (dinding di +-nilai ini). '
                                          '1.1 = setengah sisi pendek kolam latihan 2,2x4,4 m; '
                                          'naikkan ke 2.55 bila world diganti ke kki_arena.sdf (5x5 m).'),
        DeclareLaunchArgument('spawn_delay', default_value='3.0',
                              description='Detik menunda spawn ROV agar server gz '
                                          'siap dulu (naikkan bila mesin lambat).'),
        DeclareLaunchArgument('qr_letter', default_value='',
                              description='Huruf QR payload (A/B/C/D). Kosong = random.'),
        DeclareLaunchArgument('payload_x', default_value='0.4',
                              description='Posisi X payload (m); dipakai bila qr_letter di-set.'),
        DeclareLaunchArgument('payload_y', default_value='0.04',
                              description='Posisi Y payload (m); dipakai bila qr_letter di-set.'),
        DeclareLaunchArgument('payload_z', default_value='-0.80',
                              description='Posisi Z payload (m), harus = lantai kolam (top floor). '
                                          '-0.80 = lantai kolam latihan (default). '
                                          'Naikkan ke -0.90 bila world:=kki_arena.sdf (arena lomba, lantai -0.9).'),
        DeclareLaunchArgument('odom_noise', default_value='false',
                              description='P2-B: true -> /hydroships/odom disuntik noise '
                                          '(bukan ground truth langsung). false = passthrough.'),
        DeclareLaunchArgument('odom_pos_noise_std', default_value='0.03',
                              description='m, std white noise posisi odom (sigma 0.02-0.05).'),
        DeclareLaunchArgument('odom_vel_noise_std', default_value='0.02',
                              description='m/s, std white noise kecepatan odom (sigma 0.01-0.03).'),
        DeclareLaunchArgument('odom_heading_noise_std_deg', default_value='1.0',
                              description='deg/√s, std random-walk drift heading odom (0.5-2.0).'),
        DeclareLaunchArgument('odom_noise_seed', default_value='0',
                              description='0 = noise acak penuh; isi utk reproducible.'),
        DeclareLaunchArgument('camera_dropout', default_value='false',
                              description='R-8: true -> frame kamera didrop acak '
                                          '(simulasi dropout). false = passthrough.'),
        DeclareLaunchArgument('camera_drop_prob', default_value='0.05',
                              description='Peluang drop per frame kamera (0..1).'),
        DeclareLaunchArgument('camera_dropout_seed', default_value='0',
                              description='0 = dropout acak penuh; isi utk reproducible.'),
        OpaqueFunction(function=_launch_setup),
    ])
