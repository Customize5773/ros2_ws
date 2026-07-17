# Prompt untuk Claude Code: Random QR Payload Spawner (A/B/C/D)

```
Implementasi fitur random QR payload spawner untuk Gazebo Fortress.

Konteks:
- Saat ini payload QR hardcoded di `kki_arena.sdf` dengan tekstur `qr_A.png` di posisi `0.4 0.04 -0.894`.
- FSM (`mission_fsm.py`) menggunakan `payload_x=0.4`, `payload_y=0.0` hardcoded untuk navigasi APPROACH_QR.
- `qr_detector.py` membaca QR dan publish `/hydroships/qr_result` (A/B/C/D).
- Semua file pendukung (gripper, hook, stabilizer, bridge) sudah ada dan bekerja.

Tujuan:
- Payload QR spawn secara random (A/B/C/D) setiap kali sim di-launch.
- Posisi payload bisa diatur via launch argument atau di-randomize dalam bounds arena.
- FSM membaca posisi payload dari topic `/hydroships/payload_pose` sehingga bisa menavigasi ke payload di posisi manapun.

Tugas:

### 1. Buat `src/hydroships_gazebo/scripts/payload_spawner.py`

Node ROS 2 yang melakukan spawn payload model dengan QR random.

```python
#!/usr/bin/env python3
"""payload_spawner — spawn model payload QR secara random di Gazebo Fortress.

Memilih huruf QR (A/B/C/D) dan posisi secara random (atau via parameter),
kemudian spawn model payload menggunakan ros_gz_sim create.
Mempublikasikan posisi spawn ke /hydroships/payload_pose agar FSM bisa
navigasi dinamis.
"""

import os
import random
import string
import tempfile
import subprocess
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped


PAYLOAD_SDF_TEMPLATE = '''<?xml version="1.0"?>
<sdf version="1.9">
  <model name="payload">
    <static>false</static>
    <pose>{x} {y} {z} 1.5708 0 0</pose>
    <link name="payload_link">
      <inertial>
        <mass>0.3</mass>
        <inertia>
          <ixx>3.0e-4</ixx><iyy>3.0e-4</iyy><izz>1.5e-4</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <collision name="body_collision">
        <geometry><box><size>0.05 0.006 0.10</size></box></geometry>
      </collision>
      <visual name="body">
        <geometry>
          <mesh><uri>model://hydroships_gazebo/media/payload_body.obj</uri></mesh>
        </geometry>
        <material>
          <ambient>0.75 0.76 0.80 1</ambient>
          <diffuse>0.82 0.83 0.86 1</diffuse>
        </material>
      </visual>
      <visual name="qr_quiet_zone">
        <pose>0 0.0006 0.04 0 0 0</pose>
        <geometry><plane><normal>0 1 0</normal><size>0.16 0.16</size></plane></geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <specular>0 0 0 1</specular>
          <emissive>1 1 1 1</emissive>
        </material>
      </visual>
      <visual name="qr">
        <pose>0 0.0012 0.04 0 0 0</pose>
        <geometry><plane><normal>0 1 0</normal><size>0.12 0.12</size></plane></geometry>
        <material>
          <diffuse>1 1 1 1</diffuse>
          <specular>0 0 0 1</specular>
          <pbr>
            <metal>
              <albedo_map>model://hydroships_gazebo/media/qr_{letter}.png</albedo_map>
              <emissive_map>model://hydroships_gazebo/media/qr_{letter}.png</emissive_map>
              <metalness>0.0</metalness>
              <roughness>1.0</roughness>
            </metal>
          </pbr>
        </material>
      </visual>
    </link>
  </model>
</sdf>
'''


class PayloadSpawner(Node):
    def __init__(self):
        super().__init__('payload_spawner')
        p = self.declare_parameter
        p('qr_letter', '')          # '' = random A/B/C/D
        p('payload_x', 0.4)         # m, '' atau None = random
        p('payload_y', 0.04)        # m, '' atau None = random
        p('payload_z', -0.894)      # m (on floor)
        p('spawn_delay', 4.0)       # s after sim start
        p('arena_x_min', 0.2)       # m random bounds
        p('arena_x_max', 0.6)
        p('arena_y_min', -1.5)
        p('arena_y_max', 1.5)
        p('qr_media_dir', 'hydroships_gazebo/media')
        g = lambda n: self.get_parameter(n).value

        self.pub_pose = self.create_publisher(PointStamped, '/hydroships/payload_pose', 10)
        self._spawn_delay = float(g('spawn_delay'))
        self._t0 = self._now()
        self._done = False
        self.get_logger().info('payload_spawner siap (delay %.1fs)' % self._spawn_delay)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _random_letter(self):
        return random.choice(['A', 'B', 'C', 'D'])

    def _random_position(self, x_min, x_max, y_min, y_max):
        return random.uniform(x_min, x_max), random.uniform(y_min, y_max)

    def _try_remove_existing(self):
        """Coba hapus model 'payload' yang sudah ada via gz service."""
        try:
            from gz.msgs.entity_factory_pb2 import EntityFactory
            from gz.msgs.empty_pb2 import Empty
            
            # Try gz.msgs.EntityFactory delete via world control
            # Actually, easiest is to try ros2 service call to delete
            # For now, skip — ros_gz_sim create will overwrite or error
            pass
        except Exception:
            pass

    def _spawn(self):
        if self._done:
            return
        self._done = True

        g = lambda n: self.get_parameter(n).value
        letter = str(g('qr_letter')).strip().upper()
        if not letter or letter not in ('A', 'B', 'C', 'D'):
            letter = self._random_letter()

        x = float(g('payload_x'))
        y = float(g('payload_y'))
        z = float(g('payload_z'))
        x_min = float(g('arena_x_min'))
        x_max = float(g('arena_x_max'))
        y_min = float(g('arena_y_min'))
        y_max = float(g('arena_y_max'))

        # Randomize position if using defaults (0.4, 0.04)
        # We detect "default" by checking if the values match typical defaults
        # Better: add explicit parameters randomize_x/y
        # For simplicity: if qr_letter was random, also randomize position
        if not str(g('qr_letter')).strip():
            x, y = self._random_position(x_min, x_max, y_min, y_max)

        sdf = PAYLOAD_SDF_TEMPLATE.format(x=x, y=y, z=z, letter=letter)
        
        tmp = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', suffix='.sdf', delete=False) as f:
                f.write(sdf)
                tmp = f.name

            cmd = [
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-file', tmp,
                '-name', 'payload',
                '-x', str(x), '-y', str(y), '-z', str(z),
            ]
            self.get_logger().info('Spawning payload QR=%s pos=(%.2f, %.2f, %.2f)' % (letter, x, y, z))
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
            if result.returncode != 0:
                self.get_logger().warn('spawn gagal: %s' % result.stderr.strip())
            else:
                self.get_logger().info('Payload QR=%s spawned OK' % letter)
        except Exception as e:
            self.get_logger().warn('spawn exception: %s' % e)
        finally:
            if tmp and os.path.exists(tmp):
                os.unlink(tmp)

        # Publish pose untuk FSM
        ps = PointStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = 'world'
        ps.point.x = float(x)
        ps.point.y = float(y)
        ps.point.z = float(z)
        self.pub_pose.publish(ps)

    def _tick(self):
        if not self._done and self._now() - self._t0 >= self._spawn_delay:
            self._spawn()


def main(args=None):
    rclpy.init(args=args)
    node = PayloadSpawner()
    try:
        timer = node.create_timer(0.5, node._tick)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Catatan penting untuk Claude Code saat menulis file:**
- Gunakan `#!/usr/bin/env python3` shebang
- Jangan ada comment docstring panjang — cukuplah satu baris deskripsi di atas class
- Template SDF harus sama persis dengan definisi payload di `kki_arena.sdf` (mesh, collision, visuals)
- Jika `ros2 run ros_gz_sim create` tidak tersedia, fallback ke service client `gz_sim_msgs/srv/EntityFactory`
- Handle error gracefully — jika spawn gagal, FSM masih bisa pakai default `payload_x/y`

### 2. Modifikasi `src/hydroships_gazebo/worlds/kki_arena.sdf`

- **Hapus** seluruh blok model `payload` (lines 177–257):
  ```xml
  <!-- ================= Payload dgn QR (sesuai gambar KKI) ================= ... -->
  <model name="payload"> ... </model>
  ```
- **Tetap** simpan light `payload_fill` (lines 259–275)
- Tambah comment di tempat payload dihapus: `<!-- Payload di-spawn oleh payload_spawner node -->`

### 3. Modifikasi `src/hydroships_control/hydroships_control/mission_fsm.py`

- **Tambah import:** `from geometry_msgs.msg import PointStamped` (sudah ada)
- **Di `__init__`**, setelah parameter lain:
  ```python
  self.payload_pose = None  # (x, y, z) dari /hydroships/payload_pose
  self.create_subscription(PointStamped, '/hydroships/payload_pose', self._on_payload_pose, 10)
  ```
- **Tambah method `_on_payload_pose`:**
  ```python
  def _on_payload_pose(self, msg):
      self.payload_pose = (msg.point.x, msg.point.y, msg.point.z)
      self.get_logger().info('Payload pose diterima: (%.2f, %.2f, %.2f)' % self.payload_pose)
  ```
- **Modifikasi `_st_approach_qr`:**
  Ganti baris:
  ```python
  dist = self._goto_xy(self.payload_x, self.payload_y)
  ```
  Menjadi:
  ```python
  tx = self.payload_pose[0] if self.payload_pose is not None else self.payload_x
  ty = self.payload_pose[1] if self.payload_pose is not None else self.payload_y
  dist = self._goto_xy(tx, ty)
  ```
- **Update docstring** (line 15–19): tambah `/hydroships/payload_pose` ke daftar input

### 4. Modifikasi `src/hydroships_gazebo/launch/sim.launch.py`

- **Tambah launch arguments** setelah `spawn_delay`:
  ```python
  DeclareLaunchArgument('qr_letter', default_value='',
                        description='Huruf QR payload (A/B/C/D). Kosong = random.'),
  DeclareLaunchArgument('payload_x', default_value='0.4',
                        description='Posisi X payload (m). Kosong = random dalam bounds.'),
  DeclareLaunchArgument('payload_y', default_value='0.04',
                        description='Posisi Y payload (m). Kosong = random dalam bounds.'),
  ```
- **Di `_launch_setup`**, baca argumen:
  ```python
  qr_letter = LaunchConfiguration('qr_letter').perform(context)
  payload_x = LaunchConfiguration('payload_x').perform(context)
  payload_y = LaunchConfiguration('payload_y').perform(context)
  ```
- **Tambah node `payload_spawner`** sebelum return:
  ```python
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
  ```
- Tambahkan `spawner` ke list return: `return [gz_sim, bridge, rsp, spawn, depth_pub, qr, gripper, hook, spawner]`

### 5. Update `docs/HOW-TO-RUN.txt`

- Di bagian "3C. MISI AUTONOMOUS PENUH", tambahkan argumen:
  ```
  ros2 launch hydroships_bringup hydroships_mission.launch.py world:=kki_arena.sdf qr_letter:=B
  ros2 launch hydroships_bringup hydroships_mission.launch.py qr_letter:=C payload_x:=0.5 payload_y:=-1.2
  ```
- Di bagian "5. PERINTAH UJI BERGUNA", tambahkan:
  ```
  ros2 topic echo /hydroships/payload_pose
  ```

### 6. Update `docs/STATUS.md`

- Di tabel M3/M4/M5, tambahkan catatan bahwa QR payload di-spawn random via `payload_spawner`.
- Contoh: "Payload QR di-spawn otomatis oleh node `payload_spawner` (random A/B/C/D + posisi acak dalam bounds arena). Posisi dibaca FSM via `/hydroships/payload_pose`."

### 7. Update `docs/CHANGELOG.md`

- Tambah entri di bagian 2026-07-17 atau 2026-07-18:
  ```
  - **[RESOLVED] Payload QR sekarang di-spawn random (A/B/C/D) via node `payload_spawner`.**
    Model `payload` dihapus dari `kki_arena.sdf` dan diganti spawn dinamis oleh
    `payload_spawner.py`. QR letter dipilih random (atau via launch arg `qr_letter`),
    posisi bisa diatur via `payload_x`/`payload_y` atau di-randomize dalam bounds arena.
    FSM membaca posisi dari `/hydroships/payload_pose` untuk navigasi APPROACH_QR.
  ```

## Build dan Validasi

Jalankan langkah berikut secara berurutan:

1. `cd ~/ros2_ws && colcon build`
2. `source /opt/ros/humble/setup.bash && source install/setup.bash`
3. `pytest src/hydroships_control/test/ -v` (harus 62 passed)
4. `python3 -m py_compile src/hydroships_gazebo/scripts/payload_spawner.py`
5. `python3 -m py_compile src/hydroships_control/hydroships_control/mission_fsm.py`
6. Verifikasi manual:
   ```
   ros2 launch hydroships_bringup hydroships_mission.launch.py world:=kki_arena.sdf qr_letter:=C
   ros2 topic echo /hydroships/payload_pose
   ros2 topic echo /hydroships/qr_result
   ```

## Catatan Penting

- Jika `ros2 run ros_gz_sim create` tidak ditemukan, gunakan fallback service client:
  ```python
  from gz_sim_msgs.srv import EntityFactory
  # atau
  from gz.msgs.entity_factory_pb2 import EntityFactory as EntityFactoryMsg
  ```
  Tapi subprocess `ros2 run ros_gz_sim create` sudah terbukti bekerja di `sim.launch.py`.
- Payload harus non-static agar DetachableJoint bisa mengangkatnya.
- Light `payload_fill` tetap di SDF untuk iluminasi QR.
- Jika spawn gagal, FSM fallback ke `payload_x/payload_y` default (0.4, 0.04).

Setelah selesai, tulis ringkasan perubahan dan file yang diubah.
```
