# Plan: Random QR Payload Spawner (A/B/C/D) di Gazebo

## Konfirmasi Status Saat Ini

- **Payload QR saat ini hardcoded** di `kki_arena.sdf`:
  - Model `payload` single di posisi `0.4 0.04 -0.894` dengan tekstur `qr_A.png`
  - Non-static (massa 0.3 kg) agar bisa diangkat oleh DetachableJoint
- **`mission_fsm.py`** menggunakan `payload_x=0.4`, `payload_y=0.0` hardcoded untuk state `APPROACH_QR`
- **`qr_detector.py`** membaca QR dari kamera dan publish `/hydroships/qr_result` (A/B/C/D) dan `/hydroships/qr_offset`
- **`generate_qr.py`** sudah bisa generate QR A/B/C/D di `media/qr_A.png` .. `qr_D.png`

**Masalah:** Setiap run sim selalu payload A di posisi yang sama. Perlu randomisasi QR letter (A/B/C/D) dan optionally posisi.

## Solusi

### Pendekatan: Payload Spawner Node

Buat node ROS 2 `payload_spawner` yang:
1. Menjalankan spawn payload model via gz-sim `EntityFactory` (lewat `ros2 run ros_gz_sim create`)
2. Memilih QR letter secara random (A/B/C/D) atau via parameter
3. Memilih posisi random dalam bounds arena yang valid (atau via parameter)
4. Mempublikasikan posisi spawn ke `/hydroships/payload_pose` agar FSM bisa menavigasi ke payload secara dinamis

### Aliran setelah perbaikan:
1. Sim start → `payload_spawner` menunggu `spawn_delay`
2. Spawner menghapus payload lama (jika ada) dan spawn payload baru dengan QR random
3. Spawner publish posisi ke `/hydroships/payload_pose`
4. `mission_fsm` menerima posisi dan navigasi ke payload
5. `qr_detector` membaca QR yang asli → `qr_result` menentukan wall

## File yang Harus Diperbaiki/Dibuat

### 1. **BUAT** `src/hydroships_gazebo/scripts/payload_spawner.py`

Node ROS 2 yang melakukan:
- **Parameter:**
  - `qr_letter` (default: `''` = random A/B/C/D)
  - `payload_x` (default: `0.4`)
  - `payload_y` (default: `0.04`)
  - `payload_z` (default: `-0.894`)
  - `spawn_delay` (default: `4.0` detik setelah sim start)
  - `qr_media_dir` (default: `hydroships_gazebo/media`)
  - `arena_x_min`, `arena_x_max`, `arena_y_min`, `arena_y_max` (default: `0.2`, `0.6`, `-1.5`, `1.5`)
- **Logika:**
  1. Tunggu `spawn_delay` detik agar sim siap
  2. Jika `qr_letter` kosong, pick random dari `['A','B','C','D']`
  3. Jika `payload_x/y` tidak di-set (gunakan nilai default yang sudah ada), randomize dalam bounds arena
  4. Construct SDF string untuk payload model (inline, berdasarkan definisi di `kki_arena.sdf`):
     - Link `payload_link` dengan mesh `payload_body.obj`
     - Visual `qr` dengan `albedo_map` dan `emissive_map` menunjuk ke `qr_{letter}.png`
     - Visual `qr_quiet_zone` (plane putih)
     - Collision box
     - Inertial (massa 0.3 kg)
     - Pose sesuai parameter
  5. Tulis SDF ke temp file
  6. Coba hapus model `payload` yang sudah ada (jika ada) via service `gz_sim_msgs/srv/DeleteEntity` atau `WorldControl`
  7. Spawn model baru via `subprocess.run(['ros2', 'run', 'ros_gz_sim', 'create', '-file', tmp_sdf, '-name', 'payload', '-x', str(x), '-y', str(y), '-z', str(z)])`
  8. Publikasi posisi ke `/hydroships/payload_pose` (geometry_msgs/PointStamped)
  9. Log: `payload_spawner: QR=%s pos=(%.2f, %.2f, %.2f)`

- **Fallback:** Jika `ros2 run ros_gz_sim create` gagal, log error dan lanjut (FSM masih bisa pakai default position)

### 2. **MODIFIKASI** `src/hydroships_gazebo/worlds/kki_arena.sdf`

- **Hapus** model `payload` inline (lines 177–257) — spawner yang akan menanganinya
- **Tetap** simpan `payload_fill` light (lines 259–275) untuk iluminasi QR
- Update comment di bagian kosong: "Payload di-spawn oleh payload_spawner node"

### 3. **MODIFIKASI** `src/hydroships_control/hydroships_control/mission_fsm.py`

- **Tambah subscription** ke `/hydroships/payload_pose` (geometry_msgs/PointStamped)
- **Tambah state variable:** `self.payload_pose = None` (tuple x,y,z atau None)
- **Tambah callback:** `_on_payload_pose(msg)` simpan `(msg.point.x, msg.point.y, msg.point.z)`
- **Modifikasi `_st_approach_qr`:**
  - Gunakan posisi dinamis jika tersedia:
    ```python
    if self.payload_pose is not None:
        tx, ty = self.payload_pose[0], self.payload_pose[1]
    else:
        tx, ty = self.payload_x, self.payload_y
    dist = self._goto_xy(tx, ty)
    ```
  - Tambah logika tunggu pose: jika `payload_pose` masih None setelah `t_dive * 0.5`, gunakan default parameters
- **Update docstring** untuk mencantumkan `/hydroships/payload_pose` sebagai input

### 4. **MODIFIKASI** `src/hydroships_gazebo/launch/sim.launch.py`

- **Tambah launch arguments:**
  - `qr_letter` (default: `''` = random)
  - `payload_x` (default: `0.4`)
  - `payload_y` (default: `0.04`)
- **Tambah node `payload_spawner`:**
  ```python
  spawner = Node(
      package='hydroships_gazebo',
      executable='payload_spawner',
      output='screen',
      parameters=[{
          'use_sim_time': True,
          'qr_letter': LaunchConfiguration('qr_letter'),
          'payload_x': LaunchConfiguration('payload_x'),
          'payload_y': LaunchConfiguration('payload_y'),
          'spawn_delay': spawn_delay + 1.0,  # after ROV spawn
      }],
  )
  ```
- Tambahkan `spawner` ke list return

### 5. **MODIFIKASI** `src/hydroships_bringup/launch/hydroships_mission.launch.py`

- Pass through `qr_letter`, `payload_x`, `payload_y` arguments ke `sim.launch.py` inclusion, atau declare sendiri jika `sim.launch.py` tidak include `hydroships_mission.launch.py` untuk sim.

Actually, looking at the launch structure:
- `sim.launch.py` is in `hydroships_gazebo` — it starts sim + ROV + bridge + sensors
- `hydroships_mission.launch.py` is in `hydroships_bringup` — it starts sim + allocator + stabilizer + mission_fsm + gripper + hook

So `payload_spawner` should be added to `sim.launch.py` because it needs the sim running. But `hydroships_mission.launch.py` includes `sim.launch.py`... let me check.

Actually, looking at the launch files would help. Let me check `hydroships_mission.launch.py`.

Actually, I don't need to read it. The point is: `payload_spawner` should be launched together with the sim, so it makes sense to add it to `sim.launch.py` or to a separate launch include. For simplicity, add it to `sim.launch.py`.

### 6. **MODIFIKASI** `docs/HOW-TO-RUN.txt`

- Tambah dokumentasi argumen launch:
  ```
  ros2 launch hydroships_bringup hydroships_mission.launch.py qr_letter:=B
  ros2 launch hydroships_bringup hydroships_mission.launch.py qr_letter:=C payload_x:=0.5 payload_y:=-1.2
  ```
- Tambah verifikasi: `ros2 topic echo /hydroships/payload_pose`

### 7. **MODIFIKASI** `docs/STATUS.md`

- Update M3/M4: QR payload sekarang di-spawn random via `payload_spawner` node
- Catat bahwa payload position dan QR letter bisa diatur via launch arg

### 8. **MODIFIKASI** `docs/CHANGELOG.md`

- Tambah entri: "Payload QR sekarang di-spawn random (A/B/C/D) via node `payload_spawner`. FSM membaca posisi dari `/hydroships/payload_pose`."

## Strategi Perbaikan

### Langkah 1: Buat `payload_spawner.py`
- Write the full node code
- Handle SDF construction, random selection, EntityFactory spawn
- Publish to `/hydroships/payload_pose`

### Langkah 2: Modifikasi `kki_arena.sdf`
- Remove inline payload model
- Keep payload_fill light

### Langkah 3: Modifikasi `mission_fsm.py`
- Add payload_pose subscription
- Update `_st_approach_qr` to use dynamic position

### Langkah 4: Modifikasi `sim.launch.py`
- Add payload_spawner node
- Add launch arguments

### Langkah 5: Build dan test
- `colcon build`
- `pytest src/hydroships_control/test/`
- `python -m py_compile`

### Langkah 6: Update docs

## Validasi

- `colcon build` sukses
- `pytest` 62 test lolos
- `python -m py_compile` tanpa error
- Launch sim dengan `qr_letter:=B` → payload spawn dengan QR B di posisi yang benar
- `ros2 topic echo /hydroships/payload_pose` menunjukkan posisi yang benar
- FSM berhasil approach ke payload di posisi random
