# HYDROships ROV — CARA MENJALANKAN SIMULASI KOLAM (step-by-step)

KI 2026 | ROS 2 Humble + Gazebo Fortress (gz-sim 6)

Dokumen ini panduan praktis menjalankan & menguji simulasi di kolam sim.
Ringkas: (0) prasyarat -> (1) build -> (2) source -> (3) run -> (4) uji.

--------------------------------------------------------------------------------

## 0. PRASYARAT (sekali saja)

- Ubuntu 22.04 + ROS 2 Humble terpasang.
- Gazebo Fortress (gz-sim 6) + jembatan ROS<->GZ:

```bash
sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge
```

- Dependensi lain (biasanya sudah ada via rosdep):

```bash
ros-humble-xacro ros-humble-robot-state-publisher
python3-numpy python3-opencv
```

- Cara aman memasang semua dependensi paket (dijalankan di root workspace):

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

- Kalau python3-opencv tidak tersedia/gagal via apt, pasang dependensi Python
  lewat pip (opencv-python + numpy, sama seperti di README.md):

```bash
cd ~/ros2_ws
pip install -r requirements.txt
```

  Cek `cv2` sudah kebaca:

```bash
python3 -c "import cv2; print(cv2.__version__)"
```

Catatan GPU: sensor kamera (gz-sim-sensors, ogre2) butuh render. Di mesin tanpa
GPU/EGL, pakai mode headless (lihat argumen `headless` di bawah) atau kamera bisa
gagal render (lihat PROBLEM.md).

--------------------------------------------------------------------------------

## 1. BUILD WORKSPACE  (WAJIB tiap kali kode/URDF/world/config berubah)

```bash
cd ~/ros2_ws
colcon build
```

PENTING: launch memproses URDF (xacro) dari direktori `install/`, BUKAN dari `src/`.
Jadi setelah `git pull` / mengubah file, WAJIB `colcon build` lagi — kalau tidak,
sim masih memakai versi lama (mis. model/thruster/gripper lama).

Folder `build/ install/ log/` sengaja di-gitignore; regenerasi dgn `colcon build`.

--------------------------------------------------------------------------------

## 2. SOURCE ENVIRONMENT  (WAJIB tiap terminal baru)

```bash
source /opt/ros/humble/setup.bash &&
source ~/ros2_ws/install/setup.bash
```

--------------------------------------------------------------------------------

## 3. MENJALANKAN — pilih salah satu skenario

### 3A. SIM SAJA (lihat ROV di kolam, tanpa kendali)

Menyalakan: Gazebo + spawn ROV + bridge + depth_publisher + qr_detector.

```bash
ros2 launch hydroships_gazebo sim.launch.py world:=kki_arena.sdf
```

(default world sim.launch.py = `pool_practice_arena.sdf`, kolam latihan 2,2×4,4×0,8 m;
`kki_arena.sdf` = arena lomba 5×5 m; `pool_empty.sdf` = kolam kosong tanpa dinding/hook.)
Setelah spawn, verifikasi visual: body gripper (kotak kuning 0.10×0.10×0.06 m)
harus terlihat di muka depan ROV (x≈0.18 m). DUA jari kuning (masing-masing
0.08 m) menjorok ke depan (+X) dari body, satu di sisi kiri (y≈-0.025 m) dan
satu di sisi kanan (y≈+0.025 m) — bentuk gripper penjepit.

### 3B. SIM + STABILISASI + KENDALI MANUAL (teleop) (SIMULASI AIR KOLAM TIDAK ADA)

Menyalakan: sim + thruster_allocator + stabilizer (depth-hold & heading-hold).

```bash
ros2 launch hydroships_bringup hydroships_stabilized.launch.py
```

Lalu DI TERMINAL KEDUA (sudah di-source, lihat langkah 2), kemudikan horizontal:

```bash
ros2 run hydroships_control teleop_stabilized
```

### 3C. MISI AUTONOMOUS PENUH (FSM)

Menyalakan: sim + allocator + stabilizer + mission_fsm (IDLE->DIVE->...->DONE).

```bash
ros2 launch hydroships_bringup hydroships_mission.launch.py
```

Mulai dari state tertentu (lewati DIVE/SCAN):

```bash
ros2 launch hydroships_bringup hydroships_mission.launch.py start_state:=NAV_WALL
ros2 launch hydroships_bringup hydroships_mission.launch.py start_state:=NAV_WALL start_wall:=A
ros2 launch hydroships_bringup hydroships_mission.launch.py start_state:=NAV_WALL start_wall:=B
ros2 launch hydroships_bringup hydroships_mission.launch.py start_state:=NAV_WALL start_wall:=C
ros2 launch hydroships_bringup hydroships_mission.launch.py start_state:=NAV_WALL start_wall:=D
```

```bash
# Uji NAV_WALL → HANG → SURFACE tanpa lewat fase awal
ros2 launch hydroships_bringup hydroships_mission.launch.py \
    start_state:=NAV_WALL start_wall:=B
# Samakan dengan ROV spawn manual di dekat wall B
ros2 launch hydroships_bringup hydroships_mission.launch.py \
    start_state:=NAV_WALL start_wall:=B \
    rov_random_spawn:=false rov_x:=0.0 rov_y:=1.5 rov_z:=-0.5
```

Payload QR di-spawn otomatis RANDOM (A/B/C/D + posisi acak dalam bounds arena)
oleh node `payload_spawner` setiap launch. Bounds default node itu (x∈[0.2,0.6],
y∈[-1.5,1.5]) TIDAK ikut diubah saat default world diganti ke kolam latihan —
kebetulan masih valid (lebih sempit dari kolam 2,2×4,4 m) tapi cuma memanfaatkan
sebagian area kolam baru utk sebaran acak. Pilih huruf/posisi eksplisit:

```bash
ros2 launch hydroships_bringup hydroships_mission.launch.py qr_letter:=B
ros2 launch hydroships_bringup hydroships_mission.launch.py qr_letter:=C payload_x:=0.5 payload_y:=-1.2
```

FSM membaca posisi spawn dari `/hydroships/payload_pose` (navigasi APPROACH_QR
ke payload di posisi manapun; fallback ke default 0.4/0.04 bila pose belum tiba).
ROV di-spawn RANDOM dekat salah satu dinding kolam tiap run (default kontes).
Lihat posisi di log `[sim.launch] ROV spawn (random=True) di (x, y, z)`. Override
posisi manual:

```bash
ros2 launch hydroships_bringup hydroships_mission.launch.py rov_random_spawn:=false rov_x:=1.0 rov_y:=-1.0
```

### 3D. SIM + JEMBATAN GUI TIM (M7, UDP-JSON)

Menyalakan: sim + allocator + gui_bridge (adapter GUI-ROV <-> ROS via UDP).
GUI tim (Customize5773/GUI-ROV) memakai UDP-JSON/MAVLink, bukan ROS langsung
(lihat `docs/GUI-INTEGRATION.md`). `[VERIFY: belum diuji GUI live.]`

```bash
ros2 launch hydroships_bringup hydroships_gui.launch.py
```

Arahkan telemetri ke laptop GUI (server.js) & set port bila perlu:

```bash
ros2 launch hydroships_bringup hydroships_gui.launch.py \
      gui_host:=192.168.2.1 cmd_port:=14550 telem_port:=14551 telem_hz:=10
```

### 3E. SIM + TELEOP KEYBOARD 6-DOF LANGSANG

Menyalakan: sim + allocator + teleop_keyboard (kendali langsung wrench 6-DOF,
bypass stabilizer). Bila stabilizer berjalan di terminal lain, teleop_keyboard
tetap bisa competing ke `/hydroships/cmd_vel` — matikan stabilizer untuk menghindari
konflik.

```bash
ros2 launch hydroships_bringup hydroships_sim.launch.py
```

Lalu DI TERMINAL KEDUA:

```bash
ros2 run hydroships_control teleop_keyboard
```

Tombol w/s/a/d = surge, a/d = sway, i/k = heave, j/l = yaw, u/o = roll,
t/g = pitch, spasi = stop. Lihat docstring `teleop_keyboard.py` untuk detail.

### 3F. HEADLESS (SERVER SAJA, TANPA GUI)

Untuk CI, cloud, atau mesin tanpa GPU/EGL. Semua skenario di atas bisa
dijalankan headless dengan menambahkan `headless:=true`:

```bash
ros2 launch hydroships_bringup hydroships_sim.launch.py headless:=true
ros2 launch hydroships_bringup hydroships_stabilized.launch.py headless:=true
ros2 launch hydroships_bringup hydroships_mission.launch.py headless:=true
ros2 launch hydroships_bringup hydroships_gui.launch.py headless:=true
```

Catatan: kamera tetap publish `image_raw`, tetapi render headless bisa membuat
QR deteksi gagal (lihat Troubleshooting bagian QR).

### 3G. REPRODUCIBLE SPAWN (DEBUG/REPLAY)

Untuk testing yang konsisten, pakai `spawn_seed` untuk mengunci posisi spawn
ROV (gabungkan dengan `headless` + `qr_letter` + `payload_*` bila inginkan
run penuh yang deterministis). Kosong (default) = acak penuh tiap launch.

```bash
  # Run reproducibel penuh: headless + seed 1001 + QR 'A' + pose (0.4, 0.04)
  ros2 launch hydroships_bringup hydroships_mission.launch.py \
      headless:=true spawn_seed:=1001 qr_letter:=A payload_x:=0.4 payload_y:=0.04
```

Eksperimen 3-seed pakai seed 1001/1002/1003 supaya hasil bisa direproduksi
persis tiap run (covariance/RNG di `sim.launch.py` & `_spawn_rng`). Untuk replay
debug tunggal cukup set `spawn_seed` saja:

```bash
  ros2 launch hydroships_bringup hydroships_mission.launch.py spawn_seed:=1001
```

Untuk spawn manual pada posisi tetap (mis. dekat wall B untuk uji NAV_WALL):

```bash
  ros2 launch hydroships_bringup hydroships_mission.launch.py \
      start_state:=NAV_WALL start_wall:=B \
      rov_random_spawn:=false rov_x:=0.0 rov_y:=1.5 rov_z:=-0.5
```

### 3H. MID-MISSION START dengan TUNING APPROACH_HOOK

FSM bisa dimulai dari state tengah untuk isolasi testing. Contoh: mulai
langsung di APPROACH_HOOK dengan tuning parameter visual servo:

```bash
  ros2 launch hydroships_bringup hydroships_mission.launch.py \
      start_state:=APPROACH_HOOK \
      hook_size_stop:=0.40 hook_center_tol:=0.10 hook_max_age:=2.0 t_approach:=30.0
```

State lain yang bisa di-start: DIVE, APPROACH_QR, GRAB, NAV_WALL, HANG,
SURFACE, WAIT_TRIGGER, AUTO_RELEASE. Untuk NAV_WALL/HANG/SURFACE bisa
ditambah `start_wall:=A/B/C/D`.

### 3I. TRIGGER JOYSTICK UTK LEWATI WAIT_TRIGGER

Setelah SURFACE, FSM parkir di `WAIT_TRIGGER` dan menunggu pesan Empty di
`/hydroships/mission/start_autonomous` sebelum lanjut ke APPROACH_HOOK →
AUTO_RELEASE. Sejak launch arg `joy_trigger:=true` (default), tombol
joystick (default **A** / index 0 pada XInput/F310) mempublish pesan itu
via node `joy_mission_trigger`:

```bash
  # default: tombol A (index 0)
  ros2 launch hydroships_bringup hydroships_mission.launch.py

  # ganti tombol, mis. B (index 1)
  ros2 launch hydroships_bringup hydroships_mission.launch.py joy_button_index:=1

  # nonaktifkan (run battery/headless tanpa joystick)
  ros2 launch hydroships_bringup hydroships_mission.launch.py joy_trigger:=false
```

Tanpa joystick, `WAIT_TRIGGER` timeout setelah `t_wait_trigger` (default
600 s) → ABORT. Cara manual tetap tersedia:

```bash
  ros2 topic pub -1 /hydroships/mission/start_autonomous std_msgs/msg/Empty "{}"
```

### 3J. BATCH REPRODUCIBLE RUN (3-SEED)

Untuk eksperimen batch yang dapat direproduksi, jalankan loop seed dan
simpan semua output ke satu file log. Contoh: setiap seed pakai
`start_state:=AUTO_RELEASE` (uji fase HANG → AUTO_RELEASE → SURFACE) dengan
QR 'A' dan payload pada (0.4, 0.04):

```bash
  for seed in 1001 1002 1003; do
    echo "=== Menjalankan Seed: $seed ==="
    ros2 launch hydroships_bringup hydroships_mission.launch.py \
      headless:=true spawn_seed:=$seed qr_letter:=A \
      start_state:=AUTO_RELEASE 2>&1
  done | tee -a semua_output.log
```

Karena `spawn_seed` diteruskan ke `_spawn_rng` (sim.launch.py:38) + RNG payload
(`payload_spawner`), tiap seed menghasilkan pose ROV & payload yang SAMA persis
antar run. Dengan demikian varians antar seed (1001 vs 1002 vs 1003) jelas
terpisah dari varians dalam seed yang sama — pakailah untuk evaluasi
perilaku autonomous yang konsisten.

--------------------------------------------------------------------------------

## 4. ARGUMEN LAUNCH (opsional, tambahkan di belakang perintah, format arg:=nilai)

| arg | default | deskripsi |
| --- | --- | --- |
| `headless` | `false` | gz sim tanpa GUI (server saja); untuk CI / mesin tanpa GPU. |
| `world` | `pool_practice_arena.sdf` | kolam latihan 2,2×4,4×0,8 m (default). Alt: `kki_arena.sdf` (arena lomba 5×5 m), `pool_empty.sdf` (kolam kosong). |
| `start_state` | `DIVE` | (mission) state awal FSM: DIVE/APPROACH_QR/GRAB/NAV_WALL/HANG/SURFACE/WAIT_TRIGGER/APPROACH_HOOK/AUTO_RELEASE. |
| `start_wall` | `''` | (mission) seed wall target A/B/C/D utk testing mid-state (NAV_WALL/HANG/SURFACE/APPROACH_HOOK/AUTO_RELEASE). |
| `hook_size_stop` / `hook_center_tol` / `hook_max_age` / `t_approach` | `0.35` / `0.15` / `1.0` / `25.0` | (mission) tuning visual servo APPROACH_HOOK ke hook. `hook_size_stop` naik = berhenti lebih dekat; `hook_center_tol` turun = pemusatan lebih ketat; deteksi lebih tua dari `hook_max_age` = fallback ke target odometri. |
| `rov_random_spawn` | `true` | spawn ROV ACAK dekat salah satu dinding kolam (default kontes, beda tiap run). `false` = pakai `rov_x/rov_y/rov_z`. |
| `rov_x` / `rov_y` / `rov_z` | `0.0` / `0.0` / `-0.5` | posisi manual spawn ROV (dipakai bila `rov_random_spawn:=false`). `rov_z` default -0.5 (di bawah permukaan). |
| `rov_wall_margin` | `0.5` | jarak aman ROV dari dinding fisik (±`rov_arena_half`). |
| `rov_arena_half` | `1.1` | setengah lebar kolam (dinding di ±nilai ini). 1,1 = setengah sisi pendek kolam latihan (2,2 m). Naikkan ke `2.55` bila `world:=kki_arena.sdf` (arena lomba 5×5 m). |
| `spawn_delay` | `3.0` | jeda (detik) sebelum spawn ROV; naikkan bila mesin lambat (cegah race: service create belum siap). |
| `spawn_seed` | — | fix seed pose spawn acak supaya run bisa diulang persis (replay/debug). Contoh eksperimen 3-seed: 1001/1002/1003 supaya hasil konsisten tiap run. Nilai kosong (default) = acak penuh tiap launch. |
| `qr_letter` | `''` | (mission/stabilized/sim) huruf QR payload A/B/C/D. Kosong (default) = random + posisi acak dalam bounds arena. |
| `payload_x` / `payload_y` | `0.4` / `0.04` | posisi X/Y payload (m); dipakai bila `qr_letter` di-set eksplisit. |
| `scan_depth` | `0.30` | (mission) kedalaman scan QR (m). Lebih dalam = QR lebih besar tapi petak pandang menyempit. |
| `cam_gripper_dx` | `0.16` | (mission) koreksi offset kamera bawah ke gripper (m). `0.0` = QR dipusatkan di kamera (tanpa koreksi). |

Contoh gabungan:

```bash
  # Run reproducibel penuh (headless + seed + QR + payload):
  ros2 launch hydroships_bringup hydroships_mission.launch.py \
      headless:=true spawn_seed:=1001 qr_letter:=A payload_x:=0.4 payload_y:=0.04
  # Uji di arena lomba 5x5 m (bukan kolam latihan default) - rov_arena_half
  # WAJIB ikut diganti, kalau tidak ROV random-spawn dgn radius kolam kecil
  # di dalam arena besar (bukan crash, tapi tak representatif):
  ros2 launch hydroships_bringup hydroships_mission.launch.py headless:=true \
      world:=kki_arena.sdf rov_arena_half:=2.55
```

--------------------------------------------------------------------------------

## 5. PERINTAH UJI BERGUNA (terminal terpisah, sudah di-source)

Lihat daftar & data topik:

```bash
  ros2 topic list
  ros2 topic echo /hydroships/odom          # pose & twist ROV
  ros2 topic echo /hydroships/depth          # kedalaman (m, >=0)
  ros2 topic echo /hydroships/payload_pose   # posisi spawn payload QR (dari payload_spawner)
  ros2 topic echo /hydroships/qr_result      # huruf QR terbaca kamera (A/B/C/D)
```

Kendali langsung (saat mode 3B/3C berjalan):

```bash
  # target kedalaman (negatif = menyelam)
  ros2 topic pub -1 /hydroships/setpoint/depth std_msgs/msg/Float64 "{data: -0.6}"
  # target heading (rad)
  ros2 topic pub -1 /hydroships/setpoint/heading std_msgs/msg/Float64 "{data: 1.57}"
  # gaya horizontal manual (Fx maju, Fy samping) — Newton, body-frame
  ros2 topic pub -1 /hydroships/manual/cmd geometry_msgs/msg/Twist "{linear: {x: 15.0, y: 0.0}}"
```

Suntik hasil QR manual (bila QR belum terbaca visual — lihat PROBLEM.md):

```bash
  ros2 topic pub -1 /hydroships/qr_result std_msgs/msg/String "{data: 'A'}"
```

Uji thruster langsung (mode 3A, gaya N per thruster):

```bash
  ros2 topic pub -1 /hydroships/thruster_3/thrust std_msgs/msg/Float64 "{data: 20.0}"
```

--------------------------------------------------------------------------------

## 6. VERIFIKASI CEPAT PASCA-PERBAIKAN FISIKA (disarankan)

Setelah perbaikan geometri thruster (yaw pulih, lihat PROBLEM.md), cek di sim:

a) YAW berputar benar:

```bash
  ros2 topic pub -1 /hydroships/setpoint/heading std_msgs/msg/Float64 "{data: 1.57}"
```

-> ROV berputar ke ~90°. Pantau yaw via: `ros2 topic echo /hydroships/odom`

b) TANDA sway benar:

```bash
  ros2 topic pub -1 /hydroships/manual/cmd geometry_msgs/msg/Twist "{linear: {y: 15.0}}"
```

-> ROV geser ke KIRI (y+). Bila terbalik: flip axis T200-B (+y -> -y) di URDF
   & `allocation.py`.

c) DEPTH/DIVE normal:

```bash
  ros2 topic pub -1 /hydroships/setpoint/depth std_msgs/msg/Float64 "{data: -0.7}"
```

-> ROV menyelam & menahan kedalaman.

d) NAV_WALL (mode 3C): jalankan misi, suntik QR (mis. 'A'), ROV harus bergerak
   ke wall A tanpa harus memutar badan (navigasi holonomik).

e) GRIPPER BODY (mode 3A/3C): verifikasi body gripper terlihat di Gazebo
   di muka depan ROV (kotak kuning 0.10×0.10×0.06 m di x≈0.18 m).
   Dua jari kuning 0.08 m menyorong ke depan (+X) di sisi kiri & kanan body.

f) GRIPPER MECHANISME (mode 3C): selama misi, di state GRAB periksa:

```bash
  ros2 topic echo /hydroships/gripper/attach
  ros2 topic echo /hydroships/gripper/detach
```

   Payload harus ter-attach saat ROV di atasnya, dan ter-detach saat AUTO_RELEASE.
   Kedua jari bergerak serempak (menjepit/membuka) sesuai perintah — kedua
   topik ini harus menerbitkan nilai IDENTIK (0.35 = buka, 0.0 = tutup):

```bash
  ros2 topic echo /hydroships/gripper_left/cmd
  ros2 topic echo /hydroships/gripper_right/cmd
```

--------------------------------------------------------------------------------

## 7. TROUBLESHOOTING

- Model/perubahan tak muncul di Gazebo:
  Lupa `colcon build` + relaunch. Launch baca URDF dari `install/`. Rebuild dulu.

- ROV berperilaku erratic / thruster "adu perintah":
  Ada proses sim/node lama yang masih hidup. Matikan semuanya lalu ulang:

  ```bash
    pkill -f 'gz sim'; pkill -f parameter_bridge; pkill -f robot_state_publisher
    pkill -f mission_fsm; pkill -f stabilizer; pkill -f thruster_allocator
  ```

  (Selalu pastikan bersih sebelum run baru — lihat catatan PROBLEM.md.)

- Kamera hitam / sim berat / crash render di mesin tanpa GPU:
  Pakai `headless:=true`, atau jalankan di mesin ber-GPU.

- QR tidak terdeteksi walau kamera & unit test normal (KNOWN ISSUE):
  Gejala di log qr_detector, umumnya saat headless:

  ```text
  [WARN] DECODE GAGAL: QR tak terdeteksi (pts=None)
  [WARN] DECODE GAGAL: QR terdeteksi (pts ada) tapi decode kosong
  ```

  Ini bisa muncul walaupun `/hydroships/camera_bottom/image_raw` dan `camera_info`
  mengalir normal dan unit test QR (`test_qr_logic.py`) semuanya lolos — test
  memakai fixture frame, bukan hasil render headless. Penyebabnya di sisi
  render/pencahayaan sim, bukan di logika decode. Yang bisa dicoba:
  1) Jalankan TANPA headless (`headless:=false`) supaya render penuh.
  2) Turunkan ketinggian scan biar QR lebih besar di frame, mis. `scan_depth`
     lebih dekat ke payload.
  3) Untuk menguji FSM tanpa menunggu deteksi, suntik hasil QR manual:

  ```bash
    ros2 topic pub -1 /hydroships/qr_result std_msgs/msg/String "{data: 'A'}"
  ```

- Model gagal spawn (service create belum siap):
  Naikkan `spawn_delay`, mis. `spawn_delay:=6.0`.

- `ros2`/`gz` command "not found":
  Terminal belum di-source. Ulangi langkah 2.

--------------------------------------------------------------------------------

## 8. MENJALANKAN UNIT TEST (opsional, tanpa Gazebo)

```bash
  cd ~/ros2_ws
  colcon test --packages-select hydroships_control
  colcon test-result --verbose
```

--------------------------------------------------------------------------------

## 9. CONTOH SESI MULTI-TERMINAL

Prasyarat di setiap terminal:

```bash
  cd ~/ros2_ws
  source /opt/ros/humble/setup.bash
  source install/setup.bash
```

```text
┌─ CONTOH A: SIM + STABILISASI + TELEOP (skenario 3B) ────────────────────────┐
│ TERMINAL 1:                                                                    │
│   ros2 launch hydroships_bringup hydroships_stabilized.launch.py               │
│ TERMINAL 2:                                                                    │
│   ros2 run hydroships_control teleop_stabilized                                 │
│ TERMINAL 3: (opsional, cek aliran data)                                        │
│   ros2 topic echo /hydroships/cmd_vel                                           │
└───────────────────────────────────────────────────────────────────────────────┘

┌─ CONTOH B: SIM + TELEOP KEYBOARD 6-DOF (skenario 3E) ───────────────────────┐
│ TERMINAL 1:                                                                    │
│   ros2 launch hydroships_bringup hydroships_sim.launch.py                      │
│ TERMINAL 2:                                                                    │
│   ros2 run hydroships_control teleop_keyboard                                   │
│ TERMINAL 3: (opsional, cek aliran data)                                        │
│   ros2 topic echo /hydroships/thruster_1/thrust                                │
└───────────────────────────────────────────────────────────────────────────────┘

┌─ CONTOH C: MISI AUTONOMOUS (skenario 3C) ───────────────────────────────────┐
│ TERMINAL 1:                                                                    │
│   ros2 launch hydroships_bringup hydroships_mission.launch.py                  │
│ TERMINAL 2: (opsional, suntik QR manual bila deteksi gagal)                    │
│   ros2 topic pub -1 /hydroships/qr_result std_msgs/msg/String "{data: 'A'}"    │
│ TERMINAL 3: (opsional, cek state FSM)                                          │
│   ros2 topic echo /hydroships/fsm_state                                         │
└───────────────────────────────────────────────────────────────────────────────┘

┌─ CONTOH D: GUI BRIDGE (skenario 3D) ────────────────────────────────────────┐
│ TERMINAL 1:                                                                    │
│   ros2 launch hydroships_bringup hydroships_gui.launch.py                      │
│ TERMINAL 2: (laptop GUI, server.js harus sudah berjalan)                        │
│   # pastikan server.js mendengarkan port 14551                                 │
└───────────────────────────────────────────────────────────────────────────────┘
```

--------------------------------------------------------------------------------

## Ringkasan tercepat

```text
cd ~/ros2_ws && colcon build && source install/setup.bash
ros2 launch hydroships_bringup hydroships_mission.launch.py
# default: kolam latihan 2,2x4,4x0,8 m (pool_practice_arena.sdf).
# utk arena lomba 5x5 m: tambah world:=kki_arena.sdf rov_arena_half:=2.55
```
