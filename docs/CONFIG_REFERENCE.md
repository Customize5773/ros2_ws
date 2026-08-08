# Config Reference — YAML & Launch Arguments

Referensi lengkap semua file konfigurasi (`*.yaml`) dan argumen launch (`*.launch.py`)
di workspace. Untuk panduan *cara menyetel* nilai-nilai ini (bukan sekadar daftar),
lihat `docs/TUNING_GUIDE.md`. Untuk kontrak topic yang dikonsumsi/dihasilkan node
terkait, lihat `docs/ARCHITECTURE.md` dan `docs/NODES_REFERENCE.md`.

---

## 1. `src/hydroships_description/config/rov_params.yaml`

Parameter fisik ROV, dimuat URDF/xacro (`hydroships.urdf.xacro`) via `xacro.load_yaml`
saat build. **Mengubah file ini mengharuskan `colcon build` ulang** — nilainya di-bake
ke URDF saat xacro diproses, bukan dibaca runtime oleh node Python.

| Key | Nilai default | Satuan | Sumber | Arti |
|---|---|---|---|---|
| `base_mass` | 8.3 | kg | **measured** | Massa total ROV (tanpa gripper payload) |
| `thruster_mass` | 0.05 | kg | estimate | Massa per unit thruster (link URDF) |
| `fluid_density` | 1000.0 | kg/m³ | **measured** | Densitas air kolam tawar (dipakai plugin Buoyancy) |
| `buoyancy_collision.x/y/z` | 0.219 / 0.219 / 0.182 | m | estimate | Dimensi box collision dipakai plugin Buoyancy (≈0.635× bbox luar 0.345×0.345×0.286). **Neraca apung hanya sahih selama `base_link` satu-satunya link ROV yang punya `<collision>`** — plugin menurunkan volume perpindahan dari geometri collision, jadi link lain yang diberi collision ikut menghasilkan gaya apung (lihat [P0-1-BASELINE.md](P0-1-BASELINE.md)) |
| `cog.x/y/z` | 0.0 / 0.0 / 0.0 | m | estimate | Origin inersial link `base_link`. **Bukan CoG sistem**: massa gripper di haluan menggeser CoG gabungan ke x ≈ +2.37 mm |
| `cob.x/y/z` | 0.00237 / 0.0 / 0.02 | m | estimate | Center of Buoyancy — `z` di atas CoG agar ada momen pemulih pasif (self-righting); `x` disejajarkan dgn **CoG sistem** agar tidak ada lengan parasit (commit `8d6c49c`) |
| `inertia.ixx/iyy/izz` | 0.13890 / 0.13890 / 0.16465 | kg·m² | estimate | Tensor inersia diagonal, diturunkan dari model box solid via `scripts/estimate_mass_inertia.py` |
| `inertia.ixy/ixz/iyz` | 0.0 / 0.0 / 0.0 | kg·m² | estimate | Off-diagonal — diasumsikan nol (bodi simetris) |
| `added_mass.xDotU/yDotV/zDotW` | -1.571 / -1.759 / -4.614 | kg | estimate | Massa tambah (hydrodynamic added mass) translasi, diskalakan dari referensi BlueROV2 (~33.6 kg) dengan rasio massa 0.247 |
| `added_mass.kDotP/mDotQ/nDotR` | -0.0467 / -0.0334 / -0.0548 | kg·m²/rad | estimate | Massa tambah rotasi (roll/pitch/yaw) |
| `linear_damping.xU/yV/zW` | -2.890 / -4.940 / -7.880 | N·s/m | estimate | Koefisien redaman linear translasi |
| `linear_damping.kP/mQ/nR` | -6.176 / -10.869 / -1.235 | N·m·s/rad | estimate | Koefisien redaman linear rotasi |
| `quadratic_damping.xUabsU/yVabsV/zWabsW` | -4.491 / -5.351 / -9.137 | N·s²/m² | estimate | Redaman kuadratik translasi (dominan di kecepatan tinggi) |
| `quadratic_damping.kPabsP/mQabsQ/nRabsR` | -0.383 / -0.383 / -0.383 | N·m·s²/rad² | estimate | Redaman kuadratik rotasi |

> **Catatan:** semua nilai bertanda `estimate` diskalakan dari referensi BlueROV2, **bukan** hasil pengukuran ROV fisik HYDROships. Sebelum deploy hardware, lihat `docs/HARDWARE.md` §2–3 untuk daftar kalibrasi yang perlu dilakukan.

---

## 2. `src/hydroships_control/config/gains.yaml`

Parameter runtime ROS2 untuk node `stabilizer` (namespace `stabilizer: ros__parameters:`).
Dimuat via `--params-file` di launch; **bisa diubah tanpa rebuild**, cukup relaunch
(atau `ros2 param set` saat runtime untuk uji cepat, tidak persisten).

| Key | Default | Satuan | Arti |
|---|---|---|---|
| `use_sim_time` | `true` | bool | Pakai clock `/clock` dari Gazebo, bukan wall-clock. **Set `false` di hardware asli.** |
| `rate` | 20.0 | Hz | Frekuensi loop kontrol PID |
| `depth.kp` | 60.0 | N/m | Gain proporsional depth-hold |
| `depth.ki` | 8.0 | N/(m·s) | Gain integral depth-hold |
| `depth.kd` | 40.0 | N/(m/s) | Gain derivatif depth-hold |
| `depth.integral_limit` | 30.0 | N·s | Anti-windup clamp untuk term integral |
| `depth.out_limit` | 60.0 | N | Batas output |Fz| maksimum dari PID depth |
| `heading.kp` | 8.0 | N·m/rad | Gain proporsional heading-hold |
| `heading.ki` | 0.5 | N·m/(rad·s) | Gain integral heading-hold |
| `heading.kd` | 4.0 | N·m/(rad/s) | Gain derivatif heading-hold |
| `heading.integral_limit` | 5.0 | N·m·s | Anti-windup heading |
| `heading.out_limit` | 15.0 | N·m | Batas |Mz| maksimum |
| `pitch.kp/ki/kd/integral_limit/out_limit` | 6.0 / 0.3 / 3.0 / 5.0 / 15.0 | — | Sama pola dengan heading, untuk sumbu pitch (My) |
| `roll.kp/ki/kd/integral_limit/out_limit` | 6.0 / 0.3 / 3.0 / 5.0 / 15.0 | — | Sama pola, sumbu roll (Mx) |
| `buoyancy_ff` | -0.3 | N | Feed-forward kompensasi buoyancy (near-neutral, ditambahkan ke output PID depth) |
| `target_depth` | -0.1 | m | Setpoint depth default saat start (negatif = di bawah permukaan) |
| `target_heading` | 0.0 | rad | Setpoint heading default |
| `target_pitch` | 0.0 | rad | Setpoint pitch default |
| `target_roll` | 0.0 | rad | Setpoint roll default |
| `enable_depth_hold` | `true` | bool | Aktifkan/nonaktifkan loop PID depth |
| `enable_heading_hold` | `true` | bool | Aktifkan/nonaktifkan loop PID heading |
| `enable_pitch_hold` | `true` | bool | Aktifkan/nonaktifkan loop PID pitch |
| `enable_roll_hold` | `true` | bool | Aktifkan/nonaktifkan loop PID roll |

Panduan tuning nilai-nilai ini: `docs/TUNING_GUIDE.md` §1.

---

## 3. `src/hydroships_gazebo/config/bridge.yaml`

Daftar pemetaan topic `ros_gz_bridge` (ROS2 Humble ↔ Gazebo Fortress). **Hanya relevan
untuk sim** — tidak dipakai sama sekali di hardware fisik (lihat `docs/HARDWARE.md`).
Setiap entri: `ros_topic_name`, `gz_topic_name`, `ros_type_name`, `gz_type_name`, `direction`.

| ROS topic | Tipe ROS | Arah | Keterangan |
|---|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | GZ→ROS | Sinkronisasi waktu sim |
| `/hydroships/odom` | `nav_msgs/Odometry` | GZ→ROS | Ground-truth pose & twist ROV |
| `/hydroships/imu` | `sensor_msgs/Imu` | GZ→ROS | IMU simulasi |
| `/hydroships/camera_front/image_raw` | `sensor_msgs/Image` | GZ→ROS | Video kamera depan |
| `/hydroships/camera_bottom/image_raw` | `sensor_msgs/Image` | GZ→ROS | Video kamera bawah |
| `/hydroships/camera_front/camera_info` | `sensor_msgs/CameraInfo` | GZ→ROS | Intrinsics kamera depan (sim, bukan kalibrasi hardware) |
| `/hydroships/camera_bottom/camera_info` | `sensor_msgs/CameraInfo` | GZ→ROS | Intrinsics kamera bawah (sim) |
| `/hydroships/thruster_1/thrust` … `_6/thrust` | `std_msgs/Float64` | ROS→GZ | 6 entri terpisah, command gaya per thruster (N) |
| `/hydroships/gripper_left/cmd` | `std_msgs/Float64` | ROS→GZ | Sudut jari kiri gripper (rad, kosmetik) |
| `/hydroships/gripper_right/cmd` | `std_msgs/Float64` | ROS→GZ | Sudut jari kanan gripper (rad) |
| `/hydroships/gripper/attach` | `std_msgs/Empty` | ROS→GZ | Trigger `DetachableJoint` attach (grasp payload) |
| `/hydroships/gripper/detach` | `std_msgs/Empty` | ROS→GZ | Trigger `DetachableJoint` detach (lepas payload) |

---

## 4. Launch Arguments

### `src/hydroships_gazebo/launch/sim.launch.py` (core, di-include semua launch lain)

| Arg | Default | Arti |
|---|---|---|
| `world` | `kki_arena.sdf` | Nama file world (`.sdf`) di `worlds/`. Alt: `pool_empty.sdf`. |
| `headless` | `false` | `true` = gz-sim tanpa GUI (server saja), untuk CI/mesin tanpa GPU |
| `rov_random_spawn` | `true` | Spawn ROV acak dekat salah satu dinding kolam tiap run |
| `rov_x` / `rov_y` / `rov_z` | 0.0 / 0.0 / -0.5 | Posisi spawn manual (dipakai bila `rov_random_spawn:=false`) |
| `rov_wall_margin` | 0.5 | Jarak aman ROV dari dinding fisik saat spawn acak |
| `spawn_seed` | `''` (kosong) | Fix seed pose spawn acak untuk replay/debug; kosong = acak penuh |
| `rov_arena_half` | 2.55 | Setengah lebar kolam (dinding di ±nilai ini) |
| `spawn_delay` | 3.0 | Jeda (detik) sebelum spawn ROV, cegah race condition dengan service `create` |
| `qr_letter` | `''` (kosong) | Huruf QR payload A/B/C/D; kosong = random + posisi acak |
| `payload_x` / `payload_y` | 0.4 / 0.04 | Posisi payload (m), dipakai bila `qr_letter` diset eksplisit |

### `src/hydroships_bringup/launch/hydroships_sim.launch.py` (M1)

| Arg | Default |
|---|---|
| `headless` | `false` |
| `world` | `kki_arena.sdf` |

### `src/hydroships_bringup/launch/hydroships_stabilized.launch.py` (M2)

Semua arg `sim.launch.py` di atas, plus tidak ada tambahan — meneruskan langsung.

### `src/hydroships_bringup/launch/hydroships_mission.launch.py` (M6, full autonomy)

Semua arg `hydroships_stabilized.launch.py`, plus:

| Arg | Default | Arti |
|---|---|---|
| `start_state` | `DIVE` | State awal FSM: `DIVE`/`APPROACH_QR`/`GRAB`/`NAV_WALL`/`HANG`/`SURFACE`/`WAIT_TRIGGER`/`APPROACH_HOOK`/`AUTO_RELEASE` |
| `start_wall` | `''` (kosong) | Seed wall target A/B/C/D untuk testing mid-state |
| `scan_depth` | `0.30` | Kedalaman scan QR (m) — lebih dalam = QR lebih besar di frame tapi FOV menyempit |
| `cam_gripper_dx` | `0.16` | Koreksi offset kamera bawah ke gripper (m) |
| `hook_size_stop` | `0.35` | Ambang ukuran hook (ternormalisasi) untuk berhenti approach |
| `hook_center_tol` | `0.15` | Toleransi pemusatan visual servo hook |
| `hook_max_age` | `1.0` | Umur maksimum (detik) deteksi hook sebelum fallback ke target odometri |
| `t_approach` | `25.0` | Timeout (detik) state `APPROACH_HOOK` |

### `src/hydroships_bringup/launch/hydroships_gui.launch.py` (M7, GUI bridge)

| Arg | Default | Arti |
|---|---|---|
| `headless` | `false` | |
| `world` | `kki_arena.sdf` | |
| `gui_host` | `127.0.0.1` | Alamat IP tujuan telemetri UDP (laptop GUI) |
| `cmd_port` | `14550` | Port UDP untuk menerima command dari GUI |
| `telem_port` | `14551` | Port UDP untuk mengirim telemetri ke GUI |

### `src/hydroships_control/launch/teleop.launch.py`

Tidak ada argumen — hanya menjalankan node `thruster_allocator`. `teleop_keyboard`
dijalankan terpisah via `ros2 run`.

---

## 5. Parameter node `mission_fsm` (declare_parameter, bukan file YAML terpisah)

Node ini tidak punya file YAML sendiri — semua param dideklarasikan di kode
(`mission_fsm.py`) dan sebagian di-override via launch argument (lihat §4 di atas).
Daftar param lengkap (nama — default): `start_state='DIVE'`, `wall_order='random'`,
`start_delay=3.0`, `start_wall=''`, `surge_force=25.0`, `depth_bottom=0.70`,
`depth_surface=0.08`, `depth_tol=0.06`, `hook_depth=0.45`, `yaw_tol_deg=10.0`,
`qr_max_age=1.5`, `payload_x=0.4`, `payload_y=0.0`, `scan_depth=0.30`,
`approach_kp=90.0`, `approach_kd=140.0`, `approach_fmax=16.0`, `approach_tol=0.06`,
`wall_dist=2.30`, `hook_dist=0.30`, `hook_lateral_offset=0.0`, `nav_tol=0.20`,
`nav_fmax=22.0`, `hold_settle_s=2.0`, `t_dive=20.0`, `t_scan=45.0`, `t_grab=10.0`,
`t_nav=30.0`, `t_hang=20.0`, `t_surface=20.0`, `t_wait_trigger=600.0`,
`t_release=30.0`, `t_approach=25.0`, `hook_max_age=1.0`, `hook_kp_surge=40.0`,
`hook_kd_surge=30.0`, `hook_kp_sway=45.0`, `hook_kd_sway=30.0`, `hook_kp_depth=0.25`,
`hook_size_stop=0.35`, `hook_center_tol=0.15`, `hook_fmax=16.0`,
`hook_depth_range=0.20`, `t_nav_qr=30.0`, `qr_center_tol=0.12`, `qr_servo_gain=0.15`,
`qr_servo_sign=1.0`, `cam_gripper_dx=0.16`, `gripper_base_dx=0.18`,
`qr_floor_z=-0.894`, `cam_bottom_dz=0.18`, `cam_vfov_half_tan=0.6293`,
`ey_target_max=0.8`.

Konstanta tetap (bukan param, hardcoded di kode): `WALL_HEADING_DEG =
{'A': 270.0, 'B': 90.0, 'C': 0.0, 'D': 180.0}`.

Penjelasan makna & panduan tuning param FSM ini: `docs/TUNING_GUIDE.md` §2–3.
