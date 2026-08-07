# Nodes Reference

Referensi formal setiap node/modul Python di `src/hydroships_control/hydroships_control/`
(10 node ROS2 + 7 modul logika murni) plus 3 skrip pendukung di paket lain. Total 18
file `.py` (di luar `__init__.py`). Untuk arti tiap parameter secara individual lihat
`docs/CONFIG_REFERENCE.md`; untuk cara tuning lihat `docs/TUNING_GUIDE.md`.

---

## Node ROS2 (punya entry point di `setup.py`, dijalankan via `ros2 run`/launch)

### `thruster_allocator` — `thruster_allocator.py`
- **Peran:** Ubah wrench 6-DOF menjadi gaya per-thruster (N) via alokasi damped
  pseudo-inverse.
- **Subscribe:** `/hydroships/cmd_vel` (`geometry_msgs/Twist`, dipakai sebagai wrench:
  `linear`=gaya N, `angular`=torsi N·m)
- **Publish:** `/hydroships/thruster_1/thrust` … `/hydroships/thruster_6/thrust` (`std_msgs/Float64`, N)
- **Param:** `alloc_damping` (default 0.1)
- **Bergantung pada:** `allocation.py` (matriks TAM, konstanta `THRUSTERS`, `MIN_THRUST`/`MAX_THRUST`)
- **Perilaku khusus:** watchdog — bila `/hydroships/cmd_vel` berhenti >0.5s, semua thruster dinolkan.

### `teleop_keyboard` — `teleop_keyboard.py`
- **Peran:** Kontrol manual langsung 6-DOF via keyboard, bypass stabilizer.
- **Publish:** `/hydroships/cmd_vel` (`Twist`, wrench langsung)
- **Kontrol:** w/s=surge, a/d=sway, i/k=heave, j/l=yaw, u/o=roll, t/g=pitch, spasi=stop.

### `teleop_stabilized` — `teleop_stabilized.py`
- **Peran:** Kontrol manual horizontal + setpoint depth/heading, bekerja lewat `stabilizer`.
- **Publish:** `/hydroships/manual/cmd` (`Twist`, Fx/Fy saja), `/hydroships/setpoint/depth`
  (`Float64`), `/hydroships/setpoint/heading` (`Float64`)

### `stabilizer` — `stabilizer.py`
- **Peran:** 4 loop PID independen (depth, heading, pitch, roll) → hasilkan wrench penuh.
- **Subscribe:** `/hydroships/odom` (`nav_msgs/Odometry`), `/hydroships/manual/cmd`
  (`Twist`, Fx/Fy passthrough), `/hydroships/setpoint/depth` (`Float64`),
  `/hydroships/setpoint/heading` (`Float64`)
- **Publish:** `/hydroships/cmd_vel` (`Twist`, wrench lengkap)
- **Param:** lihat `config/gains.yaml` — `rate`, 4 blok gain (`depth`/`heading`/`pitch`/`roll`),
  `buoyancy_ff`, 4 target default, 4 flag `enable_*_hold`.
- **Bergantung pada:** `pid.py` (`class PID`).

### `depth_publisher` — `depth_publisher.py`
- **Peran:** Turunkan kedalaman (m, konvensi ≥0) dari pose odometry.
- **Subscribe:** `/hydroships/odom`
- **Publish:** `/hydroships/depth` (`std_msgs/Float64`)
- **Catatan hardware:** di sim ini ground-truth Gazebo; di ROV fisik harus diganti
  driver sensor MS5837 nyata yang publish ke topic sama (lihat `docs/HARDWARE.md`).

### `qr_detector` — `qr_detector.py`
- **Peran:** Decode QR code dari frame kamera (bottom + front), hasilkan huruf + offset piksel.
- **Subscribe:** topic kamera sesuai param `image_topics` (list) — `sensor_msgs/Image` + `CameraInfo`
- **Publish:** `/hydroships/qr_result` (`std_msgs/String`, isi A/B/C/D), `/hydroships/qr_offset`
  (`geometry_msgs/PointStamped`, `frame_id` menandai kamera asal)
- **Param:** `image_topics` (list), `max_rate` (default 5.0 Hz)
- **Bergantung pada:** `qr_logic.py` (`robust_decode`, `parse_wall`, `offset_from_points`),
  `image_util.py` (konversi `Image`→BGR).

### `hook_detector` — `hook_detector.py`
- **Peran:** Deteksi hook/gantungan dinding dari kamera depan (color/contour based),
  hasilkan offset ternormalisasi + ukuran (dipakai visual servo).
- **Subscribe:** `image_topic` (default `/hydroships/camera_front/image_raw`)
- **Publish:** `/hydroships/hook_offset` (`PointStamped`; x=ex, y=ey, z=size, semua ternormalisasi)
- **Param:** `image_topic`, `max_rate` (default 5.0), `min_area`
- **Bergantung pada:** `hook_logic.py` (`normalize_hook_offset`), `image_util.py`.

### `gripper_controller` — `gripper_controller.py`
- **Peran:** Terima command open/close, kelola state safety gripper, trigger attach/detach
  fisik payload di sim.
- **Subscribe:** `/hydroships/gripper/command` (`std_msgs/String`, "open"/"close"),
  `/hydroships/qr_offset` (safety gating — hanya boleh close bila QR dalam toleransi),
  `/hydroships/payload/spawned` (`std_msgs/Empty`, sinyal dari `payload_spawner`)
- **Publish:** `/hydroships/gripper_left/cmd`, `/hydroships/gripper_right/cmd` (`Float64`
  rad, nilai identik kiri-kanan), `/hydroships/gripper/attach`, `/hydroships/gripper/detach`
  (`Empty`, trigger `DetachableJoint` plugin gz)
- **Bergantung pada:** `gripper_logic.py` (`class GripperLogic`).
- **Perilaku khusus:** timer auto-detach saat startup (fallback safety).

### `gui_bridge` — `gui_bridge.py`
- **Peran:** Adapter UDP-JSON ↔ ROS2 untuk GUI eksternal tim (`Customize5773/GUI-ROV`,
  bukan ROS2 native). **Bukan** node MAVLink — lihat `docs/GUI-INTEGRATION.md`.
- **Subscribe:** `/hydroships/odom`, `/hydroships/depth`
- **Publish:** `/hydroships/cmd_vel` (`Twist`, dari command UDP masuk), `/hydroships/gripper/command`
  (`String`)
- **Param (dari launch, bukan file YAML):** `cmd_port` (default 14550), `telem_host`,
  `telem_port` (default 14551)
- **Bergantung pada:** `gui_bridge_logic.py` (`class GuiBridgeLogic`, konversi persen↔Newton).

### `mission_fsm` — `mission_fsm.py`
- **Peran:** State machine autonomi penuh misi kontes (lihat detail state di bawah).
- **Node paling kompleks** — lihat §"State Machine `mission_fsm`" di bawah untuk detail
  penuh subscribe/publish/param/scoring.

---

## Modul Logika Murni (tanpa `rclpy`, diuji unit test langsung, diimpor node di atas)

| Modul | Fungsi/kelas utama | Dipakai oleh | Diuji di |
|---|---|---|---|
| `pid.py` | `class PID(kp, ki, kd, ...)` — `.update(error, measurement, dt)`, `.reset()`, `.set_gains()`; fungsi `wrap_to_pi()` | `stabilizer.py` | `test_pid.py` |
| `allocation.py` | `build_allocation_matrix(thrusters)`, `build_damped_pinv(tam, damping)`, `allocate(wrench, tam_pinv, lo, hi)`; konstanta `THRUSTERS`, `MIN_THRUST=-40`, `MAX_THRUST=50` | `thruster_allocator.py` | `test_allocation.py` |
| `qr_logic.py` | `robust_decode(img, detector)`, `parse_wall(data)`, `offset_from_points(pts, shape)`, `_to_gray`; konstanta `CLAHE_CLIP=3.0`, `CLAHE_TILE=8`, `ADAPT_BLOCK=31`, `ADAPT_C=5`, `UPSCALE=2.0` | `qr_detector.py` | `test_qr_logic.py` |
| `hook_logic.py` | `normalize_hook_offset(center, area, frame_w, frame_h)`, `class HookServoGains(...)`, `hook_servo(off, vx, vy, hook_depth, gains)` (PD visual-servo) | `hook_detector.py`, `mission_fsm.py` | `test_hook_servo.py` |
| `gripper_logic.py` | `class GripperLogic(max_offset=0.30, min_size=0.12, offset_timeout=1.5, ...)` — `.update_offset()`, `.is_safe()`, `.on_command()`, `._do_open()`, `._do_close()`, `.force_detach()`, `.startup_detach()` | `gripper_controller.py` | `test_gripper.py` |
| `gui_bridge_logic.py` | `class GuiBridgeLogic(surge_gain=0.40, sway_gain=0.40, heave_gain=0.30, yaw_gain=0.12, ...)` — `.on_command()`, `.wrench()`, `.yaw_to_heading_deg()` (static), `.build_telemetry()` | `gui_bridge.py` | `test_gui_bridge.py` |
| `image_util.py` | `channels_for_encoding(enc)`, `reshape_with_step(buf, h, w, ch, step)`, `image_msg_to_bgr(msg)` | `qr_detector.py`, `hook_detector.py` | `test_image_util.py` |

---

## Skrip pendukung (bukan node ROS2 dengan entry_point, dijalankan langsung/`ros2 run` ad-hoc)

- **`src/hydroships_gazebo/scripts/payload_spawner.py`** — spawn model payload QR
  di Gazebo (via `ros2 run ros_gz_sim create`), publish `/hydroships/payload_pose`
  (`PointStamped`, QoS TRANSIENT_LOCAL/latched) dan `/hydroships/payload/spawned` (`Empty`).
- **`src/hydroships_gazebo/scripts/generate_qr.py`** — CLI offline, generate asset gambar
  QR (`qr_A.png`…`qr_D.png`) dipakai world/model. Tidak jalan sebagai node runtime.
- **`src/hydroships_description/scripts/estimate_mass_inertia.py`** — CLI offline untuk
  menghitung massa/inersia/CoG dari dimensi box, dipakai mengisi `rov_params.yaml`.

---

## State Machine `mission_fsm.py` — Detail Penuh

### Diagram alur

```
IDLE → DIVE → APPROACH_QR → GRAB → NAV_WALL → HANG → SURFACE → WAIT_TRIGGER
     → APPROACH_HOOK → AUTO_RELEASE → (kembali ke DIVE | DONE)
ABORT: dapat terjadi dari state manapun (timeout / guard gagal)
```

### Enum state (`class St(Enum)`)
`IDLE, DIVE, APPROACH_QR, GRAB, NAV_WALL, HANG, SURFACE, WAIT_TRIGGER,
APPROACH_HOOK, AUTO_RELEASE, DONE, ABORT`

### Subscribe
| Topic | Tipe | Catatan |
|---|---|---|
| `/hydroships/depth` | `std_msgs/Float64` | |
| `/hydroships/odom` | `nav_msgs/Odometry` | |
| `/hydroships/qr_result` | `std_msgs/String` | A/B/C/D |
| `/hydroships/qr_offset` | `geometry_msgs/PointStamped` | filter `frame_id=='camera_bottom_link'` |
| `/hydroships/hook_offset` | `geometry_msgs/PointStamped` | filter `frame_id=='camera_front_link'` |
| `/hydroships/payload_pose` | `PointStamped` | QoS TRANSIENT_LOCAL depth=1 (latched) |
| `/hydroships/mission/start_autonomous` | `std_msgs/Empty` | trigger keluar dari `WAIT_TRIGGER` |

### Publish
| Topic | Tipe | Catatan |
|---|---|---|
| `/hydroships/setpoint/depth` | `Float64` | negatif = lebih dalam |
| `/hydroships/setpoint/heading` | `Float64` | rad |
| `/hydroships/manual/cmd` | `Twist` | Fx/Fy gaya horizontal N |
| `/hydroships/gripper/command` | `String` | "open"/"close" — **⚠️ publisher `pub_grip` dideklarasikan tapi TIDAK PERNAH di-`.publish()` di state `_st_grab`; lihat bug blocking di `docs/STATUS.md` dan `docs/TROUBLESHOOTING.md`** |
| `/hydroships/gripper/detach` | `Empty` | |
| `/hydroships/qr_request` | `Empty` | |

### Konstanta tetap
`WALL_HEADING_DEG = {'A': 270.0, 'B': 90.0, 'C': 0.0, 'D': 180.0}` — dikonfirmasi
cocok dengan posisi hook arena SDF (A=-Y, B=+Y, C=+X, D=-X).

### Scoring internal
`{'m1':0, 'm2':0, 'm3':0, 'm4':0, 'm5':0}`, total maksimum 15+15+15+15+40=100
(cermin rubrik penilaian misi resmi KKI, lihat panduan lomba §4.7.4).

### Param
Daftar lengkap 40+ parameter (timeout, gain visual servo, toleransi geometri):
lihat `docs/CONFIG_REFERENCE.md` §5. Panduan tuning per-state: `docs/TUNING_GUIDE.md` §2.

---

## Cakupan Test

Semua di `src/hydroships_control/test/`, dijalankan via
`colcon test --packages-select hydroships_control` (lihat `docs/CI_CD.md`):

| File test | Menguji |
|---|---|
| `test_allocation.py` | `allocation.py` (matriks TAM, pinv, alokasi) |
| `test_gripper.py` | `gripper_logic.GripperLogic` |
| `test_gui_bridge.py` | `gui_bridge_logic.GuiBridgeLogic` |
| `test_hook_servo.py` | `hook_logic.hook_servo` |
| `test_image_util.py` | `image_util` (konversi Image→BGR) |
| `test_pid.py` | `pid.PID` |
| `test_qr_ey_target.py` | `mission_fsm.qr_ey_target` (fungsi geometri) |
| `test_qr_logic.py` | `qr_logic` (`robust_decode`/`parse_wall`/`offset_from_points`), termasuk fixture regresi frame sim asli (`qr_sim_bottom_A.png`) |

Semua test bersifat **pure-logic/headless** — tidak butuh Gazebo, tidak ada
integration test node ROS2 penuh. Status terakhir: 76/76 lolos (per `CHANGELOG.md`
2026-08-06).
