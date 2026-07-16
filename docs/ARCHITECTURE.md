# Arsitektur Simulasi HYDROships (KKI 2026)

Stack: **ROS 2 Humble + Gazebo Fortress** (gz-sim 6) + `ros_gz` bridge.

## Diagram aliran (Milestone 1–2)

```
  MODE MANUAL (M1):
    teleop_keyboard ──► /hydroships/cmd_vel (Twist = wrench 6-DOF)

  MODE STABILIZED (M2):
    teleop_stabilized ─► /hydroships/manual/cmd      (Fx, Fy)
                      └► /hydroships/setpoint/depth   (Float64)
                      └► /hydroships/setpoint/heading (Float64)
                                   │
                              stabilizer  (PID depth-hold + heading-hold)
                                   │  Fz,Mz dari PID; Fx,Fy,Mx,My pass-through
                                   ▼
                        /hydroships/cmd_vel (Twist = wrench 6-DOF)
                                   │
                                   ▼
                       thruster_allocator  (f = pinv(TAM)·wrench)
                                   │
                    /hydroships/thruster_{1..6}/thrust (Float64, N)
                                   │
                             ros_gz_bridge
                                   │
                                   ▼
                 Gazebo Fortress ── plugin Thruster ×6
                        │  plugin Hydrodynamics (drag+added mass)
                        │  plugin Buoyancy (world)
                        ▼
                 /model/hydroships/odometry ─► bridge ─► /hydroships/odom
                                   (Odometry: umpan balik z & yaw ke stabilizer)
```

## Kontrak interface topic (untuk GUI / GCS tim)

Node inti membaca/menulis topic ROS 2 standar berikut. M1–M2 (kendali & odometry)
terverifikasi di sim; kamera/QR/kedalaman (M3), manipulator (M5), integrasi GUI (M7)
**kodenya sudah dibuat** tetapi sebagian besar verifikasi runtime masih tertunda
(mesin dev tanpa ROS2/Gazebo — lihat [`STATUS.md`](STATUS.md)).

> **Catatan GUI (M7):** repo GUI tim (Customize5773/GUI-ROV) **tidak memakai ROS 2**
> melainkan UDP-JSON/MAVLink. Jembatan ke topik di bawah dilakukan node adapter
> `gui_bridge` (bukan remap langsung) — lihat `docs/GUI-INTEGRATION.md`.
>
> Legenda status: ✅ jalan & terverifikasi di sim · ⏳ direncanakan/menyusul ·
> 🧪 **kode ada, belum diverifikasi** end-to-end (mis. dgn GUI live / grasp fisik).

| Topic | Tipe | Arah | Status |
|-------|------|------|--------|
| `/hydroships/cmd_vel` | `geometry_msgs/Twist` | teleop/stabilizer → allocator | ✅ M1 (wrench body: linear=gaya N, angular=torsi N·m) |
| `/hydroships/thruster_{1..6}/thrust` | `std_msgs/Float64` | allocator → sim | ✅ M1 |
| `/hydroships/odom` | `nav_msgs/Odometry` | sim → GUI/stabilizer | ✅ M1 |
| `/clock` | `rosgraph_msgs/Clock` | sim → semua | ✅ M1 |
| `/hydroships/manual/cmd` | `geometry_msgs/Twist` | GUI/pilot → stabilizer | ✅ M2 (horizontal: Fx, Fy) |
| `/hydroships/setpoint/depth` | `std_msgs/Float64` | GUI/pilot → stabilizer | ✅ M2 (target kedalaman, m) |
| `/hydroships/setpoint/heading` | `std_msgs/Float64` | GUI/pilot → stabilizer | ✅ M2 (target yaw, rad) |
| `/hydroships/depth` | `std_msgs/Float64` | `depth_publisher` → GUI/FSM | 🧪 M3 (kode ada; verifikasi runtime tertunda) |
| `/hydroships/camera_front/image_raw` | `sensor_msgs/Image` | sim → GUI/detektor | 🧪 M3 (render kamera; verifikasi runtime tertunda) |
| `/hydroships/camera_bottom/image_raw` | `sensor_msgs/Image` | sim → GUI/detektor | 🧪 M3 (render kamera; verifikasi runtime tertunda) |
| `/hydroships/camera_front/camera_info` | `sensor_msgs/CameraInfo` | sim → servo/PBVS | 🧪 M3 (intrinsics **sim**, bukan kalibrasi hardware — lihat catatan) |
| `/hydroships/camera_bottom/camera_info` | `sensor_msgs/CameraInfo` | sim → servo/PBVS | 🧪 M3 (idem) |
| `/hydroships/qr_result` | `std_msgs/String` | `qr_detector` → FSM/GUI | 🧪 M3 (kode ada; keterbacaan QR runtime tertunda) |
| `/hydroships/qr_offset` | `geometry_msgs/PointStamped` | `qr_detector` → FSM/`gripper_controller` | 🧪 M3 (offset piksel ternorm. + ukuran; verifikasi runtime tertunda) |
| `/hydroships/hook_offset` | `geometry_msgs/PointStamped` | hook_detector → FSM | 🧪 M7 (visual servo APPROACH_HOOK; port GUI-ROV) |
| `/hydroships/gripper/command` | `std_msgs/String` | GUI/FSM → gripper_controller | 🧪 M5 ("open"/"close") |
| `/hydroships/gripper_jaw/cmd` | `std_msgs/Float64` | gripper_controller → sim | 🧪 M5 (sudut jari kosmetik, rad) |
| `/hydroships/gripper/attach` | `std_msgs/Empty` | gripper_controller → sim | 🧪 M5 (trigger DetachableJoint attach) |
| `/hydroships/gripper/detach` | `std_msgs/Empty` | gripper_controller → sim | 🧪 M5 (trigger DetachableJoint detach) |

> **Catatan CameraInfo (intrinsics):** topik `camera_info` dijembatani dari Gazebo
> (`bridge.yaml`) dan matriks K disimpan oleh `qr_detector`. **PENTING — bedakan dua hal:**
> (1) *jalur topic camera_info* → **selesai** bila `ros2 topic echo .../camera_info --once`
> menunjukkan K/D/width/height terisi masuk akal saat sim jalan (masih perlu verifikasi
> runtime); (2) *kalibrasi ke kamera fisik ROV asli* → **tetap OPEN**: intrinsics ini murni
> kalkulasi Gazebo dari FOV/resolusi SDF, **bukan** kalibrasi hardware. Jangan pakai K untuk
> estimasi jarak riil sampai data kalibrasi kamera fisik tersedia (lihat PROBLEM.md).

> Saat integrasi GUI (M7): jika GUI tim mengharapkan nama topic berbeda,
> cukup remap di launch (`--ros-args -r from:=to`) atau sesuaikan tabel ini —
> tipe pesannya sudah standar sehingga pemetaan mudah.

## Paket

| Paket | Isi |
|-------|-----|
| `hydroships_description` | URDF/xacro model + plugin gz (Thruster, Hydrodynamics, Odometry) |
| `hydroships_gazebo` | world kolam, `bridge.yaml`, `sim.launch.py` |
| `hydroships_control` | `thruster_allocator`, `teleop_keyboard`, `stabilizer`, `mission_fsm`, `qr_detector`, `gripper_controller`, `hook_detector`, `gui_bridge` (adapter GUI-ROV) |
| `hydroships_bringup` | launch top-level `hydroships_sim.launch.py` |

## File & model legacy

- **Model aktif ROV** = geometri **primitif** di `hydroships_description/urdf/hydroships.urdf.xacro`.
  Mesh berat lama **`model/rov.fbx` (FBX) & `meshes/rov.stl` (~12 MB) sudah DIHAPUS** dari
  repo (tak lagi dirujuk URDF; `meshes/` kini hanya berisi `README.md` placeholder). Catatan
  lama yang menyebut "`model/rov.fbx` 48 MB dibiarkan di repo" **tidak berlaku lagi**.
  Riwayat lengkap: [`CHANGELOG.md`](CHANGELOG.md).
- **Xacro legacy** di `hydroships_description/urdf/Model lain/` (`rov.urdf.xacro`,
  `rov_kki2026.urdf.xacro`) **BUKAN model aktif**: keduanya masih di-track git namun merujuk
  mesh (`rov_hull_visual.stl`, `hull_visual.stl`, dll.) & nama paket (`rov_description`,
  `hydroship_description`) yang **tidak ada** di workspace ini. Disimpan sebagai referensi
  desain. Saran (opsional, **jangan hapus tanpa diminta**): arsipkan ke luar `urdf/`,
  pindahkan mesh besar (bila kelak ditambahkan) ke **Git LFS**, atau hapus dari tracking bila
  sudah tak dibutuhkan sebagai referensi.

## Keputusan desain

- **Fortress dipilih** karena plugin Buoyancy/Hydrodynamics/Thruster sudah bawaan
  → tidak perlu plugin fisika underwater custom.
- **Wrench di `/hydroships/cmd_vel`**: satu titik masuk untuk teleop maupun PID
  autonomy nanti, sehingga allocator tidak perlu tahu sumber perintah.
- **Watchdog di allocator**: bila perintah berhenti > 0,5 s, thruster dinolkan
  (aman bila teleop/GUI putus).
