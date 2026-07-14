# Integrasi GUI-ROV ↔ hydroships (ROS 2) — Analisis Selisih & Adapter

Dokumen ini membandingkan antarmuka repo GUI tim **Customize5773/GUI-ROV** dengan
kontrak topik ROS 2 hydroships (`docs/ARCHITECTURE.md`), lalu menjelaskan adapter
yang dipakai untuk menjembatani keduanya **tanpa mengubah node inti** (stabilizer,
mission_fsm, thruster_allocator).

> Status: kode adapter & detektor hook **sudah dibuat** tetapi **BELUM diverifikasi
> end-to-end dengan GUI live** — lihat PROBLEM.md. Gain/tanda masih estimasi.

## 1. Temuan utama: GUI-ROV bukan ROS 2

GUI-ROV **tidak berbicara ROS 2 sama sekali**. Arsitekturnya:

```
Dashboard web (public/js) ──WebSocket──► server.js (Node) ──UDP JSON──►
   rov_agent.py / autonomy/rov_link.py ──pymavlink MANUAL_CONTROL──► Pixhawk/ArduSub
```

- **Perintah GUI → ROV**: datagram UDP JSON `{"name": <str>, "value": <val>}`.
  (dari `rov_agent.py` `command_listener`)
- **Telemetri ROV → GUI**: UDP JSON `{heading, depth, roll, pitch, temp, voltage,
  armed, light, mode, ts}`.
- **Kontrol**: axis joystick **persen −100..100** (`surge/sway/yaw/heave`) →
  MAVLink `MANUAL_CONTROL` (x/y/z/r). Gripper & lampu = servo PWM
  (`MAV_CMD_DO_SET_SERVO`, open 1900 / close 1100).

Akibatnya **transport-nya beda** (DDS vs UDP/MAVLink), bukan sekadar nama/tipe
topik. **Remap `--ros-args -r from:=to` TIDAK bisa** menjembatani ini → wajib
node adapter.

## 2. Tabel selisih antarmuka

| Aspek | GUI-ROV | hydroships (ROS 2) | Selisih & penanganan |
|------|---------|--------------------|----------------------|
| Transport | UDP JSON + MAVLink | ROS 2 DDS topics | **Beda total** → node adapter `gui_bridge` |
| Kontrol manual | `surge/sway/yaw/heave` **persen −100..100** (JSON name/value) | `/hydroships/cmd_vel` `Twist` **wrench N / N·m** | Unit & tipe beda → adapter skala persen→gaya |
| Sumbu throttle | `heave` persen (z: 0..1000 di MAVLink) | `linear.z` gaya (N) | Adapter map heave→Fz |
| Yaw | `yaw` persen | `angular.z` torsi (N·m) | Adapter map yaw→Mz |
| Arm/disarm | `{"name":"arm"}` | (sim selalu aktif) | Adapter simpan status; disarm→wrench nol |
| Stop/failsafe | `{"name":"stop"}` | — | Adapter netralkan + disarm |
| Gripper | servo PWM open/close | `/hydroships/gripper/command` String | Adapter passthrough "open"/"close" |
| Heading telemetri | `heading` **derajat 0..360** | `/hydroships/odom` yaw **rad REP-103** | Adapter konversi rad→deg |
| Depth telemetri | `depth` m (positif ke bawah) | `/hydroships/depth` m (≥0) | Sama arah → passthrough |
| Roll/pitch | derajat | odom quaternion (rad) | Adapter quaternion→deg |
| Visual servo hook | `autonomy/vision/hook_detect.py` (murni, non-ROS) | belum ada (APPROACH_HOOK timed) | **Di-port** jadi node `hook_detector` |

Frame/unit yang perlu kalibrasi lapangan: penyelarasan **heading kompas** (offset
0° kompas vs +x REP-103) dan **tanda sumbu** (surge/sway/yaw/heave) — ditandai
VERIFY karena bergantung orientasi kamera & konfigurasi ArduSub nyata.

## 3. Penanganan (tanpa mengubah node inti)

### 3a. Node adapter `gui_bridge` (hydroships_control)
Menerjemahkan dua arah, memakai titik-masuk yang SUDAH ada:
- **GUI→ROS**: UDP JSON `{name,value}` → `/hydroships/cmd_vel` (Twist wrench) &
  `/hydroships/gripper/command` (String). Sama seperti teleop_keyboard→allocator,
  jadi stabilizer/allocator tak perlu tahu sumbernya.
- **ROS→GUI**: `/hydroships/odom` + `/hydroships/depth` → telemetri UDP JSON.
- Logika murni (skala persen→wrench, rad→deg, failsafe) di `gui_bridge_logic.py`,
  **teruji headless** (`test/test_gui_bridge.py`).

Jalankan: `ros2 run hydroships_control gui_bridge` (default dengar UDP :14550,
telemetri → 127.0.0.1:14551, cocok port GUI). Node inti tak disentuh.

### 3b. Node `hook_detector` (port GUI-ROV) untuk APPROACH_HOOK
`autonomy/vision/hook_detect.py` (`detect_hook`, contour/CLAHE→Hough, murni cv2)
di-port jadi node ROS pola `qr_detector`: baca `/hydroships/camera_front/image_raw`,
publish `/hydroships/hook_offset` (PointStamped ex/ey/size — konvensi sama qr_offset).
`mission_fsm` state `APPROACH_HOOK` kini **servo** ke hook (sejajarkan heading dari
offset-x, maju hingga size ≥ ambang) menggantikan gerak *timed*; **fallback timed**
tetap ada bila deteksi tak tersedia (aman).

## 4. Yang BELUM (VERIFY/OPEN)
- Verifikasi end-to-end dgn GUI live (kirim joystick nyata → ROV sim bergerak;
  telemetri muncul di dashboard). Belum dijalankan.
- Kalibrasi gain persen→N, offset heading kompas, dan tanda sumbu.
- Tuning ambang deteksi hook di render kamera sim (nilai default = uji-meja).
- Servo hook memakai heading+surge sederhana; pose-based (PnP) menyusul bila
  `camera_info` hook dipetakan.
