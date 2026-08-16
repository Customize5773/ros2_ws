# Integrasi GUI-ROV ↔ hydroships (ROS 2) — Analisis Selisih & Adapter

Dokumen ini membandingkan antarmuka repo GUI tim **Customize5773/GUI-ROV** dengan
kontrak topik ROS 2 hydroships (`docs/ARCHITECTURE.md`), lalu menjelaskan adapter
yang dipakai untuk menjembatani keduanya **tanpa mengubah node inti** (stabilizer,
mission_fsm, thruster_allocator).

> Status: kode adapter & detektor hook **sudah diverifikasi end-to-end dengan
> dashboard GUI-ROV asli (2026-08-13, retest 2026-08-16)** untuk arm/disarm,
> yaw, gripper, dan light — lihat [`STATUS.md`](STATUS.md) untuk detail &
> item yang masih terbuka (roll/pitch spike, offset kompas).

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
| Visual servo hook | `autonomy/vision/hook_detect.py` (murni, non-ROS) | semula APPROACH_HOOK *timed* | **Di-port** jadi node `hook_detector`; APPROACH_HOOK kini servo PD holonomik (fallback timed) |

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

Jalankan sim + adapter sekaligus (disarankan):
```
ros2 launch hydroships_bringup hydroships_gui.launch.py \
    gui_host:=192.168.2.1 cmd_port:=14550 telem_port:=14551
```
(sim + thruster_allocator + gui_bridge; `gui_host` = laptop GUI/server.js.)
Atau node adapter saja: `ros2 run hydroships_control gui_bridge` (default dengar
UDP :14550, telemetri → 127.0.0.1:14551). Node inti tak disentuh.

### 3b. Node `hook_detector` (port GUI-ROV) untuk APPROACH_HOOK
`autonomy/vision/hook_detect.py` (`detect_hook`, contour/CLAHE→Hough, murni cv2)
di-port jadi node ROS pola `qr_detector`: baca `/hydroships/camera_front/image_raw`,
publish `/hydroships/hook_offset` (PointStamped ex/ey/size — konvensi sama qr_offset).
`mission_fsm` state `APPROACH_HOOK` kini **servo PD holonomik** ke hook
(`hook_logic.hook_servo`: sway dari offset-x, surge dari ukuran-tampak, koreksi setpoint
kedalaman dari offset-y, semua dgn redaman kecepatan body-frame; heading di-hold menghadap
wall) menggantikan gerak *timed*; **fallback timed** tetap ada bila deteksi tak tersedia
(aman). Teruji headless `test/test_hook_servo.py`.

## 4. Yang BELUM (VERIFY/OPEN)
- **Live test 2026-08-13** (dashboard GUI-ROV asli, `server.js` lokal via
  `RPI_ADDR=127.0.0.1`) membuktikan arm/disarm, yaw, dan gripper open/close
  round-trip.
- **[RESOLVED 2026-08-16]** Tombol **light**: diverifikasi via dashboard GUI-ROV
  asli — `[CMD] light = true/false` diterima `gui_bridge`. Investigasi kode
  (`gui_bridge_logic.py:101-103`) mengonfirmasi ini **disengaja non-aktuasi**:
  cuma disimpan sbg status flag & di-echo balik ke telemetry, tak ada model
  lampu di sim/URDF saat ini jadi tak ada aksi ROS yang dipicu. Bukan bug.
  Detail: `P2-GUI-INVESTIGATION.md` §5b.
- Kalibrasi gain persen→N: **RESOLVED 2026-08-16** (`P2-GUI-INVESTIGATION.md`
  §3) — terverifikasi benar secara aljabar (`cmd = gain × value` exact) utk
  keempat axis. Offset heading kompas & tanda sumbu tetap OPEN (butuh ROV
  fisik utk validasi).
- **[⚠️ 2026-08-16, retest dgn dashboard asli TIDAK menutup sebagai
  non-issue]** Roll/pitch melonjak besar (±25-31°) yang tercatat di run
  2026-08-13: probe UDP sintetis single-axis (`p2-experiment.py`, mode
  `sustained`/`pulsed`, yaw 100%) tidak mereproduksi (peak 0.48°/0.37°) —
  tapi retest lanjutan dgn **dashboard GUI-ROV asli** (browser + input
  manual, kombinasi surge+yaw/sway+yaw, `ROS_DOMAIN_ID=77`) menghasilkan
  peak **6.40°/2.22°** — lebih tinggi dari probe sintetis (kombinasi axis
  manusia memang berkontribusi) tapi **masih ~4-5× di bawah** klaim asli.
  Fix thrust drop-out (`853f7ff`) sudah masuk di kedua test ini, jadi
  bukan penjelasan sisa gap. Kandidat tersisa: posisi/konteks arena
  spesifik (dekat dinding) saat observasi asli 2026-08-13 — retest ini
  pakai ROV di tengah arena (`rov_x:=0 rov_y:=0`), belum menguji skenario
  dekat-dinding. **Tetap OPEN**, jangan tandai RESOLVED. Detail eksperimen
  & angka lengkap: `P2-GUI-INVESTIGATION.md` §4 & §5a.
- **[RESOLVED 2026-08-15, dikonfirmasi ulang 2026-08-16]** Timer telemetri UDP
  dipacing steady/wall clock, bukan ROS/Gazebo simulation clock. Ini mencegah
  beban headless membuat rate GUI turun bersama real-time factor. Live test
  2026-08-13 sempat mengukur ~3 Hz aktual vs target 10 Hz (sebelum fix).
  Profil ulang 2026-08-16 (`tools/p2-gui-telem-profile.py`, port 14551, 15s,
  paralel dengan eksperimen roll/pitch di atas): **10.07 Hz efektif, 151
  paket, interval median 99.99ms, 93.8% window 1-detik tepat 10 paket** — gap
  3Hz-vs-10Hz **tidak lagi terlihat**, fix pacing terkonfirmasi bekerja.
- Tuning ambang deteksi hook di render kamera sim (nilai default = uji-meja).
- Servo hook = PD holonomik IBVS (sway+surge+koreksi-depth, image-based tanpa kalibrasi);
  pose-based (solvePnP/PBVS) menyusul bila kalibrasi kamera fisik hook tersedia.
