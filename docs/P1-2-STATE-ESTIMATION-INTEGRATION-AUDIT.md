# P1-2: Audit Integrasi Estimasi State — `ros2_ws`

Audit read-only. Tidak ada kode, config, launch, FSM, controller, `gui_bridge`, `qr_logic.py`
(termasuk `robust_decode`/urutan kandidat), atau apa pun di `GUI-ROV` yang diubah untuk
menghasilkan dokumen ini. Dokumen ini melanjutkan `docs/P1-0-ARCHITECTURE-AUDIT.md` (audit
antarmuka lintas-repo) dan `docs/P1-1-ARCHITECTURE-DECISION.md` (**Keputusan A — ROS2-native
control adalah otoritas**: mission/FSM → autonomy ROS2 → stabilisasi ROS2 → alokasi thruster
ROS2 → antarmuka aktuator; Pixhawk/ArduSub bukan otoritas kendali).

Fokus dokumen ini: **dari mana `ros2_ws` sebenarnya mendapat "tahu di mana/bagaimana ROV
berada"** (pose, orientasi, kecepatan, kedalaman) — dan apakah sumber itu akan tetap ada saat
Gazebo diganti hardware asli.

---

## 1. Ringkasan eksekutif

`ros2_ws` **tidak punya node estimasi state sama sekali**. Satu-satunya sumber pose/orientasi/
kecepatan di seluruh workspace adalah `/hydroships/odom` (`nav_msgs/Odometry`), yang diterbitkan
langsung oleh plugin Gazebo `gz-sim-odometry-publisher-system` (`hydroships.urdf.xacro:403-410`)
— plugin ini membaca pose **ground-truth** model dari physics engine, bukan hasil fusi sensor
apa pun. `/hydroships/depth` (`depth_publisher.py`) hanyalah `max(0, -z)` dari `pose.z` topik
yang sama — bukan pembacaan sensor tekanan.

Sebuah IMU **sudah dijembatani** dari Gazebo ke ROS2 (`/hydroships/imu`, `sensor_msgs/Imu`,
`bridge.yaml`) tetapi **grep menyeluruh terhadap seluruh `src/*.py` untuk `hydroships/imu` dan
`from sensor_msgs.msg import Imu` tidak menghasilkan satu pun `create_subscription` di mana
pun** — topik ini dipancarkan tapi tidak pernah dibaca oleh node manapun. Temuan P1-0 tentang
ini dikonfirmasi ulang di sini dengan bukti langsung.

Konsekuensinya: setiap konsumen state (`stabilizer`, `mission_fsm`, `gui_bridge`,
`teleop_gamepad`) berasumsi implisit bahwa "odom" berarti pose/orientasi/kecepatan yang akurat
dan bertsempel-waktu benar — asumsi yang **hanya valid di simulasi**. Tidak ada jalur fallback,
tidak ada health-check kesegaran data odom, tidak ada estimator yang bisa menggantikan
ground-truth ini begitu Gazebo lepas dari gambar.

**Verdict singkat (detail di §akhir): simulation-ready, bukan target-control-ready.** Gap
estimasi state adalah blocker tunggal terbesar sebelum `ros2_ws` bisa dijalankan di ROV fisik
sama sekali — bukan hanya untuk akurasi, tapi karena **tidak ada penggantinya**.

---

## 2. Arsitektur estimasi state saat ini

```
Gazebo physics engine (ground-truth pose/twist model)
        │
        ▼
gz-sim-odometry-publisher-system (plugin, hydroships.urdf.xacro:403-410)
   odom_frame=odom, robot_base_frame=base_link, odom_publish_frequency=30 Hz
        │  gz topic: /model/hydroships/odometry (gz.msgs.Odometry)
        ▼
ros_gz_bridge (bridge.yaml, GZ_TO_ROS)
        │  ros topic: /hydroships/odom (nav_msgs/Odometry)
        ▼
   ┌────┴─────────────────────────────────────────────┐
   ▼                    ▼                    ▼         ▼
stabilizer.py     mission_fsm.py      gui_bridge.py  depth_publisher.py
(z,yaw,roll,pitch) (yaw,x,y,vx,vy)    (rpy utk telem) (z -> depth)
```

Tidak ada node fusi/EKF, tidak ada filter komplementer, tidak ada TF tree (dikonfirmasi ulang:
`grep -rn "tf2\|TransformBroadcaster" src/` — nol hasil, konsisten dengan temuan P1-0 §6). IMU
dijembatani tapi menjadi **dead-end topic** — tidak ada consumer.

Setiap node yang butuh yaw/roll/pitch **mengekstrak ulang dari quaternion `/hydroships/odom`
secara independen** dengan implementasi masing-masing (temuan DUPLIKASI P1-0 §6, dikonfirmasi
di sini dengan lokasi baris):
- `stabilizer.py:40-56` (`yaw_from_quaternion`, `roll_pitch_from_quaternion`)
- `mission_fsm.py` (yaw-only, dipanggil di `_on_odom`, fungsi `yaw_from_quaternion` diimpor —
  lihat §7)
- `gui_bridge.py:36-47` (`_yaw_rpy`, full RPY)

Tidak ada satu pun dari ketiganya membaca `/hydroships/imu`.

---

## 3. Inventaris sumber sensor/state

| Sumber | Topik ROS | Tipe pesan | Publisher | Consumer(s) | Rate | Semantik timestamp | Sim vs hardware | Estimated vs ground-truth |
|---|---|---|---|---|---|---|---|---|
| Pose+twist penuh | `/hydroships/odom` | `nav_msgs/Odometry` | Plugin Gazebo `OdometryPublisher` (`hydroships.urdf.xacro:403-410`) via `ros_gz_bridge` | `stabilizer.py`, `mission_fsm.py`, `gui_bridge.py`, `depth_publisher.py`, `teleop_gamepad.py` (grep) | 30 Hz (`odom_publish_frequency`) | Stempel dari clock sim (`use_sim_time`); tidak diverifikasi apakah `header.stamp` diisi benar oleh plugin (tidak diaudit lebih dalam — di luar cakupan kode Python) | **Sim-only** — plugin Gazebo, tidak ada padanan hardware apa pun | **100% ground-truth** — dibaca langsung dari state physics engine, bukan estimasi/fusi |
| Kedalaman | `/hydroships/depth` | `std_msgs/Float64` | `depth_publisher.py` (turunan `max(0,-z)` dari odom) | `mission_fsm.py`, `gui_bridge.py` | Mengikuti rate callback `/hydroships/odom` (≈30 Hz, tanpa throttle sendiri) | Tidak berstempel (`Float64` polos, tidak ada header) | **Sim-only** secara turunan (bergantung odom ground-truth); `HARDWARE.md` sudah punya rencana adapter (`depth_sensor_driver` MS5837 → topik sama) | Ground-truth (turunan langsung dari ground-truth odom, bukan sensor) |
| IMU | `/hydroships/imu` | `sensor_msgs/Imu` | Sensor Gazebo `imu_link` (`hydroships.urdf.xacro:246-250`) via `ros_gz_bridge` | **Tidak ada** — grep `create_subscription.*Imu` di seluruh `src/*.py`: nol hasil | Ditentukan oleh sensor plugin gz-sim (default per-`<update_rate>`, tidak diset eksplisit di xacro yang dibaca — lihat catatan) | N/A (tak ada consumer) | **Sim-only, dead-end** | Simulasi IMU (noise model gz default, bukan ground-truth sekaligus bukan hardware nyata) |
| Setpoint kedalaman (kontrol, bukan state) | `/hydroships/setpoint/depth` | `std_msgs/Float64` | `teleop_stabilized`, `teleop_gamepad`, `mission_fsm` | `stabilizer.py` | Event-driven | Tidak berstempel | Agnostik sim/hardware (murni target) | N/A — bukan pengukuran |
| Payload pose (ground-truth eksternal, dipakai autonomy) | `/hydroships/payload_pose` | `geometry_msgs/PointStamped`, QoS TRANSIENT_LOCAL | `payload_spawner` (di luar `hydroships_control`, latch-once) | `mission_fsm.py` (`_on_payload_pose`) | Sekali per spawn (latched) | Berstempel `PointStamped` tapi nilainya statis-per-run | **Sim-only leak eksplisit** — komentar kode sendiri: "Tanpa ini FSM navigasi ke param payload_x/y yg statis" | Ground-truth spawner, **tidak ada padanan hardware** — pada ROV fisik, posisi payload harus ditemukan lewat persepsi visual (QR), bukan dibaca dari ground-truth |
| Offset QR (visual, relatif) | `/hydroships/qr_offset` | `geometry_msgs/PointStamped` (overload: x/y=offset piksel ternorm., z=proxy ukuran) | `qr_detector.py` | `mission_fsm.py`, `gripper_controller.py` | Mengikuti rate frame kamera | Berstempel per-frame kamera | Sim kamera hari ini; jalur hardware sudah discope (`usb_cam`/`v4l2_camera`) tapi belum ada driver nyata | Diukur (visual), relatif terhadap kamera — bukan ground-truth absolut, tapi intrinsik kamera masih sim-only (lihat `ARCHITECTURE.md` catatan CameraInfo) |
| Offset hook (visual, relatif) | `/hydroships/hook_offset` | `geometry_msgs/PointStamped` | `hook_detector.py` | `mission_fsm.py` | Mengikuti rate frame kamera | Berstempel per-frame | Sama seperti QR offset | Diukur (visual), relatif |
| Voltage/temp | (tidak ada topik) | — | — | — | — | — | — | **Tidak diimplementasikan sama sekali** — `gui_bridge_logic.py` mengirim `0.0` hardcoded (P1-0 Finding #7) |

**Catatan penting:** tidak ada satu pun baris di tabel di atas yang merupakan hasil estimasi/fusi
sensor. Baris "ground-truth" berarti benar-benar dibaca dari state internal simulator, bukan
"cukup akurat sehingga dianggap ground-truth" — perbedaan ini krusial untuk §6.

---

## 4. Grafik dependensi loop kendali

```
[GROUND-TRUTH SIM]                         [EDGE STATUS]
gz odom plugin ──► /hydroships/odom ──► stabilizer.on_odom()      IMPLEMENTED (sim-only sumbernya)
                                    ├──► mission_fsm._on_odom()   IMPLEMENTED (sim-only sumbernya)
                                    ├──► gui_bridge._on_odom()    IMPLEMENTED (sim-only sumbernya)
                                    └──► depth_publisher._on_odom() IMPLEMENTED (sim-only sumbernya)

depth_publisher ──► /hydroships/depth ──► mission_fsm, gui_bridge  IMPLEMENTED (turunan sim-only)

gz imu sensor ──► /hydroships/imu ──► (TIDAK ADA CONSUMER)         MISSING (bridge ada, consumer tidak)

stabilizer (PID) ──► /hydroships/cmd_vel ──► thruster_allocator ──► /hydroships/thruster_{1..6}/thrust
   IMPLEMENTED, ROS2-native, hardware-ready DI SISI SOFTWARE (unit-testable, tidak bergantung sim
   untuk *logikanya* — tapi bergantung total pada odom sim untuk *input*-nya)

thruster_allocator ──► gz thruster plugin (sim) / [MISSING: ESC/PWM driver] (hardware)
   SIMULATED-ONLY di ujung aktuator — HARDWARE.md sudah men-scope adapter ini

mission_fsm ──► /hydroships/payload_pose (ground-truth eksternal)
   SIMULATED-ONLY, TIDAK ADA JALUR PENGGANTI hardware yang discope — beda dari depth/ESC yang
   sudah punya rencana adapter di HARDWARE.md
```

**Interpretasi:** rantai kendali dari `cmd_vel` ke aktuator sudah punya jalur adapter yang
di-scope (`HARDWARE.md` item 1 untuk ESC). Rantai **sensor ke `cmd_vel`** (odom → stabilizer/
mission_fsm) tidak punya jalur adapter yang di-scope sama sekali — `HARDWARE.md` hanya
menjanjikan driver MS5837 untuk `/hydroships/depth` (skalar tunggal), bukan pengganti
`/hydroships/odom` (pose+orientasi+kecepatan penuh) yang dipakai `stabilizer` untuk
roll/pitch/yaw dan `mission_fsm` untuk x/y/vx/vy.

---

## 5. Klasifikasi simulasi-vs-target per sinyal state

| Sinyal | Klasifikasi | Alasan |
|---|---|---|
| Posisi z (kedalaman-turunan) | **ADAPTER-REQUIRED** | Rencana sudah ada (`depth_sensor_driver` MS5837 → `/hydroships/depth`), topic contract sudah kompatibel (`HARDWARE.md §3 item 2`) |
| Roll/pitch/yaw (orientasi) | **MISSING** | Sumber saat ini (odom ground-truth) tidak punya padanan hardware yang direncanakan; IMU sudah dijembatani tapi tanpa consumer DAN tanpa filter (raw IMU quaternion/gyro tidak otomatis setara "orientasi terfilter" yang diasumsikan `stabilizer`) |
| Kecepatan linear x/y (vx, vy, dipakai `mission_fsm` untuk navigasi NAV_WALL/APPROACH_HOOK) | **MISSING** | Tidak ada sensor kecepatan (DVL/dead-reckoning) di proposal maupun `HARDWARE.md`; odom ground-truth memberi kecepatan sempurna yang tidak mungkin direplikasi tanpa sensor tambahan |
| Posisi x/y absolut (dipakai `mission_fsm` untuk navigasi ke `payload_pose`/`_hook_xy`) | **MISSING** | Tidak ada localization sama sekali di proposal atau kode; x/y saat ini murni ground-truth simulator. Underwater positioning (USBL/DVL/dead-reckoning) adalah kategori sensor yang **tidak disebut sama sekali** di `HARDWARE.md` |
| IMU raw (accel/gyro/orientation quaternion) | **SIM-ONLY** (dibridge tapi tak dipakai — secara efektif tidak ada nilai praktis hari ini) | Data ada di topik tapi nol consumer; menjadi ADAPTER-REQUIRED hanya jika sebuah node baru dibangun untuk memakainya |
| Posisi payload (`/hydroships/payload_pose`) | **SIM-ONLY** | Berasal dari `payload_spawner`, murni konsep simulasi (random-spawn untuk keperluan uji); pada hardware, "tahu di mana payload" harus 100% berasal dari deteksi QR visual (`/hydroships/qr_offset`), bukan sinyal pose absolut |
| Offset QR/hook (visual relatif) | **ADAPTER-REQUIRED** (untuk kamera fisik + kalibrasi), tapi **logikanya sudah portable** | `qr_logic.py`/`hook_logic.py` adalah modul murni yang tidak bergantung sim; yang perlu diganti hanya driver kamera + intrinsics (sudah discope `HARDWARE.md` item 3) |
| Voltage/temp | **MISSING** | Tidak ada sensor sama sekali di kedua sisi (sim maupun kode hardware); stub `0.0` di `gui_bridge_logic.py` |
| Thruster force feedback (apakah thruster benar-benar menghasilkan gaya yang diminta) | **MISSING** (implisit, tak ada topik sama sekali) | Tidak ada sensor arus/RPM thruster di manapun; `thruster_allocator` mengasumsikan gaya yang dikirim = gaya yang terjadi, valid di sim (physics langsung), tidak valid di hardware tanpa kalibrasi kurva PWM↔gaya (`HARDWARE.md` item 1) |

---

## 6. Analisis gap fusi state/IMU

Menjawab langsung temuan P1-1 §11 ("state-estimation gap adalah risiko teknis terbesar yang
belum terselesaikan") dengan bukti kode konkret per sumbu state:

| Sumbu | Diasumsikan oleh | Diukur / diestimasi / disimulasikan / ground-truth / diasumsikan-implisit? | Bukti |
|---|---|---|---|
| **Attitude (roll/pitch)** | `stabilizer.py` (roll/pitch-hold PID), `gui_bridge.py` (telemetri) | **Ground-truth simulasi**, diekstrak dari quaternion `/hydroships/odom` — bukan dari IMU sama sekali | `stabilizer.py:47-56` (`roll_pitch_from_quaternion`, dipanggil dari `on_odom`, `stabilizer.py:163-164`) |
| **Heading/yaw** | `stabilizer.py` (heading-hold PID), `mission_fsm.py` (semua state navigasi berbasis wall heading), `gui_bridge.py` | **Ground-truth simulasi**, sama seperti attitude — tidak ada offset kompas, tidak ada estimasi magnetometer (magnetometer bahkan tidak disebut di proposal) | `stabilizer.py:40-44`, `mission_fsm._on_odom` |
| **Angular velocity (rate roll/pitch/yaw)** | **Tidak ada satu pun konsumen** — PID di `stabilizer.py` memakai `PID.update(err, measurement, dt)` yang secara internal menghitung d(error)/dt dari histori error, **bukan** dari rate langsung IMU/odom `twist.angular` | Tidak dipakai; `msg.twist.twist.angular` dari `Odometry` (yang sebenarnya tersedia di pesan) tidak diambil di manapun (grep `twist.angular` di `src/`: nol hasil) — turunan D-term PID dihitung numerik dari error posisi, bukan dari rate terukur |
| **Linear velocity (vx, vy)** | `mission_fsm.py` (dipakai untuk redaman servo saat `APPROACH_HOOK`/`NAV_WALL` — cek `_on_odom`) | **Ground-truth simulasi**, langsung dari `msg.twist.twist.linear` — sinyal yang secara fisik tak mungkin didapat presisi sama di air nyata tanpa DVL, yang **tidak ada di daftar komponen proposal** | `mission_fsm._on_odom: self.vx = msg.twist.twist.linear.x` (baris ~452 area, lihat kutipan §7 di bawah) |
| **Position x/y** | `mission_fsm.py` (navigasi ke `payload_pose`, `_hook_xy` per dinding) | **Ground-truth simulasi** — tak ada localization apa pun; pada hardware nyata di bawah air, posisi x/y absolut umumnya butuh USBL/DVL dead-reckoning, yang tidak disebut di `HARDWARE.md` atau proposal sama sekali | `mission_fsm._on_odom: self.x/self.y = msg.pose.pose.position.{x,y}` |
| **Depth/kedalaman** | `stabilizer.py` (via setpoint, bukan langsung — lihat catatan dua-konvensi tanda di P1-0 §6), `mission_fsm.py`, `gui_bridge.py` | **Ground-truth simulasi** hari ini; **satu-satunya sumbu yang sudah punya rencana adapter hardware konkret** (MS5837, `HARDWARE.md` item 2) | `depth_publisher.py:26-28` |
| **Acceleration** | Tidak dipakai secara eksplisit di mana pun (PID tidak butuh accel langsung; IMU accel di-bridge tapi tak dibaca) | **Tidak diasumsikan sama sekali** — tidak ada gap di sini karena tidak ada konsumen | grep `linear_acceleration` di `src/*.py`: nol hasil |
| **Timestamp/kesegaran data** | Semua consumer (`stabilizer`, `mission_fsm`, `gui_bridge`) memakai nilai state **terakhir diterima**, tanpa staleness check | **Diasumsikan-implisit selalu segar** — tidak ada mekanisme timeout untuk odom seperti yang ada untuk `thruster_allocator`'s cmd_vel watchdog (0.5s). Jika `/hydroships/odom` berhenti mengalir (mis. sensor drop-out hardware), `stabilizer` akan terus memakai `self.cur_z`/`self.cur_yaw` basi tanpa peringatan | `stabilizer.py:130-133` (state disimpan sebagai atribut instance, tidak ada `last_odom_time` yang dicek); dikonfirmasi tidak ada pola serupa `_hook_fresh()`/`hook_max_age` (yang memang ada di `mission_fsm.py` untuk offset hook — jadi tim tahu polanya, tapi **tidak diterapkan ke odom**) |

**Kesimpulan bagian ini:** IMU yang dijembatani tapi tak terpakai bukan sekadar "belum
dimanfaatkan" — ia menandakan bahwa **desain awal mengasumsikan IMU akan menjadi bagian dari
sebuah node fusi yang belum pernah dibangun**. Tanpa node itu, transisi ke hardware tidak
"tinggal ganti sumber data" (seperti kasus depth/MS5837 yang topic-compatible) — perlu
**membangun sebuah estimator/fusi baru dari nol** yang menghasilkan sesuatu yang bentuknya
seperti `nav_msgs/Odometry` dari kombinasi IMU + (opsional) DVL/USBL yang belum ada di daftar
komponen proposal sama sekali. Kecepatan linear (vx/vy) dan posisi x/y absolut adalah gap yang
**bahkan lebih dalam** dari attitude, karena tidak ada sensor kandidat apa pun (DVL/USBL) yang
disebut di `HARDWARE.md` untuk menggantikannya — attitude setidaknya punya IMU sebagai kandidat
mentah, posisi/kecepatan linear di bawah air tidak punya kandidat sensor sama sekali dalam
dokumentasi proyek saat ini.

---

## 7. Dependensi misi/autonomy

Kutipan langsung `mission_fsm.py` (baris dikonfirmasi via `grep -n`):

```python
def _on_depth(self, msg): self.depth = msg.data

def _on_odom(self, msg):
    self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)
    self.x = msg.pose.pose.position.x
    self.y = msg.pose.pose.position.y
    self.vx = msg.twist.twist.linear.x
    self.vy = msg.twist.twist.linear.y

def _on_payload_pose(self, msg):
    """Pose payload sebenarnya dari spawner (payload di-random tiap run).
    Tanpa ini FSM navigasi ke param payload_x/y yg statis -> ROV mendarat di
    tempat salah & QR tak pernah masuk frame kamera bawah."""
    self.payload_pose = (msg.point.x, msg.point.y, msg.point.z)
```

`mission_fsm.py` memperoleh:
- **Depth** → dari `/hydroships/depth` (turunan ground-truth odom). Dipakai untuk gating semua
  transisi state berbasis kedalaman (`DIVE`, `SURFACE`, dll — lihat `depth_ok` di sekitar baris
  524).
- **Heading (yaw)** → dari `/hydroships/odom`, ground-truth. Dipakai untuk penyelarasan arah
  dinding (`WALL_HEADING_DEG`) di `NAV_WALL`/`APPROACH_HOOK`.
- **Posisi x/y & kecepatan vx/vy** → dari `/hydroships/odom`, ground-truth. Dipakai untuk
  navigasi menuju `payload_pose`/hook (redaman kecepatan servo, lihat komentar `hook_logic` di
  `GUI-INTEGRATION.md §3b`).
- **Info target QR** → dari `/hydroships/qr_offset` (visual, sim-kamera hari ini, tidak
  ground-truth — ini satu-satunya sinyal state di FSM yang **bukan** ground-truth sim).
- **Posisi payload absolut** → `/hydroships/payload_pose`, **ground-truth eksternal murni dari
  spawner**, dengan komentar kode sendiri yang secara eksplisit mengakui ini adalah kebutuhan
  hard: tanpa sinyal ini, FSM navigasi ke koordinat statis dan QR tidak akan pernah masuk
  frame kamera bawah — **ini adalah sim-only leak yang FSM secara struktural bergantung
  padanya untuk mencapai `APPROACH_QR`/`GRAB` sama sekali**, bukan hanya untuk presisi.

**Yang akan lenyap di hardware, urut dari paling kritis:**
1. `/hydroships/payload_pose` — tidak ada padanan hardware; pada ROV fisik, penemuan lokasi
   payload harus 100% dari visual servo QR (`qr_offset`) tanpa bantuan ground-truth XY, yang
   berarti behavior `DIVE→APPROACH_QR` FSM saat ini (mengandalkan kombinasi keduanya) perlu
   dipikirkan ulang, bukan sekadar "hapus baris kode".
2. `self.x`, `self.y`, `self.vx`, `self.vy` dari odom ground-truth — dipakai navigasi
   `NAV_WALL`/`APPROACH_HOOK`; tanpa localization pengganti, FSM tidak tahu posisinya sendiri.
3. `self.yaw` dari odom ground-truth — dipakai penyelarasan `WALL_HEADING_DEG`; IMU mentah
   (belum difilter) adalah kandidat pengganti tapi belum ada node yang menghasilkannya.
4. `self.depth` — satu-satunya yang **punya jalur adapter konkret** (MS5837).

---

## 8. Dampak ke GUI

Berdasarkan `gui_bridge.py`/`gui_bridge_logic.py` yang sudah ditelusuri di P1-0 §4/§7, dipetakan
ulang di sini per kategori kepentingan:

| Field | Kategori | Sumber saat ini | Status setelah ROS2 owns estimation |
|---|---|---|---|
| `heading`, `roll`, `pitch` | Operator-telemetry (juga dipakai untuk display, bukan loop kendali GUI) | Odom ground-truth, via `_yaw_rpy()` lokal di `gui_bridge.py` | Akan otomatis mengikuti apa pun yang jadi sumber `/hydroships/odom` — **tidak perlu ubah `gui_bridge`** begitu ada node fusi yang publish ke topik yang sama, sesuai desain topic-agnostic `ARCHITECTURE.md §4` |
| `depth` | Control-critical (dipakai operator untuk depth-hold set) + operator-telemetry | `/hydroships/depth` | Sama — tinggal ganti publisher `depth_publisher.py` dengan driver MS5837 sesuai `HARDWARE.md` |
| `armed` | Control-critical (state echo) | In-process `GuiBridgeLogic.armed`, bukan dari sensor | Tidak terpengaruh gap estimasi state |
| `mode` | Operator-telemetry | Konstanta hardcoded `"manual"` | Tidak terpengaruh gap estimasi state (masalah terpisah, P1-0 Finding #3) |
| `poshold`, `depth_target`, `depth_hold` | Diagnostic/operator-telemetry | **Tidak ada** — gap independen dari state estimation (P1-0 Finding #6) | Tetap gap, tidak terkait dokumen ini |
| Voltage/temp | Diagnostic | Stub `0.0` | Tidak terkait state estimation (sensor daya/lingkungan terpisah) |
| Mission/FSM state | Diagnostic (operator ingin tahu FSM sedang di state apa) | Tidak dipublikasikan sama sekali (P1-0 Finding #5) | Tidak terkait state estimation |

**Kesimpulan §8:** Karena `gui_bridge` sepenuhnya topic-agnostic terhadap *bagaimana*
`/hydroships/odom`/`hydroships/depth` diproduksi (ia hanya subscribe ke nama topik), gap
estimasi state di dokumen ini **tidak menambah pekerjaan baru untuk `gui_bridge` itu sendiri**
— begitu ada node fusi/estimator baru yang mempublish ke topik yang sama, `gui_bridge` otomatis
ikut benar. Risiko sebenarnya bagi GUI bukan di `gui_bridge`, melainkan di **kualitas data**
yang lewat: operator akan melihat heading/depth yang berasal dari estimator baru yang belum
tentu setepat ground-truth sim — ini murni risiko kualitas, bukan risiko integrasi kode. Tidak
ada perubahan `gui_bridge` yang diusulkan di sini, sesuai batasan tugas.

---

## 9. Disposisi komponen legacy `GUI-ROV`

| Komponen | Peran | Disposisi | Alasan |
|---|---|---|---|
| `rov_agent.py` | Bridge produksi MAVLink↔Pixhawk, sumber `state["depth"]`/attitude dari `ATTITUDE`/pressure MAVLink | **Retire** (sudah diputuskan P1-1 §6: "retired as authoritative bridge") | Berasumsi Pixhawk mengestimasi state (ATTITUDE cascade firmware) — bertentangan langsung dengan Keputusan A yang menjadikan ROS2 otoritas. Tidak reusable sebagai bridge, tapi lihat baris berikutnya untuk sub-modulnya. |
| `attitude_filter.py` | Filter komplementer roll/pitch/yaw dari `ATTITUDE` Pixhawk mentah (docstring: "ATTITUDE membawa dua sumber info: sudut absolut & rate...") | **Reference-only, kandidat reusable-pure-logic** | Ini justru **satu-satunya kode filter/fusi attitude yang sudah ada di kedua repo** — algoritmanya (complementary filter dari absolute-angle + rate) berpotensi diadaptasi untuk node fusi IMU baru di `ros2_ws` (§11 di bawah), tapi butuh port karena inputnya saat ini adalah field MAVLink `ATTITUDE`, bukan `sensor_msgs/Imu`. Bukan port langsung — ini referensi desain algoritma, bukan kode yang bisa dipindah utuh. |
| `rov_axes.py` | Pemetaan murni axis GUI (-1000..1000) → field MANUAL_CONTROL | **Reference-only** | Sudah dicatat P1-1 §6 sebagai boleh dirujuk untuk P1.2 rekonsiliasi `gui_bridge` axis-scaling (bukan soal state estimation) — tidak relevan untuk dokumen P1.2-state-estimation ini secara langsung, hanya relevan untuk kontrak command di P1.3. |
| `rov_modes.py` | Pemetaan nama mode GUI ↔ ArduSub HEARTBEAT strings, termasuk aturan keselamatan per-mode (mis. kenapa "stabilize" dialiaskan ke ALT_HOLD) | **Reference-only** | Konsep "mode" di sini terikat erat ke perilaku firmware ArduSub (ALT_HOLD = cascade PID kedalaman internal Pixhawk) yang tidak relevan di bawah Keputusan A — `ros2_ws` sudah punya `control_mode` sendiri (`stabilizer.py` `CONTROL_MODES` dict) dengan semantik berbeda. Tidak untuk diporting; hanya referensi kenapa desain lama memisahkan mode vs depth-set. |
| `autonomy/fsm/mission5.py` | FSM misi kedua yang independen dari `mission_fsm.py` | **Requires-project-owner-decision** (sudah dicatat P1-1 §12 item 2, tidak diulang keputusannya di sini) | Di luar cakupan estimasi state secara langsung, tapi relevan karena `mission5.py` mengasumsikan state datang dari MAVLink telemetry (`ATTITUDE`/pressure), bukan ROS2 — jika `mission5.py` dipertahankan sebagai fallback, ia mewarisi ketergantungan Architecture B pada estimasi state Pixhawk yang sama sekali terpisah dari gap yang didokumentasikan di sini. |
| `autonomy/control/visual_servo.py` | Logika visual servo (PBVS) untuk `mission5.py` | **Candidate-for-porting (logika murni)**, tidak diaudit detail baris di sini (di luar cakupan file yang diminta secara eksplisit) | Sama seperti `hook_detect.py` yang sudah pernah di-port jadi `hook_logic.py` (dicatat `GUI-INTEGRATION.md §3b`) — pola porting logika-murni sudah terbukti berhasil sekali; visual servo QR/hook `ros2_ws` sendiri (`qr_logic.py`, `hook_logic.py`) sudah ada dan tidak butuh input dari modul ini, jadi porting hanya bernilai jika ditemukan algoritma yang lebih baik di dalamnya — bukan kebutuhan struktural. |
| `rov_pid.py`, `rov_heading.py` | PID/heading util pendukung `rov_agent.py` | **Reference-only** | Fungsinya sudah punya padanan ROS2-native (`pid.py`, `wrap_to_pi`) yang sudah dipakai & diuji di `ros2_ws`; tidak ada celah yang butuh diisi dari sini. |

---

## 10. Tabel blocker/risiko

| # | Item | Kategori | Bukti |
|---|---|---|---|
| 1 | Tidak ada node estimasi/fusi state sama sekali — `/hydroships/odom` 100% ground-truth Gazebo, tidak ada padanan hardware yang direncanakan untuk attitude/posisi/kecepatan (hanya depth yang punya rencana adapter) | **Blocker** | §2, §5, §6; `hydroships.urdf.xacro:403-410`; `HARDWARE.md` §3 (item depth ada, item odom/attitude/posisi tidak ada) |
| 2 | `mission_fsm._on_odom`/`_on_payload_pose` menggunakan ground-truth x/y/vx/vy dan pose payload absolut untuk navigasi — perilaku FSM saat ini **tidak akan berjalan sama sekali** tanpa localization pengganti, bukan hanya "kurang akurat" | **Blocker** | §7, kutipan `mission_fsm.py` |
| 3 | Tidak ada sensor kandidat untuk posisi/kecepatan linear bawah air (DVL/USBL) di proposal maupun `HARDWARE.md` — gap ini lebih dalam dari sekadar "belum ada driver", karena tidak ada hardware yang dibeli/direncanakan untuk mengisinya | **Blocker** | §5, §6; grep `HARDWARE.md` untuk DVL/USBL: nol hasil |
| 4 | IMU sudah dijembatani (`/hydroships/imu`) tapi nol consumer — kesempatan "cepat" untuk mulai mengisi gap attitude belum dimanfaatkan sama sekali | **Important** | §2, §6; grep `create_subscription.*Imu`: nol hasil |
| 5 | Tidak ada staleness/health-check untuk data odom di consumer manapun (`stabilizer`, `mission_fsm`, `gui_bridge`) — kontras dengan pola `_hook_fresh()` yang sudah ada untuk offset hook, menunjukkan tim tahu pola ini tapi belum menerapkannya ke odom | **Important** | §6 (baris timestamp), `mission_fsm.py` `_hook_fresh` vs `stabilizer.py` `on_odom` |
| 6 | Dua konvensi tanda kedalaman berbeda pada dua topik berbeda (P1-0 Finding #8) — akan menjadi jebakan nyata begitu depth diganti sensor MS5837 riil, karena satu-satunya keteguhan hari ini adalah keduanya diturunkan dari sumber yang sama (odom) sehingga selalu konsisten secara aljabar; sensor riil terpisah bisa memutus konsistensi implisit ini | **Important** | P1-0 §6, `depth_publisher.py:28` vs `mission_fsm.py:367-368` |
| 7 | Thruster force feedback tidak ada — `thruster_allocator` mengasumsikan gaya yang dikirim = gaya yang terjadi; valid di sim (fisika langsung), bukan asumsi yang otomatis benar di hardware tanpa kalibrasi kurva PWM↔gaya | **Cleanup** (sudah discope sebagai bagian `HARDWARE.md` item 1, bukan gap baru) | §5, `HARDWARE.md` §3 item 1 |
| 8 | Payload localization ground-truth (`/hydroships/payload_pose`) adalah simplifikasi uji yang FSM bergantung secara struktural — bukan cuma dipakai untuk mempercepat testing, tapi jadi prasyarat sebelum `APPROACH_QR` bisa dimulai secara efektif menurut komentar kode sendiri | **Blocker** (untuk transisi hardware M6, bukan blocker sim) | §7 |

---

## 11. Usulan dekomposisi implementasi

Urutan dari yang paling mendasar (tanpa ini, tidak ada yang lain bisa diverifikasi di hardware)
ke yang paling downstream:

1. **Driver IMU + node fusi attitude minimal** (roll/pitch/yaw dari IMU saja, tanpa magnetometer/
   GPS/DVL — filter komplementer sederhana, bukan EKF penuh)
   - Tujuan: menggantikan sumber roll/pitch/yaw `stabilizer`/`mission_fsm` yang saat ini
     ground-truth sim.
   - Input: `/hydroships/imu` (sudah ada topiknya, tinggal konsumsi) di sim; IMU fisik on
     hardware (bukan bagian Pixhawk — proposal menyebut IMU sebagai bagian Pixhawk, tapi
     Keputusan A berarti IMU perlu dibaca independen dari mixing Pixhawk, mis. via I2C langsung
     atau MAVLink pass-through read-only).
   - Output: topik baru (mis. `/hydroships/attitude` atau tetap menulis ke `nav_msgs/Odometry`
     parsial di `/hydroships/odom` jika ingin tetap topic-compatible) berisi roll/pitch/yaw
     terfilter.
   - Owning repo: `ros2_ws` (`hydroships_control`), pola node baru + modul pure-logic
     (`attitude_fusion_logic.py`) sesuai konvensi proyek.
   - Dependencies: tidak ada — bisa dikerjakan segera, bahkan sebelum hardware fisik ada,
     dengan menguji terhadap `/hydroships/imu` yang sudah tersedia di sim hari ini.
   - Verifikasi: unit test murni terhadap deret IMU sintetis (bandingkan filter output vs
     ground-truth odom di sim sebagai referensi, TANPA mengganti odom asli — cukup jalankan
     paralel dan log selisih). Sim-first sepenuhnya feasible.
   - Referensi desain (bukan port kode langsung): algoritma complementary filter di
     `GUI-ROV/attitude_filter.py` (lihat §9).

2. **Health/validity gating pada consumer odom** (staleness check)
   - Tujuan: mencegah `stabilizer`/`mission_fsm` memakai data odom basi tanpa sadar (Finding #5
     di §10), pola yang sudah ada untuk hook offset (`_hook_fresh()`) tinggal direplikasi.
   - Input: timestamp `/hydroships/odom`.
   - Output: flag `odom_valid`/pemakaian nilai terakhir dengan batas umur, mengikuti pola
     `_hook_fresh()`.
   - Owning repo: `ros2_ws`.
   - Dependencies: tidak ada — independen, bisa dikerjakan sebelum #1.
   - Verifikasi: unit test dengan mock timestamp basi.
   - Sim-first: sepenuhnya feasible, tidak butuh hardware.

3. **Keputusan desain: localization x/y/vx/vy** — ini bukan item implementasi tunggal, melainkan
   pertanyaan arsitektur yang harus dijawab project-owner sebelum kode ditulis (lihat §13 item
   1): apakah kompetisi menerima navigasi dead-reckoning kasar (integrasi IMU+thruster-command),
   ataukah dibutuhkan sensor tambahan (DVL kecil, USBL, atau sekadar mengandalkan visual servo
   QR/hook sepenuhnya tanpa localization absolut sama sekali, mengikuti pola `APPROACH_HOOK` yang
   sudah IBVS relatif, bukan PBVS absolut).
   - Setelah keputusan ini, item implementasi konkretnya baru bisa didekomposisi lebih lanjut
     (di luar cakupan P1.2 untuk memutuskan sendiri).

4. **Driver kedalaman MS5837** — sudah sepenuhnya discope di `HARDWARE.md` item 2, tidak
   diulang detailnya di sini; ini item yang paling siap dikerjakan karena topic-compatible dan
   tanpa ambiguitas desain.
   - Sim-first: N/A (sudah sim-ready via `depth_publisher.py`; item ini murni pekerjaan
     hardware-side).

5. **Integrasi ke `stabilizer`/`mission_fsm`** — mengganti konsumsi `/hydroships/odom` ground-
   truth dengan kombinasi (#1 attitude fusion) + (#3 hasil keputusan localization) + (#4 depth
   asli), sambil mempertahankan topic name yang sama supaya `stabilizer.py`/`mission_fsm.py`
   tidak perlu diubah sama sekali (mengikuti prinsip `ARCHITECTURE.md §"Keputusan desain"` dan
   `HARDWARE.md §4`) — **atau**, jika desain akhirnya butuh topik terpisah (mis. attitude-only
   vs full odom), scope perubahan minimal di consumer node menjadi bagian eksplisit item ini.
   - Dependencies: #1, #3.
   - Verifikasi: jalankan `stabilizer`/`mission_fsm` di sim dengan node fusi baru menggantikan
     odom plugin secara paralel (bukan mengganti odom plugin itu sendiri — hanya untuk
     verifikasi kesetaraan), bandingkan hasil hold/navigasi.

6. **GUI telemetry** — tidak ada pekerjaan tambahan diperlukan di `gui_bridge` (lihat §8),
   dicantumkan di sini hanya untuk kelengkapan urutan: begitu #5 selesai, `gui_bridge` otomatis
   mengalirkan data dari sumber baru tanpa perubahan kode.

---

## 12. Dependensi P1.3

- P1.3 (implementasi rekonsiliasi wire-contract GUI, per `P1-0 §12`/`P1-1 §9-10`) **tidak
  bergantung** pada penyelesaian gap estimasi state di dokumen ini — keduanya independen
  (dikonfirmasi §8: `gui_bridge` topic-agnostic).
- Namun, P1.3 **harus mewarisi kesadaran** bahwa field telemetry seperti `heading`/`depth` yang
  direkonsiliasi kontraknya akan, di masa depan, berasal dari estimator baru (bukan ground-truth
  sim) — kualitas/latensi data itu bisa berubah setelah item §11 diimplementasikan, meski
  kontrak wire-nya tidak berubah. Ini catatan silang, bukan blocker P1.3.
- Prioritas relatif: gap estimasi state (dokumen ini) lebih fundamental untuk kesiapan hardware
  daripada P1.3 (rekonsiliasi GUI), tapi keduanya bisa dikerjakan paralel karena tidak saling
  memblokir secara teknis.

---

## 13. Keputusan project-owner yang belum terjawab

1. **Tingkat localization yang dibutuhkan** — apakah kompetisi KKI 2026 menerima navigasi
   berbasis visual-servo relatif sepenuhnya (tanpa posisi x/y absolut, seperti pola
   `APPROACH_HOOK` IBVS saat ini), ataukah `NAV_WALL`/pencarian payload butuh localization
   absolut yang perlu sensor tambahan (DVL/USBL) yang belum ada di proposal maupun anggaran tim?
   Ini menentukan apakah item #3 di §11 perlu hardware baru atau cukup desain ulang algoritma.
2. **Sumber IMU fisik** — proposal menyebut IMU sebagai "bagian dari Pixhawk" (`HARDWARE.md`
   baris IMU). Di bawah Keputusan A (Pixhawk bukan otoritas kendali), apakah IMU tetap dibaca
   lewat Pixhawk (mode pass-through/telemetry-only, tanpa Pixhawk melakukan mixing) atau tim
   perlu IMU standalone terpisah dari Pixhawk? Ini soal fisik/pengadaan, bukan keputusan
   software murni.
3. **Toleransi terhadap dead-reckoning drift** — jika keputusan #1 di atas memilih dead-reckoning
   (integrasi IMU+command tanpa koreksi posisi eksternal), berapa lama durasi misi yang harus
   ditoleransi sebelum drift dianggap tidak dapat diterima? Ini menentukan apakah filter
   komplementer sederhana (item #1 §11) cukup atau dibutuhkan EKF lebih kompleks dengan model
   dinamika ROV.
4. **Nasib `/hydroships/payload_pose`** — apakah spawner ground-truth ini dipertahankan
   selamanya sebagai *simulation-only test aid* (dengan `mission_fsm` didesain ulang agar tidak
   bergantung strukturil padanya untuk mencapai `APPROACH_QR`), atau FSM memang perlu ditulis
   ulang sebelum uji hardware pertama? Keputusan ini punya implikasi jadwal langsung terhadap
   kapan uji kolam fisik pertama bisa dilakukan secara bermakna.
5. **Siapa yang mengerjakan node fusi/estimator baru** — ini pekerjaan teknik baru yang cukup
   besar (bukan port/adapter sederhana seperti depth), perlu penugasan eksplisit dan estimasi
   waktu dari project owner, sejalan dengan catatan P1-1 §12 item 4 tentang timeline gap
   state-estimation.

---

## Verdict akhir

**Status: simulation-ready-but-not-target-ready.**

Justifikasi: seluruh rantai kendali (`stabilizer` → `thruster_allocator` → aktuator) berjalan
dan sudah terverifikasi sebagian di Gazebo (lihat `STATUS.md` M1/M2 ✅), dan desainnya secara
struktural topic-agnostic sehingga *secara prinsip* siap menerima sumber data pengganti. Namun
**tidak ada satu pun sumber state (attitude, posisi, kecepatan) yang punya jalur pengganti
hardware yang direncanakan atau dibangun**, kecuali kedalaman (yang sudah punya rencana MS5837
konkret). IMU sudah dijembatani tapi nol baris kode di seluruh workspace membacanya — bridge itu
saat ini dekoratif, bukan fungsional. `mission_fsm.py` bahkan bergantung struktural pada sinyal
ground-truth eksternal (`/hydroships/payload_pose`) yang tidak dan tidak bisa punya padanan
hardware sama sekali — itu harus digantikan pendekatan berbeda (visual-servo murni), bukan
sekadar "dipasangi sensor".

**Tugas implementasi P1.2 paling penting satu ini:** membangun node fusi attitude minimal
(filter komplementer dari `/hydroships/imu`) yang menghasilkan roll/pitch/yaw sebagai pengganti
ekstraksi quaternion ground-truth saat ini — ini satu-satunya sumbu state yang (a) sudah punya
sensor dijembatani, (b) punya referensi algoritma yang bisa dipelajari (`GUI-ROV/attitude_filter.py`),
dan (c) bisa dikerjakan & diverifikasi sepenuhnya di sim sebelum hardware fisik ada, menjadikannya
titik mulai paling murah-risiko untuk mulai menutup gap estimasi state yang jauh lebih besar.

**Tugas konkret berikutnya setelah P1.2:** menjawab Pertanyaan #1 di §13 (tingkat localization
yang dibutuhkan) bersama project owner — jawaban ini menentukan apakah pekerjaan berikutnya
adalah "tulis filter komplementer sederhana dan lanjutkan" atau "hentikan dulu, perlu pengadaan
sensor DVL/USBL sebelum `NAV_WALL`/pencarian payload bisa didesain ulang untuk hardware". Tanpa
jawaban ini, P1.3 dan seterusnya berisiko dibangun di atas asumsi localization yang salah.
