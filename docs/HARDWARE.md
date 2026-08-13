# Hardware — Status & Gap Analysis

> **Ringkas:** workspace ini (kode ROS2 di `src/`) saat ini **100% simulasi Gazebo**.
> Tidak ada driver hardware asli (Pixhawk/MAVLink, ESC/PWM, kamera fisik, GPIO,
> serial port) di dalam `src/`. Dokumen ini memetakan apa yang sudah ada di sim vs
> apa yang perlu dibangun untuk ROV fisik, berdasarkan desain hardware di
> proposal KKI 2026 (`DOKUMENTASI ROV/`, lihat lampiran proposal tim).

## 1. Kenapa dokumen ini ada

Semua dokumentasi lain di `docs/` (ARCHITECTURE.md, HOW-TO-RUN.md, STATUS.md, dst.)
menjelaskan node ROS2 dan topic contract yang berjalan di atas **Gazebo Fortress**.
Proposal tim (KKI 2026) mendeskripsikan ROV fisik dengan komponen nyata: Pixhawk
PX4, Raspberry Pi 4B, 6× thruster (T100/T200 BlueRobotics), 2× kamera DWE ExploreHD,
sensor kedalaman MS5837, ESC 20A, tether interface board, gripper servo waterproof.
**Tidak satu pun dari komponen ini punya driver/interface code di `src/` workspace
ini.** Dokumen ini eksplisit menandai gap tersebut supaya tidak diasumsikan "sudah
jalan" hanya karena ada di proposal.

## 2. Peta komponen: Proposal (fisik) → Sim (ROS2) → Status kode

| Komponen fisik (proposal) | Peran | Padanan di sim ROS2 saat ini | Status kode hardware asli |
|---|---|---|---|
| 6× Thruster (3× T100 + 3× T200, BlueRobotics) | Propulsi 6-DOF | `gz-sim-thruster-system` plugin, dikomando via `/hydroships/thruster_{1..6}/thrust` (Float64, Newton) | **Tidak ada.** Perlu node ESC/PWM driver yang menerjemahkan Newton → sinyal PWM (via Pixhawk `MAIN OUT` atau ESC langsung). |
| Pixhawk PX4 | Kontroler utama (thruster mixing, servo gripper) | Tidak dipakai — mixing dilakukan `thruster_allocator.py` (software, damped pseudo-inverse) | **Tidak ada.** Lihat `docs/GUI-INTEGRATION.md` — hanya repo GUI eksternal (`Customize5773/GUI-ROV`) yang bicara MAVLink/pymavlink ke Pixhawk/ArduSub. Workspace ini tidak punya kode MAVLink. |
| Raspberry Pi 4B | Onboard computer (jalankan ROS2 node + kamera + sensor) | N/A — semua node dijalankan di mesin dev/sim (laptop) | **Belum divalidasi di RPi.** Perlu uji `colcon build` + performa real-time node (`stabilizer`, `qr_detector`, `hook_detector`) di RPi 4B aktual (CPU/RAM terbatas dibanding laptop dev). |
| Sensor kedalaman MS5837 (I2C) | Kedalaman real-time | `depth_publisher.py` — kedalaman diturunkan dari `/hydroships/odom` (ground-truth Gazebo, BUKAN sensor) | **Tidak ada.** Perlu node baru `depth_sensor_driver` (I2C via `smbus2`/library MS5837 resmi) yang publish ke topic yang sama, `/hydroships/depth` (`std_msgs/Float64`), sehingga `stabilizer`/`mission_fsm` tidak perlu diubah (topic contract sudah cocok — lihat ARCHITECTURE.md). |
| 2× Kamera DWE ExploreHD (USB) | Video real-time (bottom + front) | Kamera simulasi Gazebo (`gz-sim-sensors-system`, ogre2 render) → `/hydroships/camera_{front,bottom}/image_raw` | **Tidak ada.** Perlu node `usb_cam`/`v4l2_camera` (paket ROS2 standar tersedia) yang publish `sensor_msgs/Image` + `CameraInfo` ke topic sama. **Kalibrasi kamera fisik juga belum dilakukan** — intrinsics sim (`camera_info`) murni dihitung dari FOV SDF, bukan kalibrasi checkerboard riil (lihat `docs/VERIFICATION-CHECKLIST.md` P5). |
| Tether Interface Board + tether cable (25m) | Distribusi daya + komunikasi + video | N/A (semua proses lokal dalam satu mesin sim) | **Tidak ada.** Ini murni hardware fisik (bukan sesuatu yang perlu "kode"), tapi implikasinya untuk software: latency/bandwidth tether harus diuji untuk topic image (frame rate kamera bisa perlu di-throttle — lihat param `max_rate` di `qr_detector`/`hook_detector`). |
| ESC 20A (per thruster) | Driver motor BLDC | Diwakili plugin Thruster Gazebo (langsung terima Newton) | **Tidak ada.** Perlu kalibrasi kurva Newton→PWM (atau RPM→PWM) khusus T100/T200 sebelum `thruster_allocator` output bisa dipakai langsung sebagai command hardware. |
| Servo gripper waterproof (2× aktuator, 2-axis) | Capit + elevasi payload | `gripper_controller.py` publish `/hydroships/gripper_left/cmd`, `/hydroships/gripper_right/cmd` (Float64 rad) → digerakkan sebagai `DetachableJoint` rigid-attach di Gazebo (BUKAN gripping mekanis fisik — lihat `docs/STATUS.md`) | **Tidak ada driver servo.** Perlu node yang menerjemahkan sudut (rad) → PWM servo (via Pixhawk AUX/RC output atau driver PCA9685 dari RPi). **Catatan penting:** logika grasp di sim disederhanakan jadi "magnetic attach" (DetachableJoint), bukan simulasi fisik jepitan capit — ini axis desain yang sengaja diambil untuk sim, tapi berarti perilaku gripper fisik (kekuatan cengkeram, slip, dsb.) **tidak divalidasi sama sekali** oleh sim ini. |
| IMU (bagian dari Pixhawk) | Orientasi (roll/pitch/yaw) | IMU simulasi Gazebo → `/hydroships/imu` | **Tidak ada** integrasi IMU fisik Pixhawk ke ROS2 workspace ini (kembali ke poin Pixhawk di atas — jalur MAVLink hanya ada di repo GUI eksternal). |
| GCS / Panel Operator (laptop) | Monitoring + kontrol | `gui_bridge.py` — adapter UDP-JSON↔ROS2, dipakai *bila* GUI eksternal tersambung | Ada **jalur adapter** (`gui_bridge`), tapi **belum pernah diuji dengan GUI fisik/live** — lihat `docs/VERIFICATION-CHECKLIST.md` P4 ("UDP round-trip terkonfirmasi dgn synthetic client" saja, bukan GUI asli). |

## 3. Yang HARUS dibangun sebelum deploy ke ROV fisik

Urutan disarankan (dari yang paling blocking untuk "ROV bisa berenang" ke yang paling
untuk "misi lomba lengkap"):

1. **ESC/PWM driver untuk 6 thruster** — tanpa ini, `thruster_allocator` cuma
   menghasilkan angka Newton yang tidak kemana-mana. Perlu kalibrasi kurva
   thrust↔PWM per unit T100/T200 (BlueRobotics menyediakan datasheet kurva referensi,
   tapi kalibrasi ulang disarankan karena voltase suplai tim beda dari referensi pabrik).
2. **Sensor kedalaman MS5837 driver** — blocking untuk `stabilizer` depth-hold
   bekerja di air asli (saat ini `depth_publisher` memakai ground-truth sim, tidak ada
   di dunia nyata).
3. **Kamera USB driver (v4l2/usb_cam) + kalibrasi intrinsics** — blocking untuk
   `qr_detector`/`hook_detector` bekerja dengan kamera fisik; geometri servo visual
   (mis. `qr_servo_gain`, `hook_kp_surge`, dst. di `gains.yaml`/param `mission_fsm`)
   kemungkinan besar perlu re-tuning karena FOV, distorsi lensa, dan pencahayaan
   bawah air asli berbeda drastis dari render Gazebo.
4. **Servo gripper driver** — blocking untuk `gripper_controller` menggerakkan
   aktuator fisik. Perlu keputusan desain: PWM langsung dari RPi (mis. via
   PCA9685 I2C) atau lewat Pixhawk AUX output.
5. **Integrasi Pixhawk/MAVLink (opsional, tergantung keputusan tim)** — proposal
   menyebut Pixhawk sebagai "kontroler utama", tapi arsitektur software saat ini
   (`thruster_allocator` + `stabilizer` di ROS2) bisa berjalan **tanpa** Pixhawk sama
   sekali jika ESC dikendalikan langsung dari RPi. Jika tim tetap ingin memakai
   Pixhawk (mis. untuk redundansi IMU/failsafe), perlu node MAVLink baru — **jangan
   duplikasi kerja repo GUI eksternal**, cek dulu apakah `Customize5773/GUI-ROV`
   sudah menutupi kebutuhan ini via `pymavlink`.
6. **Uji performa node di Raspberry Pi 4B asli** — semua node saat ini hanya
   diuji di mesin dev/CI (headless, CPU laptop). RPi 4B punya CPU/RAM jauh lebih
   terbatas; `qr_detector`/`hook_detector` (OpenCV per-frame) berisiko paling besar
   soal frame rate — verifikasi `max_rate` param cukup rendah untuk RPi.

## 4. Yang TIDAK perlu diubah (topic contract sudah hardware-agnostic)

Desain arsitektur (lihat `docs/ARCHITECTURE.md`) sengaja memisahkan node logika
(`stabilizer`, `thruster_allocator`, `mission_fsm`, `qr_detector`, `hook_detector`,
`gripper_controller`) dari sumber data via topic ROS2 standar. Ini artinya: **node
driver hardware baru (poin 1–4 di atas) hanya perlu publish/subscribe ke topic
yang sama dengan yang dipakai plugin Gazebo** — tidak perlu mengubah `stabilizer.py`,
`mission_fsm.py`, dll. Contoh: driver MS5837 baru cukup publish `Float64` ke
`/hydroships/depth`, persis seperti `depth_publisher.py` di sim. Lihat tabel topic
lengkap di `docs/ARCHITECTURE.md` dan `docs/CONFIG_REFERENCE.md`.

## 5. Referensi silang

- Spesifikasi komponen fisik lengkap (part number, harga, datasheet ringkas):
  proposal tim, `DOKUMENTASI ROV/` (Bab 2: Desain dan Spesifikasi).
- Kontrak topic ROS2 yang harus dipenuhi driver hardware baru: `docs/ARCHITECTURE.md`.
- Semua parameter yang kemungkinan perlu re-tuning setelah pindah ke hardware asli
  (gain PID, threshold visual servo): `docs/TUNING_GUIDE.md`.
- Status blocking bug software saat ini (independen dari isu hardware):
  `docs/STATUS.md`.
