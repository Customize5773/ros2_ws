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

   **Prosedur kalibrasi (jalur sudah disiapkan di kode, belum dites hardware):**
   1. Jalankan driver kamera (`usb_cam` atau `v4l2_camera`, paket ROS2 standar)
      publish ke topic yang sama seperti sim: `/hydroships/camera_bottom/image_raw`
      dan `/hydroships/camera_front/image_raw`.
   2. Kalibrasi tiap kamera dengan paket ROS resmi — **jangan tulis ulang**
      `cv2.calibrateCamera` sendiri:

      ```bash
      sudo apt install ros-humble-camera-calibration
      ros2 run camera_calibration cameracalibrator \
          --size 8x6 --square 0.024 \
          --ros-args -r image:=/hydroships/camera_bottom/image_raw
      ```

      (papan checkerboard 8x6 internal corner, kotak 24mm — sesuaikan ukuran papan
      yang tersedia). Gerakkan papan sampai bar kalibrasi penuh, klik **CALIBRATE**
      lalu **SAVE** — hasilnya file `ost.yaml` (per default di `/tmp/calibrationdata.tar.gz`,
      ekstrak `ost.yaml`). Verifikasi reprojection error yang dilaporkan tool (idealnya < 0.5 px)
      sebelum dipakai.
   3. Ulangi untuk kamera front.
   4. Muat hasilnya ke `qr_detector` lewat param (lihat `qr_logic.load_calibration_yaml`):

      ```bash
      ros2 run hydroships_control qr_detector --ros-args \
          -p calib_file_bottom:=/path/ke/ost_bottom.yaml \
          -p calib_file_front:=/path/ke/ost_front.yaml
      ```

      Kosong (default) = perilaku tak berubah, tetap pakai `camera_info` sim.
      `load_calibration_yaml` juga menerima `.npz` (`K`/`dist`/`image_size`/`rms`
      lewat `np.savez`, format hasil `cv2.calibrateCamera` langsung) selain `.yaml`.
   5. **Status: kalibrasi mentah SUDAH ADA, belum divalidasi/dipakai** — `dwe.npz` di
      root repo berisi hasil kalibrasi nyata kamera DWE ExploreHD (`K`, `dist` 5-koef,
      `image_size=[1280,720]`, `rms=4.97 px`). RMS 4.97 px **jauh di atas ambang wajar
      (idealnya < 0.5 px)** — indikasi kalibrasi ini kasar/kurang foto papan atau papan
      kurang bervariasi sudut, BUKAN siap pakai langsung untuk presisi visual servo.
      Sebelum dipakai: (a) load lewat `calib_file_bottom:=dwe.npz`, verifikasi
      `fx/fy/cx/cy` masuk akal untuk 1280x720, (b) idealnya rekalibrasi dengan lebih
      banyak sudut/jarak papan untuk turunkan RMS, (c) belum ada bukti file ini cocok
      dengan kamera bottom vs front (nama generik `dwe.npz`, tidak per-kamera) — cek
      dulu sebelum diasumsikan salah satu kamera tertentu. `dist` (distorsi lensa)
      disimpan tapi **belum dipakai** di `qr_logic`/`qr_detector` (offset dihitung dari
      corner piksel mentah, tanpa undistort) — gap terpisah, bukan blocking utk
      memuat `K` saja.
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

## 4. Validasi geometri hook fisik & retuning deteksi (M4, OPEN)

**Status: prosedur siap, belum dieksekusi** (butuh hook PVC fisik + kolam —
sejajar §3, kalibrasi mentah sudah ada tapi belum divalidasi hardware).

1. **Cek geometri.** Hook di `worlds/kki_arena.sdf` adalah PVC ¾" (Ø25mm,
   `radius=0.0125m`) bentuk J: stub atas 60mm, batang vertikal 550mm, lengkung
   bawah 80mm. Ukur hook fisik tim terhadap angka ini. Konfirmasi juga tinggi
   pasang **0,45 m dari dasar kolam** (`docs/GUIDEBOOK.md` §4.7.1, "Layout &
   Konsep GUI ROV") cocok dengan pose hook di SDF.
2. **Retuning threshold deteksi.** `hook_detector.py` docstring menandai
   default threshold-nya "titik-awal uji-meja, WAJIB di-tuning ulang di
   kolam" — `min_area`, `canny_lo`, `canny_hi`, `aspect_min`, `aspect_max`
   kini semua **declared ROS param** (bukan cuma konstanta module), jadi bisa
   diubah tanpa edit-kode-lalu-rebuild:

   ```bash
   ros2 run hydroships_control hook_detector --ros-args \
       -p canny_lo:=40 -p canny_hi:=120 -p aspect_min:=0.2 -p aspect_max:=5.0
   # atau live saat node jalan:
   ros2 param set /hook_detector canny_lo 40
   ```

   Jalankan terhadap footage/rosbag kamera front di kolam nyata (atau
   rekaman uji-meja hook fisik dulu bila kolam belum tersedia), sesuaikan
   sampai deteksi stabil (`/hydroships/hook_offset` konsisten, bukan
   berosilasi hilang-muncul), lalu catat nilai final balik ke konstanta
   default di `hook_detector.py` (`HOOK_CANNY_LO`, dst., baris ~40-46) supaya
   default besok tak perlu di-set ulang tiap run.
3. **Warna PVC (opsional, catatan tambahan).** `detect_hook()` sengaja
   melewati jalur deteksi warna GUI-ROV — komentar di kode: "warna PVC hook
   tak pasti". Sesi retuning poolside adalah momen tepat untuk
   mengonfirmasi/membantah asumsi ini dengan warna hook asli; kalau warnanya
   ternyata konsisten & kontras dari background kolam, jalur warna bisa
   diaktifkan lagi sebagai optimisasi terpisah — bukan bagian wajib validasi
   ini.
4. **Setelah threshold deteksi stabil**: gain servo `hook_logic.py`
   (`size_stop`, `center_tol`, `kp_surge`/`kd_surge`, dst.) kemungkinan besar
   juga perlu re-tuning — sama seperti gain visual servo QR di §3 poin 3 —
   tapi itu tuning parameter kontrol, terpisah dari validasi geometri hook
   ini.

## 5. Referensi silang

- Spesifikasi komponen fisik lengkap (part number, harga, datasheet ringkas):
  proposal tim, `DOKUMENTASI ROV/` (Bab 2: Desain dan Spesifikasi).
- Kontrak topic ROS2 yang harus dipenuhi driver hardware baru: `docs/ARCHITECTURE.md`.
- Semua parameter yang kemungkinan perlu re-tuning setelah pindah ke hardware asli
  (gain PID, threshold visual servo): `docs/TUNING_GUIDE.md`.
- Status blocking bug software saat ini (independen dari isu hardware):
  `docs/STATUS.md`.
