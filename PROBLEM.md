# PROBLEM.md — Catatan Masalah & Verifikasi Tertunda (HYDROships ros2_ws)

Dokumen ini mengumpulkan masalah diketahui, keputusan sementara, dan hal yang **masih
perlu diverifikasi di kolam/hardware asli**. Dijadikan catatan akhir saat semua milestone
selesai. Format: `[status]` OPEN / VERIFY / RESOLVED.

> Konteks lintas-repo: dokumen arsitektur & roadmap referensi ada di repo lama
> **ROV-GAZEBO-SIMULATION** (GitHub), bukan di workspace ini.

---

## Arena (M4) — sudah dibangun
- `[VERIFY]` Pemetaan label hook **A/B/C/D → sisi kolam** masih sementara (A=−Y, B=+Y,
  C=+X, D=−X di `worlds/kki_arena.sdf`). Pengacakan posisi ("Posisi A,B,C,D diacak")
  belum diimplementasi — menyusul bersama logika tugas.
- `[VERIFY]` Geometri hook = aproksimasi silinder J (Ø25 mm), titik terendah z=−0.45.
  Belum divalidasi apakah cukup untuk uji sangkut ROV nyata.

## Sensor & Persepsi (M3) — sudah dibangun
- `[VERIFY]` Kamera di-render headless via `gz-sim-sensors-system` (ogre2). Jalan di mesin
  dev ini (~22 Hz). Di mesin/CI tanpa GPU/EGL, sensor kamera bisa gagal render — perlu cek.
- `[OPEN]` Kalibrasi/CameraInfo kamera sim belum dipetakan ke intrinsics nyata; `camera_info`
  di-publish gz tapi belum dijembatani ke ROS (baru image_raw). Tambah bila node PBVS butuh.
- `[VERIFY]` `depth_publisher` menurunkan kedalaman = max(0,−z) dari odom. Benar di sim
  (permukaan z=0). ROV near-neutral **sedikit positif** → mengapung ke permukaan saat
  thruster mati (depth→0). Ini sesuai desain M1, bukan bug.
- `[OPEN]` Node deteksi QR (`camera_bottom/image_raw` → `/hydroships/qr_result` A/B/C/D)
  belum dibuat. Butuh `cv_bridge` + OpenCV di ROS 2.

## Manipulator (M5) — sudah dibangun
- Gripper 2 jari (revolute sumbu z) di depan ROV, dikontrol gz JointPositionController.
  Perintah semantik `/hydroships/gripper/command` ("open"/"close") → node
  `gripper_controller` → setpoint 2 jari (bridge → gz). State jari di
  `/hydroships/joint_states`. **Terverifikasi**: open ≈ +0.50/−0.50, close ≈ −0.14/+0.15 rad.
- `[VERIFY]` Sudut `open_angle=0.5` / `close_angle=-0.15` (param node) & arah buka/tutup
  masih perlu dicek visual di GUI/kolam agar celah jepit pas ukuran payload.
- `[OPEN]` **Grasp fisik belum diuji**: jari punya collision tapi belum ada model payload
  (bagian M4). Menjepit andal kemungkinan butuh penyetelan friction atau plugin
  attach/detach (mis. gz-sim DetachableJoint) — belum diimplementasi.
- `[OPEN]` `/hydroships/joint_states` (dari gz) hanya berisi 2 joint gripper, bukan
  thruster. TF jari via robot_state_publisher belum tersambung (rsp mendengar
  `/joint_states`, sedangkan bridge ke `/hydroships/joint_states`). Remap bila butuh TF.
- `[note]` `ros2 topic pub --once` ke command bisa meleset karena race discovery;
  node menerbitkan ulang setpoint 2 Hz sehingga joint tetap menahan posisi. Konsumen
  nyata (GUI/autonomy) mengirim berulang, jadi aman.

## FISIKA ROV — DUA BUG BESAR DITEMUKAN & DIPERBAIKI (RESOLVED)
Menjawab "kenapa ROV makin dibiarkan makin melayang, tidak menggenang di air":
1. `[RESOLVED]` **Buoyancy tanpa permukaan.** World pakai `<uniform_fluid_density>` yang
   memberi gaya apung **di MANA SAJA** (tak ada permukaan bebas). ROV sedikit-positif →
   net gaya ke atas selalu → melayang naik tanpa henti. **Fix:** ganti ke `<graded_buoyancy>`
   (air 1000 di bawah z=0, udara 1 di atas) di `worlds/kki_arena.sdf` & `pool_empty.sdf`.
   Hasil: ROV **menetap di permukaan** (odom z ≈ −0.14, stabil), tidak melayang lagi.
2. `[RESOLVED]` **Thrust tak pernah masuk.** Plugin Thruster `<namespace>hydroships</namespace>`
   MEN-prepend namespace ke `<topic>hydroships/…</topic>` → subscribe ke
   `/hydroships/hydroships/thruster_N/thrust`, sedangkan bridge publish ke
   `/hydroships/thruster_N/thrust` → **tak nyambung, gaya thruster nol**. (Gerakan naik
   sebelumnya murni buoyancy, bukan thrust.) **Fix:** ubah `<topic>` jadi `${name}/thrust`
   (tanpa prefix) di `hydroships.urdf.xacro`. Hasil: wrench −40 N → ROV **menyelam** dari
   −0.14 ke −0.75 m dan terkendali.

> Catatan: M1/M2 sebelumnya ditandai ✅ tapi ternyata **thrust tak pernah benar-benar
> menggerakkan ROV** (topik tak nyambung) — verifikasi lama kurang teliti. Kini teruji nyata.

## Model Visual ROV (mesh dari model/rov.fbx)
- `[RESOLVED]` Body kotak base_link diganti mesh ROV asli. Fortress (ign-common4)
  **tidak bisa memuat FBX** (tak ada loader assimp) & FBX asli 48 MB / 1.26 M tri
  (DAE hasil konversi 378 MB — terlalu berat). **Solusi:** konversi `model/rov.fbx`
  → decimate 1.26 M → **40 k tri** → `meshes/rov.stl` (2 MB) via pyassimp +
  fast-simplification. Collision TETAP kotak (buoyancy/fisika). Mesh ter-load tanpa error.
- `[RESOLVED]` `package://` di-resolve gz jadi `model://` → tak ketemu. Ditambah
  `IGN_GAZEBO_RESOURCE_PATH`/`GZ_SIM_RESOURCE_PATH` ke folder share di `sim.launch.py`.
- `[VERIFY]` **Skala (0.0048) & orientasi (rpy 0) & offset origin** mesh masih TEBAKAN
  (footprint disamakan ~0.345 m). Belum bisa dicek visual headless — buka GUI
  (`ros2 launch hydroships_gazebo sim.launch.py`) untuk pastikan ukuran/arah bow benar,
  lalu sesuaikan `scale`/`rpy`/`origin` di `hydroships.urdf.xacro` (base_link visual).
- `[VERIFY]` STL monokrom (tanpa warna/material asli). Bila perlu warna, ekspor DAE
  ter-decimate (butuh tool decimate + jaga ukuran) atau set material di URDF.
- `[note]` Sumber `model/rov.fbx` (48 MB) dibiarkan di repo; yang dipakai sim = `meshes/rov.stl`.

## Autonomy (M6) — kode dibangun, INTEGRASI JALAN (setelah fix fisika)
Node `mission_fsm` (ROS 2) + launch `hydroships_bringup/launch/hydroships_mission.launch.py`
(sim + allocator + stabilizer + FSM). FSM mengendalikan lewat setpoint stabilizer
(`setpoint/depth`, `setpoint/heading`, `manual/cmd`) + `/hydroships/gripper/command`.

- `[RESOLVED]` Setelah dua fix fisika di atas, misi **berjalan**: FSM `IDLE→DIVE` →
  "Dasar tercapai (0.76 m)" → `DIVE→SCAN_QR` → (inject QR "A") "QR → wall A (+15)" →
  `SCAN_QR→GRAB`. Depth-hold & heading-hold bekerja.
- `[VERIFY]` Timeout tiap state, gaya (`surge_force` dll), & sudut belum di-tune untuk
  gerak nyata di arena; baru diuji transisi awal.
- `[OPEN]` `SCAN_QR` menunggu `/hydroships/qr_result` (node QR belum ada) → tanpa QR akan
  timeout; sementara uji dgn `ros2 topic pub /hydroships/qr_result` manual atau `start_state:=`.
- `[TODO]` `APPROACH_HOOK` masih *timed* (visual servo ArUco ROS 2 belum ada) — referensi
  port ada di `GUI-ROV/autonomy/`.

## Umum / lintas-milestone
- `[VERIFY]` Massa & koefisien hidrodinamika ROV masih **placeholder** near-neutral
  (dari `hydroships.urdf.xacro`), belum data ROV asli. Setel di M2+ dgn data nyata.
- `[OPEN]` Integrasi GUI tim (repo GUI-ROV) ↔ topik ROS 2 (M7) belum dijembatani.
