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
- `[RESOLVED]` Node deteksi QR **sudah dibuat**: `qr_detector` (baca
  `camera_bottom/image_raw`, decode `cv2.QRCodeDetector`, publish `/hydroships/qr_result`
  A/B/C/D). Tanpa `cv_bridge` (decode Image manual via numpy). **Terverifikasi** dgn QR
  sintetik "A" → qr_result "A". Otomatis mengumpani FSM SCAN_QR.
- `[RESOLVED]` **Model payload dibangun** sesuai gambar KKI: braket-L (pelat 5×0.6×10 cm,
  lubang dia 3 cm 0.6 cm dari atas, kaki 3 cm) = mesh `media/payload_body.obj` (dibuat
  programatis, lubang bulat asli), + **QR 4×4 cm** di muka depan. Berdiri di dasar arena
  (pose `0.4 0 -0.9 ... yaw 90°`), QR menghadap −X. Model ada di scene, OBJ ter-load tanpa error.
- `[RESOLVED]` **Rendering tekstur QR** di kamera sensor Fortress headless: PBR `albedo_map`
  saja RENDER HITAM (butuh environment/IBL). **Fix:** pakai `<emissive_map>` (self-lit) →
  QR tampil & **terbaca** (`cv2` decode "A" dari kamera). Tekstur RGB (bukan grayscale) di
  plane (bukan box — box tak punya UV).
- `[OPEN]` **Scan QR autonomous di sim belum andal.** QR asli 4 cm terlalu kecil untuk
  di-decode dari jarak misi (butuh ~3 px/modul; 4 cm @ 0.2 m hanya ~60 px). Plus ROV
  positif-buoyant (naik ke permukaan), tak ada position-hold XY (drift), dan SCAN_QR
  menyapu heading → kamera sulit tetap mengarah ke QR kecil. **Untuk uji misi penuh sekarang:
  inject `/hydroships/qr_result` manual** (logika FSM sudah terverifikasi). **Opsi perbaikan
  (belakangan):** perbesar QR di sim, tambah perilaku APPROACH ke payload sebelum scan,
  tambah depth+XY hold, atau baringkan payload (QR ke atas) tepat di bawah jalur menyelam.

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
- `[RESOLVED]` **Struktur mesh "acak-acak" diperbaiki.** Sebelumnya 279 sub-mesh FBX
  digabung pakai vertex LOKAL tanpa menerapkan transform node → semua bagian
  terkumpul salah posisi. **Fix:** load dgn assimp `aiProcess_PreTransformVertices`
  (bake transform hierarki ke world-space) sebelum merge.
- `[RESOLVED]` **Skala terpecahkan.** Setelah transform benar, bbox mesh =
  **350.7 × 344.5 × 286.0 mm** — persis ukuran kotak desain (0.345×0.345×0.286 m) →
  **FBX satuan MILIMETER → scale = 0.001**. Mesh di-recenter ke origin di file, jadi
  URDF origin xyz=0. (Fortress tak bisa FBX → dipakai STL hasil konversi.)
- `[RESOLVED]` `package://`→`model://` tak ketemu → tambah `IGN_GAZEBO_RESOURCE_PATH`
  di `sim.launch.py`. Mesh ter-load tanpa error (0 geometry-load-failures).
- `[OPEN]` **Poly-count berat.** Model punya 279 komponen terpisah; fast-simplification
  mentok di ~237 k segitiga (STL 12 MB) — tak bisa turun ke ~40 k. Akibat: **rate kamera
  turun ~22 → ~10 Hz** (render lebih berat). Untuk lebih ringan: pakai decimator quadric
  (open3d/pymeshlab) atau buang komponen kecil (baut/pipa) sebelum merge.
- `[VERIFY]` **Arah bow (haluan) belum dipastikan.** Footprint mesh ~persegi (X≈Y) jadi
  tak bisa ditebak dari bbox. Diatur via properti `bow_yaw` (rad) di `hydroships.urdf.xacro`
  (default 0 = bow menghadap +X). Cek di GUI & set 1.5708/−1.5708/3.14159 bila perlu.
- `[VERIFY]` STL monokrom (tanpa warna/material asli). Bila perlu warna, ekspor DAE
  ter-decimate (jaga ukuran) atau set material per-bagian di URDF.
- `[note]` Sumber `model/rov.fbx` (48 MB) dibiarkan di repo; yang dipakai sim = `meshes/rov.stl` (12 MB).

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
