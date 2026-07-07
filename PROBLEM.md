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
- `[RESOLVED]` **Perilaku APPROACH + depth/XY hold** (state `APPROACH_QR` di mission_fsm):
  ROV menyelam ke `scan_depth`, lalu **PD posisi** (kp/kd, redaman dari odom twist body-frame)
  mendorong ROV **DI ATAS** payload/QR datar & **menahannya**. Terverifikasi bersih: ROV
  menetap di **(0.41, 0.00, ~0.63)** tepat di target `payload_x/y` (0.4, 0) dan stabil.
  Kamera bawah digeser sedikit ke bawah badan (z=-0.18) agar tak terhalang mesh sendiri.
- `[OPEN]` **QR belum terbaca meski posisi sudah tepat.** Di kedalaman scan, kamera bawah
  menampilkan lantai berfaset + bayangan; **payload/QR tak tampil jelas** (QR emissif 4 cm
  terlalu kecil pada render ini, atau isu render payload jarak dekat). Loop kontrol (approach)
  sudah beres — sisanya murni keterbacaan visual. **Uji misi penuh sekarang: inject
  `/hydroships/qr_result` manual.** Fix: **perbesar QR khusus sim** (lihat "Opsi ditunda"),
  atau debug pencahayaan/scene agar QR emissif kontras & cukup besar di frame.
- `[note-uji]` Run sim yang tumpang-tindih meninggalkan proses `mission_fsm`/`parameter_bridge`
  lama yang **saling adu perintah** → ROV berperilaku erratic. Selalu pastikan proses lama
  mati (`ps | grep`) sebelum run baru. Bukan bug kode.

## Manipulator (M5) — DIHAPUS (akan dirancang ulang)
- `[REMOVED]` Seluruh subsistem gripper **dihapus** atas permintaan (rencana dibuat ulang).
  Yang dibuang: link `gripper_base` + 2 jari & plugin `JointPositionController`/
  `JointStatePublisher` di `hydroships.urdf.xacro`; node `gripper_controller`
  (+ entry-point `setup.py` & Node di `sim.launch.py`); topik `gripper_left/right/cmd`
  & `joint_states` di `bridge.yaml`; publisher `/hydroships/gripper/command`, method
  `_grip`, & semua panggilannya di `mission_fsm.py`.
- `[note]` State `GRAB`/`HANG`/`AUTO_RELEASE` di `mission_fsm` **dipertahankan sebagai
  kerangka gerakan** (depth/surge saja, tanpa jepit) agar misi tetap jalan; logika
  manipulasi menyusul saat rancangan gripper baru siap.
- `[TODO]` Rancang ulang manipulator (mekanik + kontrol + integrasi misi + grasp fisik,
  mis. gz-sim DetachableJoint).

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

## FISIKA ROV — TEMUAN KETIGA: geometri thruster near-singular (YAW ~tak terkendali)
- `[OPEN]` **Bidang horizontal (surge/sway/yaw) badly-conditioned; YAW praktis mati.**
  Analisis numerik Thrust Allocation Matrix (dari `hydroships_control/allocation.py`):
  `cond(TAM) ≈ 12.377`, singular value terkecil ≈ `1e-4` (arah yaw). Dengan
  pseudo-inverse polos (invers eksak utk TAM 6×6 full-rank), perintah:
  - **yaw 1 N·m** menuntut T3/T4 = **±5000 N** (batas ±50 N),
  - **surge 1 N** menuntut T3/T4 = **±648 N**
  → semua ter-clip lalu **merusak DOF lain**. Heading-hold & NAV_WALL karena itu
  tak bisa bekerja andal (cocok dgn catatan "belum di-tune untuk gerak nyata").
- **Akar masalah (geometri):** `thruster_positions.csv` hanya berisi POSISI dgn
  konvensi **X=kiri/kanan (lateral), Y=depan/belakang** (lihat `thruster_topview.png`),
  sedangkan frame body ROS/URDF **x=maju, y=kiri** (gambar body-frame `Gambar ROV/`).
  Posisi thruster disalin **mentah** `(X,Y,Z)→(x,y,z)` tanpa rotasi frame, sementara
  arah dorong (`axis`) di-set by-intent (T3/T4=+x). Akibatnya dua thruster horizontal
  yang secara FISIK terpisah kiri-kanan (mestinya beri yaw dari differensial) di model
  jadi terpisah depan-belakang → **lengan momen yaw ≈ 0**. Gejala serupa berpotensi
  menukar roll↔pitch pada pasangan vertikal.
- **Belum bisa diperbaiki tuntas:** orientasi/vectoring asli T100-A & T100-C **belum
  pasti** (investigasi FBX ambigu karena konvensi sumbu instance ber-"Mirror"; CSV tak
  punya kolom orientasi). Membetulkan geometri = menebak sudut → ditunda sampai data
  orientasi asli tersedia (dari pipeline pembuat CSV / CAD).
- `[RESOLVED-mitigasi]` **Allocator damped least-squares.** `allocation.build_damped_pinv`
  (dipakai `thruster_allocator`, param `alloc_damping=0.1`) menggantikan `np.linalg.pinv`
  polos. Efek (terverifikasi numerik): surge 25 N tetap **tersalur bersih ~24.5 N**
  (sebelumnya ±16.200 N lalu jenuh/rusak); perintah yaw **"menyerah anggun" ~0** alih-alih
  meledak & merusak heave/sway. Heave/sway/surge terjaga ≥98%. Ini **mitigasi numerik**,
  bukan penyembuh: yaw tetap lemah sampai geometri dibetulkan. Node kini juga
  **log cond(TAM)** & memperingatkan bila > 100.
- `[TODO]` Saat orientasi thruster asli diketahui: betulkan posisi/axis di URDF +
  `allocation.py` (idealnya satu sumber-kebenaran parametrik utk hindari drift), lalu
  turunkan `alloc_damping` bila TAM sudah well-conditioned.

## Model Visual ROV — DIBUAT ULANG jadi PRIMITIF SEDERHANA (RESOLVED tahap-1)
- `[RESOLVED]` **Model visual dibuat ULANG dari primitif ringan** (ganti mesh STL 12 MB
  yang acak-acak & berat). Visual `base_link` kini = rangka kotak HITAM (bawah) + busa
  apung ORANYE (atas) + tabung elektronik abu-abu + dome kamera depan (penanda haluan +X),
  bergaya BlueROV Heavy sesuai foto `Gambar ROV/`. Tetap mengisi bbox desain
  0.345×0.345×0.286 m; collision box (fisika/buoyancy) TIDAK diubah. Tak ada lagi mesh
  berat → render kamera tak terbebani mesh 237k segitiga. Detail per-komponen (thruster
  ducted, gripper mesh, dsb.) menyusul sebagai penyempurnaan berikutnya.
- `[RESOLVED]` `meshes/rov.stl` & `model/rov.fbx` **sudah dihapus** dari repo (tak lagi
  dirujuk URDF). Bersama itu artefak build colcon (`build/`, `install/`, `log/`) & cache
  `__pycache__/*.pyc` juga dikeluarkan dari git + ditambah `.gitignore` (praktik standar
  ROS2). Sim perlu `colcon build` ulang di mesin lokal untuk regen `install/`.

### (Catatan lama — mesh FBX, sudah tidak dipakai)
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
- `[MOOT]` **Poly-count berat.** (Historis — mesh STL 12 MB sudah dihapus & diganti
  primitif ringan; lihat "Model Visual ROV — DIBUAT ULANG". Tak lagi relevan kecuali
  mesh detail dihidupkan lagi.) Dulu: 279 komponen, fast-simplification mentok ~237 k
  segitiga → rate kamera ~22→10 Hz. Bila kelak pakai mesh: decimator quadric
  (open3d/pymeshlab) atau buang komponen kecil sebelum merge.
- `[VERIFY]` **Arah bow (haluan) belum dipastikan.** Footprint mesh ~persegi (X≈Y) jadi
  tak bisa ditebak dari bbox. Diatur via properti `bow_yaw` (rad) di `hydroships.urdf.xacro`
  (default 0 = bow menghadap +X). Cek di GUI & set 1.5708/−1.5708/3.14159 bila perlu.
- `[VERIFY]` STL monokrom (tanpa warna/material asli). Bila perlu warna, ekspor DAE
  ter-decimate (jaga ukuran) atau set material per-bagian di URDF.
- `[note]` Sumber `model/rov.fbx` (48 MB) dibiarkan di repo; yang dipakai sim = `meshes/rov.stl` (12 MB).

## Autonomy (M6) — kode dibangun, INTEGRASI JALAN (setelah fix fisika)
Node `mission_fsm` (ROS 2) + launch `hydroships_bringup/launch/hydroships_mission.launch.py`
(sim + allocator + stabilizer + FSM). FSM mengendalikan lewat setpoint stabilizer
(`setpoint/depth`, `setpoint/heading`, `manual/cmd`). (Perintah gripper dihapus — lihat M5.)

- `[RESOLVED]` Setelah dua fix fisika di atas, misi **berjalan**: FSM `IDLE→DIVE` →
  "Dasar tercapai (0.76 m)" → `DIVE→SCAN_QR` → (inject QR "A") "QR → wall A (+15)" →
  `SCAN_QR→GRAB`. Depth-hold & heading-hold bekerja.
- `[VERIFY]` Timeout tiap state, gaya (`surge_force` dll), & sudut belum di-tune untuk
  gerak nyata di arena; baru diuji transisi awal.
- `[VERIFY]` `SCAN_QR`/`APPROACH_QR` mengonsumsi `/hydroships/qr_result` — node
  `qr_detector` **sudah ada** (lihat M3, RESOLVED). Yang tersisa murni keterbacaan
  visual QR di render (lihat item "QR belum terbaca"); sampai itu beres, uji misi penuh
  dgn inject `ros2 topic pub /hydroships/qr_result` manual atau `start_state:=`.
- `[TODO]` `APPROACH_HOOK` masih *timed* (visual servo ArUco ROS 2 belum ada) — referensi
  port ada di `GUI-ROV/autonomy/`.

## Sistem Launch Simulasi Gazebo — DIPERBAIKI (RESOLVED)
- `[RESOLVED]` **`stabilizer` tak diberi `use_sim_time`.** Di
  `hydroships_stabilized.launch.py` node stabilizer jalan di wall-clock sementara
  sim & node lain di sim-time → PID d-term & laju setpoint salah timing. **Fix:**
  `parameters=[gains, {'use_sim_time': True}]`.
- `[RESOLVED]` **Race condition spawn ROV.** Node `create` dijalankan bersamaan
  dgn server gz; bila service `/world/<world>/create` belum siap, model gagal
  di-spawn. **Fix:** bungkus spawn di `TimerAction` (delay default 3 s, arg
  `spawn_delay` bisa dinaikkan untuk mesin lambat) di `sim.launch.py`.
- `[note]` `install/` kini di-gitignore → jalankan `colcon build` lokal dulu
  sebelum `ros2 launch` agar share (launch/world/config) ter-regen.

## Opsi ditunda
- `[OPEN]` **Perbesar QR khusus sim.** Alternatif membuat scan andal tanpa approach presisi:
  pakai QR jauh lebih besar dari 4 cm (mis. 15–25 cm) HANYA untuk sim, agar mudah di-decode
  dari jarak. Trade-off: tak sesuai ukuran asli KKI (4 cm). Dipilih belakangan bila perilaku
  APPROACH + hold belum cukup.

## Umum / lintas-milestone
- `[VERIFY]` Massa & koefisien hidrodinamika ROV masih **placeholder** near-neutral
  (dari `hydroships.urdf.xacro`), belum data ROV asli. Setel di M2+ dgn data nyata.
- `[OPEN]` Integrasi GUI tim (repo GUI-ROV) ↔ topik ROS 2 (M7) belum dijembatani.
