# CHANGELOG — Riwayat Kronologis HYDROships (KKI 2026)

Riwayat lengkap keputusan, temuan bug, dan perubahan — **termasuk keputusan yang
sudah dibatalkan/diganti**. Untuk status ringkas terkini lihat [STATUS.md](STATUS.md).
Format status warisan `PROBLEM.md`: `[RESOLVED]` selesai · `[VERIFY]` perlu uji
runtime · `[OPEN]` gap desain/hardware · `[REMOVED]`/`[MOOT]` dibatalkan/tak relevan.

Commit hash & tanggal dari `git log` (rentang 2026-07-07 … 2026-07-17).

---

## 2026-07-07

- **`5978f90`** — Bersihkan resource tak berguna & tambah `.gitignore` (artefak
  colcon `build/`/`install/`/`log/` & `__pycache__` dikeluarkan dari git).
- **`53e5dfc`** — Perbaiki alokasi thruster near-singular (yaw) + audit fisika.
  Diperkenalkan **allocator damped least-squares** (`build_damped_pinv`,
  `alloc_damping=0.1`) sebagai jaring pengaman; node log `cond(TAM)`.
- **`245c5df`** — Perbaiki sistem launching simulasi Gazebo.
- **`3ad812c`** — **[REMOVED] Hapus seluruh subsistem gripper** (rencana rancang ulang):
  link `gripper_base` + 2 jari, plugin `JointPositionController`/`JointStatePublisher`,
  node `gripper_controller`, topik `gripper_left/right/cmd` & `joint_states`, publisher
  `/hydroships/gripper/command`, method `_grip` di `mission_fsm`.
- **`74a63c4`** — Hilangkan model gripper dari ROV (lanjutan pembersihan di atas).

## 2026-07-08

- **`14cf649`** (Fase 1a) — **[RESOLVED] Betulkan frame posisi thruster → YAW pulih.**
  Akar masalah: `thruster_positions.csv` berkonvensi X=lateral / Y=fore-aft (depan
  negatif) / Z=atas, disalin **mentah** ke frame body ROS tanpa rotasi → posisi terputar
  90°, momen yaw T100-A/C saling meniadakan (`cond(TAM)≈1.2e4`). Fix: konversi
  `x_body=-Y_csv, y_body=-X_csv, z_body=Z_csv` di `allocation.py` & `hydroships.urdf.xacro`
  → `cond` turun ke ~20, yaw pulih (yaw 5 N·m ~ 18 N, dulu butuh 25.000 N).
- **`5387b55`** (Fase 1b) — FSM: navigasi wall holonomik (mitigasi yaw lemah).
- **`b8b0623`** (Fase 3) — Bridge `camera_info` + skrip generator QR payload.
- **`fa4dc69`** (Fase 3) — Perbesar QR khusus sim (SIM_ONLY 0.04→0.12 m) +
  `qr_detector` publish `/hydroships/qr_offset`.

### Fisika ROV — dua bug besar (RESOLVED, periode awal)
- **[RESOLVED] Buoyancy tanpa permukaan.** World memakai `<uniform_fluid_density>`
  (gaya apung di mana saja) → ROV melayang naik tanpa henti. Fix: ganti ke
  `<graded_buoyancy>` (air 1000 di bawah z=0, udara 1 di atas).
- **[RESOLVED] Thrust tak pernah masuk.** Plugin Thruster dengan `<namespace>hydroships`
  men-*prepend* namespace → subscribe `/hydroships/hydroships/thruster_N/thrust`,
  sedangkan bridge publish `/hydroships/thruster_N/thrust` → gaya nol. Fix: `<topic>`
  jadi `${name}/thrust` (tanpa prefix). Catatan: M1/M2 sebelumnya keliru ✅ — gerak lama
  murni buoyancy, bukan thrust.

### Model visual ROV (RESOLVED tahap-1)
- **[RESOLVED] Model visual dibuat ulang dari primitif ringan** (rangka kotak hitam +
  busa apung oranye + tabung elektronik + dome kamera penanda haluan +X), menggantikan
  mesh STL 12 MB (~237k segitiga). Collision box (fisika/buoyancy) tak diubah.
- **[RESOLVED] `meshes/rov.stl` & `model/rov.fbx` dihapus dari repo** (tak lagi dirujuk
  URDF). Lihat catatan "File legacy" di [ARCHITECTURE.md](ARCHITECTURE.md).

## 2026-07-11
- **`7d6a3e1`** — refactor: sesuaikan gain PID depth stabilization.

## 2026-07-12
- **`0b2aa8d`** — feat: tambah model URDF ROV & desain KKI 2026.

## 2026-07-14

- **`fd06b0a`** — **[RESOLVED] Redesign manipulator: DetachableJoint grasp +
  `gripper_controller` node.** Rancang ulang dari nol (bukan menghidupkan gripper 2-jari
  lama yang grasp fisiknya tak pernah lolos uji). Grasp = gz-sim `DetachableJoint`
  (`parent_link=gripper_base`, `child_model=payload`, attach/detach via
  `/hydroships/gripper/attach` & `/detach`); jari `gripper_jaw` 1-DOF kosmetik via
  `/hydroships/gripper_jaw/cmd`. Attach hanya saat "close" DAN ROV di atas payload dalam
  jangkauan aman (dari `/hydroships/qr_offset`). Kontrak `/hydroships/gripper/command`
  "open"/"close" dipertahankan. Logika murni `gripper_logic.py` teruji headless.
- **`df8f71e`** — **[RESOLVED] Fix DetachableJoint initial-attach (Fortress).** Tag
  `<suppress_initial_attach>` TIDAK valid di Fortress (fitur `<initial_attach>` baru
  diusulkan PR gz-sim #3268 utk gz-sim10) → DetachableJoint selalu attached saat load.
  Fix: hapus tag, `gripper_controller` menerbitkan **satu detach otomatis saat startup**
  (`startup_detach_delay=1.5 s`). Idempoten.
- **`acac770`** — **[RESOLVED] Bridge GUI-ROV via adapter + port hook visual servo.**
  GUI-ROV ternyata **bukan ROS2** (UDP-JSON + MAVLink/ArduSub) → node adapter `gui_bridge`
  (bukan remap): UDP JSON `{name,value}` → `/hydroships/cmd_vel` & `/hydroships/gripper/command`;
  `/hydroships/odom`+`/depth` → telemetri UDP JSON. `autonomy/vision/hook_detect.py`
  di-port jadi node `hook_detector` → `/hydroships/hook_offset`.
- **`499ab31`** — **[RESOLVED→VERIFY] APPROACH_HOOK: upgrade servo proporsional-heading →
  PD holonomik** (`hook_logic.hook_servo`): sway dari offset-x, surge dari ukuran-tampak,
  koreksi setpoint kedalaman dari offset-y, redaman kecepatan body-frame; heading di-hold
  ke wall; fallback timed aman. Teruji `test/test_hook_servo.py`.
- **`a143306`** — QR decode robustness (`qr_logic.robust_decode`: grayscale+CLAHE →
  adaptive/Otsu → upscale) + dokumentasi gap intrinsics `camera_info`.
- **`e84b619`** — `qr_detector`: hormati `msg.step` (row stride) + instrumentasi log
  diagnosis (frame pertama per kamera; decode-gagal throttled yang membedakan
  "QR tak terdeteksi" vs "terdeteksi tapi decode kosong").
- **`045d7c4`** — **[RESOLVED-parameterisasi] Externalize param fisik** ke
  `hydroships_description/config/rov_params.yaml` (dibaca URDF via `xacro.load_yaml`):
  `base_mass`, `thruster_mass`, `fluid_density`, `cog`/`cob`, tensor inertia, 18 koefisien
  hidrodinamika. Alat `scripts/estimate_mass_inertia.py`. Angka masih `[estimate]` sampai
  diukur pada ROV asli. URDF hasil identik dgn versi hardcode; test tetap lolos.
- **`c5988f0`** — Deklarasi dependency Python (opencv, numpy).
- **`b9c97f5`** — `hydroships_gui.launch.py` (sim + `thruster_allocator` + `gui_bridge`).
- **`45b3df6` / `f98923c`** — docs: HOW-TO-RUN (multi-terminal teleop + argumen GUI launch).
- **`3a685b7`** — Update PROBLEM.md.

## 2026-07-15 … 07-16
- **`6872143`** — approach hook.
- **`d64a589` / `8607f26`** — Merge PR #10 & #11 (feature/approach-hook-navigation).
- **`ac8c243`** — chore(docs): dokumentasi proyek & metadata workspace.
- **`ef48529`** — graphify-integration.

## 2026-07-17
- **`612eee3`** — uv add.
- **[RESOLVED] Fix DetachableJoint init gagal — `gripper_base` tak ditemukan di SDF.**
  Gejala: `[Err] [DetachableJoint.cc:62] Link with name gripper_base not found in
  model hydroships` saat `ros2 launch hydroships_gazebo sim.launch.py world:=kki_arena.sdf`.
  Akar masalah: joint `gripper_base_joint` bertipe `fixed` → saat `sdformat`
  convert URDF→SDF, child link dari joint `fixed` **di-lump/collapse** ke parent
  (`base_link`) secara default. `robot_state_publisher` (baca URDF asli) tetap kenal
  `gripper_base`, tapi plugin `gz-sim-detachable-joint-system` (baca SDF hasil convert)
  tidak → `parent_link` invalid. `gripper_jaw` tak kena karena joint-nya `revolute`.
  Fix: `<gazebo reference="gripper_base_joint"><preserveFixedJoint>true</preserveFixedJoint></gazebo>`
  di `hydroships.urdf.xacro` — menahan joint fixed jadi joint SDF nyata sehingga
  `gripper_base` tetap link tersendiri (kinematika kaku tak berubah). Diverifikasi
  dgn `ign sdf -p` (Fortress): link `gripper_base` kini muncul di SDF hasil convert.
  Catatan: `<dontcollapse>` TIDAK dikenali sdformat, dan `<disableFixedJointLumping>`
  pada link TIDAK cukup di Fortress bila `base_link` punya banyak child fixed-joint
  lain (imu/kamera) — link tetap ter-lump; `preserveFixedJoint` pada joint andal.
- **[RESOLVED] Fix `NAV_WALL` langsung ABORT saat FSM di-start mid-state utk testing.**
  Gejala: `start_state:=NAV_WALL` → `NAV_WALL -> ABORT` dalam 1 tick (~250ms), robot
  tak bergerak. Akar masalah: `self.wall` hanya di-set di `_st_approach_qr`/`_st_scan_qr`
  saat QR terbaca; start mid-FSM via `start_state` melewati state itu → `self.wall`
  masih `None` → guard `if self.wall is None: self._to(St.ABORT)` di `_st_nav_wall`
  langsung memicu ABORT (guard-nya benar utk operasi normal). Fix: parameter baru
  `start_wall` di `mission_fsm.py` yg men-seed `self.wall` (divalidasi thd
  `WALL_HEADING_DEG`) setelah init `self.wall = None`. Guard tak diubah. State lain
  yg baca `self.wall` (`HANG`/`SURFACE`/`APPROACH_HOOK`/`AUTO_RELEASE`) kini bisa
  dites langsung dgn `start_wall` yg sama. Catatan: `hydroships_mission.launch.py`
  juga di-update untuk mendeklarasikan & meneruskan arg `start_wall` ke node —
  tanpa ini arg CLI diabaikan diam-diam & node tetap pakai default `''` (ABORT).
- **[RESOLVED] QR detection akhirnya TERBUKTI runtime — root cause = FRAMING, bukan
  render/decode.** Diverifikasi di mesin dgn ROS 2 Humble + Gazebo Fortress + GPU/EGL
  (sebelumnya selalu [VERIFY], mesin dev lama tak punya sim). Diagnosis berurutan:
  (1) image MENGALIR (`FRAME PERTAMA` bottom & front, `rgb8` 640x480 `step=1920` = tanpa
  padding) → render/bridge OK. (2) Simpan frame mentah kamera bottom → QR ter-render
  TAJAM & kontras tinggi (std ~88, range 0–255) → material/emissive/PBR OK, decode-logic
  OK. (3) TAPI QR ter-CROP di tepi bawah frame (finder-pattern bawah keluar frame) +
  gripper ROV menutupi ~1/3 atas frame → `cv2.QRCodeDetector` gagal (`pts=None`). Sebab:
  di `scan_depth=0.62` kamera bawah cuma ~9 cm di atas QR (world z=-0.893) → QR 12 cm
  memenuhi/melebihi frame. **Bukan** orientasi (dihitung: normal plane QR = +Z dunia,
  searah pandang `camera_bottom_link` — sudah benar), **bukan** kontras/ukuran, **bukan**
  quiet-zone. Fix: `scan_depth 0.62 → 0.46` (`mission_fsm.py`) → kamera ~25 cm di atas QR
  → QR utuh + quiet-zone di frame. Dibuktikan: frame nyata decode `'A'` (raw & robust),
  DAN misi penuh headless: `qr_detector: QR terbaca "A" -> sisi A` → `mission_fsm:
  QR -> wall A (+15) [dist 0.01m]` → `APPROACH_QR -> GRAB -> NAV_WALL`. Tambahan:
  `t_scan 45→60` (ROV spawn lebih dalam dari scan_depth → APPROACH_QR harus NAIK ~0.27 m
  dulu, makan ~40 s; 45 s terlalu mepet). Centering kamera (offset +0.02 m) TIDAK perlu —
  standoff lebih tinggi memberi margin cukup (diuji uncentered tetap decode `'A'`).
  Aset: `qr_B/C/D.png` di-generate (`generate_qr.py`, isi = huruf tunggal spy QR versi
  rendah/modul besar; menyamakan konvensi qr_A.png ter-commit; parse_wall tetap terima
  string panjang). Regresi frame-nyata ditambah ke `test_qr_logic.py`
  (`test_robust_decodes_real_sim_frame`, fixture `qr_sim_bottom_A.png`).

---

## Keputusan yang DIBATALKAN / diganti (arsip)

Disimpan sebagai referensi agar tidak dihidupkan ulang tanpa sadar.

### Gripper 2-jari (dibatalkan → diganti DetachableJoint)
Desain lama: 2 jari revolute sumbu-z di depan ROV, dikontrol gz `JointPositionController`,
state di `/hydroships/joint_states`, topik `gripper_left/right/cmd`. Sudut terverifikasi
visual (open ≈ +0.50/−0.50, close ≈ −0.14/+0.15 rad), **tapi grasp fisik tak pernah lolos
uji** (butuh tuning friction/contact). Dihapus di `3ad812c`/`74a63c4`, diganti pendekatan
DetachableJoint di `fd06b0a`. **Jangan** kembalikan gripper 2-jari; desain aktif = jari
kosmetik + DetachableJoint (lihat [STATUS.md](STATUS.md)).

### Mesh berat FBX/STL (dibatalkan → primitif)
`model/rov.fbx` (FBX satuan mm, 279 sub-mesh; masalah transform di-fix via assimp
`aiProcess_PreTransformVertices`) dikonversi ke `meshes/rov.stl` (~237k segitiga, 12 MB)
→ menurunkan rate kamera (~22→10 Hz). **Kedua file sudah dihapus dari repo**; model aktif
= primitif ringan di `hydroships.urdf.xacro`. (Catatan lama "`model/rov.fbx` 48 MB dibiarkan
di repo" **tidak berlaku lagi** — sudah dihapus.)

### `<suppress_initial_attach>` (tidak valid di Fortress)
Tag ini diabaikan diam-diam oleh gz-sim Fortress; digantikan mekanisme auto-detach startup
(lihat `df8f71e`).

### Opsi ditunda
- Perbesar QR jauh lebih besar dari 4 cm (15–25 cm) khusus sim — hanya bila approach+hold
  presisi belum cukup untuk decode.
- Servo hook pose-based (solvePnP/PBVS) — menyusul bila kalibrasi kamera fisik hook tersedia.
