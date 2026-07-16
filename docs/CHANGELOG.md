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
