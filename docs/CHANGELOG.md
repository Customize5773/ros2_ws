# CHANGELOG — Riwayat Kronologis HYDROships (KKI 2026)

Riwayat lengkap keputusan, temuan bug, dan perubahan — **termasuk keputusan yang
sudah dibatalkan/diganti**. Untuk status ringkas terkini lihat [STATUS.md](STATUS.md).
Format status warisan `PROBLEM.md`: `[RESOLVED]` selesai · `[VERIFY]` perlu uji
runtime · `[OPEN]` gap desain/hardware · `[REMOVED]`/`[MOOT]` dibatalkan/tak relevan.

Commit hash & tanggal dari `git log` (rentang 2026-07-07 … 2026-08-06).

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
- **[RESOLVED] Restore gripper + hook-servo integration yang hilang di merge PR #14.**
  PR #14 (`76dae05`) mengambil sisi `3f50a69` (revert lengkap gripper oleh `lockkers844-web`
  yang menghapus 103 baris) sambil hanya menarum QR fixes dari branch `rasya/dev2`
  (`4fbc6f8`). Hasilnya: `mission_fsm.py` HEAD kehilangan integrasi gripper (`_grip`),
  hook visual servo PD (`APPROACH_HOOK`), `done_hooks` tracking di `_st_surface` +
  `_st_auto_release`, serta loop kembali DIVE bila <4 hook. Semua itu di-restore dari
  `4fbc6f8` sambil mempertahankan QR fixes (`scan_depth=0.46`, `t_scan=60`,
  `start_wall`). Test headless 62 tetes lolos.
- **[RESOLVED] Bangun body gripper yang terlihat di Gazebo, dipasang di muka depan ROV.**
  Body gripper sebelumnya hanya box 0.05×0.05×0.03 m (5 cm³) di perut bawah ROV
  sehingga hampir tidak terlihat di sim. Sekarang dipasang di muka depan ROV
  (joint `xyz="0.18 0 0"`), body 0.10×0.10×0.06 m, massa 0.12 kg, material kuning
  kontras. Jari `gripper_jaw` menjorok ke depan (+X) sepanjang 0.12 m. Mekanisme
  DetachableJoint + plugin tidak diubah.

---

## 2026-07-18

- **[RESOLVED] ROV kini di-spawn RANDOM dekat dinding kolam (posisi kontes) via launch arg.**
  Sebelumnya ROV selalu spawn di tengah (0,0,-0.5). Kontes mendeploy ROV dekat dinding lalu
  jalankan misi autonomous, jadi spawn perlu acak & realistis. `sim.launch.py` dapat helper
  `_rov_spawn_xyz`: `rov_random_spawn=true` (default) → pilih acak 1 dari 4 dinding (konvensi
  `mission_fsm._wall_inward`: A=-Y, B=+Y, C=+X, D=-X), koordinat mepet dinding di
  `±(rov_arena_half - rov_wall_margin) = ±2.05 m`, koordinat lain tersebar acak sepanjang
  dinding dalam rentang aman sama; `false` → pakai `rov_x/rov_y/rov_z`. Param baru:
  `rov_random_spawn`, `rov_x/y/z`, `rov_wall_margin` (0.5), `rov_arena_half` (2.55) — mengganti
  arg lama `x/y/z`. Diteruskan konsisten lewat rantai launch `hydroships_mission →
  hydroships_stabilized → sim`. Log `[sim.launch] ROV spawn (random=…) di (x,y,z)`. `z` selalu
  dari `rov_z` (kedalaman aman −0.5, di bawah permukaan); clearance dari dinding fisik ±2.55 =
  0.50 m (≥ 0.4 m aman). `payload_spawner` TAK diubah (payload tetap acak terpisah). Build OK;
  unit-test helper: 2000 sampel semua on-wall & in-bounds, 4 dinding tersebar merata, override
  manual `(1.0,-1.0,-0.5)` tepat; ketiga launch expose semua `rov_*` (`--show-args`).
  **[VERIFY]** runtime di sim: ROV muncul di posisi beda dekat dinding tiap run tanpa nabrak,
  misi autonomous tetap jalan (DIVE dari posisi mana pun) sampai DONE/ABORT wajar.

- **[RESOLVED] ROV stuck di GRAB→NAV_WALL & menabrak dinding keras — safety standoff + HANG aman.**
  Gejala: setelah APPROACH_QR→GRAB→NAV_WALL, ROV tiba di dinding lalu "idle" & misi tak
  lanjut. Dua akar masalah di `mission_fsm.py`: (1) **`_st_hang` menabrak dinding** — fase
  `e<8.0` memanggil `_move_world(ux,uy,15.0)` (gerak MENUJU dinding) lalu mundur; placeholder
  "manipulasi dihapus" yg berbahaya (rusak struktur ROV). (2) **NAV_WALL tanpa wall-avoidance**
  — target `wall_dist=2.30` sedangkan muka dalam dinding fisik di ±2.5 m (`kki_arena.sdf`),
  clearance cuma 0.20 m → PD (`approach_kp=90`, `nav_fmax=22`) overshoot → ROV mentok, odom
  loncat, `dist` tak pernah < `nav_tol=0.15` → osilasi/"idle". Fix: **(A)** target NAV_WALL
  kini `wall_face(2.5) - wall_standoff(0.45) = 2.05 m` (clearance aman 0.45 m); param baru
  `wall_face`, `wall_standoff` (ganti `wall_dist`). **SOFT-STOP**: helper `_wall_clearance()`
  hitung sisa jarak ke muka dinding; bila < `wall_standoff` → `_move_world` MENJAUHI dinding
  (tak pernah didorong lebih dekat), di NAV_WALL & HANG. **(B)** `_st_hang` ditulis ulang:
  HOLD lembut di standoff (`_goto_xy` ke target standoff + soft-stop) selama `hang_hold=6 s`
  (simulasi gantung) lalu SURFACE — TANPA gerak agresif ke dinding. **(C)** transisi
  NAV_WALL→HANG kini butuh `dist < nav_tol(0.25)` **DAN** `|v| < nav_settle_vel(0.10)` (settle,
  tak transisi mid-osilasi); log jelas "Tiba di standoff wall X -> HANG" & "HANG: tahan di
  standoff wall X". Timeout NAV_WALL tetap berlaku walau soft-stop aktif. APPROACH_QR/GRAB/
  SCAN_QR **tak diubah**. Build + 62 test lolos; smoke-test geometri: target wall C = (2.05,0),
  clearance @x=2.2 = 0.30 m → soft-stop aktif, @x=2.0 = 0.50 m aman. **[VERIFY]** end-to-end
  di sim: APPROACH_QR→GRAB→NAV_WALL→HANG→SURFACE…→DONE (4 hook) tanpa tabrakan; ROV berhenti
  ~0.45 m dari dinding (cek GUI Gazebo). Tuning `wall_standoff`/gain bila perlu.

- **[RESOLVED] Urutan spawn payload vs auto-detach gripper diperbaiki (payload nempel salah saat spawn).**
  Gejala: payload spawn LEBIH LAMBAT dari startup-detach gripper (timer 1.5 s), jadi saat
  model `payload` muncul gz-sim Fortress langsung auto-attach DetachableJoint ke ROV (perilaku
  default load) dan tak ada detach lagi setelahnya → payload "nempel" ke gripper sejak awal,
  melanggar alur spawn→QR→GRAB(attach). Fix: **startup-detach kini dipicu topik**
  `/hydroships/payload/spawned` (`std_msgs/Empty`, QoS latched) yg diterbitkan `payload_spawner`
  SETELAH `ros_gz_sim create` sukses — detach dijamin terjadi *setelah* payload ada, bukan
  pada timer buta. `gripper_controller` timer lama (`startup_detach_delay=1.5`) diganti
  `startup_detach_fallback=8.0` sbg jaring pengaman saja (bila spawner tak jalan); keduanya
  idempoten (`_do_startup_detach`, guard `_did_startup_detach`). `gripper_logic.startup_detach`
  (murni) TAK berubah → 62 test tetap lolos. Selain itu `payload_spawner` kini **publish
  `/hydroships/payload_pose` SEGERA di awal `_spawn`** (sebelum subprocess create yg bisa lambat)
  agar FSM tak menganggur menunggu pose; sinyal `spawned` hanya terbit bila create benar-benar
  sukses (create gagal → tak ada payload → tak perlu detach). Launch: delay spawner
  `spawn_delay+1.0 → +0.5` (payload muncul lebih awal; urutan attach/detach dijaga topik, bukan
  timing). Diverifikasi smoke-test: gripper TIDAK detach sebelum sinyal `spawned` (0 detach dlm
  2 s), detach sekali begitu sinyal tiba (latched terkirim ke subscriber). **[VERIFY]** urutan
  end-to-end di sim: log `Payload QR=… spawned OK` → `payload/spawned diterbitkan` → gripper
  `auto-detach startup [pemicu: payload spawn terdeteksi]`; payload TIDAK ikut gerak ROV di awal;
  attach hanya di GRAB saat qr_offset aman.

- **[RESOLVED] ROV susah/lama baca QR di misi 3C (APPROACH_QR) — 3 root cause diperbaiki.**
  Gejala: di misi autonomous penuh ROV masuk APPROACH_QR lalu seakan diam/tak sampai
  di atas payload, `qr_result` tak terbit → timeout `t_scan` (60 s) → ABORT. Penyebab &
  fix: (1) **Lampu payload gelap di posisi random.** `payload_fill` di `kki_arena.sdf`
  hardcode di (0.4,0,-0.45) range 0.8 m, sedangkan `payload_spawner` me-random payload
  ke x∈[0.2,0.6] y∈[-1.5,1.5] → payload sering di luar radius lampu → QR kontras rendah.
  Range 0.8→3.0 m, atenuasi dilandaikan (constant 0.3→0.6, linear 0.5→0.15, quad 1.0→0.08),
  diffuse 0.8→0.9 → menutupi seluruh area spawn. (2) **`_st_approach_qr` tanpa guard odom
  & timeout navigasi.** `_goto_xy` return 999 tanpa publish gaya bila `self.x/self.yaw`
  belum ada, & state hanya menunggu `qr_result`. Ditambah: guard odom (log sekali, reset
  baseline timeout, tak dianggap "sampai"), timeout navigasi `t_nav_qr` (30 s) dgn recovery
  (naik 0.10 m perluas FOV kamera bawah), pesan ABORT jelas "gagal capai payload [dist]".
  (3) **Tak ada centering.** FSM kini subscribe `/hydroships/qr_offset` (ternormalisasi
  [-1..1]) & lakukan visual servo halus: bila QR di pinggir frame (|offset|>`qr_center_tol`
  0.12), geser target hold sebesar `qr_servo_gain` (0.15 m) agar QR ke tengah (sign x/y via
  param, perlu **[VERIFY]** runtime mounting kamera). Param baru: `t_nav_qr`, `qr_off_max_age`,
  `qr_center_tol`, `qr_servo_gain`, `qr_servo_sign_x/y`. `scan_depth` (0.46) TAK diubah.
  Reliabilitas pose: `payload_spawner` publish `/hydroships/payload_pose` kini **latched**
  (QoS transient_local) + republish periodik 2 Hz → subscriber late-join (FSM) selalu dapat.
  Build + 62 test lolos; node FSM smoke-test konstruksi OK (subscription qr_offset/payload_pose
  terdaftar, nudge logic benar). **[VERIFY]** perilaku end-to-end di sim (gerak ke payload,
  QR terbaca <~10 s di posisi random, tanpa ABORT gelap) belum diuji runtime.

- **[RESOLVED] Payload QR sekarang di-spawn RANDOM (A/B/C/D) via node `payload_spawner`.**
  Model `payload` dihapus dari `worlds/kki_arena.sdf` dan diganti spawn dinamis oleh
  `hydroships_gazebo/scripts/payload_spawner.py` (`ros2 run ros_gz_sim create` + template
  SDF inline `PAYLOAD_SDF_TEMPLATE`, identik dgn definisi lama: mesh body, collision,
  quiet-zone, QR pbr, massa 0.3 kg non-static). Huruf QR dipilih random (atau via launch
  arg `qr_letter:=A/B/C/D`); posisi acak dalam bounds arena (`arena_x/y_min/max`) saat
  huruf random, atau eksplisit via `payload_x`/`payload_y` bila `qr_letter` di-set.
  Node publikasi posisi ke `/hydroships/payload_pose` (`PointStamped`); `mission_fsm`
  `_st_approach_qr` navigasi ke pose tsb (fallback ke param `payload_x/payload_y` bila
  belum tiba). Argumen diteruskan lewat rantai launch
  `hydroships_mission → hydroships_stabilized → sim`. `payload_fill` light tetap di SDF.
  Executable dipasang via CMake `install(PROGRAMS ... RENAME payload_spawner)`.
  Build + 62 test lolos; SDF/launch tervalidasi headless. **[VERIFY]** spawn & grasp
  fisik di sim belum diuji runtime (butuh mesin ber-display / gz server).
  Catatan: DetachableJoint (`child_model=payload`) kini me-resolve payload yg di-spawn
  belakangan; perlu verifikasi attach tetap bekerja saat payload muncul pasca-load.

---

## 2026-07-28

- **[RESOLVED] Redesign gripper: jari tunggal → DUA JARI OPPOSING (tetap kosmetik).**
  Jari kosmetik lama `gripper_jaw` memakai revolute sumbu **Y**, jadi mengayun naik-turun
  di bidang XZ — terlihat seperti flap/scoop, bukan gripper yang menjepit. Diganti dua jari
  `gripper_finger_left`/`gripper_finger_right` (box 0.08×0.016×0.03 m, massa 0.06 kg) dengan
  revolute sumbu **Z** di pivot `(0.04, ∓0.025, 0)` relatif `gripper_base` → menjepit di
  bidang XY seperti parallel gripper. Sudut `0.0` = tertutup, `0.35` = terbuka (limit joint
  `[-0.1, 0.5]`; `jaw_open` diturunkan dari 0.6 agar muat). **Mirroring diurus tanda `axis`**
  (kiri `0 0 -1`, kanan `0 0 1`) sehingga nilai Float64 yang **sama** dikirim ke kedua joint —
  `GripperLogic.jaw_target` tetap **skalar tunggal**, API logic tak berubah sama sekali.
  URDF pakai `xacro:macro` `gripper_finger` + `gripper_finger_plugin` (dua instance
  `JointPositionController`, satu per joint — cara standar Fortress). Topik jari
  `/hydroships/gripper_jaw/cmd` **diganti** `/hydroships/gripper_left/cmd` +
  `/hydroships/gripper_right/cmd` (bridge.yaml, `Float64` ↔ `gz.msgs.Double`).
  **Tidak berubah**: `DetachableJoint` (grasp fisik sesungguhnya), `preserveFixedJoint` pada
  `gripper_base_joint`, kontrak `/hydroships/gripper/command` & `attach`/`detach`, `mission_fsm`,
  GUI. **Ini BUKAN pemulihan gripper 2-jari lama** (lihat arsip di bawah) — jari baru tetap
  murni kosmetik, tidak menahan payload lewat gesekan.
  **[VERIFY]** runtime di sim: kedua jari terlihat & bergerak serempak, dan jari tidak
  menutupi pusat frame kamera bawah (deteksi QR tetap jalan).

- **[RESOLVED] `APPROACH_QR` navigasi ke pose payload ASLI + visual servo QR benar-benar
  di-wire.** Investigasi "ROV susah di APPROACH_QR" menemukan bahwa beberapa fitur yang
  sebelumnya dicatat `[RESOLVED]` di CHANGELOG **implementasinya tidak pernah ada** di
  `mission_fsm.py` — variabel state-nya dideklarasikan tapi `create_subscription`-nya
  hilang. Empat akar masalah, semua diverifikasi langsung ke kode:
  - **RC4 (utama).** `self.payload_pose` dideklarasi tapi **tak pernah subscribe**
    `/hydroships/payload_pose`; FSM selalu navigasi ke param statis `payload_x/y`
    (default `0.4, 0.0`) padahal payload di-random tiap run → ROV berhenti di tempat
    salah sehingga QR jarang masuk frame kamera bawah (log run lama didominasi
    `DECODE GAGAL: QR tak terdeteksi (pts=None)`). Kini subscribe dgn QoS
    `TRANSIENT_LOCAL` (menyamai publisher latched di `payload_spawner.py`); param
    tinggal fallback sampai pesan tiba.
  - **RC5.** `self.qr_off`/`qr_off_time` dideklarasi tapi **tak pernah subscribe**
    `/hydroships/qr_offset` → visual servo centering yang dideskripsikan di CHANGELOG
    tak pernah berjalan. Kini di-wire, dan **difilter `frame_id == 'camera_bottom_link'`**
    — `qr_detector` menerbitkan offset untuk kamera **bawah maupun depan** di topic yang
    sama, offset kamera depan akan menyesatkan servo saat memusatkan diri di atas payload.
  - **RC6.** Hanya ada satu timeout `t_scan` (45 s, bukan 60 s; dan tidak ada state
    `SCAN_QR` — `t_scan` cuma dipakai `APPROACH_QR`). Ditambah `t_nav_qr` (30 s) sebagai
    pemicu **RECOVERY** — bukan abort: bila QR belum terbaca, setpoint depth dinaikkan
    +0.10 m untuk memperlebar FOV kamera bawah (QR 12 cm mudah memenuhi frame & finder
    pattern ter-crop saat terlalu rendah, lih. catatan `scan_depth` 0.62→0.46). Abort
    tetap hanya di `t_scan`.
  - **RC7.** `self._approach_move_t0` dead code (di-assign, tak pernah dibaca) — dihapus.

  **Alur baru: pusatkan dulu, baru GRAB.** Saat QR terbaca, `self.wall` dikunci & skor m1
  diberi, tapi FSM **tetap di APPROACH_QR** menjalankan servo sampai QR terpusat
  (`|ex|,|ey| < qr_center_tol`) atau `dist < approach_tol` → jepitan lebih presisi.
  Ini **menggantikan** perilaku sementara sebelumnya yang langsung `→ GRAB` begitu QR
  terbaca (yang membuat servo praktis tak pernah terpakai). Fallback `wall_order` tetap
  ada untuk kasus QR tak pernah terbaca. Flag `_wall_scored`/`_approach_recovered`
  di-reset di `_to()` saat masuk `APPROACH_QR` karena misi berulang per payload
  (`AUTO_RELEASE → DIVE → APPROACH_QR`).

  **Catatan frame (rawan salah tanda).** `ex,ey` dari `offset_from_points` bersumbu
  **CITRA** ternormalisasi `[-1..1]`, sedangkan `_goto_xy` menerima target **DUNIA**;
  yaw ROV berubah terus sehingga pemetaan tetap akan salah arah saat ROV berputar.
  Nudge dihitung di body-frame (`body_dx=-ey·k`, `body_dy=-ex·k`) lalu diputar dgn yaw.
  **Salah tanda = umpan balik POSITIF (ROV menjauh dari payload)** — karena itu tanda
  dibuat parameter eksplisit `qr_servo_sign` (default `+1.0`) dan `ex/ey` di-log tiap
  siklus. Param baru: `t_nav_qr`, `qr_center_tol` (0.12), `qr_servo_gain` (0.15),
  `qr_servo_sign`.
  **[VERIFY]** runtime: (a) log `APPROACH_QR dbg` menampilkan `target=` pose payload
  sebenarnya (mis. `(0.50,-1.20)`), bukan `(0.40,0.00)`; (b) `ex/ey` **mengecil** menuju 0
  saat mendekat — bila **membesar**, balik `qr_servo_sign:=-1.0` sebelum mengubah logika
  lain; (c) recovery depth-ascent memicu di 30 s saat QR tak terbaca.

---

## 2026-07-28 … 2026-08-03 (belum dicatat sebelumnya)

- **`24dd349`** — Model gripper baru (lihat entry dua-jari-opposing di atas, 2026-07-28).
- **`4f86d4a`** — Raw model FUSION 360 + payload hook approach dengan visual servo
  (lihat entry `APPROACH_HOOK` upgrade PD holonomik di atas).
- **`b754a53`** — Evaluasi kamera ROV HEAD & BOTTOM (dokumentasi/analisis mounting kamera).
- **[RESOLVED] Physics: skala volume buoyancy & koefisien hidrodinamika; massa ROV
  33.6 kg → 8.3 kg.** — `00d4aaa` (chore(physics): update mass properties and buoyancy
  compensation) + `aa2410d` (feat(physics): scale buoyancy volume and hydrodynamic
  coefficients, faktor skala ~0.247, `buoyancy_ff -120.0 → -0.3`). Mengubah
  `src/hydroships_description/config/rov_params.yaml`, `gains.yaml`, `stabilizer.py`, URDF.
  **Regresi yang ditemukan akibat perubahan ini** — lihat item NAV_WALL di bawah.
- **`99d990a`** — Launch diagnostics & service discovery (turut menyentuh `mission_fsm.py`,
  ~130 baris; tak ada regresi fungsional teridentifikasi dari perubahan ini secara terpisah).
- **Koreksi commit hash**: entry lama yang menyebut `b754a33` seharusnya `b754a53` (hash
  `b754a33` tidak ada di riwayat git repo ini).

## 2026-08-06 — Verifikasi runtime penuh (mesin dev kini punya ROS 2 Humble + Gazebo
Fortress 6.18 + EGL/mesa) — build, smoke-run, & seluruh state FSM diuji end-to-end/isolasi

Metodologi: `colcon build` bersih, `pytest` **76/76** lolos (bukan 62 seperti tercatat di
entry lama — jumlah tes bertambah sejak `test_qr_ey_target.py` & lainnya ditambahkan).
Smoke-run headless (`spawn_seed:=42`), lalu misi penuh dari `DIVE` (`spawn_seed:=42
qr_letter:=A`), lalu isolasi `start_state:=APPROACH_HOOK start_wall:=B` (`spawn_seed:=12`)
utk memisahkan verifikasi hook dari bug NAV_WALL di bawah, lalu `hydroships_gui.launch.py`
(`spawn_seed:=5`) + client UDP JSON sintetis ke port 14550/14551.

- **[RESOLVED] `camera_info` & render kamera headless — TERBUKTI ulang.** `fx=fy=381.4
  cx=320 cy=240 (640x480)`, `camera_bottom/front/image_raw` mengalir `rgb8 step=1920`
  segera setelah spawn.
- **[RESOLVED] scan_depth=0.30 (revisi dari 0.46, `mission_fsm.py:107-121`, alasan
  offset gripper `cam_gripper_dx=0.16 m` di depan kamera bawah) — QR 'A' TETAP terdecode
  runtime & misi lanjut ke GRAB** (`DIVE → APPROACH_QR → GRAB` sukses, skor m1 +15 lalu
  m2 +15 "GRAB terverifikasi"). Regresi yang dikhawatirkan render CHANGELOG lama
  (0.62→0.46) **tidak berulang** di 0.30. QR huruf B/C/D **tidak** diuji ulang sesi ini
  (kehabisan waktu memprioritaskan bug baru di bawah) — tetap `[VERIFY]`.
- **[REGRESI BARU — OPEN, BLOCKING] `GRAB` tidak pernah benar-benar meng-attach payload.**
  `mission_fsm._st_grab` (`mission_fsm.py:606-618`) HANYA menunggu `hold_settle_s` lalu
  memberi skor +15 dan pindah ke `NAV_WALL` — **tidak pernah** publish ke
  `/hydroships/gripper/command` ("close"). Diverifikasi via `grep`: publisher
  `self.pub_grip` (`mission_fsm.py:221`) dideklarasikan tapi **tidak sekalipun dipanggil
  `.publish()`** di seluruh file; satu-satunya jalur `gripper_controller` menerbitkan
  `/hydroships/gripper/attach` adalah lewat `_on_cmd` yang men-subscribe
  `/hydroships/gripper/command` (`gripper_controller.py:56-129`) — tanpa pesan "close"
  masuk, attach **tidak pernah terpicu**. Docstring `_st_grab` sendiri mengonfirmasi ini
  sebagai desain sementara: *"Verifikasi ROV diam sejenak di atas QR sebagai pengganti
  event attach nyata"* — tapi ini **berlawanan** dengan deskripsi `STATUS.md`/`CHANGELOG`
  lama ("Attach hanya di GRAB saat qr_offset aman", commit `fd06b0a`) yang mengasumsikan
  `_st_grab` memicu attach nyata. **Akibat**: payload TETAP lepas (hasil startup
  auto-detach) sepanjang `NAV_WALL/HANG/SURFACE/...` — ROV "mengantar" tanpa payload
  secara fisik. Ini bukan sekadar `[VERIFY]` tertunda — ini **fungsi inti misi yang
  hilang**, prioritas perbaikan tertinggi sebelum klaim M5 bisa ✅. **Belum** diverifikasi
  via topic-echo langsung (`ros2 topic echo /hydroships/gripper/attach` selama GRAB) —
  disarankan sebagai langkah pertama sesi berikut untuk konfirmasi empiris tambahan,
  meski bukti `grep` (tak ada call site) sudah meyakinkan.
- **[REGRESI BARU — OPEN, BLOCKING] `NAV_WALL` tidak konvergen ke `nav_tol`, timeout →
  ABORT.** Repro: `spawn_seed:=42 qr_letter:=A start_state:=DIVE`, wall target
  `(0.00, 2.30)` (dari `wall_dist=2.30`, `mission_fsm.py:126`). ROV mendekat sampai
  `dist≈0.26 m` (tepat di atas `nav_tol=0.20 m`, `mission_fsm.py:129`) lalu **posisi x/y
  membeku** (x=0.25, y=2.38 tak berubah di banyak tick berturut-turut) sementara yaw
  perlahan drift (~4° dalam 30 s) — bukan osilasi cepat seperti bug lama 2026-07-18,
  melainkan gaya dorong yang tampak nyaris tak menghasilkan gerak berarti pada jarak
  kecil ini. `T['nav']` habis → `NAV_WALL timeout (dist 0.26m)` → `ABORT`. `cond(TAM)=20`
  (sehat, allocator BUKAN penyebab). **Catatan penting**: mekanisme soft-stop
  `wall_face`/`wall_standoff`/`_wall_clearance()` yang dijelaskan RESOLVED di entry
  2026-07-18 ("target NAV_WALL `wall_face(2.5)-wall_standoff(0.45)=2.05 m`... SOFT-STOP
  `_wall_clearance()`...") **TIDAK ADA lagi di `mission_fsm.py` saat ini** — kode aktif
  memakai `wall_dist=2.30` langsung tanpa clearance push-away terpisah (`grep` tak
  menemukan `wall_face`/`wall_standoff`/`_wall_clearance` di file). STATUS.md M6 saat ini
  masih mendeskripsikan mekanisme lama itu sebagai aktif — **keliru**, sudah dikoreksi di
  STATUS.md. Dugaan penyebab (belum dipastikan, JANGAN ubah gain sebelum mengukur lebih
  lanjut): `approach_kp`/`approach_kd` di `_goto_xy_yaw_first` (`mission_fsm.py:327-355`)
  ditala untuk massa lama (33.6 kg); dengan massa baru 8.3 kg + koefisien hidrodinamika
  diskalakan ~0.247× (`aa2410d`), taper gaya dekat target (`freeze_dist=0.08`,
  `slow_dist=1.5`) mungkin menghasilkan gaya di bawah ambang yang bisa mengatasi
  drag/buoyancy residual pada `dist` sekecil 0.26 m. **Akibat**: siklus misi penuh
  (4 hook) TIDAK BISA diselesaikan end-to-end saat ini — blocker utama sebelum M6 ✅.
- **[RESOLVED, via isolasi state] `APPROACH_HOOK → AUTO_RELEASE → SURFACE → loop DIVE`
  TERBUKTI runtime** (start mid-state `start_wall:=B`, memotong bug NAV_WALL di atas).
  Urutan lengkap terkonfirmasi: hook terdeteksi & servo merespons (`ex` -0.64→~0,
  `size` 0.30→0.65 sempat), `t_approach=25.0 s` (`mission_fsm.py:135`, **terpakai** —
  mengoreksi catatan draft rencana sebelumnya yang keliru menyebut param ini tak
  terpakai; sebenarnya di-wire via `self.T['approach']`, `mission_fsm.py:211-213`) habis
  → **fallback ke AUTO_RELEASE (bukan ABORT), sesuai desain** → "AUTO_RELEASE: posisi
  stabil, publish detach..." → depth naik ke permukaan → skor m5 +40 → `done_hooks`
  bertambah → loop balik ke `DIVE` untuk hook berikutnya. Mekanisme fallback/looping
  terbukti solid.
- **[VERIFY — belum konvergen andal] Servo visual `APPROACH_HOOK` (`hook_logic.hook_servo`)
  terbukti WIRED & responsif** (offset kamera depan memengaruhi `ex/ey/size` sesuai
  ekspektasi), **tapi tidak sekali pun mencapai kriteria `near AND aligned` serentak**
  dalam window `t_approach=25 s` pada run ini — `ex` berosilasi antara ~0 (terpusat) dan
  ~0.9 (nyaris keluar frame) dalam hitungan detik, mengindikasikan deteksi hook
  flickering atau overshoot approach. Sistem **aman** (fallback timeout bekerja, tidak
  pernah ABORT), tapi konvergensi visual servo hook itu sendiri **butuh tuning**
  (`hook_kp_*`/`hook_kd_*`, atau longgarkan `hook_center_tol=0.15`/`hook_size_stop=0.35`,
  atau perpanjang `t_approach`) sebelum bisa diklaim ✅ penuh.
- **[RESOLVED] Payload spawn → sinyal `spawned` → gripper auto-detach startup — urutan
  TERBUKTI benar** (diverifikasi di ≥2 run terpisah): log `Spawn payload QR=… pos=(…)`
  → `Payload QR=… spawned OK` → `Sinyal /hydroships/payload/spawned diterbitkan` →
  `gripper open: auto-detach startup … [pemicu: payload spawn terdeteksi]`, selalu dalam
  urutan itu.
- **[RESOLVED] Integrasi GUI (`gui_bridge`) — round-trip UDP TERBUKTI.** `hydroships_gui.launch.py`
  **tidak** menjalankan `stabilizer` (hanya sim + `thruster_allocator` + `gui_bridge`) —
  jadi tak ada konflik publisher `/hydroships/cmd_vel` seperti dikhawatirkan awalnya.
  UDP JSON `{"name":"arm","value":true}` → `armed:true` di telemetri balik; `{"name":
  "surge","value":75}` (armed) → `/hydroships/cmd_vel` berubah dari nol. Telemetri UDP
  balik ke port 14551 berisi `heading/depth/roll/pitch` real (bukan placeholder). Gain
  (`surge_gain=0.40, sway_gain=0.40, heave_gain=0.30, yaw_gain=0.12`,
  `gui_bridge_logic.py:49-102`) **tetap estimasi** — kalibrasi lapangan masih `[OPEN]`.
- **[RESOLVED] Pemetaan hook A–D ↔ sisi kolam dikonfirmasi cocok, kode vs world SDF.**
  `WALL_HEADING_DEG = {'A':270°,'B':90°,'C':0°,'D':180°}` (`mission_fsm.py:81`) cocok
  presisi dengan pose `hook_a=(0,-2.5)`, `hook_b=(0,2.5)`, `hook_c=(2.5,0)`,
  `hook_d=(-2.5,0)` di `worlds/kki_arena.sdf` (A=-Y, B=+Y, C=+X, D=-X, sesuai catatan lama).
  Dikonfirmasi juga secara langsung: run `qr_letter:=A` menghasilkan `NAV_WALL` menuju
  `(0.00, 2.30)` = sisi +Y.
- **Ringkasan prioritas perbaikan berikutnya (urut dampak):** (1) `GRAB` tidak attach —
  blocking, cari kenapa `_grip`/publish "close" hilang dari `_st_grab` (kemungkinan
  regresi lanjutan dari insiden PR #14 yang disebut "direstore" di entry 2026-07-17 —
  tampaknya restorasi tsb tidak lengkap untuk bagian ini). (2) `NAV_WALL` tak konvergen —
  blocking, ukur closed-loop response `approach_kp/kd` vs fisika baru sebelum ubah gain.
  (3) Tuning konvergensi servo `APPROACH_HOOK`. (4) Uji QR B/C/D & kalibrasi gain GUI
  (non-blocking, [VERIFY]/[OPEN] lama tetap berlaku).

## 2026-08-08 — [RESOLVED] Akar `DIVE timeout` = model apung, bukan controller.
Investigasi P0-1a→e; DIVE `CLOSED`, tag `p0-1-baseline`

Ringkasan penuh + seluruh angka: **[P0-1-BASELINE.md](P0-1-BASELINE.md)**.

Metodologi: diagnosis berlapis dgn gerbang anti-kontaminasi per run, tanpa mengubah satu
pun parameter kendali. Setiap tahap memisahkan satu pertanyaan: P0-1a (rantai kendali) →
P0-1b (aktuator open-loop) → P0-1c.1 (audit trim statis) → P0-1c.2/.3 (koreksi) →
P0-1c.4 (infrastruktur uji) → P0-1d (karakterisasi bersih) → P0-1e (regresi tertutup).

- **`9219735`** — **[RESOLVED] Gripper tidak lagi menyumbang volume apung.** Plugin
  `gz-sim-buoyancy-system` menurunkan volume perpindahan air dari geometri `<collision>`;
  `gripper_base` + kedua jari punya collision box sehingga ikut menghasilkan gaya apung
  yang tak pernah masuk neraca `rov_params.yaml`. Akibatnya net apung **+6.92 N** (bukan
  +0.28 N, 24.7×) dan CoB bergeser **+13.6 mm** ke haluan → momen bow-up 1.04 N·m melawan
  momen pemulih maks 1.69 N·m → trim pasif **31.5°**. Contact fisik gripper tak dibutuhkan
  (grasp = DetachableJoint + proximity visual, jari kosmetik), jadi collision dihapus
  mengikuti idiom thruster. Terukur setelah: net apung +0.28 N, trim +4.44°/+3.58°.
- **`8d6c49c`** — **[RESOLVED] `cob.x` disejajarkan dgn CoG sistem** (0.0 → 0.00237 m).
  `cog` di YAML adalah origin inersial `base_link`, **bukan** CoG sistem — massa gripper di
  haluan menggesernya ke x = +2.37 mm. Sisa lengan parasit itu menutup trim: prediksi 0.0°,
  terukur **−0.02° / −0.01°**.
- **`0941cd4`** — **[RESOLVED] Nama world diambil dari isi SDF, bukan nama file.**
  `create -world <nama>` memakai nama yang dideklarasikan di SDF; `sim.launch.py`
  menurunkannya dari nama file. `pool_empty.sdf` memakai `<world name="pool">` dan
  `kki_arena_test.sdf` memakai `<world name="kki_arena">` → 2 dari 3 world tak pernah
  spawn ROV, **gagal tanpa error yang terlihat**. Kode-keluar node `create` kini
  dilaporkan lewat `OnProcessExit`.

**Hasil regresi DIVE** (4 run: 3 random spawn + 1 deterministik, `kki_arena`, stack penuh):
lolos **4/4**, ambang 0.24 m tercapai **1.65–1.76 s** dari anggaran 20 s, pitch maks
**0.30°**, roll maks **0.19°**, fidelity allocator **99.4%**, thrust puncak **2.44 N** dari
batas 50 N, tanpa saturasi & tanpa kontak. Pembanding baseline gagal (2026-08-06 dst.):
timeout 20 s, kedalaman mentok ~0.215 m, pitch divergen −33°, thrust 16–19 N.

**[DEFERRED] TAM tidak berubah dan TIDAK terbukti benar** — P0-1 hanya menunjukkan kopling
Fz→My bukan blocker pada titik operasi DIVE yang diuji. Celah `INCONCLUSIVE` yang sengaja
tidak diekstrapolasi: B/B′ pada −10/−14 N, kontribusi individual T2/T6 (jendela bersih
terpotong kontak lantai), dan skala thrust absolut η.

**[OPEN] Status lama GRAB & NAV_WALL tidak lagi valid sebagai deskripsi.** Dalam jendela
60 s pasca-DIVE, FSM berlanjut `APPROACH_QR → GRAB → NAV_WALL → HANG → SURFACE →
WAIT_TRIGGER` tanpa ABORT di keempat run — tetapi **"berjalan tanpa ABORT" bukan acceptance
evidence**. Ketiganya menunggu karakterisasi P0-2/P0-3/P0-4; jangan dicatat sebagai selesai.

Skrip eksperimen disimpan di `tools/p0-experiments/` agar P0-1d/P0-1e dapat direproduksi.

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

---

## 2026-08-12 — M5-D lanjutan: gripper diturunkan + latch hak attach

Melanjutkan `e8ee4d2` (state `DESCEND`). Run verifikasi commit itu menunjukkan
NAV_WALL sembuh tapi `gripper closed: tutup TAPI payload di luar jangkauan -> tak
attach`. Sebabnya **bukan** `DECODE GAGAL` yang kebetulan: di `grasp_depth`, kamera
bawah (`cam_bottom_dz=0.18`) turun sampai sejajar bidang QR (`qr_floor_z=-0.894`),
jadi **tak mungkin** ada deteksi segar saat `"close"` dikirim — sementara
`is_safe()` mensyaratkannya. Gerbang lama tak bisa lolos di desain apa pun.

- **[RESOLVED] URDF `gripper_base_joint` z `0` → `-0.13`.** Gripper dulu setinggi
  PUSAT hull, 0.14 m di atas dasar hull: walau ROV turun sampai hull hampir menyentuh
  lantai, gripper tetap ~0.19 m di atas payload — `DetachableJoint` tetap mengelas
  lintas celah. Sekarang di `grab_depth=0.70`: dasar gripper `-0.860` (**0.034 m** di
  atas bidang QR), dasar collision hull `-0.771` (0.12 m di atas lantai). Aman thd
  neraca apung: `gripper_base` memang tanpa `<collision>`; yang bergeser hanya massa
  0.08 kg turun 0.13 m (CoG sedikit turun = lebih stabil).
- **[RESOLVED] Latch "armed" di `GripperLogic`** (`arm_timeout=8.0 s`): syarat visual
  yang terpenuhi di `APPROACH_QR` memberi hak attach yang bertahan melewati fase buta
  `DESCEND`. Di-reset pada `open`/`force_detach`/`startup_detach` supaya payload
  BERIKUTNYA tak mewarisi arm payload sebelumnya (kelas bug yang sama dgn EMA basi
  `_qr_ex_filt` di `mission_fsm._to(APPROACH_QR)`).
- **[RESOLVED] `alt_gap` pindah dari `_on_offset` ke `_on_depth`.** Menumpang
  `qr_offset` berarti gerbang fisik mati persis saat dibutuhkan (topiknya berhenti
  terbit begitu QR hilang). Sekarang dari `/hydroships/depth` yang terbit terus, dan
  diukur dari **dasar gripper** (`gripper_bottom_dz=0.16`), bukan base_link;
  `max_alt_gap` 0.15 → **0.08** menyesuaikan celah rancangan 0.034 m.
  Kontrak akhir `is_safe()`: **latch/segar (arah) DAN altitude (fisik)** — latch tanpa
  gerbang fisik mengizinkan attach dari mana saja; gerbang fisik tanpa latch tak pernah
  attach sama sekali.
- Param `grasp_standoff` → **`grab_depth` (0.70)**, dgn turunan angkanya ditulis di
  komentar `mission_fsm.py`. `ey_target` selama `DESCEND` kini dihitung dari kedalaman
  **aktual**, bukan target (kalau tidak, nilainya ter-clamp ke `ey_max` dan servo
  mendorong ke arah salah selama detik pertama turun, saat QR masih terlihat).
- Test: `test/test_grab_geometry.py` (baru) membaca nilai sesungguhnya dari xacro +
  `rov_params.yaml` + kedua node, lalu mengunci tiga syarat sekaligus (gripper
  menjangkau QR, hull tak menabrak lantai, gerbang attach konsisten dgn `grab_depth`)
  — ubah satu file tanpa yang lain, test gagal. Plus 3 test latch di `test_gripper.py`.
  **101 test hijau.**
- **[RESOLVED, 3/3 run]** Diverifikasi runtime pada sesi diagnosis di bawah:
  `attach (payload dalam jangkauan)` muncul di ketiga run. **[VERIFY] tersisa:**
  payload benar-benar terangkat (pose payload runtime belum diukur).

---

## 2026-08-12 (lanjutan) — Diagnosis gerbang attach (DIAGNOSIS ONLY)

Instrumentasi `GATEDBG` (commit terpisah, non-fungsional: `GripperLogic.explain()`
+ satu baris log di `gripper_controller._on_cmd` dan `qr_detector._publish_offset`).
Tanpa perubahan threshold, urutan FSM, kriteria transisi, atau topic. 3 run
(`headless:=true`, payload di (0.40,0.04) / (−0.30,0.55) / (0.75,−0.45), spawn ROV
acak). Log mentah: `~/m5d-diagnosis-logs/run{1,2,3}.log`.

- **[RESOLVED] Root cause penolakan gerbang = dropout deteksi QR (kandidat c).**
  Offset terakhir dari kamera bawah tiba **0.90 / 2.81 / 2.03 detik** sebelum tick
  `"close"`; pada run2 `age=1.53 s` sudah melewati `offset_timeout=1.5` sehingga
  `fresh=False` dan attach lolos **hanya lewat latch** (`armed=True`). Ini bukan
  kebetulan timing melainkan konsekuensi geometri: di `grab_depth` kamera bawah
  sejajar bidang QR. Gerbang lama (freshness wajib) memang tak bisa lolos.
- **[MOOT] Kandidat (a) `alt_gap`** bukan penyebab: 0.073/0.074/0.075 ≤ 0.08 (3/3).
  Tapi marginnya 5–7 mm → gap baru **R-10** di P1-OWNER-DECISIONS-AND-ROADMAP.
- **[MOOT] Kandidat (b) `ey_target`** bukan penyebab: `|y−ey_target|` =
  0.235/0.057/0.164 vs 0.30. Catatan: `ey_target` ter-clamp di `ey_target_max`
  (−0.800) pada run1 & run3.
- **[OPEN] Docstring `_st_grab` BASI (komentar, bukan logika).** Klaimnya "payload
  sudah ter-DetachableJoint ke ROV sejak spawn ... attach praktis no-op" dibantah
  log di 3/3 run: `auto-detach startup [pemicu: payload spawn terdeteksi]` terbit
  **14.6 / 19.3 / 15.3 detik SEBELUM** `close`. Sengaja TIDAK diedit sesi ini —
  redaksi diserahkan ke penulis repo.
- **[OPEN] `_st_grab` tanpa ack** dari `gripper_controller` (skor `m2=15` &
  transisi `NAV_WALL` tanpa konfirmasi attach) → item roadmap **R-9**, tidak
  diimplementasikan sesi ini.

## 2026-08-13 — Live test pertama GUI-ROV ↔ gui_bridge (M7)

Test manual end-to-end pertama dengan dashboard GUI-ROV asli (`server/server.js`
lokal, `RPI_ADDR=127.0.0.1`) terhadap `hydroships_gui.launch.py` (headless).
Log: `ros2_ws.log` (sisi ROS) + `GUI-ROV.log` (sisi server, 2413 baris).

- **[RESOLVED] Round-trip arm/disarm, yaw, dan gripper open/close terbukti**
  dengan dashboard asli (sebelumnya hanya klien UDP sintetis 2026-08-06).
  Gripper: perintah dari GUI sampai ke `gripper_controller` dan gate jarak
  bekerja benar (menolak attach saat payload di luar jangkauan, sesuai desain).
- **[VERIFY tersisa]** Tombol **light** tidak sempat ditekan saat run ini (nol
  command di log). Efek gerak sim dari surge/sway tak diverifikasi (perlu
  echo `/hydroships/odom` di run berikutnya).
- **[OPEN] Roll/pitch melonjak ±25-31° selama yaw ditahan lama**, redam pelan
  setelah yaw berhenti. Kontrol saat run ini adalah keyboard (pulsa on/off
  berulang), bukan stick kontinu — belum jelas apakah lonjakan ini karakter
  fisik wajar dari pulsa berulang atau indikasi allocator/gain perlu ditinjau.
  Perlu run pembanding dengan joystick asli.
- **[OPEN] Telemetry rate terukur ~3 Hz** di beberapa jendela 1 detik, di
  bawah `telem_hz=10` default (`gui_bridge.py:57`). Perlu profiling.
- **[MOOT, bukan bug]** Command `pool_depth`/`controller` yang dikirim dashboard
  diam-diam diabaikan `gui_bridge_logic.on_command()` (nama tak dikenal →
  `{}`) — sesuai desain adapter saat ini, tapi berarti tombol/slider terkait
  di dashboard tak berefek ke sim.
- **[RESOLVED] Docstring `gui_bridge_logic.py` dibetulkan**: klaim telemetri
  punya field `ts` tidak sesuai `build_telemetry()` (tak pernah menyertakan
  `ts`) — diperbaiki jadi deskriptif.

## 2026-08-13 (lanjutan) — R-9 ditutup + battery 4-hook ×3 seed (Fase 1)

- **[RESOLVED] R-9**: topic ack `/hydroships/gripper/status` ternyata **sudah
  ada & sudah di-subscribe** di `mission_fsm.py`, cuma disimpan tanpa dipakai
  ("observability, tak memicu transisi"). Bukan pelanggaran batasan "tanpa
  topic baru" dari sesi diagnosis 2026-08-12 — diff jadi kecil: `_to()`
  me-reset `gripper_status=None` saat masuk `GRAB`; `_st_grab` kini menunggu
  `'attached'` (skor m2=15, lanjut `NAV_WALL`) atau `'rejected'` (publish
  ulang "close", gerbang bisa berubah tick berikutnya) sampai `T['grab']`
  habis → `ABORT`. Docstring lama yang bilang "tak ada jalur ack" dihapus.
  Ditambah baris tabel topic `/hydroships/gripper/status` di
  `docs/ARCHITECTURE.md` (sebelumnya terimplementasi tapi tak terdaftar).
  110/110 test pure-logic tetap hijau (tak ada `test_mission_fsm.py`,
  sesuai pola repo saat ini — tak ditambah infra rclpy baru utk perubahan ini).
- **Battery verifikasi runtime, 3 seed baru** (`spawn_seed:=3001/3002/3003`,
  `headless:=true`, `tools/p0-experiments/run_mission_cycle.sh`, log di
  `/tmp/p1-fase1-r9/R9-*.log`):
  - **3001, 3003**: 4 hook penuh `DIVE→...→AUTO_RELEASE→DONE`,
    `SKOR m1=15 m2=15 m3=15 m4=15 m5=40 TOTAL=100/100`.
  - **3002**: GRAB hook pertama ditolak terus-menerus (`GATEDBG close
    result=False`, `x=+0.799` tak pernah masuk `max_offset=0.30` selama
    ~16 s, `armed=False`) → retry loop R-9 bekerja sesuai desain, lalu
    **`GRAB timeout (tak ada ack attached)` → `ABORT` jujur**. Sebelum
    perbaikan R-9, ini akan tetap mencatat skor m2=15 palsu (persis pola
    lama di STATUS M5/M6).
  - **[OPEN, dicatat R-11]** Root cause penolakan 3002 bukan bug R-9 — gerbang
    justru bekerja benar. Kemungkinan presisi centering `APPROACH_QR`/`DESCEND`
    untuk posisi spawn payload tertentu; belum didiagnosis, butuh run
    berulang/data lebih banyak sebelum diubah.
  - Fase 1 roadmap exit criteria ("4-hook ×3 seed berturut") jadi **2/3** —
    bukan regresi, melainkan R-9 mengungkap kegagalan nyata yang sebelumnya
    tersembunyi.
