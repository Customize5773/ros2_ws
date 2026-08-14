# VERIFICATION-CHECKLIST — Uji Runtime HYDROships

Daftar item yang **butuh uji sim (Gazebo) / hardware nyata**. Mesin dev sejak 2026-08-06
**sudah punya** ROS 2 Humble + Gazebo Fortress + EGL/mesa — sebagian besar item di bawah
sudah diuji sesi itu (ditandai `[x]` + nilai terukur). Dua regresi baru **memblokir**
penyelesaian siklus misi penuh: GRAB tak attach payload, NAV_WALL tak konvergen — lihat
[`CHANGELOG.md`](CHANGELOG.md) entry 2026-08-06 untuk detail & repro. Konteks & status:
[`STATUS.md`](STATUS.md).

Prasyarat tiap sesi: `colcon build` lokal, `source install/setup.bash`, dan pastikan tidak
ada proses `mission_fsm`/`parameter_bridge` lama yang tersisa (`ps aux | grep -E 'mission_fsm|parameter_bridge'`)
agar tidak saling adu perintah.

Format: `[ ]` belum diuji · `[x]` terbukti · `[~]` parsial/tak andal — deskripsi — cara
verifikasi — file/commit terkait.

## Prioritas 1 — Persepsi dasar (banyak state FSM bergantung)

- [x] **`camera_info` mengalir & masuk akal** — TERBUKTI: `qr_detector` menerima
  `camera_bottom/front/camera_info` (`fx=fy=381.4 cx=320 cy=240 640x480`), bridge nyambung. — `bridge.yaml`, `qr_detector.py`, commit `b8b0623`/`a143306`.
- [x] **Render kamera headless jalan di GPU/EGL** — TERBUKTI: `camera_bottom/front/image_raw`
  mengalir (`rgb8` 640x480, `step=1920` = tanpa padding), frame ter-render tajam. — `gz-sim-sensors-system` (ogre2).
- [x] **QR terbaca otomatis di render sim** (BUKAN inject manual) — TERBUKTI runtime (Fortress+GPU):
  misi penuh `qr_detector: QR terbaca "A" -> sisi A [camera_bottom]` → `mission_fsm: QR -> wall A (+15)`
  → lanjut `GRAB`. Root cause bug lama = FRAMING (kamera ~9 cm di atas QR → finder bawah ter-crop +
  gripper menutupi atas frame), diperbaiki `scan_depth 0.62→0.46` (kamera ~25 cm). BUKAN kontras/
  ukuran/orientasi. **Update 2026-08-06**: `scan_depth` kini `0.30` (revisi lanjutan, alasan
  offset gripper `cam_gripper_dx=0.16 m`) — RE-VERIFIED tetap decode 'A' tanpa regresi. — `mission_fsm.py`, `qr_detector.py`, `qr_logic.robust_decode`, lihat [CHANGELOG](CHANGELOG.md).
- [ ] **QR sisi B/C/D terbaca** (VERIFY, belum diuji ulang 2026-08-06) — aset `qr_B/C/D.png` sudah ada; ganti `<albedo_map>/<emissive_map>`
  di `kki_arena.sdf` (payload visual `qr`) ke huruf lain lalu ulangi misi; `parse_wall` sudah teruji
  A/B/C/D di `test_qr_logic.py`. — `generate_qr.py`, `kki_arena.sdf`.
- [x] **`qr_offset` valid** — TERBUKTI fungsional 2026-08-06: `scan_depth=0.30`, misi
  `DIVE→APPROACH_QR→GRAB` sukses (centering servo menuntaskan APPROACH_QR). Nilai numerik
  `ex/ey` presisi belum dicatat sesi ini (belum topic-echo langsung) — cara verifikasi
  numerik: `ros2 topic echo /hydroships/qr_offset` saat ROV di atas payload. — `qr_detector.py`.

## Prioritas 2 — Manipulator (butuh persepsi + arena)

- [x] **Startup auto-detach bekerja** — TERBUKTI 2026-08-06 (≥2 run terpisah): log
  `Payload QR=… spawned OK` → `Sinyal /hydroships/payload/spawned diterbitkan` →
  `gripper open: auto-detach startup … [pemicu: payload spawn terdeteksi]`, urutan selalu
  benar (dipicu topik, bukan timer). — `gripper_controller.py`, `gripper_logic.startup_detach`, commit `df8f71e`.
- [ ] **[REGRESI BLOCKING] Grasp DetachableJoint mengangkat payload** — **GAGAL**:
  `mission_fsm._st_grab` (`mission_fsm.py:606-618`) tidak pernah publish "close" ke
  `/hydroships/gripper/command`; `pub_grip` dideklarasikan tapi tak sekali pun dipanggil
  `.publish()` di seluruh `mission_fsm.py` (dikonfirmasi `grep`). Akibat: `/hydroships/gripper/attach`
  tidak pernah terbit selama misi autonomous → payload tak pernah ter-attach, tak terbawa
  NAV_WALL. `AUTO_RELEASE` publish `/detach` TERBUKTI jalan (state-machine benar) tapi jadi
  no-op karena tak ada yang di-detach. **Perbaikan diperlukan**: tambahkan publish "close"
  (mis. via `self.pub_grip`) di `_st_grab` sebelum/saat transisi ke `NAV_WALL`, gated pada
  `qr_offset` aman sesuai desain asli commit `fd06b0a`. — `mission_fsm.py:221,606-618`, `gripper_controller.py:56-129`.
- [ ] **Tuning ambang jarak-aman & massa payload** — `max_offset=0.30`, `min_size=0.12`,
  massa payload 0.3 kg; setel agar attach terpicu tepat & payload tak melayang/menembus air.
  (Blocked oleh item di atas — tak ada attach untuk ditala.) — `gripper_controller.py` params.

## Prioritas 3 — Autonomy & servo hook

- [ ] **[REGRESI BLOCKING] NAV_WALL konvergen ke standoff** — **GAGAL** 2026-08-06:
  repro `spawn_seed:=42 qr_letter:=A start_state:=DIVE`, target `(0.00,2.30)`
  (`wall_dist=2.30`), ROV berhenti bergerak di `dist≈0.26 m` (di atas `nav_tol=0.20`),
  timeout `T['nav']` → ABORT. `cond(TAM)=20` sehat (bukan allocator). Mekanisme soft-stop
  `wall_face`/`wall_standoff`/`_wall_clearance()` dari CHANGELOG 2026-07-18 **tidak ada
  lagi di kode**. Dugaan: `approach_kp/kd` (`_goto_xy_yaw_first`) belum ditala ulang utk
  massa baru 8.3 kg + hydro coefficients diskalakan (`aa2410d`/`00d4aaa`) — **ukur dulu,
  jangan ubah gain langsung**. — `mission_fsm.py:126,327-355,620-641`.
- [~] **APPROACH_HOOK servo konvergen** — **PARSIAL**: TERBUKTI wired & responsif
  (`ex` -0.64→~0, `size` 0.30→0.65 sempat tercapai) tapi **tidak pernah** mencapai
  `near AND aligned` serentak dalam `t_approach=25s` pada run ini — `ex` berosilasi
  0↔0.9 (indikasi deteksi flickering/overshoot). Verifikasi ulang:
  `start_state:=APPROACH_HOOK start_wall:=B`; `ros2 topic echo /hydroships/hook_offset`. — `hook_detector.py`, `hook_logic.hook_servo`, `mission_fsm._st_approach_hook`.
- [x] **Fallback open-loop APPROACH_HOOK** — TERBUKTI 2026-08-06: `t_approach=25s`
  habis → log "APPROACH_HOOK timeout -> lanjut AUTO_RELEASE" → state **maju** ke
  `AUTO_RELEASE` (tidak abort), detach terpublish, skor +40, loop balik `DIVE`.
  Catatan: `t_approach` **memang terpakai** (`self.T['approach']`,
  `mission_fsm.py:211-213`) — koreksi atas catatan sebelumnya yang keliru menyebut param
  ini dead. — `mission_fsm._hook_fresh`, param `hook_max_age`, `t_approach`.
- [x] **Deteksi hook di kamera sim** — TERBUKTI: `hook_detector` mendeteksi hook arena
  (`hook terdeteksi: center=(…) area=…` di log), merespons posisi ROV secara real-time. — `hook_detector.py` params.
- [ ] **Tuning gain PD servo hook** — `hook_kp_*`/`hook_kd_*` (estimasi) perlu ditala
  lebih lanjut — lihat catatan osilasi di item "APPROACH_HOOK servo konvergen" di atas. — `mission_fsm.py` params.
- [ ] **Tuning timeout/gaya FSM** — timeout tiap state, `surge_force`, sudut & toleransi
  belum di-tune untuk gerak nyata arena; NAV_WALL kini prioritas tertinggi (lihat blocker). — `mission_fsm.py` params.

## Prioritas 4 — Integrasi GUI live (M7)

- [x] **End-to-end GUI ↔ sim (klien UDP sintetis)** — TERBUKTI 2026-08-06:
  `hydroships_gui.launch.py spawn_seed:=5` (tanpa `stabilizer`, tak ada konflik
  `cmd_vel`); UDP JSON `{"name":"arm","value":true}` → telemetri balik `armed:true`;
  `{"name":"surge","value":75}` (armed) → `/hydroships/cmd_vel` berubah dari nol.
  Telemetri UDP :14551 berisi `heading/depth/roll/pitch` real. **Belum** diuji dengan
  dashboard GUI/joystick fisik asli — hanya klien UDP JSON buatan sendiri. — `gui_bridge.py`, `hydroships_gui.launch.py`, commit `acac770`/`b9c97f5`.
- [ ] **Kalibrasi gain & tanda** — gain persen→N saat ini `surge_gain=0.40, sway_gain=0.40,
  heave_gain=0.30, yaw_gain=0.12` (`gui_bridge_logic.py:49`) — masih ESTIMASI, offset
  heading kompas (0° vs +x REP-103) & tanda sumbu belum dikalibrasi ke hardware/GUI asli. — `gui_bridge_logic.py`, `docs/GUI-INTEGRATION.md`.

## Prioritas 5 — Validasi arena & hardware

- [x] **Pemetaan label hook A/B/C/D → sisi kolam** — TERBUKTI cocok 2026-08-06: kode
  `WALL_HEADING_DEG={'A':270°,'B':90°,'C':0°,'D':180°}` vs pose SDF
  `hook_a=(0,-2.5)/hook_b=(0,2.5)/hook_c=(2.5,0)/hook_d=(-2.5,0)` → A=-Y/B=+Y/C=+X/D=-X
  terkonfirmasi (bukan lagi "sementara"). Dikonfirmasi juga via run langsung (`qr_letter:=A`
  → NAV_WALL target sisi +Y). — `worlds/kki_arena.sdf`, `mission_fsm.py:81`.
- [x] **Geometri hook Ø25 mm** — dikonfirmasi via SDF (`radius 0.0125 m` konsisten di semua
  4 hook). Validasi fisik "cukup untuk uji sangkut nyata" tetap **OPEN** (butuh hook & ROV
  fisik). — `worlds/kki_arena.sdf`.
- [ ] **Arah bow (haluan)** — cek `bow_yaw` di GUI (footprint ~persegi, tak bisa ditebak bbox). — `hydroships.urdf.xacro`.
- [ ] **Kalibrasi kamera fisik ROV** (OPEN, gap hardware) — intrinsics sim ≠ kalibrasi
  hardware; jangan pakai K sim untuk estimasi jarak riil sampai kalibrasi kamera fisik tersedia.
  Tooling & jalur muat sudah tersedia (`calib_file_bottom`/`calib_file_front` param,
  `qr_logic.load_calibration_yaml` — baca `.yaml` ROS ATAU `.npz`, prosedur
  `camera_calibration` di `docs/HARDWARE.md` §3). Kalibrasi mentah kamera DWE ExploreHD
  SUDAH ADA (`dwe.npz` di root repo) tapi RMS reprojection 4.97 px terlalu kasar untuk
  dipercaya, dan tidak jelas untuk kamera bottom atau front — lihat `docs/HARDWARE.md` §3
  langkah 5 sebelum dipakai. — `qr_detector.py`.
- [ ] **Data fisik ROV asli** — massa/inertia/koefisien hidrodinamika masih `[estimate]`
  (termasuk setelah revisi massa 33.6→8.3 kg di `aa2410d`/`00d4aaa`, lihat CHANGELOG
  2026-08-06); ukur ROV nyata, isi `rov_params.yaml`, ubah tag `[estimate]`→`[measured]`. — `src/hydroships_description/config/rov_params.yaml`, `scripts/estimate_mass_inertia.py`, commit `045d7c4`.
