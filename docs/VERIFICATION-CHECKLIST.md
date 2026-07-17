# VERIFICATION-CHECKLIST — Uji Runtime HYDROships

Daftar item yang **butuh uji sim (Gazebo) / hardware nyata** — belum bisa dijalankan di
mesin dev (tanpa ROS2/Gazebo). Diurutkan **prioritas eksekusi** begitu ada akses mesin
ber-GPU/EGL + Gazebo Fortress. Konteks & status: [`STATUS.md`](STATUS.md) ·
[`CHANGELOG.md`](CHANGELOG.md).

Prasyarat tiap sesi: `colcon build` lokal, `source install/setup.bash`, dan pastikan tidak
ada proses `mission_fsm`/`parameter_bridge` lama yang tersisa (`ps aux | grep -E 'mission_fsm|parameter_bridge'`)
agar tidak saling adu perintah.

Format: `[ ]` deskripsi — cara verifikasi — file/commit terkait.

## Prioritas 1 — Persepsi dasar (banyak state FSM bergantung)

- [x] **`camera_info` mengalir & masuk akal** — TERBUKTI: `qr_detector` menerima
  `camera_bottom/front/camera_info` (`fx=fy=381.4 cx=320 cy=240 640x480`), bridge nyambung. — `bridge.yaml`, `qr_detector.py`, commit `b8b0623`/`a143306`.
- [x] **Render kamera headless jalan di GPU/EGL** — TERBUKTI: `camera_bottom/front/image_raw`
  mengalir (`rgb8` 640x480, `step=1920` = tanpa padding), frame ter-render tajam. — `gz-sim-sensors-system` (ogre2).
- [x] **QR terbaca otomatis di render sim** (BUKAN inject manual) — TERBUKTI runtime (Fortress+GPU):
  misi penuh `qr_detector: QR terbaca "A" -> sisi A [camera_bottom]` → `mission_fsm: QR -> wall A (+15)`
  → lanjut `GRAB`. Root cause bug lama = FRAMING (kamera ~9 cm di atas QR → finder bawah ter-crop +
  gripper menutupi atas frame), diperbaiki `scan_depth 0.62→0.46` (kamera ~25 cm). BUKAN kontras/
  ukuran/orientasi. — `mission_fsm.py`, `qr_detector.py`, `qr_logic.robust_decode`, lihat [CHANGELOG](CHANGELOG.md).
- [ ] **QR sisi B/C/D terbaca** (VERIFY) — aset `qr_B/C/D.png` sudah ada; ganti `<albedo_map>/<emissive_map>`
  di `kki_arena.sdf` (payload visual `qr`) ke huruf lain lalu ulangi misi; `parse_wall` sudah teruji
  A/B/C/D di `test_qr_logic.py`. — `generate_qr.py`, `kki_arena.sdf`.
- [ ] **`qr_offset` valid** — `ros2 topic echo /hydroships/qr_offset` saat ROV di atas
  payload; offset piksel ternormalisasi + ukuran-tampak masuk akal (dipakai gate attach gripper). — `qr_detector.py`.

## Prioritas 2 — Manipulator (butuh persepsi + arena)

- [ ] **Startup auto-detach bekerja** — saat sim mulai, payload TIDAK ikut nempel ROV
  sebelum GRAB (cek payload diam di dasar); node `gripper_controller` mengirim satu detach
  di `startup_detach_delay`=1.5 s. — `gripper_controller.py`, `gripper_logic.startup_detach`, commit `df8f71e`.
- [ ] **Grasp DetachableJoint mengangkat payload** — di state `GRAB` kirim "close":
  cek payload ter-attach & terangkat; saat `NAV_WALL` terbawa; saat `AUTO_RELEASE` ("open")
  terlepas. `ros2 topic echo /hydroships/gripper/attach` & `/detach`. — `gripper_controller.py`, `mission_fsm.py`, commit `fd06b0a`.
- [ ] **Tuning ambang jarak-aman & massa payload** — `max_offset=0.30`, `min_size=0.12`,
  massa payload 0.3 kg; setel agar attach terpicu tepat & payload tak melayang/menembus air. — `gripper_controller.py` params.

## Prioritas 3 — Autonomy & servo hook

- [ ] **APPROACH_HOOK servo konvergen** — `ros2 topic echo /hydroships/hook_offset`;
  amati ROV servo (sway+surge+koreksi-depth) menuju hook di render kamera depan; cek fallback
  timed tak dipakai kecuali deteksi hilang. — `hook_detector.py`, `hook_logic.hook_servo`, `mission_fsm._st_approach_hook`, commit `499ab31`/`acac770`.
- [ ] **Deteksi hook di kamera sim** — verifikasi `hook_detector` mendeteksi hook arena;
  tuning ambang (`min_area`, CLAHE) untuk glare/kekeruhan/kontras render. — `hook_detector.py` params.
- [ ] **Tuning gain PD servo hook** — `hook_kp_*`/`hook_kd_*` (estimasi) disetel dari perilaku sim. — `mission_fsm.py` params.
- [ ] **Tuning timeout/gaya FSM** — timeout tiap state, `surge_force`, sudut & toleransi
  belum di-tune untuk gerak nyata arena. — `mission_fsm.py` params.

## Prioritas 4 — Integrasi GUI live (M7)

- [ ] **End-to-end GUI ↔ sim** — joystick GUI (UDP :14550) → ROV sim bergerak; telemetri
  (heading/depth/roll/pitch) muncul di dashboard (UDP :14551). — `gui_bridge.py`, `hydroships_gui.launch.py`, commit `acac770`/`b9c97f5`.
- [ ] **Kalibrasi gain & tanda** — gain persen→N (`surge/sway/heave/yaw_gain`), offset
  heading kompas (0° vs +x REP-103), tanda sumbu, port UDP. Semua estimasi. — `gui_bridge_logic.py`, `docs/GUI-INTEGRATION.md`.

## Prioritas 5 — Validasi arena & hardware

- [ ] **Pemetaan label hook A/B/C/D → sisi kolam** & pengacakan posisi (A=−Y/B=+Y/C=+X/D=−X sementara). — `worlds/kki_arena.sdf`.
- [ ] **Geometri hook Ø25 mm** (silinder J, z terendah −0.45) cukup untuk uji sangkut nyata. — `worlds/kki_arena.sdf`.
- [ ] **Arah bow (haluan)** — cek `bow_yaw` di GUI (footprint ~persegi, tak bisa ditebak bbox). — `hydroships.urdf.xacro`.
- [ ] **Kalibrasi kamera fisik ROV** (OPEN, gap hardware) — intrinsics sim ≠ kalibrasi
  hardware; jangan pakai K sim untuk estimasi jarak riil sampai kalibrasi kamera fisik tersedia. — `qr_detector.py`.
- [ ] **Data fisik ROV asli** — massa/inertia/koefisien hidrodinamika masih `[estimate]`;
  ukur ROV nyata, isi `rov_params.yaml`, ubah tag `[estimate]`→`[measured]`. — `config/rov_params.yaml`, `scripts/estimate_mass_inertia.py`, commit `045d7c4`.
