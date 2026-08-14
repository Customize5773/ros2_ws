# STATUS — Progres Milestone HYDROships (KKI 2026)

Ringkasan **status terkini** tiap milestone. Riwayat kronologis lengkap (termasuk
keputusan yang sudah dibatalkan/diganti) ada di [CHANGELOG.md](CHANGELOG.md).

> ## ✅ DIVE `CLOSED` — model fisik terkoreksi (baseline `p0-1-baseline`, 2026-08-08)
>
> Investigasi P0-1a→e menemukan bahwa `DIVE timeout` **bukan** masalah controller,
> allocator, maupun plugin thruster, melainkan **model apung**: `<collision>` pada ketiga
> link gripper ikut dihitung sebagai volume perpindahan air → net apung **+6.92 N**
> (bukan +0.28 N) dan CoB bergeser **+13.6 mm** ke haluan → ROV trim **~31° bow-up** →
> dorongan "vertikal" terproyeksi jadi gerak horizontal. Setelah dikoreksi (`9219735`,
> `8d6c49c`) trim pasif **−0.01°** dan **DIVE lolos 4/4 run** dalam 1.65–1.76 s dari
> anggaran 20 s, **tanpa mengubah satu pun parameter kendali**. Bukti lengkap & angka:
> [P0-1-BASELINE.md](P0-1-BASELINE.md).
>
> **Yang BELUM terbukti — jangan dibaca lebih jauh dari ini:**
> (1) **TAM tetap `DEFERRED`, bukan `VERIFIED`** — kopling Fz→My hanya terbukti bukan
> blocker pada titik operasi DIVE yang diuji. (2) Status lama **GRAB tidak attach** dan
> **NAV_WALL tak konvergen** (entry 2026-08-06) **tidak lagi valid sebagai deskripsi**:
> keduanya diamati berjalan tanpa ABORT dalam jendela 60 s pasca-DIVE, tetapi *"berjalan
> tanpa ABORT" bukan acceptance evidence*. Ketiga state (APPROACH_QR/GRAB/NAV_WALL)
> berstatus **`OPEN`, menunggu karakterisasi P0-2/P0-3/P0-4**.
>
> Mesin dev sudah punya ROS 2 Humble + Gazebo Fortress 6.18 + EGL/mesa (blocker
> environment lama tidak berlaku). Item yang masih bergantung verifikasi lanjutan
> ditandai **🧪**; daftar uji berprioritas:
> [VERIFICATION-CHECKLIST.md](VERIFICATION-CHECKLIST.md).

> ## ⚠️ Angka Gate 4 APPROACH_QR: baca sebagai DUA populasi (2026-08-12)
>
> `entered_band_with_dwell` — dasar verdict Gate 4 — adalah **OR** antara kriteria kamera
> dan kriteria odometri murni terhadap `/hydroships/payload_pose` (ground truth spawner).
> Ia **tidak bisa** menjawab "apakah kamera berkontribusi". Setelah pelaporan dipisah
> (`e071667`): pada battery terbesar (n=28) angka combined **39%** tapi decoded-QR hanya
> **11%**, dan **12/28 run tidak pernah men-decode QR sama sekali**. Kinerja persepsi
> selama ini dilebihkan ~3–4×. Verdict Gate 4 tetap FAIL, tapi diagnosisnya berubah:
> sebagian besar run tidak punya input vision untuk dipresisikan. Detail:
> [P1-0-FASE0-VISION-ATTRIBUTION.md](P1-0-FASE0-VISION-ATTRIBUTION.md).
>
> Audit lintas-repo, transfer map, dan roadmap:
> [P1-OWNER-DECISIONS-AND-ROADMAP.md](P1-OWNER-DECISIONS-AND-ROADMAP.md) — termasuk
> keputusan pemilik proyek **"ArduSub tetap yang mixing"**, yang **membatalkan** Decision A
> di `P1-1-ARCHITECTURE-DECISION.md` §6.

> ## ✅ qr_detector `CLOSED` — crash "double free or corruption" (2026-08-13)
>
> Eksperimen 3-seed (spawn_seed 1001/1002/1003, mission FSM DIVE→WAIT_TRIGGER) menemukan
> `qr_detector` mati **2/3 run** dengan `double free or corruption (out)` (SIGABRT native,
> bukan exception Python), non-deterministik (t≈69–107 s). **Root cause**: environment
> `~/.local` (pip user-install, dipakai interpreter `/usr/bin/python3` yang sebenarnya
> menjalankan node — bukan `.venv` proyek) resolve ke **`opencv-python 5.0.0.93`**
> (rilis pip terbaru & kurang teruji), bukan `python3-opencv` 4.5.4 (apt) yang
> dideklarasikan di `package.xml`. `cv2.QRCodeDetector` dipanggil ~7×/frame (kandidat
> mentah/clahe/threshold/upscale) selama ribuan frame — heap corruption baru memicu
> crash belakangan, cocok dgn simptom offset absurd (`ex=-4.436 ey=-12.641 size=26.135`)
> yang sempat teramati sesaat sebelum satu crash.
>
> **Percobaan downgrade ke apt 4.5.4 GAGAL** — build itu tidak ter-link **QUIRC**,
> sama sekali tak bisa decode isi QR (`Library QUIRC is not linked`), mematikan fitur
> inti misi. **Fix final**: pin `opencv-python==4.10.0.84` (matang, QUIRC aktif) di
> `requirements.txt`, install ulang lewat `/usr/bin/python3 -m pip install --user`
> (interpreter yang benar dipakai runtime node, bukan `.venv`). Hardening tambahan
> (defense-in-depth, bukan akar masalah): `image_util.py` copy eksplisit
> (`np.array(copy=True)`, bukan `ascontiguousarray` yg no-op saat sudah contiguous)
> menutup celah zero-copy view atas `msg.data`; `qr_detector.py::_publish_offset`
> menolak `size` offset di luar rentang wajar (0, 5.0] sebelum dipublikasikan.
>
> **Diverifikasi**: 0 crash dalam 4 run berturut (~10 menit qr_detector jalan
> nonstop, seed 1001/1002/1003) pasca-fix, vs crash <2 menit sebelum fix. 101/101
> unit test lolos. **Verifikasi kedua (independen, sesi terpisah)**: 3 seed
> (1001/1002/1003) × 600 detik penuh (`timeout 600s`, ~30 menit total qr_detector
> nonstop) — **0 crash** di ketiganya (`semua_output.log`), menguatkan hasil di atas.

Legenda: ✅ jalan & terverifikasi di sim · 🧪 kode ada, verifikasi runtime tertunda/parsial
· ⏳ direncanakan/menyusul · OPEN gap desain/hardware.

| Milestone | Status | Kondisi sekarang |
|-----------|--------|------------------|
| **M1** — Kendali dasar & thruster allocation | ✅ | Wrench `/hydroships/cmd_vel` → allocator (damped pinv) → 6 thruster; odom umpan balik. Dulu keliru ✅ (thrust tak nyambung) — sudah diperbaiki (topic namespace + graded buoyancy + frame thruster). |
| **M2** — Stabilizer PID (depth/heading hold) | ✅ | `stabilizer` PID depth & heading menulis wrench; `use_sim_time` diperbaiki. Test PID lolos. |
| **M3** — Sensor & persepsi | 🧪 | `camera_info` (`fx=fy=381.4 cx=320 cy=240`) & render kamera headless **TERBUKTI ulang runtime 2026-08-06**. `scan_depth=0.30` (revisi dari 0.46, lihat M6/CHANGELOG) **TERBUKTI TETAP decode QR 'A'** & misi lanjut ke GRAB — regresi framing lama tidak berulang. Payload QR **di-spawn RANDOM** (A/B/C/D) oleh `payload_spawner`; FSM baca posisi via `/hydroships/payload_pose` (latched); lampu `payload_fill` range 3.0 m menutupi area spawn. Visual servo `/hydroships/qr_offset` **berfungsi** (GRAB tercapai mengonfirmasi centering jalan secara fungsional). **VERIFY tersisa**: huruf B/C/D belum diuji ulang sesi 2026-08-06 (kehabisan waktu); nilai `ex/ey` `qr_offset` presisi belum dicatat numerik. Kalibrasi ke kamera fisik = **OPEN** (gap hardware). |
| **M4** — Arena / world | ✅ | `worlds/kki_arena.sdf` dibangun (QR + hook). Payload **tidak lagi inline di world** — di-spawn runtime oleh `payload_spawner`. **Pemetaan label hook A–D ↔ sisi kolam TERBUKTI cocok** (2026-08-06): kode `WALL_HEADING_DEG` vs pose SDF `hook_a=(0,-2.5)/hook_b=(0,2.5)/hook_c=(2.5,0)/hook_d=(-2.5,0)` — A=-Y/B=+Y/C=+X/D=-X terkonfirmasi. Geometri hook Ø25 mm terkonfirmasi di SDF; validasi fisik "cukup untuk uji sangkut nyata" tetap **OPEN** (butuh arena fisik). |
| **M5** — Manipulator | ✅ **[M5-D TERTUTUP: attach + siklus penuh 3/3 run, 2026-08-12]** | **Diagnosis gerbang attach — 3 run, instrumentasi `GATEDBG` (2026-08-12; log mentah `~/m5d-diagnosis-logs/run{1,2,3}.log`, payload di (0.40,0.04)/(−0.30,0.55)/(0.75,−0.45), `headless:=true`, spawn ROV acak):** **VERIFIED (3/3)** — `close` diterima gerbang di ketiga run (`attach (payload dalam jangkauan)`), lalu `NAV_WALL` konvergen (`dist` 0.17/0.19/0.19 m ke wall A/C/C) dan siklus lanjut sampai `SURFACE -> WAIT_TRIGGER` tanpa satu pun ABORT. **Root cause penolakan pra-perbaikan = kandidat (c), VERIFIED:** deteksi QR memang MATI sebelum `"close"` dikirim — offset terakhir dari kamera bawah datang 0.90 s / 2.81 s / 2.03 s sebelum tick keputusan, dan pada run2 `age=1.53 s > offset_timeout=1.5` → `fresh=False`; attach di run itu **lolos hanya lewat latch** (`arm_age=1.53 s`, `armed=True`). Tanpa latch, run2 pasti ditolak — persis mekanisme kegagalan lama. Kandidat **(a) alt_gap TIDAK menyebabkan penolakan** (0.073/0.074/0.075 ≤ 0.08 di ketiga run) **tapi marginnya cuma 5–7 mm** → gap struktural baru, dicatat `P1-OWNER-DECISIONS-AND-ROADMAP.md` **R-10**. Kandidat **(b) ey_target BUKAN penyebab**: `\|y−ey_target\|` = 0.235/0.057/0.164 vs `max_offset=0.30` (catatan: `ey_target` ter-**clamp** di −0.800 = `ey_target_max` pada run1 & run3, artinya geometri sudah menuntut QR di luar frame — lolos karena QR nyatanya masih tampak di ey≈−0.57…−0.64). `\|x\|` = 0.136/0.147/0.025, tak pernah marginal. **Gap struktural terpisah (BUKAN diperbaiki sesi ini):** `_st_grab` tak menunggu ack — skor `m2=15` & transisi `NAV_WALL` tetap jalan walau attach ditolak → **R-9**. **Doc-conflict (komentar, bukan logika):** docstring `_st_grab` masih menyatakan "payload sudah ter-DetachableJoint ke ROV sejak spawn ... attach praktis no-op"; log membantahnya di 3/3 run — `auto-detach startup [pemicu: payload spawn terdeteksi]` terbit **14.6 / 19.3 / 15.3 detik SEBELUM** `close`, jadi payload sudah lepas jauh sebelum GRAB. Redaksi perbaikan diserahkan ke penulis repo. **[CLOSED 2026-08-14, lihat subsection "Payload-terangkat & ketahanan grip" di bawah]** payload benar-benar terangkat (pose payload runtime belum diukur) & slip/gesekan fisik. Perbaikan yang diukur di sini: **Perbaikan gerbang attach (2026-08-12):** penyebab `tutup TAPI payload di luar jangkauan` di run sebelumnya sudah terisolasi — **memang mustahil**, bukan kebetulan `DECODE GAGAL`: di grasp_depth kamera bawah (`cam_bottom_dz=0.18`) turun sampai **sejajar bidang QR**, jadi tak akan pernah ada deteksi segar saat `"close"` dikirim, sementara `is_safe()` menuntutnya. Tiga perubahan: (1) `gripper_logic` kini me-**latch** kondisi visual yang terpenuhi di APPROACH_QR selama `arm_timeout=8 s` — hak attach bertahan melewati fase buta DESCEND, dan hangus saat `open`/`detach` supaya payload berikutnya tak mewarisinya; (2) `alt_gap` tak lagi menumpang `qr_offset` (yang berhenti terbit persis saat dibutuhkan) melainkan dihitung dari `/hydroships/depth` yang terbit terus, dan diukur dari **dasar gripper**, bukan base_link; (3) **URDF: `gripper_base_joint` z 0 → −0.13** — dulu gripper setinggi pusat hull, jadi walau ROV turun mentok, gripper tetap ~0.19 m di atas payload; sekarang celah rancangan **0.034 m** di `grab_depth=0.70` (dasar hull masih 0.12 m di atas lantai). `grasp_standoff` diganti `grab_depth`. Geometri lintas-file (xacro ↔ rov_params ↔ mission_fsm ↔ gripper_controller) dikunci `test/test_grab_geometry.py`; latch dikunci 3 test di `test_gripper.py` (101 test hijau). **Yang MASIH harus dibuktikan runtime:** log `attach (payload dalam jangkauan)` benar-benar muncul, dan payload **ikut terangkat** saat depth naik ke `hook_depth` — kesuksesan NAV_WALL saja TIDAK membuktikannya (payload sudah ter-DetachableJoint sejak spawn). **M5-D** verifikasi runtime (2026-08-12, `start_state:=APPROACH_QR start_wall:=B headless:=true`, 1 run C-siklus):** State `St.DESCEND` **TERBUKTI bekerja** — depth turun dari scan_depth 0.29 m ke grasp_depth 0.76 m dalam ~3 s (`DESCEND: kedalaman grasp tercapai (0.76m) -> GRAB`), lalu **NAV_WALL TERBUKTI konvergen** (`Tiba di standoff wall B (dist 0.20m, v 0.30m/s) -> HANG`, tanpa ABORT) dan misi lanjut mulus sampai `SURFACE -> WAIT_TRIGGER`. Blocker fundamental lama (ROV terjangkar ke lantai, NAV_WALL timeout) **tidak lagi terjadi** — root cause di STATUS lama (tak ada fase turun-untuk-mencengkeram) **sudah tertutup**. **TAPI ketahuan blocker baru saat run yang sama:** persis saat GRAB memicu `"close"`, `gripper_controller` log **`gripper closed: tutup TAPI payload di luar jangkauan -> tak attach`** — gerbang visual `GripperLogic.is_safe()` (offset x/y, ukuran QR, `alt_gap`, freshness — lihat `gripper_logic.py:79-102`) menolak attach meski DESCEND sudah di grasp_depth yang benar. Attach via `DetachableJoint` **tidak pernah benar-benar terpicu oleh gripper_controller** di run ini; misi tetap "sukses" karena — sesuai catatan `_st_grab` di kode — payload di sim **sudah ter-DetachableJoint ke ROV sejak spawn**, independen dari path gripper. Jadi kesuksesan NAV_WALL run ini **membuktikan fase DESCEND**, bukan **membuktikan mekanisme grasp visual-gated**. Perlu investigasi kenapa `is_safe()` gagal tepat di titik itu (kandidat: `alt_gap` dihitung dari `qr_floor_z` yang mungkin belum match hasil DESCEND, atau QR offset basi/hilang — log qr_detector menunjukkan beberapa `DECODE GAGAL` berturutan persis di sekitar transisi DESCEND→GRAB) sebelum item ini bisa ditutup. Item lama di bawah (fase DESCEND belum dikerjakan) **CLOSED**; item baru: **gerbang visual attach gripper_controller OPEN**. Riwayat sebelum fix DESCEND: (1) `_st_grab` kini publish `"close"` sekali per masuk state — TERBUKTI runtime (`GRAB: perintah "close" -> gripper_controller`). (2) **Bug lebih dalam yang baru ketahuan setelah (1) diperbaiki:** `gripper_controller` tak memfilter `frame_id` padahal `qr_detector` menerbitkan KEDUA kamera ke `/hydroships/qr_offset` → gerbang dinilai dari kamera depan (terukur `ex=0.90 ey=0.75`) saat ROV justru terpusat rapi (`gripper_err=0.032 m`). (3) **Bug ketiga, akar sesungguhnya:** `GripperLogic.is_safe()` menuntut `|ey| <= 0.30` (acuan pusat kamera) sedangkan `mission_fsm` sengaja membidik `ey_target ≈ −0.52` (acuan gripper, karena gripper 0.16 m di depan kamera bawah) — **dua kriteria mustahil dipenuhi bersamaan**, terukur **0/34 tick GRAB lolos** di run C1. Diperbaiki: `qr_ey_target` dipindah ke `qr_logic` sbg satu sumber, `is_safe()` kini menguji `|ey − ey_target|`. Dikunci 3 test regresi di `test_gripper.py`. **M5-D — BLOCKER BARU, paling fundamental (2026-08-12):** begitu attach benar-benar terpicu, ROV **terjangkar dan misi ABORT**. `DetachableJoint` mengelas kedua link **pada pose saat itu**; ia TIDAK menarik payload ke gripper. Saat GRAB, ROV melayang **0.602/0.601/0.605 m di atas payload** (konsisten 3/3 run) karena `scan_depth=0.30` memang dirancang tinggi agar QR muat di frame. Akibatnya ROV mengelas diri ke benda yang masih di lantai → surge perintah **8–16 N menghasilkan |v|≈0.005 m/s**, depth terkunci 0.2 m di atas setpoint, NAV_WALL timeout. **Akar desain: FSM tak punya fase TURUN-UNTUK-MENCENGKERAM** — APPROACH_QR (melayang 0.6 m) langsung ke GRAB (attach). Perilaku lama menyembunyikan ini: misi "selesai" justru KARENA grasp tak pernah terjadi. Butuh keputusan desain (fase DESCEND / turunkan depth sebelum close / attach yang memindahkan payload) — **belum dikerjakan, menunggu persetujuan**. **[CLOSED 2026-08-14]** sim tak memvalidasi cengkeraman/slip fisik sama sekali. Riwayat lama: | Node `gripper_controller` + `gripper_logic` aktif; body gripper dua-jari opposing kosmetik di muka depan ROV **terlihat & bergerak** (dikonfirmasi via startup-detach log). **Startup-detach TERBUKTI berurutan benar** (`payload spawned OK` → sinyal → auto-detach). `AUTO_RELEASE` publish detach **TERBUKTI** (diuji via start mid-state). **TAPI: `_st_grab` (`mission_fsm.py:606-618`) TIDAK PERNAH mengirim "close" ke `/hydroships/gripper/command`** — publisher `pub_grip` dideklarasikan (baris 221) tapi tak sekali pun dipanggil `.publish()` di seluruh file. Akibatnya **DetachableJoint attach tidak pernah terpicu** selama misi autonomous — payload tetap lepas sepanjang NAV_WALL/HANG/SURFACE dst. Ini **fungsi grasp inti yang hilang**, bukan sekadar item verifikasi tertunda. Detail & rencana perbaikan: [CHANGELOG](CHANGELOG.md) entry 2026-08-06. |
| **M6** — Autonomy (FSM misi) | 🧪 **[NAV_WALL ditutup HANYA tanpa payload; blocker baru M5-D]** | ⚠️ **KUALIFIKASI PENTING:** 47/47 di bawah diukur dari battery di mana **attach tidak pernah terpicu**, jadi ROV tak pernah benar-benar membawa payload. Setelah attach diperbaiki (2026-08-12), **NAV_WALL ABORT di 3/3 siklus ber-attach** (macet di 0.50–0.55 m). Kontrol dalam-run E13: siklus-1 tanpa attach **selesai penuh**, siklus-2 dengan attach **gagal**. Jadi NAV_WALL terbukti sehat **hanya untuk kasus tanpa beban**. Lihat M5-D & [P1-FASE1-RESULTS.md](P1-FASE1-RESULTS.md). **UPDATE 2026-08-13 — stall ini TAMPAKNYA SUDAH RESOLVED oleh fix DESCEND+URDF (`e8ee4d2`/`df4f46f`, sama-sama 2026-08-12):** analisis akar masalah (bukan force-budget — komando surge historis 8–16N jauh di bawah `nav_fmax=22N`/budget gabungan ~120N, tidak pernah saturasi; bukan payload mass — cuma 0.02kg; melainkan `DetachableJoint` mengelas ROV↔payload pada gap 0.60m saat GRAB dulu, menjangkarkan ROV secara kinematik) menunjuk ke gap attach sbg akar. Battery verifikasi **n=10** (seed 1001/1002/2001-2008, `headless:=true`, siklus penuh `DIVE→...→WAIT_TRIGGER`, attach ter-trigger dgn `alt_gap` 0.072–0.080 m — dalam rentang fix): **10/10 NAV_WALL konvergen** (`dist` 0.17–0.20 m, median ~0.18m — setara kasus tanpa-beban 0.197m), 0 ABORT, 0 timeout, 0 crash. Data lama "3/3 macet" kemungkinan direkam pada state kode antara `e8ee4d2` dan `df4f46f`, sebelum gap benar-benar mengecil. **Force budget & `approach_kp/kd` TERBUKTI BUKAN penyebab** (lihat catatan dugaan gain di akhir entri ini — sudah tidak relevan, jangan diubah). Catatan: seed 2008 sempat 1x attach REJECTED (`GATEDBG close result=False`, QR offset meleset `|y-ey|=0.805`) tapi misi tetap lanjut ke WAIT_TRIGGER — pola dikenal sbg **R-9** (`_st_grab` tak menunggu ack attach). **R-9 DITUTUP 2026-08-13**: `_st_grab` kini menunggu ack `/hydroships/gripper/status` (`'attached'`/`'rejected'`), retry saat rejected, `ABORT` bila tak pernah attached dalam `T['grab']`. Battery 3-seed baru (`R9-3001/3002/3003`, headless): **2/3 selesai 4-hook penuh SKOR 100/100** (3001, 3003); **3002 ABORT jujur** — GRAB ditolak terus (`x=+0.799` tak pernah masuk `max_offset=0.30`), gerbang visual bekerja benar, R-9 mencegah skor m2=15 palsu yang dulu akan terjadi. Root cause penolakan 3002 dicatat sbg **R-11** (kemungkinan presisi centering APPROACH_QR/DESCEND utk posisi spawn tertentu, belum didiagnosis). Fase 1 roadmap exit criteria (4-hook ×3 seed) jadi **2/3**, lihat `P1-OWNER-DECISIONS-AND-ROADMAP.md` §5-6. Bukti kasus tanpa beban: **47/47 run konvergen, 0 timeout** (battery P0-2.6/2.7/2.8, `d_min` median 0.197 m vs `nav_tol=0.20`, durasi median 5.4 s dari anggaran 30 s). Akar penyebab lama sudah hilang sejak commit **`f9a2d84` (2026-08-07)** yang mengubah `wall_dist` **2.30 → 2.15**; entri blocker di bawah ditulis 2026-08-06, sehari sebelum perbaikan itu, lalu tak pernah ditutup. Siklus 4-hook penuh **TERBUKTI berjalan end-to-end** (run C1: 3 siklus berurutan `DIVE→APPROACH_QR→GRAB→NAV_WALL→HANG→SURFACE→WAIT_TRIGGER→APPROACH_HOOK→AUTO_RELEASE` tanpa satu pun ABORT; siklus ke-4 terpotong batas rekaman, bukan kegagalan). Catatan tersisa: `APPROACH_HOOK` masih sering keluar lewat **fallback timeout** `t_approach=25s`, bukan kriteria centering asli. Riwayat lama: | Alur: `DIVE → APPROACH_QR → GRAB → NAV_WALL → HANG → SURFACE → WAIT_TRIGGER → APPROACH_HOOK → AUTO_RELEASE`. **TERBUKTI runtime 2026-08-06**: `DIVE→APPROACH_QR→GRAB` sukses (skor m1/m2); `APPROACH_HOOK→AUTO_RELEASE→SURFACE→loop DIVE` sukses via isolasi mid-state (termasuk **fallback timeout `t_approach=25s` ke AUTO_RELEASE, bukan ABORT** — bekerja sesuai desain). **BLOCKER BARU: `NAV_WALL` tidak konvergen ke `nav_tol=0.20`** — ROV berhenti bergerak di `dist≈0.26 m` lalu timeout → ABORT; siklus 4-hook penuh **tidak bisa diselesaikan** end-to-end saat ini. Catatan koreksi: mekanisme soft-stop `wall_face`/`wall_standoff`/`_wall_clearance()` yang tercatat RESOLVED 2026-07-18 **tidak ada lagi di kode saat ini** (kode aktif pakai `wall_dist=2.30` langsung) — deskripsi lama di sini sudah usang, dikoreksi. `t_approach` **memang terpakai** (koreksi atas catatan draft rencana yang sempat keliru menyebut param ini dead). Servo `APPROACH_HOOK` terbukti wired & responsif tapi **konvergensi tak andal** (osilasi ex/size) — bergantung fallback timeout, bukan kriteria centering asli. Dugaan akar NAV_WALL: closed-loop gain (`approach_kp/kd`) belum ditala ulang utk massa baru 8.3 kg + hydro coefficient scaling (`aa2410d`/`00d4aaa`) — **JANGAN ubah gain sebelum mengukur lebih lanjut**. |
| **M7** — Integrasi GUI tim | 🧪 | Adapter `gui_bridge` **diuji live dengan dashboard GUI-ROV asli 2026-08-13** (`server.js` lokal, `RPI_ADDR=127.0.0.1`): arm/disarm, yaw (heading berputar penuh & wrap 360°), gripper open/close (round-trip sampai `gripper_controller`, gate jarak bekerja benar) **lolos**. Surge/sway terkirim tapi efek gerak sim belum diverifikasi (perlu echo `/hydroships/odom` di run berikutnya); tombol **light** belum sempat dites (tak ada command di log run ini). Command `pool_depth`/`controller` dari dashboard diabaikan diam-diam oleh `gui_bridge_logic.py` (nama tak dikenal → no-op) — bukan bug, tapi tombol/slider terkait di dashboard saat ini tak berefek ke sim. **[OPEN] Ditemukan saat test**: roll/pitch melonjak besar (±25-31°) selama yaw ditahan lama, redam pelan setelah yaw berhenti — belum jelas apakah ini karakter fisik wajar dari pulsa kontrol keyboard (bukan stick kontinu) atau indikasi allocator/gain perlu ditinjau; perlu run ulang dengan joystick asli utk pembanding. Telemetry rate terukur ~3 Hz di beberapa jendela 1 detik, di bawah `telem_hz=10` default (`gui_bridge.py:57`) — perlu profiling. Gain persen→N (`surge_gain=0.40` dst.) & kalibrasi tanda/offset kompas tetap **OPEN**/estimasi (bergerak namun tanda/skala belum divalidasi terhadap gerak sim yang diharapkan). Lihat [GUI-INTEGRATION.md](GUI-INTEGRATION.md). |

## Manipulator (M5) — status final (satu-satunya versi aktif)

Untuk menghindari kebingungan dari riwayat lama (gripper 2-jari lama → dihapus →
dirancang ulang), berikut **satu-satunya desain yang aktif sekarang**:

- **Grasp sesungguhnya** = plugin gz-sim **`DetachableJoint`** yang menyambung kaku
  `gripper_base` (di muka depan ROV, +X) ↔ model `payload`. Attach/detach via topik
  `/hydroships/gripper/attach` & `/hydroships/gripper/detach` (`std_msgs/Empty`).
- **Dua jari opposing** `gripper_finger_left` / `gripper_finger_right` = revolute sumbu Z,
  menjepit di bidang XY (bentuk parallel gripper). **Kosmetik** — indikator visual
  buka-tutup saja, tidak ikut menahan payload. Dikendalikan lewat
  `/hydroships/gripper_left/cmd` & `/hydroships/gripper_right/cmd` (`Float64`, rad);
  kedua topik selalu menerima **nilai yang sama**, arah tutup dibedakan oleh tanda
  `axis` di URDF. Sudut 0 = tertutup, 0.35 = terbuka (limit joint [-0.1, 0.5]).
- Node **`gripper_controller`** menerima `/hydroships/gripper/command` (`String` "open"/"close").
  Attach hanya dipicu saat "close" **dan** ROV berada di atas payload dalam jangkauan aman
  (dinilai dari `/hydroships/qr_offset`). Satu detach otomatis saat startup membatalkan
  attach bawaan Fortress.
  > ⚠️ **Regresi 2026-08-06**: `mission_fsm._st_grab` saat ini **tidak pernah** publish
  > "close" ke topik ini — jalur attach di atas benar secara desain tapi **tidak pernah
  > dipicu** oleh FSM autonomous. Lihat [CHANGELOG](CHANGELOG.md) 2026-08-06 untuk detail.
- **Bukan** gripper 2-jari (versi lama, dibatalkan) dan **bukan** hook servo. `hook_detector`
  adalah subsistem terpisah untuk mendeteksi hook arena (state `APPROACH_HOOK`).

Detail keputusan & riwayat pembatalan: [CHANGELOG.md](CHANGELOG.md).

### Payload-terangkat & ketahanan grip — CLOSED (2026-08-14)

Menindaklanjuti dua gap lama (**"Masih UNVERIFIED: payload benar-benar terangkat"** &
**"TETAP OPEN juga: sim tak memvalidasi cengkeraman/slip fisik"**): **kedua gap DITUTUP**,
dibuktikan battery **3/3 seed PASS** kedua kriteria (`spawn_seed:=4001/4002/4003`,
`headless:=true`, `ros2 run hydroships_gazebo validate_grab_lift`):

| Seed | TERANGKAT (delta_z odom / max drift, tol 0.030m) | TAK SLIP (delta pasca-gangguan) |
|------|---------------------------------------------------|----------------------------------|
| 4001 | PASS — 0.138m / 0.010m | PASS — 0.009m |
| 4002 | PASS — 0.138m / 0.012m | PASS — 0.012m |
| 4003 | PASS — 0.145m / 0.012m | PASS — 0.012m |

Payload **benar-benar ikut naik** bersama ROV saat NAV_WALL→HANG (drift ROV↔payload ≤1.2cm,
jauh di bawah toleransi 3cm) dan **tetap tersambung** (`gripper/status` tetap `'attached'`)
setelah gangguan gaya 6N/1s dikirim ke `/hydroships/cmd_vel` — `DetachableJoint` terbukti
menahan beban dinamis, bukan cuma ack status tanpa efek fisik nyata.

**Perjalanan instrumentasi (untuk konteks, jangan diulang):**
1. Percobaan awal memasang plugin gz-sim `PosePublisher` langsung di model `payload`
   **KELIRU** — dengan `publish_link_pose=true`, topik `/model/payload/pose` ternyata
   menerbitkan pose **link relatif ke model** (selalu `(0,0,0)`, dikonfirmasi via
   `ign topic -e -t /model/payload/pose` yang menunjukkan `position {}` kosong/nol dan
   `child_frame_id: "payload::payload_link"`), bukan pose model relatif dunia. Ini
   menghasilkan **battery pertama FALSE FAIL 3/3** (drift 0.115–0.122m — persis sama
   dengan kenaikan `odom.z` milik ROV sendiri, karena `payload.z` yang terbaca stuck di 0).
2. Perbaikan: **hapus plugin custom sepenuhnya** — pakai topik bawaan gz-sim
   `/world/kki_arena/pose/info` (`gz.msgs.Pose_V`, diterbitkan otomatis oleh scene
   broadcaster, tak perlu plugin tambahan), dibridge sbg `tf2_msgs/msg/TFMessage` ke
   `/hydroships/world_pose_tf` (`bridge.yaml`), lalu `validate_grab_lift.py` memfilter
   transform dengan `child_frame_id == 'payload'`. Terbukti membawa pose dunia yang benar
   (nonzero, berubah seiring waktu).
3. Bug kedua (di skrip, bukan sim): `rclpy.shutdown()` dipanggil dari dalam callback timer
   saat `rclpy.spin(node)` berjalan membuat proses menggantung & `print()` ke file tak
   pernah ter-flush. Diperbaiki dengan loop `spin_once` manual + flag `done` + `flush=True`.

`/hydroships/payload_pose` lama (snapshot statis saat spawn, dipakai APPROACH_QR/Gate-4)
**tidak diubah**.

Cara reproduksi:
```bash
ros2 launch hydroships_bringup hydroships_mission.launch.py headless:=true spawn_seed:=4001
# di terminal lain, setelah gripper attached (lihat log GRAB):
ros2 run hydroships_gazebo validate_grab_lift
```
