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

Legenda: ✅ jalan & terverifikasi di sim · 🧪 kode ada, verifikasi runtime tertunda/parsial
· ⏳ direncanakan/menyusul · OPEN gap desain/hardware.

| Milestone | Status | Kondisi sekarang |
|-----------|--------|------------------|
| **M1** — Kendali dasar & thruster allocation | ✅ | Wrench `/hydroships/cmd_vel` → allocator (damped pinv) → 6 thruster; odom umpan balik. Dulu keliru ✅ (thrust tak nyambung) — sudah diperbaiki (topic namespace + graded buoyancy + frame thruster). |
| **M2** — Stabilizer PID (depth/heading hold) | ✅ | `stabilizer` PID depth & heading menulis wrench; `use_sim_time` diperbaiki. Test PID lolos. |
| **M3** — Sensor & persepsi | 🧪 | `camera_info` (`fx=fy=381.4 cx=320 cy=240`) & render kamera headless **TERBUKTI ulang runtime 2026-08-06**. `scan_depth=0.30` (revisi dari 0.46, lihat M6/CHANGELOG) **TERBUKTI TETAP decode QR 'A'** & misi lanjut ke GRAB — regresi framing lama tidak berulang. Payload QR **di-spawn RANDOM** (A/B/C/D) oleh `payload_spawner`; FSM baca posisi via `/hydroships/payload_pose` (latched); lampu `payload_fill` range 3.0 m menutupi area spawn. Visual servo `/hydroships/qr_offset` **berfungsi** (GRAB tercapai mengonfirmasi centering jalan secara fungsional). **VERIFY tersisa**: huruf B/C/D belum diuji ulang sesi 2026-08-06 (kehabisan waktu); nilai `ex/ey` `qr_offset` presisi belum dicatat numerik. Kalibrasi ke kamera fisik = **OPEN** (gap hardware). |
| **M4** — Arena / world | ✅ | `worlds/kki_arena.sdf` dibangun (QR + hook). Payload **tidak lagi inline di world** — di-spawn runtime oleh `payload_spawner`. **Pemetaan label hook A–D ↔ sisi kolam TERBUKTI cocok** (2026-08-06): kode `WALL_HEADING_DEG` vs pose SDF `hook_a=(0,-2.5)/hook_b=(0,2.5)/hook_c=(2.5,0)/hook_d=(-2.5,0)` — A=-Y/B=+Y/C=+X/D=-X terkonfirmasi. Geometri hook Ø25 mm terkonfirmasi di SDF; validasi fisik "cukup untuk uji sangkut nyata" tetap **OPEN** (butuh arena fisik). |
| **M5** — Manipulator | 🧪 **[REGRESI BLOCKING]** | Node `gripper_controller` + `gripper_logic` aktif; body gripper dua-jari opposing kosmetik di muka depan ROV **terlihat & bergerak** (dikonfirmasi via startup-detach log). **Startup-detach TERBUKTI berurutan benar** (`payload spawned OK` → sinyal → auto-detach). `AUTO_RELEASE` publish detach **TERBUKTI** (diuji via start mid-state). **TAPI: `_st_grab` (`mission_fsm.py:606-618`) TIDAK PERNAH mengirim "close" ke `/hydroships/gripper/command`** — publisher `pub_grip` dideklarasikan (baris 221) tapi tak sekali pun dipanggil `.publish()` di seluruh file. Akibatnya **DetachableJoint attach tidak pernah terpicu** selama misi autonomous — payload tetap lepas sepanjang NAV_WALL/HANG/SURFACE dst. Ini **fungsi grasp inti yang hilang**, bukan sekadar item verifikasi tertunda. Detail & rencana perbaikan: [CHANGELOG](CHANGELOG.md) entry 2026-08-06. |
| **M6** — Autonomy (FSM misi) | 🧪 **[REGRESI BLOCKING]** | Alur: `DIVE → APPROACH_QR → GRAB → NAV_WALL → HANG → SURFACE → WAIT_TRIGGER → APPROACH_HOOK → AUTO_RELEASE`. **TERBUKTI runtime 2026-08-06**: `DIVE→APPROACH_QR→GRAB` sukses (skor m1/m2); `APPROACH_HOOK→AUTO_RELEASE→SURFACE→loop DIVE` sukses via isolasi mid-state (termasuk **fallback timeout `t_approach=25s` ke AUTO_RELEASE, bukan ABORT** — bekerja sesuai desain). **BLOCKER BARU: `NAV_WALL` tidak konvergen ke `nav_tol=0.20`** — ROV berhenti bergerak di `dist≈0.26 m` lalu timeout → ABORT; siklus 4-hook penuh **tidak bisa diselesaikan** end-to-end saat ini. Catatan koreksi: mekanisme soft-stop `wall_face`/`wall_standoff`/`_wall_clearance()` yang tercatat RESOLVED 2026-07-18 **tidak ada lagi di kode saat ini** (kode aktif pakai `wall_dist=2.30` langsung) — deskripsi lama di sini sudah usang, dikoreksi. `t_approach` **memang terpakai** (koreksi atas catatan draft rencana yang sempat keliru menyebut param ini dead). Servo `APPROACH_HOOK` terbukti wired & responsif tapi **konvergensi tak andal** (osilasi ex/size) — bergantung fallback timeout, bukan kriteria centering asli. Dugaan akar NAV_WALL: closed-loop gain (`approach_kp/kd`) belum ditala ulang utk massa baru 8.3 kg + hydro coefficient scaling (`aa2410d`/`00d4aaa`) — **JANGAN ubah gain sebelum mengukur lebih lanjut**. |
| **M7** — Integrasi GUI tim | 🧪 | Adapter `gui_bridge` (UDP-JSON ↔ ROS 2) **round-trip TERBUKTI 2026-08-06**: `hydroships_gui.launch.py` tidak menjalankan `stabilizer` (tak ada konflik publisher `cmd_vel`); UDP `arm`/`surge` → `armed` berubah & `cmd_vel` berubah; telemetri UDP balik (heading/depth/roll/pitch nyata) diterima di port 14551. **Belum** diuji dengan dashboard GUI asli (hanya klien UDP sintetis); gain persen→N (`surge_gain=0.40` dst.) & kalibrasi tanda/offset kompas tetap **OPEN**/estimasi. Lihat [GUI-INTEGRATION.md](GUI-INTEGRATION.md). |

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
