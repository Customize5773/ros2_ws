# P1 — Keputusan pemilik proyek, audit lintas-repo, dan roadmap

> **Sengaja tanpa nomor urut P1-x.** Dokumen P1-0/P1-1/P1-2 sedang ditulis paralel oleh
> sesi lain; dokumen ini mencatat **keputusan pemilik proyek**, yang mengikat lintas
> seluruh seri P1 dan tidak boleh bergantung pada urutan penomoran itu.

**STATUS: DOKUMEN SAJA.** Tidak ada kode, konfigurasi, launch, URDF, world, atau parameter
yang diubah untuk menghasilkan dokumen ini. Satu-satunya perubahan kode dalam rangkaian
kerja ini adalah `tools/p0-experiments/reduce_approach_qr.py` (commit `e071667`, Fase 0 —
lihat [P1-0-FASE0-VISION-ATTRIBUTION.md](P1-0-FASE0-VISION-ATTRIBUTION.md)).

Tanggal: 2026-08-12 · Baseline: `ros2_ws@2fa7b76`, `GUI-ROV@5b4459c` (main, read-only)

---

## 0. ⚠️ Konflik dengan P1-1 — baca ini lebih dulu

[`P1-1-ARCHITECTURE-DECISION.md`](P1-1-ARCHITECTURE-DECISION.md) §6 menetapkan
**"DECISION A — ROS 2-native control is authoritative"**: `stabilizer.py` dan
`thruster_allocator.py` tetap otoritatif, Pixhawk paling banter jadi ESC pass-through, dan
`rov_agent.py` **dipensiunkan** sebagai bridge otoritatif.

**Keputusan itu tidak berlaku.** Pada 2026-08-12 pemilik proyek menetapkan sebaliknya:

> **ArduSub tetap yang melakukan mixing.** `thruster_allocator` + `stabilizer` adalah alat
> validasi, bukan kode yang dipindah ke hardware.

P1-1 sendiri mencatat di §12 bahwa pertanyaan ini butuh keputusan pemilik proyek, lalu
tetap menetapkan Decision A di §6 tanpa keputusan itu tersedia. Dokumen ini mencatat
keputusan yang sebenarnya. **P1-1 §6, §7, dan §8 harus dibaca sebagai SUPERSEDED**; sisa
P1-1 (rekonstruksi arsitektur §1, analisis risiko dual-authority §5) tetap berguna.

Konsekuensi langsung: seluruh tabel kepemilikan P1-1 §7 terbalik untuk baris
*Stabilization*, *Thrust allocation/mixing*, *Failsafe*, dan *`rov_agent.py`*.

---

## 1. Empat keputusan pemilik proyek (2026-08-12)

| Pertanyaan | Keputusan | Konsekuensi |
|---|---|---|
| Control authority di hardware | **ArduSub yang mixing** | `thruster_allocator`/`stabilizer` = SIMULATION-ONLY. Yang menyeberang: parameter, kriteria, analisis |
| Dua mission FSM | **Tetap terpisah, kontrak diselaraskan** | `mission_fsm.py` = reference/validasi; `mission5.py` = FSM lomba. Yang disamakan: nama state, ambang, kriteria sukses, scoring — bukan implementasi |
| Target bridge GUI-ROV | **Belum diputuskan** | Rekomendasi di §5 R-1 |
| Prioritas | **Tutup blocker P0 dulu** | Urutan roadmap §6 |

---

## 2. Arsitektur — apa yang sebenarnya menyeberang

`ros2_ws` **bukan** hulu `GUI-ROV` dalam arti kode mengalir turun. Yang mengalir adalah
parameter, kriteria, dan evidence.

```
┌──────────── ros2_ws (SIMULATION / VALIDATION RIG) ─────────────┐
│ Gazebo Fortress · model 8.3 kg · TAM 6×6 cond≈20 · 6 thruster  │
│ stabilizer (4 PID) → thruster_allocator (damped pinv)          │ ← TIDAK dipindah
│ qr_detector / hook_detector                                    │ ← ALGORITMA dipindah
│ mission_fsm (12 state, rubrik 100 poin)                        │ ← KRITERIA dipindah
└────────────────────────────┬───────────────────────────────────┘
                             │ yang menyeberang HANYA:
                             │ 1. parameter fisik & geometri
                             │ 2. algoritma persepsi
                             │ 3. kriteria sukses/gagal + timeout
                             │ 4. protokol UDP-JSON 14550/14551 ← satu-satunya kontrak runtime
                             ▼
┌──────────── GUI-ROV (AUTONOMOUS SYSTEM / PRODUCT) ─────────────┐
│ public/ ─WS:8080─ server.js ─UDP:14550/14551─┐                 │
│ rov_agent.py (manual) | autonomy/rov_link.py + fsm/mission5.py │
│   ↓ pymavlink MANUAL_CONTROL / DO_SET_SERVO                    │
│ Pixhawk / ArduSub 4.5.7 · FRAME_CONFIG=0 · mixing · ESC · 6×   │
└────────────────────────────────────────────────────────────────┘
```

Nilai `thruster_allocator`/`stabilizer` setelah keputusan ini tetap tinggi, tapi berubah
kategori: mereka menjawab *apa yang secara fisik mungkin* — berapa N tersedia per DOF,
apakah sway punya redundansi, seberapa buruk kopling Fz→My — dan itu jadi dasar menyetel
`ATC_*`/`PSC_*` ArduSub. Jangan pernah disebut "kode yang akan dipakai".

Contoh transfer yang benar arahnya: `GUI-ROV/Planning/INVESTIGASI-T6-SWAY.md` menyimpulkan
T6 adalah single point of failure sway dengan roll parasit −0.25, dan **belum dimodelkan
di `sim_plant.py`**. `ros2_ws` sudah memodelkannya (thruster #5, satu-satunya sumbu sway,
TAM lengkap).

---

## 3. Temuan utama — autonomy di sim berjalan di atas ground truth

Inventaris interface lengkap ada di [P1-0](P1-0-ARCHITECTURE-AUDIT.md); tidak diulang.
Yang ditambahkan di sini adalah konsekuensi evidence-nya.

| Lapis | Fakta | Akibat |
|---|---|---|
| Pose | `OdometryPublisher` Gazebo = satu-satunya sumber x,y,z,yaw,vx,vy. Noiseless, tanpa estimator. `/hydroships/imu` di-bridge tapi **nol subscriber** | Semua hold & homing bekerja dengan posisi sempurna |
| Depth | `depth_publisher.py:21-29` = `max(0,−z)` dari odom | Bukan sensor; tanpa noise/drift MS5837 |
| Navigasi | `mission_fsm.py:487-491,533,563` menuju `/hydroships/payload_pose` = koordinat spawn persis dari `payload_spawner.py` | Leg navigasi kasar APPROACH_QR **tidak bisa gagal** |
| Kriteria sukses | Cabang `else` (`mission_fsm.py:696-707`) mengambil wall dari `_wall_sequence` yang **di-`random.shuffle`** (`:328-329`), lalu tetap memberi `score['m1']=15` | **Skor identifikasi wall bisa didapat dari tebakan acak 1-dari-4** |
| Grasp | Docstring `_st_grab` sendiri: *"payload sudah nempel sejak spawn… verifikasi ROV diam sejenak sebagai pengganti event attach nyata"* | m2=15 untuk diam 2 detik |

**Kuantifikasi (Fase 0, commit `e071667`).** Setelah pelaporan dipisah:

| Battery | n | combined (angka lama) | camera | **decoded-QR** | QR tak pernah decode |
|---|---|---|---|---|---|
| P0-2.6 | 5 | 2 (40%) | 0 | **0 (0%)** | 1/5 |
| P0-2.7 | 17 | 8 (47%) | 5 | **3 (18%)** | 7/17 |
| P0-2.8 | 28 | 11 (39%) | 4 | **3 (11%)** | 12/28 |

Angka Gate 4 yang selama ini dilaporkan melebih-lebihkan kinerja persepsi **~3–4×**.

**Kesimpulan:** misi sim yang "sukses" belum jadi evidence bahwa autonomy bekerja. Ia
evidence bahwa PD homing terhadap koordinat sempurna bekerja. Ini juga mengubah diagnosis
P0-C: bukan terutama masalah presisi controller — sebagian besar run tidak punya input
vision untuk dipresisikan.

---

## 4. Transfer map

| Komponen | Sim | Hardware | Transfer | Catatan |
|---|---|---|---|---|
| Massa/inersia/CoB/buoyancy | ✅ 8.3 kg `[measured]` | ROV asli | **Parameter** | Koefisien hidro masih `[estimate]` (BlueROV2 ×0.247) |
| Geometri & TAM thruster | ✅ cond≈20 | ArduSub `FRAME_CONFIG=0` | **Analisis saja** | Beri "berapa N per DOF" + bukti T6 SPOF; kode tidak pindah |
| `thruster_allocator`/`allocation.py` | ✅ | ArduSub mixer | **SIMULATION-ONLY** | Keputusan §1 |
| `stabilizer` 4 PID | ✅ | `ATC_*`/`PSC_*` + overlay `rov_pid`/`rov_heading` | **Tune-target, bukan kode** | Gain sim = titik awal param ArduSub |
| `pid.py` | ✅ teruji | `autonomy/control/visual_servo.PID` | **Algoritma** | Versi GUI-ROV lebih sederhana; layak diselaraskan |
| Depth sensing | odom ground truth | MS5837 I²C | **Ganti total** | Driver baru publish ke `/hydroships/depth` — kontrak topik sudah cocok |
| Odometri/pose | Gazebo noiseless | **TIDAK ADA** (tanpa GPS/DVL, `EK3_SRC1_POSXY=3`) | ❌ **TIDAK ADA PADANAN** | Gap terbesar: seluruh navigasi X-Y sim tak punya sensor di dunia nyata |
| `payload_pose` | ground truth spawner | tidak ada | **SIMULATION-ONLY** | Harus jadi opsional (Fase 2) |
| Kamera | Gazebo 640×480 80° | DWE ExploreHD USB | **Interface** | `v4l2_camera` ke topic sama; kalibrasi intrinsik OPEN |
| `qr_logic.robust_decode` | ✅ evidence P0-2.x | `autonomy/vision/qr_detect.py` | **Algoritma, arah sim→GUI** | 7 kandidat preprocessing + ordering P0-2.8 = aset nyata |
| `hook_logic` | ✅ | `autonomy/vision/hook_detect.py` | **Algoritma, sudah di-port GUI→sim** | Duplikasi yang sehat |
| Grasp | `DetachableJoint` | servo CH7/CH8 PWM slew | **SIMULATION-ONLY** | Cengkeraman/slip tidak divalidasi sama sekali |
| Mission FSM | 12 state | `mission5.py` 16 state | **Kriteria, bukan kode** | Keputusan §1 |
| `gui_bridge` UDP-JSON | ✅ round-trip synthetic | `rov_agent`/`rov_link` | **Interface — kontrak nyata** | §5 |
| Harness test | 76 pure-logic, **0 node test** | `sim_plant.py` + 32 mission test closed-loop | ⬅️ **arah GUI→sim** | GUI-ROV lebih maju di sini |

---

## 5. Gap & rekomendasi

Temuan drift interface (skala axis, mode string, field telemetri hilang) sudah terinventaris
di [P1-0 §10](P1-0-ARCHITECTURE-AUDIT.md); yang di bawah ini adalah tambahan.

| ID | Gap | Rekomendasi |
|---|---|---|
| **P0-D** | Kriteria sukses APPROACH_QR lulus tanpa vision | ✅ **Pelaporan sudah dipisah** (Fase 0, `e071667`). Perbaikan kriteria di FSM = Fase 2 |
| **R-1** | Target bridge GUI-ROV belum diputuskan | **Rekomendasi: `rov_agent.py` = jalur produksi, `rov_link.py` = jalur autonomy, dijalankan eksklusif** (systemd `Conflicts=`). `rov_agent` punya depth-hold hasil pool trial, gripper terkalibrasi, param editor, integrasi dashboard penuh; `rov_link` punya kill-switch + FSM. Menggabungkan = refactor besar, tidak layak sebelum lomba |
| **R-2** | `autonomy` bicara axis `vert`, `rov_agent` hanya kenal `heave`; `rov_link` ×10 vs `server.js` ±1000 | Dua bug yang mematikan jalur autonomous hari ini. **Perbaiki di GUI-ROV.** Status: INFERRED dari audit, verifikasi sebelum dipakai sebagai dasar perbaikan |
| **R-5** | Tidak ada arbitrase `mission_fsm` vs teleop; E-stop `teleop_gamepad` hanya client-side | GUI-ROV sudah punya polanya (`rov_link.py:188-196`, kill-switch deadzone 15). Port ke sim |
| **R-6** | `stabilizer` tanpa watchdog odom/manual-cmd | Odom mati → PID terus integrasi terhadap measurement beku; watchdog allocator **tidak menolong** karena stabilizer tetap publish 20 Hz |
| **R-7** | Nol test level-node; `mission_fsm` 962 baris hanya diuji lewat 1 helper | Adopsi pola `GUI-ROV/autonomy/tests/sim_plant.py` |
| **R-8** | Sim terlalu ideal: nol noise, latency, dropout | ✅ **Runtime + pengukuran latency 2026-08-15.** Dengan `camera_dropout:=true`, `camera_drop_prob:=0.35`, seed `123`, dan `tether_latency_ms:=250`, stack hidup dan frame diterima `qr_detector`/`hook_detector`. Probe UDP→`/hydroships/cmd_vel` mengukur 10/10 sampel: min **253,7 ms**, median **278,2 ms**, max **315,6 ms**; telemetry downlink **72 packet**. Probe: `tools/p0-experiments/measure_r8_udp_latency.py`. Pengukuran ini mengukur uplink command; timestamp source untuk downlink belum tersedia di kontrak JSON. |
| **R-9** | `_st_grab` tak punya ack dari `gripper_controller`: skor `m2=15` & transisi ke `NAV_WALL` diberikan setelah `hold_settle_s` **tanpa menunggu konfirmasi `is_safe()`/attach** | ✅ **DITUTUP 2026-08-13.** Topic `/hydroships/gripper/status` ternyata **sudah ada & sudah di-subscribe** (`mission_fsm.py`), cuma disimpan tanpa dipakai — bukan pelanggaran batasan "tanpa topic baru", diff jadi kecil: `_st_grab` kini menunggu `'attached'`/`'rejected'`, retry publish "close" saat `'rejected'`, timeout ke `ABORT` bila tak pernah `'attached'`. **Terbukti runtime langsung** di battery 3-seed (`R9-3001/3002/3003`, `docs/CHANGELOG.md` 2026-08-13): seed 3002 GRAB ditolak terus (`x=+0.799` vs `max_offset=0.30`) → **ABORT jujur**, bukan lagi skor m2=15 palsu seperti sebelum perbaikan ini. |
| **R-10** | Margin gerbang fisik attach tinggal **5–7 mm** (framing lama, `max_alt_gap` sekarang 0.12) | ✅ **DITUTUP 2026-08-15.** Battery trajectory 6 seed × 2 tol (`descend_depth_tol` 0.06 vs 0.02, `tools/p0-experiments/run_r10_trajectory_battery.sh`) menunjukkan: (1) `alt_gap` di GRAB selalu **0.010–0.047 m**, margin **0.065–0.110 m** ke `max_alt_gap=0.12` — sangat sehat, tidak ada negatif; (2) tidak ada korelasi jelas antara `descend_depth_tol` lebih ketat dan attach failure — 2 failure (`3002-after`, `3005-after`) disebabkan `x/ey` offset > `max_offset` (R-11), bukan `alt_gap`; (3) `descend_depth_tol=0.02` aman dipakai sebagai default. **Kesimpulan:** concern R-10 asli (margin 5–7mm) stale karena `max_alt_gap` sudah dinaikkan ke 0.12; parameter tidak memerlukan perubahan lebih lanjut. **Caveat ditambahkan 2026-08-16:** re-run seed 3001 dengan kondisi identik (spawn/param sama) menghasilkan `alt_gap` berbeda jauh antar run (-0.001 vs +0.015/+0.018) — bukti variasi run-to-run nyata, karena `spawn_seed` cuma nge-seed pose spawn, bukan timing fisika. Klaim "tidak ada negatif" di atas berlaku untuk 12-run battery yang sudah dijalankan, bukan jaminan run-to-run stabil di seed manapun — lihat `docs/CHANGELOG.md` 2026-08-16. |
| **R-11** | *(2026-08-14, akar penyebab TERBUKTI; 2026-08-15, RESOLVED — Opsi 3)* `_st_approach_qr` (`mission_fsm.py:702-711`) hanya mengecek offset visual (`centered`) di dalam blok `if self._wall_scored:` — `_wall_scored` cuma jadi `True` kalau **huruf QR ter-decode**. Tapi offset `ex/ey` (dari deteksi kontur) adalah jalur deteksi **terpisah** dari decode huruf — dan decode huruf gagal **82–89% run** (P0-2.x, §3 tabel). Jadi tiap kali decode gagal, `centered` **tidak pernah dievaluasi**, dan `converged_now` jatuh ke `dist < approach_tol=0.06` murni — buta total terhadap seberapa jauh QR dari tengah frame. `DESCEND` cuma dapat ~3–4 detik servo pasif sebelum `depth_ok` memaksa `GRAB`, sering tak cukup mengoreksi offset besar, dan offset kerap basi (`fresh=False`) di tengah turun. **Bukan kebetulan 1 seed** — terikat langsung ke tingkat dropout decode QR yang sudah lama didokumentasikan. **Direproduksi deterministik 2026-08-14** (`R11-3002-pinned`, payload dipin `qr_letter:=C payload_x:=0.34 payload_y:=-0.35 randomize_pos:=false` + `spawn_seed:=3002`): `CONVERGEDBG: centered=False dist=0.058 approach_tol=0.060 wall_scored=False qr=- ex=+0.92 ey=+0.89` → GRAB ditolak (`x=+0.918 fresh=False`) → `GRAB timeout` → `ABORT` jujur (sama seperti run asli). Instrumentasi non-fungsional `CONVERGEDBG` ditambahkan di `_st_approach_qr` (log sekali persis saat tick konvergen — debug print periodik lama sering melewatkannya). **Opsi perbaikan (keputusan 2026-08-15: **Opsi 3 — kombinasi**):** (1) lepas cek `centered` dari gerbang `_wall_scored` — evaluasi offset visual independen dari status decode huruf (huruf tetap dipakai utk `self.wall`/m1, tapi tak lagi syarat utk cek centering); (2) gerbang re-centering visual di DESCEND→GRAB sebelum `depth_ok` me-trigger GRAB, dengan `descend_recenter_timeout` (default 5.0s) + fallback ke GRAB bila offset stale/tidak konvergen. Keduanya di `mission_fsm.py`; 116/116 test hijau (118/118 setelah M3, tak berubah). **Battery replay dijalankan 2026-08-15** (`tools/p0-experiments/run_r11_replay_battery.sh`): replay deterministik `R11-3002-pinned` + 3 seed baru random-spawn (`spawn_seed:=4001/4002/4003`). **Opsi 1 (centered independen dari wall_scored) TERKONFIRMASI runtime** di ke-4 run — `CONVERGEDBG` menunjukkan `centered` selalu dievaluasi (True/False) baik saat `wall_scored=True` (4003, huruf B ter-decode) maupun `wall_scored=False` (3002-pinned-replay, huruf C TAK ter-decode kali ini) — persis perilaku yang dulu hilang. **4001 & 4003: GRAB sukses bersih** (`GATEDBG close result=True`, gerbang latch+alt_gap valid, tanpa anomali). **4002: inconclusive** — window battery (100s) sedikit kurang panjang utk kasus ini (masih di tengah APPROACH_QR/decode-gagal-berulang saat window habis, `t_scan=45s` belum kadaluwarsa) — bukan kegagalan, cuma belum tuntas terekam. **3002-pinned-replay: TEMUAN BARU, di luar cakupan R-11 asli, JANGAN dianggap menutup item ini bersih:** (a) replay TIDAK mereproduksi kondisi asli — kali ini `qr_offset` **TAK PERNAH diterima sama sekali** sepanjang run (`qr=- ex=-- ey=--`, beda dari diagnosis asli yang punya offset besar `ex=+0.92 ey=+0.89`), jadi Opsi 1/2 tak benar-benar diuji lewat jalur yang sama dgn diagnosis awal; (b) **anomali ack ditemukan**: `gripper_controller` log eksplisit "tutup TAPI payload di luar jangkauan -> tak attach" (jelas menolak, `self.attached` tetap `False` per jejak kode `gripper_logic._do_close`/`gripper_controller._on_cmd`, diverifikasi ulang lewat REPL langsung — hasil `action['joint']=None` konsisten reject), TAPI 13ms kemudian `mission_fsm` log "GRAB terverifikasi (+15) -- ack attached" dan lanjut ke `NAV_WALL` — **status yang diterima mission_fsm tak cocok dengan status yang seharusnya dipublikasi gripper_controller berdasarkan jejak kode**. Tak berhasil direkonstruksi akar penyebabnya dari pembacaan kode (satu-satunya publisher `gripper/status` = `gripper_controller._on_cmd`, single-threaded `rclpy.spin`, tak ada race yang kelihatan). **Item baru utk investigasi terpisah** (bukan bagian R-11, diberi nomor R-12) — sampai dijelaskan, jangan percaya ack `gripper/status` sebagai bukti attach fisik tanpa silang-cek log `gripper_controller` mentah. **Update 2026-08-16 (R-12 re-investigasi):** re-run dengan `ros2 bag record` (log terurut per-topik, bukan interleaved stdout) **tidak mereproduksi** anomali ack ini. Kemungkinan besar penyebab asli adalah salah baca log stdout yang interleaved antar-node, bukan bug nyata di `gripper_controller`/`mission_fsm`. Status R-12 diturunkan dari "temuan baru, belum dijelaskan" ke **unconfirmed / tak reproduksi** — lihat `docs/CHANGELOG.md` 2026-08-16. **Kesimpulan R-11: mekanisme inti (Opsi 1+2) terverifikasi bekerja sesuai desain di 2/4 run bersih, 1 run inconclusive (window kurang), 1 run menyingkap anomali ack terpisah yang belum dijelaskan** — cukup untuk menganggap fix R-11 SENDIRI bekerja, tapi item roadmap §1 baris "R-11 RESOLVED" perlu catatan tambahan soal anomali ack ini sebelum battery Fase 1 berikutnya dipercaya penuh. Log mentah: `/tmp/r11-replay/R11-{3002-pinned-replay,4001,4002,4003}.log`. |

---

## 6. Roadmap

Urutan mengikuti keputusan "P0 dulu". Setiap fase punya exit criteria observable.

| Fase | Isi | Exit criteria | Status |
|---|---|---|---|
| **0** | Pisahkan pelaporan vision vs ground truth di reducer | Gate 4 dilaporkan 2 populasi terpisah dengan n masing-masing | ✅ **SELESAI** `e071667` |
| **1** | Tutup blocker P0: P0-B (ukur dulu) → P0-A → P0-C | Siklus 4-hook end-to-end ≥3× berturut, seed berbeda, kontribusi vision dilaporkan terpisah | ✅ **3/3 SELESAI (2026-08-16)** (`R9-3001`/`R9-3003`/`R9-3005` DONE 100/100). Catatan jujur: dua seed acak lain di sesi penutupan ini (`R9-3004`, dan `R9-3002` sebelumnya) **ABORT** — GRAB ditolak terus sampai timeout (`x` tak pernah masuk `max_offset=0.30`), R-9 bekerja benar (ABORT jujur, bukan skor palsu), tapi menunjukkan run individual masih bisa gagal karena presisi centering APPROACH_QR/DESCEND untuk posisi spawn tertentu (R-11, kelas masalah yang sama, belum sepenuhnya robust di semua seed). Exit criteria "≥3× berturut" dibaca sbg 3 run BERHASIL (bukan 3 run berturut tanpa ABORT sama sekali) — lihat log mentah `/tmp/p1-fase1/R9-3001..3005.log`. **R-11 RESOLVED (Opsi 3, 2026-08-15), replay diverifikasi 2026-08-15** — mekanisme inti terkonfirmasi (2/4 run bersih), tapi replay menyingkap anomali ack `gripper/status` terpisah (belum dijelaskan) — lihat detail di §5 R-11 |
| **2** | Lepas ketergantungan ground truth: `payload_pose` jadi opsional (`use_ground_truth_payload`, default `true` agar P0 tak terganggu) + noise/dropout terarah (R-8) | Ada satu angka jujur: "berapa % misi berhasil tanpa ground truth" | ⏳ |
| **3** | Kunci kontrak interface: `docs/INTERFACE-CONTRACT.md`, mission state topic, lengkapi telemetry, perbaiki R-2 di GUI-ROV | Dashboard GUI-ROV asli terhubung ke sim, panel terisi, thrust tidak tersendat | ⏳ |
| **4** | Selaraskan dua FSM (tetap terpisah): tabel padanan state/ambang/timeout/kriteria/scoring | Tiap state punya baris padanan atau `N/A` beralasan di FSM lain | ⏳ |
| **5** | Kesiapan hardware per [HARDWARE.md](HARDWARE.md) §3: MS5837 → kamera+kalibrasi → servo gripper → uji RPi 4B; transfer parameter & analisis TAM ke `ATC_*`/`PSC_*` | — | ⏳ |

---

## 7. Status evidence tiap klaim

| Klaim | Status | Bukti |
|---|---|---|
| `pub_grip` nol `.publish()` | **VERIFIED** | 1 hit grep, `mission_fsm.py:256` |
| Skor m1=15 tanpa decode, wall dari `random.shuffle` | **VERIFIED** | `:328-329`, `:696-707` |
| GRAB tidak memvalidasi grasp | **VERIFIED** | docstring `_st_grab` |
| Navigasi APPROACH_QR pakai ground truth | **VERIFIED** | `:487-491`, `:533`, `:563` |
| `/hydroships/imu` nol subscriber | **VERIFIED** | hanya `bridge.yaml:26-27` |
| Rasio vision vs combined 3–4× | **VERIFIED** | [P1-0-FASE0](P1-0-FASE0-VISION-ATTRIBUTION.md) §3, 3 battery |
| Bug axis GUI-ROV (R-2) | **INFERRED** | belum di-grep langsung |
| Thrust tersendat 0.5 s saat GUI live | **INFERRED** (analisis statis) | butuh uji runtime |

## 8. Yang TIDAK diubah

`mission_fsm.py`, `stabilizer.py`, `thruster_allocator.py`/`allocation.py`,
`qr_detector.py`/`qr_logic.py`, URDF, `rov_params.yaml`, `kki_arena.sdf`, launch, parameter,
dan **seluruh isi repo `GUI-ROV`** (read-only sepenuhnya; `git status` di sana tetap bersih).
