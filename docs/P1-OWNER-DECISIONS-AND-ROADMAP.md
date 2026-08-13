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
| **R-8** | Sim terlalu ideal: nol noise, latency, dropout | Tambah **hanya** yang punya test objective: noise MS5837, dropout kamera, latency tether |
| **R-9** | `_st_grab` tak punya ack dari `gripper_controller`: skor `m2=15` & transisi ke `NAV_WALL` diberikan setelah `hold_settle_s` **tanpa menunggu konfirmasi `is_safe()`/attach** | ✅ **DITUTUP 2026-08-13.** Topic `/hydroships/gripper/status` ternyata **sudah ada & sudah di-subscribe** (`mission_fsm.py`), cuma disimpan tanpa dipakai — bukan pelanggaran batasan "tanpa topic baru", diff jadi kecil: `_st_grab` kini menunggu `'attached'`/`'rejected'`, retry publish "close" saat `'rejected'`, timeout ke `ABORT` bila tak pernah `'attached'`. **Terbukti runtime langsung** di battery 3-seed (`R9-3001/3002/3003`, `docs/CHANGELOG.md` 2026-08-13): seed 3002 GRAB ditolak terus (`x=+0.799` vs `max_offset=0.30`) → **ABORT jujur**, bukan lagi skor m2=15 palsu seperti sebelum perbaikan ini. |
| **R-10** | Margin gerbang fisik attach tinggal **5–7 mm**: `alt_gap` terukur 0.073/0.074/0.075 m vs `max_alt_gap=0.08` (3/3 run) | Bukan kegagalan, tapi rapuh. Sebabnya kriteria keluar DESCEND `depth >= grab_depth - depth_tol` melepas pada ~0.66 m padahal `grab_depth=0.70` — `depth_tol=0.06` memakan hampir seluruh margin rancangan (celah rancangan 0.034 m). Opsi (belum diputuskan): toleransi keluar DESCEND yang lebih ketat, atau `max_alt_gap` dinaikkan sesuai `depth_tol`. **Jangan diubah tanpa run pembanding** |
| **R-11** | *(baru, 2026-08-13)* Seed 3002 (battery 4-hook ×3): offset lateral `x` GRAB tak pernah masuk gerbang (`x=+0.799` vs `max_offset=0.30`, `ok=False` di semua sample ~16s) → attach ditolak terus, `armed=False` (latch APPROACH_QR tak pernah kena) | Kemungkinan presisi centering `APPROACH_QR`/`DESCEND` untuk posisi spawn tertentu — bukan bug R-9 (gerbangnya justru bekerja benar, menolak dgn tepat). **Belum didiagnosis**; butuh run berulang seed 3002 (dan seed lain dgn payload di kuadran serupa) utk lihat apakah ini konsisten reproducible atau kebetulan 1 seed. **Jangan diubah tanpa data lebih banyak.** |

---

## 6. Roadmap

Urutan mengikuti keputusan "P0 dulu". Setiap fase punya exit criteria observable.

| Fase | Isi | Exit criteria | Status |
|---|---|---|---|
| **0** | Pisahkan pelaporan vision vs ground truth di reducer | Gate 4 dilaporkan 2 populasi terpisah dengan n masing-masing | ✅ **SELESAI** `e071667` |
| **1** | Tutup blocker P0: P0-B (ukur dulu) → P0-A → P0-C | Siklus 4-hook end-to-end ≥3× berturut, seed berbeda, kontribusi vision dilaporkan terpisah | ⏳ **2/3** (`R9-3001`/`R9-3003` DONE 100/100; `R9-3002` ABORT jujur karena R-11 — lihat §5) |
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
