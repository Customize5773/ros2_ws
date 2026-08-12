# Graph Report - ros2_ws  (2026-08-12)

## Corpus Check
- 111 files · ~195,564 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 1196 nodes · 1642 edges · 82 communities (67 shown, 15 thin omitted)
- Extraction: 99% EXTRACTED · 1% INFERRED · 0% AMBIGUOUS · INFERRED: 16 edges (avg confidence: 0.67)
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `acb9cc9d`
- Run `git rev-parse HEAD` and compare to check if the graph is stale.
- Run `graphify update .` after code changes (no API cost).

## Community Hubs (Navigation)
- MissionFSM
- GuiBridgeLogic
- GripperLogic
- PID
- test_allocation.py
- PayloadSpawner
- ComplementaryFilter
- test_qr_logic.py
- CHANGELOG — Riwayat Kronologis HYDROships (KKI 2026)
- hook_detector.py
- TeleopStabilized
- TeleopKeyboard
- estimate_mass_inertia.py
- STATUS.md
- DepthPublisher
- test_qr_ey_target.py
- Node ROS2 (punya entry point di `setup.py`, dijalankan via `ros2 run`/launch)
- sim.launch.py
- P0-2.3 SPEC — Positioning/centering accuracy at GRAB (KKI 2026)
- hydroships_gui.launch.py
- hydroships_mission.launch.py
- hydroships_sim.launch.py
- hydroships_stabilized.launch.py
- teleop.launch.py
- P0-2.2 SPEC — QR-driven vs Ground-truth-driven APPROACH_QR (KKI 2026)
- meshes/README.md
- ros2-ws
- P0-1 BASELINE — Investigasi & Koreksi DIVE (KKI 2026)
- 4.7 Sub-Kategori Remotely Operated Underwater Vehicle (ROV)
- P0-2.4 SPEC — APPROACH_QR precision convergence retest (Gate 4) (KKI 2026)
- CI/CD
- Deployment Checklist — Hari-H Kontes
- Troubleshooting
- Performance — Metrik Harapan & Cara Mengukur
- Hardware — Status & Gap Analysis
- teleop_gamepad.launch.py
- analyze_qr_gate_correlation.py
- P0-2.3 SEPARATION EXPERIMENT SPEC — AABB/model error vs detection/decode quality (KKI 2026)
- P1-2A: Desain Estimasi Orientasi (Attitude) — `ros2_ws`
- analyze_qr_separation.py
- reduce_approach_qr.py
- P1-1: Architecture Decision Audit — ROS2-native vs. Pixhawk/ArduSub control authority
- P1-2A: Verifikasi Runtime IMU — Gate Sebelum Implementasi Estimator
- P1-0: Cross-Repo Architecture Audit — `ros2_ws` ↔ `GUI-ROV`
- P1-2: Audit Integrasi Estimasi State — `ros2_ws`
- P1.2B — Consumer-Integration Review for the P1.2A Orientation Estimator
- RecorderQR
- Recorder
- P0-2.8 — engineering review: does adaptive_thresh_denoised justify an ordering experiment?
- Driver
- P0-2.7 SPEC — failure-focused APPROACH_QR battery (design only, NOT run)
- P1 — Keputusan pemilik proyek, audit lintas-repo, dan roadmap
- P0-2.5 ENGINEERING ANALYSIS & DESIGN — why Gate 4 is 5/17 (KKI 2026)
- Integrasi GUI-ROV ↔ hydroships (ROS 2) — Analisis Selisih & Adapter
- P0-2.6 PERCEPTION PIPELINE AUDIT & DIAGNOSTIC ANALYSIS (KKI 2026)
- P0-2.4 RESULTS — Gate 4 convergence retest (KKI 2026)
- P0-2.5 Candidate 1 (`qr_servo_range=0.6`) Acceptance Report (KKI 2026)
- P0-2.5 Candidate 3 (`approach_min_fmax_frac=0.15`) Acceptance Report (KKI 2026)
- P0-2.5 Candidate 4 (`approach_dwell_ticks=3`) Acceptance Report (KKI 2026)
- P1 Fase 1 — Hasil: tutup blocker P0
- JoyMissionTrigger
- JoyTeleop
- Arsitektur Simulasi HYDROships (KKI 2026)
- P0-2.5 Candidate 2 (α=0.3) Acceptance Report (KKI 2026)
- P1-0 Fase 0 — Pemisahan evidence: vision vs ground truth
- Konfigurasi & Alokasi 6 Thruster — HYDROships
- VERIFICATION-CHECKLIST — Uji Runtime HYDROships
- select_failure_seeds.py
- ARENA ROV KKI 2026
- extract_gripper_err.py
- gate_mission.sh
- run_approach_qr_battery.sh
- run_approach_qr_smoke.sh
- run_mission.sh
- run_mission_cycle.sh
- run_qr_failure_battery.sh

## God Nodes (most connected - your core abstractions)
1. `GripperLogic` - 37 edges
2. `MissionFSM` - 36 edges
3. `ComplementaryFilter` - 22 edges
4. `P0-2.3 SPEC — Positioning/centering accuracy at GRAB (KKI 2026)` - 22 edges
5. `GuiBridgeLogic` - 20 edges
6. `hook_servo()` - 17 edges
7. `PID` - 17 edges
8. `TeleopGamepad` - 16 edges
9. `P0-2.3 SEPARATION EXPERIMENT SPEC — AABB/model error vs detection/decode quality (KKI 2026)` - 16 edges
10. `RecorderQR` - 15 edges

## Surprising Connections (you probably didn't know these)
- `goto_xy_predict()` --calls--> `clamp()`  [INFERRED]
  tools/p0-experiments/reduce_approach_qr.py → src/hydroships_control/hydroships_control/gui_bridge_logic.py
- `ey()` --calls--> `qr_ey_target()`  [INFERRED]
  src/hydroships_control/test/test_qr_ey_target.py → src/hydroships_control/hydroships_control/qr_logic.py
- `test_tanpa_offset_gripper_target_nol()` --calls--> `qr_ey_target()`  [INFERRED]
  src/hydroships_control/test/test_qr_ey_target.py → src/hydroships_control/hydroships_control/qr_logic.py
- `test_tam_full_rank()` --calls--> `build_allocation_matrix()`  [EXTRACTED]
  src/hydroships_control/test/test_allocation.py → src/hydroships_control/hydroships_control/allocation.py
- `AttitudeEstimator` --uses--> `ComplementaryFilter`  [INFERRED]
  src/hydroships_control/hydroships_control/attitude_estimator.py → src/hydroships_control/hydroships_control/attitude_filter_logic.py

## Import Cycles
- None detected.

## Communities (82 total, 15 thin omitted)

### Community 0 - "MissionFSM"
Cohesion: 0.08
Nodes (27): Enum, HookServoGains, Gain PD visual-servo APPROACH_HOOK (holonomik: sway+surge+depth-setpoint).…, main(), MissionFSM, Node, Non-holonomik: putar dulu menghadap target, baru maju (surge saja, tanpa sway).…, PD posisi HOLONOMIK: dorong ROV ke (tx,ty) dunia via gaya horizontal body-frame… (+19 more)

### Community 1 - "GuiBridgeLogic"
Cohesion: 0.05
Nodes (48): GuiBridge, GuiBridgeLogic, _num(), gui_bridge_logic — inti terjemahan GUI-ROV <-> ROS 2 (murni, tanpa ROS/UDP).…, yaw REP-103 (rad, CCW dari +x) -> heading GUI (derajat 0..360)., Susun dict telemetri utk GUI (JSON). Nilai None -> 0 agar GUI aman., Terjemahan stateless-ish GUI<->ROS. Simpan axis manual terakhir & status. Gain…, Proses satu pesan {name,value} GUI. Kembalikan dict aksi: {'wrench':… (+40 more)

### Community 2 - "GripperLogic"
Cohesion: 0.06
Nodes (44): Empty, GripperController, main(), Float64, Node, String, gripper_controller — node manipulator ROV (rancang ulang M5, DetachableJoint).…, GripperLogic (+36 more)

### Community 3 - "PID"
Cohesion: 0.05
Nodes (32): PID, Bungkus sudut (rad) ke rentang [-pi, pi]., Hitung output kendali dari error & pengukuran saat ini., wrap_to_pi(), main(), Float64, Node, Odometry (+24 more)

### Community 4 - "test_allocation.py"
Cohesion: 0.07
Nodes (37): allocate(), build_allocation_matrix(), build_damped_pinv(), Kembalikan TAM 6xN: kolom i = [axis_i ; pos_i x axis_i]., Pseudo-inverse teredam (damped least-squares / Tikhonov). pinv_damped = TAM^T…, Peta wrench body 6-DOF -> gaya per thruster (N), sudah di-clip., main(), Node (+29 more)

### Community 5 - "PayloadSpawner"
Cohesion: 0.39
Nodes (3): main(), PayloadSpawner, Node

### Community 6 - "ComplementaryFilter"
Cohesion: 0.06
Nodes (43): fixture, Imu, AttitudeEstimator, main(), Node, attitude_estimator — node ROS2 tipis di sekitar ComplementaryFilter (P1.2A).…, ComplementaryFilter, _is_finite() (+35 more)

### Community 7 - "test_qr_logic.py"
Cohesion: 0.09
Nodes (31): CameraInfo, PointStamped, main(), Image, Node, QRDetector, qr_detector — deteksi QR dari kamera → sisi kolam A/B/C/D + offset piksel (M3).…, P0-2.3: raw corner points + decode_success, SAME pts/data already computed for… (+23 more)

### Community 8 - "CHANGELOG — Riwayat Kronologis HYDROships (KKI 2026)"
Cohesion: 0.10
Nodes (20): 2026-07-07, 2026-07-08, 2026-07-11, 2026-07-12, 2026-07-14, 2026-07-15 … 07-16, 2026-07-17, 2026-07-18 (+12 more)

### Community 9 - "hook_detector.py"
Cohesion: 0.10
Nodes (26): _best_contour(), detect_hook(), HookDetector, main(), Image, Node, hook_detector — deteksi hook (pipa-U) dari kamera depan -> offset (visual…, Deteksi hook -> (center, area) atau None. Jenjang: contour/CLAHE lalu Hough.… (+18 more)

### Community 10 - "TeleopStabilized"
Cohesion: 0.35
Nodes (4): get_key(), main(), Node, TeleopStabilized

### Community 11 - "TeleopKeyboard"
Cohesion: 0.31
Nodes (5): get_key(), main(), Node, Baca satu karakter dari stdin (non-canonical)., TeleopKeyboard

### Community 12 - "estimate_mass_inertia.py"
Cohesion: 0.38
Nodes (9): box_inertia(), build_components(), combine(), format_yaml(), main(), _parse_args(), Tensor inertia kotak pejal (di pusatnya), massa seragam. Ixx = m/12 (sy^2 +…, Gabungkan daftar komponen -> (massa_total, cog, inertia_full_di_cog).… (+1 more)

### Community 13 - "STATUS.md"
Cohesion: 0.36
Nodes (3): Manipulator (M5) — status final (satu-satunya versi aktif), STATUS — Progres Milestone HYDROships (KKI 2026), PROBLEM.md — pindah ke docs/

### Community 14 - "DepthPublisher"
Cohesion: 0.29
Nodes (5): DepthPublisher, main(), Node, Odometry, depth_publisher — turunkan KEDALAMAN ROV dari odometry (Milestone 3). Di…

### Community 15 - "test_qr_ey_target.py"
Cohesion: 0.14
Nodes (18): ey(), Uji qr_ey_target: koreksi offset kamera bawah -> gripper (APPROACH_QR). Kamera…, TAN harus = tan(atan(0.75 * tan(40°))) utk sensor 640x480, hFOV 80°., Depth operasional: h_cam=0.414, ½-tinggi=0.261 -> ey=-0.61 (aman)., Bukti kenapa scan_depth harus dinaikkan. Pada 0.46, h_cam=0.254 ->…, Konvensi offset_from_points: ey<0 = QR di ATAS pusat = payload di DEPAN., Naik = petak pandang melebar = offset 0.16 m jadi fraksi frame lebih kecil., Sedalam apa pun, hasil tak pernah keluar frame. (+10 more)

### Community 16 - "Node ROS2 (punya entry point di `setup.py`, dijalankan via `ros2 run`/launch)"
Cohesion: 0.08
Nodes (23): Cakupan Test, `depth_publisher` — `depth_publisher.py`, Diagram alur, Enum state (`class St(Enum)`), `gripper_controller` — `gripper_controller.py`, `gui_bridge` — `gui_bridge.py`, `hook_detector` — `hook_detector.py`, Konstanta tetap (+15 more)

### Community 17 - "sim.launch.py"
Cohesion: 0.24
Nodes (11): _f(), generate_launch_description(), _launch_setup(), Launch simulasi Gazebo Fortress + spawn ROV HYDROships + ros_gz_bridge.…, RNG utk pose spawn. spawn_seed diisi -> reproducible (replay/debug); kosong…, Nama world dari ISI file SDF (<world name="...">), bukan dari nama file.…, Ambil LaunchConfiguration sbg float; fallback ke default bila kosong/invalid., Kembalikan (x, y, z, yaw) string utk spawn ROV. rov_random_spawn=true -> acak… (+3 more)

### Community 18 - "P0-2.3 SPEC — Positioning/centering accuracy at GRAB (KKI 2026)"
Cohesion: 0.05
Nodes (42): 1. Ringkasan evidence (rekap, bukan analisis baru), 2. Opsi-opsi, 3. Yang tidak berubah apa pun opsi yang dipilih, 4. Keputusan, Opsi A — CLOSE: "QR di dalam loop, presisi belum terbukti", Opsi B — CLOSE-PARTIAL: pisahkan pertanyaan acceptance, Opsi C — ITERATE: kumpulkan data lebih banyak sebelum memutuskan, Opsi D — REDESIGN: pertanyakan apakah `qr_center_tol` bar yang tepat (+34 more)

### Community 24 - "P0-2.2 SPEC — QR-driven vs Ground-truth-driven APPROACH_QR (KKI 2026)"
Cohesion: 0.06
Nodes (34): 10. Status P0-2 (update), 1. Mengapa menghitung transisi state saja tidak cukup, 2. Prinsip metodologis (dibawa dari P0-1 & diperkuat pengguna), 3. Enam gate → kriteria terukur, 4. P0-2.2a — instrumentation extension (CLOSED), 5. P0-2.2b — Rancangan run battery (dispesifikasi, belum dijalankan), 6. Dua prasyarat kecil sebelum eksekusi (ditemukan saat implementasi P0-2.2b), 7. P0-2.2b — Hasil eksekusi battery (EVIDENCE, bukan verdict) (+26 more)

### Community 31 - "P0-1 BASELINE — Investigasi & Koreksi DIVE (KKI 2026)"
Cohesion: 0.05
Nodes (38): 1. `src/hydroships_description/config/rov_params.yaml`, 2. `src/hydroships_control/config/gains.yaml`, 3. `src/hydroships_gazebo/config/bridge.yaml`, 4. Launch Arguments, 5. Parameter node `mission_fsm` (declare_parameter, bukan file YAML terpisah), Config Reference — YAML & Launch Arguments, `src/hydroships_bringup/launch/hydroships_gui.launch.py` (M7, GUI bridge), `src/hydroships_bringup/launch/hydroships_mission.launch.py` (M6, full autonomy) (+30 more)

### Community 32 - "4.7 Sub-Kategori Remotely Operated Underwater Vehicle (ROV)"
Cohesion: 0.18
Nodes (10): 4.7.1 Deskripsi dan Misi, 4.7.2 Ketentuan Teknis Prototipe ROV, 4.7.3 Sistem Kontes dan Lintasan ROV, 4.7.4 Penilaian dan Penentuan Pemenang ROV, 4.7.5 Penilaian Proposal ROV, 4.7.6 Penilaian Laporan Kemajuan ROV, 4.7 Sub-Kategori Remotely Operated Underwater Vehicle (ROV), 4.8 Sistem Penilaian Kategori Prototipe (+2 more)

### Community 33 - "P0-2.4 SPEC — APPROACH_QR precision convergence retest (Gate 4) (KKI 2026)"
Cohesion: 0.07
Nodes (26): 1. Tolerance/matrix acuan (dari `docs/P0-2-AUDIT.md`), 2.1 "ROV konvergen ke acceptance region?" (`dist < approach_tol` atau band `qr_center_tol`), 2.2 Penyebab residual bias (AABB/model error vs detection/decode quality), 2.3 `decode_success` sebagai correlate independen, 2.4 "Error relatif berkurang?" (tren dalam satu run), 2.5 "Tidak oscillatory/divergent?", 2. Kriteria per pertanyaan — PASS / FAIL / INCONCLUSIVE, 3. Rekonsiliasi: dukungan hipotesis vs acceptance (+18 more)

### Community 34 - "CI/CD"
Cohesion: 0.17
Nodes (11): 1. Proses build & test manual (kondisi saat ini), 2. Kenapa belum ada CI otomatis (analisis gap), 3. Rancangan pipeline CI yang disarankan, 4. Rekomendasi prioritas, Build, CI/CD, Dependensi sistem (prasyarat sebelum build/test bisa jalan), Tahap 1 — Lint + Unit Test (mudah, tidak butuh GPU, ROI tinggi) (+3 more)

### Community 35 - "Deployment Checklist — Hari-H Kontes"
Cohesion: 0.22
Nodes (8): Deployment Checklist — Hari-H Kontes, Fase Eksekusi Misi (10 menit), Fase Evakuasi (5 menit), Fase Persiapan (5 menit, saat giliran run dimulai), H-0, saat tiba di venue (sebelum giliran/slot waktu tim), H-1 (sehari sebelum, di penginapan/base camp), Pasca-run, Prasyarat software sebelum checklist ini bisa dipakai penuh di hari-H

### Community 36 - "Troubleshooting"
Cohesion: 0.22
Nodes (8): 1. Masalah umum & solusi cepat, 2. Metodologi debugging (untuk masalah yang tidak ada di tabel di atas), 3. Kasus dalam: QR tidak terdeteksi meski unit test lolos (KNOWN ISSUE), 4. Bug blocking aktif (per `docs/STATUS.md`, cek dulu apakah masih berlaku), 4a. Gripper tidak pernah benar-benar "close" saat state `GRAB`, 4b. `NAV_WALL` tidak konvergen ke `nav_tol`, 5. Kapan minta bantuan / eskalasi, Troubleshooting

### Community 37 - "Performance — Metrik Harapan & Cara Mengukur"
Cohesion: 0.25
Nodes (7): 1. Metrik waktu misi (mission timing), 2. Metrik komputasi (CPU/latency), 3. Metrik thrust/energi, 4. Metrik akurasi/keandalan, 5. Log pengukuran aktual, Performance — Metrik Harapan & Cara Mengukur, Referensi

### Community 38 - "Hardware — Status & Gap Analysis"
Cohesion: 0.29
Nodes (6): 1. Kenapa dokumen ini ada, 2. Peta komponen: Proposal (fisik) → Sim (ROS2) → Status kode, 3. Yang HARUS dibangun sebelum deploy ke ROV fisik, 4. Yang TIDAK perlu diubah (topic contract sudah hardware-agnostic), 5. Referensi silang, Hardware — Status & Gap Analysis

### Community 40 - "analyze_qr_gate_correlation.py"
Cohesion: 0.17
Nodes (25): candidate_name(), extract_observations(), h_cam_from_depth(), is_nan(), load_csv(), main(), quad_metrics(), Same as reduce_qr_precision.py:94-109 -- AABB-side / mean-edge inflation. (+17 more)

### Community 41 - "P0-2.3 SEPARATION EXPERIMENT SPEC — AABB/model error vs detection/decode quality (KKI 2026)"
Cohesion: 0.08
Nodes (24): 11. Review internal — hasil dan revisi (pass terpisah dari penulisan awal §1-10), 12. Status (superseded oleh §14 — battery separation sudah dieksekusi), 13. Eksekusi battery separation (stopping-rule, disetujui pengguna), 14. Hasil analisis separation — EVIDENCE, BUKAN VERDICT, 15. Status (menggantikan §12; verdict akhir di §16), 16. Final acceptance review — lihat `docs/P0-2-3-ACCEPTANCE-REVIEW.md`, 1. Pertanyaan yang belum terjawab, 2. Metode utama — perbandingan `decode_success` distratifikasi inflasi (+16 more)

### Community 42 - "P1-2A: Desain Estimasi Orientasi (Attitude) — `ros2_ws`"
Cohesion: 0.08
Nodes (23): 10. Asumsi belum terselesaikan, 11. Ruang lingkup implementasi (untuk ticket P1.2A masa depan), 1. Grafik dependensi saat ini, 2. Input yang dibutuhkan estimator — semantik pesan IMU, 3.1 Kontribusi accelerometer (roll/pitch dari vektor gravitasi), 3.2 Kontribusi gyro (dead-reckoning + koreksi frekuensi-tinggi), 3.3 Yaw — TIDAK ada magnetometer, drift tidak terhindarkan, 3.4 Inisialisasi (+15 more)

### Community 43 - "analyze_qr_separation.py"
Cohesion: 0.22
Nodes (21): bin_of(), collect_observations(), group_stats(), main(), pearson(), rotation_angle_deg(), squareness_and_angledev(), collect_observations() (+13 more)

### Community 44 - "reduce_approach_qr.py"
Cohesion: 0.17
Nodes (22): clamp(), analyze_run(), attribute_convergence(), classify_exit(), first_dwell(), fmt(), goto_xy_predict(), is_nan() (+14 more)

### Community 45 - "P1-1: Architecture Decision Audit — ROS2-native vs. Pixhawk/ArduSub control authority"
Cohesion: 0.12
Nodes (17): 10. P1.3 dependencies (telemetry/observability), 11. Migration risks, 12. Unresolved questions requiring human/project-owner decision, 1. Architecture reconstruction, 2. Evidence for intended target architecture, 3. `gui_bridge` role assessment, 4. Architecture decision matrix, 5. Dual-authority / safety risk analysis (+9 more)

### Community 46 - "P1-2A: Verifikasi Runtime IMU — Gate Sebelum Implementasi Estimator"
Cohesion: 0.12
Nodes (16): 1.1 Sampel mentah saat diam (init, ~t=13.9s sim time), 1.2 Beberapa sampel berurutan (steady-state, ~t=21.7-21.8s), 1.3 Rate publish aktual, 1.4 Uji rotasi — respons terhadap `angular_velocity.z`, 1.5 `/hydroships/odom` — pembanding (sekali, saat diam), 1. Bukti runtime IMU, 2. Consumer yang ada saat ini, 3. Determinasi viabilitas input estimator (+8 more)

### Community 47 - "P1-0: Cross-Repo Architecture Audit — `ros2_ws` ↔ `GUI-ROV`"
Cohesion: 0.12
Nodes (16): 10. Findings table, 11. Top integration risks, 12. Recommended next steps, 1. Executive architecture summary, 2. Repository/module responsibility map, 3. ROS interface inventory (`ros2_ws`), 4. `gui_bridge` boundary trace, 5. Parameter/configuration map (+8 more)

### Community 48 - "P1-2: Audit Integrasi Estimasi State — `ros2_ws`"
Cohesion: 0.12
Nodes (15): 10. Tabel blocker/risiko, 11. Usulan dekomposisi implementasi, 12. Dependensi P1.3, 13. Keputusan project-owner yang belum terjawab, 1. Ringkasan eksekutif, 2. Arsitektur estimasi state saat ini, 3. Inventaris sumber sensor/state, 4. Grafik dependensi loop kendali (+7 more)

### Community 49 - "P1.2B — Consumer-Integration Review for the P1.2A Orientation Estimator"
Cohesion: 0.12
Nodes (15): 1. Scope & inputs, 2. Ground truth source being replaced, 3.1 `stabilizer.py`, 3.2 `mission_fsm.py`, 3.3 `gui_bridge.py`, 3.4 `teleop_gamepad.py`, 3.5 Non-consumers (checked, ruled out), 3. Consumer inventory (+7 more)

### Community 50 - "RecorderQR"
Cohesion: 0.13
Nodes (3): main(), Node, RecorderQR

### Community 51 - "Recorder"
Cohesion: 0.20
Nodes (3): main(), Node, Recorder

### Community 52 - "P0-2.8 — engineering review: does adaptive_thresh_denoised justify an ordering experiment?"
Cohesion: 0.18
Nodes (10): 0. Why this review exists, 1. Code trace, 2. Evidence review (both existing batteries, no new data collected), 3. Verdict of this review, 4. Formulated experiment (single-variable, P0-2.5-style isolation), 5. Explicit hypothesis (falsifiable, stated before any run), 6. Acceptance metrics for the (future, separately approved) battery, 7. Explicit non-goals of this document (+2 more)

### Community 53 - "Driver"
Cohesion: 0.20
Nodes (3): Driver, main(), Node

### Community 54 - "P0-2.7 SPEC — failure-focused APPROACH_QR battery (design only, NOT run)"
Cohesion: 0.20
Nodes (9): 0. Why this spec exists, 1. Failure-prone spawn pose selection, 2. V1-class failure definition, 3. Minimum qualifying sample, 4. Stopping rule, 5. Tooling (written, not executed by this spec), 6. Explicit non-goals, 7. Status (+1 more)

### Community 55 - "P1 — Keputusan pemilik proyek, audit lintas-repo, dan roadmap"
Cohesion: 0.20
Nodes (10): 0. ⚠️ Konflik dengan P1-1 — baca ini lebih dulu, 1. Empat keputusan pemilik proyek (2026-08-12), 2. Arsitektur — apa yang sebenarnya menyeberang, 3. Temuan utama — autonomy di sim berjalan di atas ground truth, 4. Transfer map, 5. Gap & rekomendasi, 6. Roadmap, 7. Status evidence tiap klaim (+2 more)

### Community 56 - "P0-2.5 ENGINEERING ANALYSIS & DESIGN — why Gate 4 is 5/17 (KKI 2026)"
Cohesion: 0.22
Nodes (8): 0. Analisis tambahan (read-only) yang mendasari dokumen ini, A.1 — Dua mode kegagalan yang berbeda, bukan satu, A.2 — Yang secara eksplisit BUKAN penyebab (dikonfirmasi, bukan diasumsikan), A. Root Cause Trace, B. Candidate Changes (Measurable), C. Sequential Experiment Roadmap, P0-2.5 ENGINEERING ANALYSIS & DESIGN — why Gate 4 is 5/17 (KKI 2026), Status

### Community 57 - "Integrasi GUI-ROV ↔ hydroships (ROS 2) — Analisis Selisih & Adapter"
Cohesion: 0.25
Nodes (7): 1. Temuan utama: GUI-ROV bukan ROS 2, 2. Tabel selisih antarmuka, 3. Penanganan (tanpa mengubah node inti), 3a. Node adapter `gui_bridge` (hydroships_control), 3b. Node `hook_detector` (port GUI-ROV) untuk APPROACH_HOOK, 4. Yang BELUM (VERIFY/OPEN), Integrasi GUI-ROV ↔ hydroships (ROS 2) — Analisis Selisih & Adapter

### Community 58 - "P0-2.6 PERCEPTION PIPELINE AUDIT & DIAGNOSTIC ANALYSIS (KKI 2026)"
Cohesion: 0.25
Nodes (7): 0. Kenapa pass ini ada, 1. Re-audit evidence log P0-2.3 (dari angka terpublikasi, data mentah tak tersedia), 2. Trace coupling pipeline persepsi (`qr_detector.py` → `qr_logic.py` → `mission_fsm.py`), 3. Baseline observables, dihitung segar dari 17 run valid P0-2.4, 4. Kandidat perbaikan persepsi terisolasi (BELUM disetujui, hanya diformulasikan), 5. Status, P0-2.6 PERCEPTION PIPELINE AUDIT & DIAGNOSTIC ANALYSIS (KKI 2026)

### Community 59 - "P0-2.4 RESULTS — Gate 4 convergence retest (KKI 2026)"
Cohesion: 0.29
Nodes (6): 1. Tooling yang diimplementasikan, 2. Eksekusi battery, 3. Hasil per run (ringkasan), 4. Agregat dan verdict Gate 4, 5. Verdict akhir, P0-2.4 RESULTS — Gate 4 convergence retest (KKI 2026)

### Community 60 - "P0-2.5 Candidate 1 (`qr_servo_range=0.6`) Acceptance Report (KKI 2026)"
Cohesion: 0.29
Nodes (6): A. Comparative Summary Table, B. Physical Convergence vs. Metric Analysis, C. Recommendation & Next Step, P0-2.5 Candidate 1 (`qr_servo_range=0.6`) Acceptance Report (KKI 2026), Protocol deviation (reported, not hidden), Status

### Community 61 - "P0-2.5 Candidate 3 (`approach_min_fmax_frac=0.15`) Acceptance Report (KKI 2026)"
Cohesion: 0.29
Nodes (6): A. Comparative Summary Table, B. Physical Convergence vs. Metric Analysis, C. Recommendation & Next Step, Mandatory guardrail (this is the highest-risk candidate — checked after every batch), P0-2.5 Candidate 3 (`approach_min_fmax_frac=0.15`) Acceptance Report (KKI 2026), Status

### Community 62 - "P0-2.5 Candidate 4 (`approach_dwell_ticks=3`) Acceptance Report (KKI 2026)"
Cohesion: 0.29
Nodes (6): A. Results, B. Physical Convergence vs. Metric Analysis, C. Recommendation & Next Step, P0-2.5 Candidate 4 (`approach_dwell_ticks=3`) Acceptance Report (KKI 2026), Status, Why this candidate is evaluated differently

### Community 63 - "P1 Fase 1 — Hasil: tutup blocker P0"
Cohesion: 0.29
Nodes (7): 1. P0-B (NAV_WALL) — DITUTUP untuk kasus tanpa beban, tanpa perubahan kode, 2. P0-A (GRAB tak pernah "close") — tiga bug bertumpuk, 3. Siklus 4-hook end-to-end, 4. ⛔ BLOCKER BARU M5-D — ROV terjangkar begitu benar-benar mencengkeram, 5. Status evidence, 6. Catatan kontaminasi, P1 Fase 1 — Hasil: tutup blocker P0

### Community 64 - "JoyMissionTrigger"
Cohesion: 0.33
Nodes (4): JoyMissionTrigger, main(), Joy, Node

### Community 65 - "JoyTeleop"
Cohesion: 0.33
Nodes (4): JoyTeleop, main(), Joy, Node

### Community 66 - "Arsitektur Simulasi HYDROships (KKI 2026)"
Cohesion: 0.33
Nodes (6): Arsitektur Simulasi HYDROships (KKI 2026), Diagram aliran (Milestone 1–2), File & model legacy, Keputusan desain, Kontrak interface topic (untuk GUI / GCS tim), Paket

### Community 67 - "P0-2.5 Candidate 2 (α=0.3) Acceptance Report (KKI 2026)"
Cohesion: 0.33
Nodes (5): A. Comparative Summary Table, B. Physical Convergence vs. Metric Analysis, C. Recommendation & Next Step, P0-2.5 Candidate 2 (α=0.3) Acceptance Report (KKI 2026), Status

### Community 68 - "P1-0 Fase 0 — Pemisahan evidence: vision vs ground truth"
Cohesion: 0.33
Nodes (6): 1. Masalah, 2. Perubahan, 3. Hasil pada data battery yang ada, 4. Verifikasi, 5. Batasan, P1-0 Fase 0 — Pemisahan evidence: vision vs ground truth

### Community 69 - "Konfigurasi & Alokasi 6 Thruster — HYDROships"
Cohesion: 0.33
Nodes (5): Catatan penyetelan (menyusul), Konfigurasi & Alokasi 6 Thruster — HYDROships, Konvensi posisi & sumber data, Tabel thruster (sesuai `allocation.py`), Thrust Allocation Matrix (TAM)

### Community 70 - "VERIFICATION-CHECKLIST — Uji Runtime HYDROships"
Cohesion: 0.33
Nodes (6): Prioritas 1 — Persepsi dasar (banyak state FSM bergantung), Prioritas 2 — Manipulator (butuh persepsi + arena), Prioritas 3 — Autonomy & servo hook, Prioritas 4 — Integrasi GUI live (M7), Prioritas 5 — Validasi arena & hardware, VERIFICATION-CHECKLIST — Uji Runtime HYDROships

### Community 72 - "select_failure_seeds.py"
Cohesion: 0.60
Nodes (4): is_failure_prone(), main(), Exact replica of _rov_spawn_pose()'s RNG call order for rov_random_spawn=true…, spawn_pose_for_seed()

### Community 73 - "ARENA ROV KKI 2026"
Cohesion: 0.67
Nodes (3): ARENA ROV KKI 2026, Instalasi Dependensi, Status Milestone

## Knowledge Gaps
- **409 isolated node(s):** `ros2-ws`, `gate_mission.sh script`, `run_approach_qr_battery.sh script`, `run_approach_qr_smoke.sh script`, `run_mission.sh script` (+404 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **15 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `qr_ey_target()` connect `MissionFSM` to `GripperLogic`, `test_qr_ey_target.py`, `test_qr_logic.py`?**
  _High betweenness centrality (0.015) - this node is a cross-community bridge._
- **What connects `ros2-ws`, `gate_mission.sh script`, `run_approach_qr_battery.sh script` to the rest of the system?**
  _409 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `MissionFSM` be split into smaller, more focused modules?**
  _Cohesion score 0.08418079096045197 - nodes in this community are weakly interconnected._
- **Should `GuiBridgeLogic` be split into smaller, more focused modules?**
  _Cohesion score 0.050595238095238096 - nodes in this community are weakly interconnected._
- **Should `GripperLogic` be split into smaller, more focused modules?**
  _Cohesion score 0.056051587301587304 - nodes in this community are weakly interconnected._
- **Should `PID` be split into smaller, more focused modules?**
  _Cohesion score 0.054098360655737705 - nodes in this community are weakly interconnected._
- **Should `test_allocation.py` be split into smaller, more focused modules?**
  _Cohesion score 0.07439613526570048 - nodes in this community are weakly interconnected._