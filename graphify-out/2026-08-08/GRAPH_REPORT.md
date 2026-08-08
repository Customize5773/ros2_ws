# Graph Report - ros2_ws  (2026-08-07)

## Corpus Check
- 59 files · ~114,337 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 610 nodes · 913 edges · 40 communities (31 shown, 9 thin omitted)
- Extraction: 99% EXTRACTED · 1% INFERRED · 0% AMBIGUOUS · INFERRED: 11 edges (avg confidence: 0.64)
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `af6a7855`
- Run `git rev-parse HEAD` and compare to check if the graph is stale.
- Run `graphify update .` after code changes (no API cost).

## Community Hubs (Navigation)
- MissionFSM
- GuiBridgeLogic
- GripperLogic
- PID
- test_allocation.py
- GripperController
- hook_servo
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
- TeleopGamepad
- hydroships_gui.launch.py
- hydroships_mission.launch.py
- hydroships_sim.launch.py
- hydroships_stabilized.launch.py
- teleop.launch.py
- CLAUDE.md
- meshes/README.md
- ros2-ws
- 1. Tuning PID `stabilizer` (`config/gains.yaml`)
- 4.7 Sub-Kategori Remotely Operated Underwater Vehicle (ROV)
- 4. Launch Arguments
- CI/CD
- Deployment Checklist — Hari-H Kontes
- Troubleshooting
- Performance — Metrik Harapan & Cara Mengukur
- Hardware — Status & Gap Analysis
- teleop_gamepad.launch.py

## God Nodes (most connected - your core abstractions)
1. `MissionFSM` - 35 edges
2. `GripperLogic` - 31 edges
3. `GuiBridgeLogic` - 20 edges
4. `hook_servo()` - 17 edges
5. `PID` - 17 edges
6. `TeleopGamepad` - 16 edges
7. `CHANGELOG — Riwayat Kronologis HYDROships (KKI 2026)` - 13 edges
8. `build_allocation_matrix()` - 12 edges
9. `GripperController` - 12 edges
10. `_fresh()` - 12 edges

## Surprising Connections (you probably didn't know these)
- `GripperController` --uses--> `GripperLogic`  [INFERRED]
  src/hydroships_control/hydroships_control/gripper_controller.py → src/hydroships_control/hydroships_control/gripper_logic.py
- `MissionFSM` --uses--> `HookServoGains`  [INFERRED]
  src/hydroships_control/hydroships_control/mission_fsm.py → src/hydroships_control/hydroships_control/hook_logic.py
- `test_tam_full_rank()` --calls--> `build_allocation_matrix()`  [EXTRACTED]
  src/hydroships_control/test/test_allocation.py → src/hydroships_control/hydroships_control/allocation.py
- `test_jaw_targets_within_urdf_joint_limits()` --calls--> `GripperLogic`  [EXTRACTED]
  src/hydroships_control/test/test_gripper.py → src/hydroships_control/hydroships_control/gripper_logic.py
- `test_no_offset_not_safe()` --calls--> `GripperLogic`  [EXTRACTED]
  src/hydroships_control/test/test_gripper.py → src/hydroships_control/hydroships_control/gripper_logic.py

## Import Cycles
- None detected.

## Communities (40 total, 9 thin omitted)

### Community 0 - "MissionFSM"
Cohesion: 0.11
Nodes (18): MissionFSM, Node, Non-holonomik: putar dulu menghadap target, baru maju (surge saja, tanpa sway).…, PD posisi HOLONOMIK: dorong ROV ke (tx,ty) dunia via gaya horizontal body-frame…, Offset QR di frame kamera. qr_detector menerbitkan utk kamera BAWAH maupun…, Offset hook di frame kamera DEPAN dari hook_detector: x=ex ternormalisasi (+ =…, (ex, ey, size) bila deteksi hook masih segar, else None. hook_detector hanya…, Pose payload sebenarnya dari spawner (payload di-random tiap run). Tanpa ini… (+10 more)

### Community 1 - "GuiBridgeLogic"
Cohesion: 0.06
Nodes (34): GuiBridge, clamp(), GuiBridgeLogic, _num(), gui_bridge_logic — inti terjemahan GUI-ROV <-> ROS 2 (murni, tanpa ROS/UDP).…, yaw REP-103 (rad, CCW dari +x) -> heading GUI (derajat 0..360)., Susun dict telemetri utk GUI (JSON). Nilai None -> 0 agar GUI aman., Terjemahan stateless-ish GUI<->ROS. Simpan axis manual terakhir & status. Gain… (+26 more)

### Community 2 - "GripperLogic"
Cohesion: 0.10
Nodes (28): GripperLogic, gripper_logic — inti keputusan manipulator ROV (murni Python, tanpa ROS).…, Paksa lepas tanpa perintah (mis. saat shutdown/abort)., Aksi auto-detach saat node START. gz-sim Fortress SELALU meng-attach…, Mesin keputusan gripper. Semua waktu (``now``, ``stamp``) dalam detik.…, Simpan sinyal visual servo terbaru (dari /hydroships/qr_offset)., True bila payload ada di jangkauan aman untuk di-attach: offset kecil (ROV…, Proses perintah semantik. Kembalikan dict aksi tingkat-rendah: {'jaw': <sudut… (+20 more)

### Community 3 - "PID"
Cohesion: 0.09
Nodes (23): PID, Bungkus sudut (rad) ke rentang [-pi, pi]., Hitung output kendali dari error & pengukuran saat ini., wrap_to_pi(), main(), Float64, Node, Odometry (+15 more)

### Community 4 - "test_allocation.py"
Cohesion: 0.12
Nodes (24): allocate(), build_allocation_matrix(), build_damped_pinv(), Kembalikan TAM 6xN: kolom i = [axis_i ; pos_i x axis_i]., Pseudo-inverse teredam (damped least-squares / Tikhonov). pinv_damped = TAM^T…, Peta wrench body 6-DOF -> gaya per thruster (N), sudah di-clip., main(), Node (+16 more)

### Community 5 - "GripperController"
Cohesion: 0.14
Nodes (9): Empty, GripperController, main(), Node, String, gripper_controller — node manipulator ROV (rancang ulang M5, DetachableJoint).…, main(), PayloadSpawner (+1 more)

### Community 6 - "hook_servo"
Cohesion: 0.14
Nodes (21): Enum, _clamp(), hook_servo(), HookServoGains, hook_logic — helper murni deteksi/servo hook (tanpa ROS/cv2), agar testable.…, Gain PD visual-servo APPROACH_HOOK (holonomik: sway+surge+depth-setpoint).…, PD visual servo hook -> perintah gerak (fungsi MURNI, testable). Args: off :…, main() (+13 more)

### Community 7 - "test_qr_logic.py"
Cohesion: 0.09
Nodes (30): CameraInfo, PointStamped, main(), Image, Node, QRDetector, qr_detector — deteksi QR dari kamera → sisi kolam A/B/C/D + offset piksel (M3).…, _candidates() (+22 more)

### Community 8 - "CHANGELOG — Riwayat Kronologis HYDROships (KKI 2026)"
Cohesion: 0.11
Nodes (19): 2026-07-07, 2026-07-08, 2026-07-11, 2026-07-12, 2026-07-14, 2026-07-15 … 07-16, 2026-07-17, 2026-07-18 (+11 more)

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
Cohesion: 0.06
Nodes (30): Arsitektur Simulasi HYDROships (KKI 2026), Diagram aliran (Milestone 1–2), File & model legacy, Keputusan desain, Kontrak interface topic (untuk GUI / GCS tim), Paket, 1. Temuan utama: GUI-ROV bukan ROS 2, 2. Tabel selisih antarmuka (+22 more)

### Community 14 - "DepthPublisher"
Cohesion: 0.29
Nodes (5): DepthPublisher, main(), Node, Odometry, depth_publisher — turunkan KEDALAMAN ROV dari odometry (Milestone 3). Di…

### Community 15 - "test_qr_ey_target.py"
Cohesion: 0.13
Nodes (20): qr_ey_target(), Offset vertikal ternormalisasi tempat QR HARUS tampak di kamera bawah. Gripper…, ey(), Uji qr_ey_target: koreksi offset kamera bawah -> gripper (APPROACH_QR). Kamera…, TAN harus = tan(atan(0.75 * tan(40°))) utk sensor 640x480, hFOV 80°., Depth operasional: h_cam=0.414, ½-tinggi=0.261 -> ey=-0.61 (aman)., Bukti kenapa scan_depth harus dinaikkan. Pada 0.46, h_cam=0.254 ->…, Konvensi offset_from_points: ey<0 = QR di ATAS pusat = payload di DEPAN. (+12 more)

### Community 16 - "Node ROS2 (punya entry point di `setup.py`, dijalankan via `ros2 run`/launch)"
Cohesion: 0.08
Nodes (23): Cakupan Test, `depth_publisher` — `depth_publisher.py`, Diagram alur, Enum state (`class St(Enum)`), `gripper_controller` — `gripper_controller.py`, `gui_bridge` — `gui_bridge.py`, `hook_detector` — `hook_detector.py`, Konstanta tetap (+15 more)

### Community 17 - "sim.launch.py"
Cohesion: 0.29
Nodes (9): _f(), generate_launch_description(), _launch_setup(), Launch simulasi Gazebo Fortress + spawn ROV HYDROships + ros_gz_bridge.…, Ambil LaunchConfiguration sbg float; fallback ke default bila kosong/invalid., Kembalikan (x, y, z, yaw) string utk spawn ROV. rov_random_spawn=true -> acak…, RNG utk pose spawn. spawn_seed diisi -> reproducible (replay/debug); kosong…, _rov_spawn_pose() (+1 more)

### Community 18 - "TeleopGamepad"
Cohesion: 0.14
Nodes (9): Joy, Ekstrak yaw (rad) dari geometry_msgs/Quaternion., yaw_from_quaternion(), main(), Node, Odometry, Kunci setpoint ke keadaan saat ini supaya perpindahan mode bumpless. Tanpa ini,…, Nilai axis setelah deadzone, expo, dan inversi. (+1 more)

### Community 31 - "1. Tuning PID `stabilizer` (`config/gains.yaml`)"
Cohesion: 0.13
Nodes (14): 1. Tuning PID `stabilizer` (`config/gains.yaml`), 2. Tuning `mission_fsm` — Timeout per state, 3. Tuning Visual Servo (QR approach & Hook approach), 3a. `APPROACH_QR` — servo ke QR code, 3b. `APPROACH_HOOK` — servo ke hook dinding (dipakai `hook_logic.hook_servo`), 4. Tuning Thruster Allocation (`alloc_damping`), 5. Urutan re-tuning setelah pindah ke hardware fisik, Catatan khusus `buoyancy_ff` (+6 more)

### Community 32 - "4.7 Sub-Kategori Remotely Operated Underwater Vehicle (ROV)"
Cohesion: 0.18
Nodes (10): 4.7.1 Deskripsi dan Misi, 4.7.2 Ketentuan Teknis Prototipe ROV, 4.7.3 Sistem Kontes dan Lintasan ROV, 4.7.4 Penilaian dan Penentuan Pemenang ROV, 4.7.5 Penilaian Proposal ROV, 4.7.6 Penilaian Laporan Kemajuan ROV, 4.7 Sub-Kategori Remotely Operated Underwater Vehicle (ROV), 4.8 Sistem Penilaian Kategori Prototipe (+2 more)

### Community 33 - "4. Launch Arguments"
Cohesion: 0.15
Nodes (12): 1. `src/hydroships_description/config/rov_params.yaml`, 2. `src/hydroships_control/config/gains.yaml`, 3. `src/hydroships_gazebo/config/bridge.yaml`, 4. Launch Arguments, 5. Parameter node `mission_fsm` (declare_parameter, bukan file YAML terpisah), Config Reference — YAML & Launch Arguments, `src/hydroships_bringup/launch/hydroships_gui.launch.py` (M7, GUI bridge), `src/hydroships_bringup/launch/hydroships_mission.launch.py` (M6, full autonomy) (+4 more)

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

## Knowledge Gaps
- **122 isolated node(s):** `ros2-ws`, `graphify`, `PROBLEM.md — pindah ke docs/`, `Status Milestone`, `Instalasi Dependensi` (+117 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **9 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `GripperController` connect `GripperController` to `GripperLogic`?**
  _High betweenness centrality (0.057) - this node is a cross-community bridge._
- **Why does `MissionFSM` connect `MissionFSM` to `hook_servo`?**
  _High betweenness centrality (0.050) - this node is a cross-community bridge._
- **Why does `GripperLogic` connect `GripperLogic` to `GripperController`?**
  _High betweenness centrality (0.049) - this node is a cross-community bridge._
- **What connects `ros2-ws`, `graphify`, `PROBLEM.md — pindah ke docs/` to the rest of the system?**
  _122 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `MissionFSM` be split into smaller, more focused modules?**
  _Cohesion score 0.10638297872340426 - nodes in this community are weakly interconnected._
- **Should `GuiBridgeLogic` be split into smaller, more focused modules?**
  _Cohesion score 0.06294326241134751 - nodes in this community are weakly interconnected._
- **Should `GripperLogic` be split into smaller, more focused modules?**
  _Cohesion score 0.09672830725462304 - nodes in this community are weakly interconnected._