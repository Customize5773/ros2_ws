# Graph Report - ros2_ws  (2026-07-28)

## Corpus Check
- 51 files · ~100,682 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 477 nodes · 727 edges · 31 communities (23 shown, 8 thin omitted)
- Extraction: 87% EXTRACTED · 13% INFERRED · 0% AMBIGUOUS · INFERRED: 97 edges (avg confidence: 0.78)
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `99d990a8`
- Run `git rev-parse HEAD` and compare to check if the graph is stale.
- Run `graphify update .` after code changes (no API cost).

## Community Hubs (Navigation)
- MissionFSM
- GuiBridgeLogic
- GripperLogic
- PID
- build_allocation_matrix
- QRDetector
- hook_servo
- test_qr_logic.py
- PROBLEM.md — Catatan Masalah & Verifikasi Tertunda (HYDROships ros2_ws)
- hook_detector.py
- TeleopStabilized
- TeleopKeyboard
- estimate_mass_inertia.py
- Integrasi GUI-ROV ↔ hydroships (ROS 2) — Analisis Selisih & Adapter
- DepthPublisher
- test_qr_ey_target.py
- sim.launch.py
- PayloadSpawner
- hydroships_gui.launch.py
- hydroships_mission.launch.py
- hydroships_sim.launch.py
- hydroships_stabilized.launch.py
- teleop.launch.py
- CLAUDE.md
- README.md
- ros2-ws
- 4.7 Sub-Kategori Remotely Operated Underwater Vehicle (ROV)

## God Nodes (most connected - your core abstractions)
1. `MissionFSM` - 32 edges
2. `GripperLogic` - 29 edges
3. `GuiBridgeLogic` - 18 edges
4. `PID` - 15 edges
5. `hook_servo()` - 14 edges
6. `GripperController` - 12 edges
7. `_fresh()` - 12 edges
8. `CHANGELOG — Riwayat Kronologis HYDROships (KKI 2026)` - 11 edges
9. `build_allocation_matrix()` - 10 edges
10. `GuiBridge` - 10 edges

## Surprising Connections (you probably didn't know these)
- `test_tam_full_rank()` --calls--> `build_allocation_matrix()`  [INFERRED]
  src/hydroships_control/test/test_allocation.py → src/hydroships_control/hydroships_control/allocation.py
- `GripperController` --uses--> `GripperLogic`  [INFERRED]
  src/hydroships_control/hydroships_control/gripper_controller.py → src/hydroships_control/hydroships_control/gripper_logic.py
- `test_jaw_targets_within_urdf_joint_limits()` --calls--> `GripperLogic`  [INFERRED]
  src/hydroships_control/test/test_gripper.py → src/hydroships_control/hydroships_control/gripper_logic.py
- `test_no_offset_not_safe()` --calls--> `GripperLogic`  [INFERRED]
  src/hydroships_control/test/test_gripper.py → src/hydroships_control/hydroships_control/gripper_logic.py
- `test_open_without_attach_no_detach()` --calls--> `GripperLogic`  [INFERRED]
  src/hydroships_control/test/test_gripper.py → src/hydroships_control/hydroships_control/gripper_logic.py

## Import Cycles
- None detected.

## Communities (31 total, 8 thin omitted)

### Community 0 - "MissionFSM"
Cohesion: 0.12
Nodes (15): MissionFSM, Node, Non-holonomik: putar dulu menghadap target, baru maju (surge saja,         tanpa, PD posisi HOLONOMIK: dorong ROV ke (tx,ty) dunia via gaya horizontal         bod, Offset QR di frame kamera. qr_detector menerbitkan utk kamera BAWAH         maup, Pose payload sebenarnya dari spawner (payload di-random tiap run).         Tanpa, Metrik alignment sesungguhnya: jarak XY GRIPPER (bukan base_link) ke QR., Misi 1: dekati payload holonomik (tanpa terikat heading, cegah         osilasi s (+7 more)

### Community 1 - "GuiBridgeLogic"
Cohesion: 0.07
Nodes (28): GuiBridge, clamp(), GuiBridgeLogic, _num(), gui_bridge_logic — inti terjemahan GUI-ROV <-> ROS 2 (murni, tanpa ROS/UDP).  Re, yaw REP-103 (rad, CCW dari +x) -> heading GUI (derajat 0..360)., Susun dict telemetri utk GUI (JSON). Nilai None -> 0 agar GUI aman., Terjemahan stateless-ish GUI<->ROS. Simpan axis manual terakhir & status.      G (+20 more)

### Community 2 - "GripperLogic"
Cohesion: 0.10
Nodes (28): GripperLogic, gripper_logic — inti keputusan manipulator ROV (murni Python, tanpa ROS).  Dipis, Paksa lepas tanpa perintah (mis. saat shutdown/abort)., Aksi auto-detach saat node START.          gz-sim Fortress SELALU meng-attach De, Mesin keputusan gripper. Semua waktu (``now``, ``stamp``) dalam detik.      Para, Simpan sinyal visual servo terbaru (dari /hydroships/qr_offset)., True bila payload ada di jangkauan aman untuk di-attach:         offset kecil (R, Proses perintah semantik. Kembalikan dict aksi tingkat-rendah:             {'jaw (+20 more)

### Community 3 - "PID"
Cohesion: 0.08
Nodes (21): PID, Bungkus sudut (rad) ke rentang [-pi, pi]., Hitung output kendali dari error & pengukuran saat ini., wrap_to_pi(), main(), Float64, Node, Odometry (+13 more)

### Community 4 - "build_allocation_matrix"
Cohesion: 0.10
Nodes (24): allocate(), build_allocation_matrix(), build_damped_pinv(), Kembalikan TAM 6xN: kolom i = [axis_i ; pos_i x axis_i]., Pseudo-inverse teredam (damped least-squares / Tikhonov).          pinv_damped =, Peta wrench body 6-DOF -> gaya per thruster (N), sudah di-clip., main(), Node (+16 more)

### Community 5 - "QRDetector"
Cohesion: 0.22
Nodes (6): Empty, GripperController, main(), Node, gripper_controller — node manipulator ROV (rancang ulang M5, DetachableJoint)., String

### Community 6 - "hook_servo"
Cohesion: 0.12
Nodes (23): _clamp(), hook_servo(), HookServoGains, normalize_hook_offset(), hook_logic — helper murni deteksi/servo hook (tanpa ROS/cv2), agar testable.  Di, (center px, area px^2, ukuran frame) -> (ex, ey, size) ternormalisasi.      Konv, Gain PD visual-servo APPROACH_HOOK (holonomik: sway+surge+depth-setpoint)., PD visual servo hook -> perintah gerak (fungsi MURNI, testable).      Args: (+15 more)

### Community 7 - "test_qr_logic.py"
Cohesion: 0.08
Nodes (30): CameraInfo, PointStamped, main(), Image, Node, QRDetector, qr_detector — deteksi QR dari kamera → sisi kolam A/B/C/D + offset piksel (M3)., _candidates() (+22 more)

### Community 8 - "PROBLEM.md — Catatan Masalah & Verifikasi Tertunda (HYDROships ros2_ws)"
Cohesion: 0.12
Nodes (17): 2026-07-07, 2026-07-08, 2026-07-11, 2026-07-12, 2026-07-14, 2026-07-15 … 07-16, 2026-07-17, 2026-07-18 (+9 more)

### Community 9 - "hook_detector.py"
Cohesion: 0.10
Nodes (26): _best_contour(), detect_hook(), HookDetector, main(), Image, Node, hook_detector — deteksi hook (pipa-U) dari kamera depan -> offset (visual servo), Deteksi hook -> (center, area) atau None. Jenjang: contour/CLAHE lalu Hough. (+18 more)

### Community 10 - "TeleopStabilized"
Cohesion: 0.35
Nodes (4): get_key(), main(), Node, TeleopStabilized

### Community 11 - "TeleopKeyboard"
Cohesion: 0.31
Nodes (5): get_key(), main(), Node, Baca satu karakter dari stdin (non-canonical)., TeleopKeyboard

### Community 12 - "estimate_mass_inertia.py"
Cohesion: 0.38
Nodes (9): box_inertia(), build_components(), combine(), format_yaml(), main(), _parse_args(), Tensor inertia kotak pejal (di pusatnya), massa seragam.      Ixx = m/12 (sy^2 +, Gabungkan daftar komponen -> (massa_total, cog, inertia_full_di_cog).      compo (+1 more)

### Community 13 - "Integrasi GUI-ROV ↔ hydroships (ROS 2) — Analisis Selisih & Adapter"
Cohesion: 0.06
Nodes (30): Arsitektur Simulasi HYDROships (KKI 2026), Diagram aliran (Milestone 1–2), File & model legacy, Keputusan desain, Kontrak interface topic (untuk GUI / GCS tim), Paket, 1. Temuan utama: GUI-ROV bukan ROS 2, 2. Tabel selisih antarmuka (+22 more)

### Community 14 - "DepthPublisher"
Cohesion: 0.29
Nodes (5): DepthPublisher, main(), Node, Odometry, depth_publisher — turunkan KEDALAMAN ROV dari odometry (Milestone 3).  Di simula

### Community 15 - "test_qr_ey_target.py"
Cohesion: 0.10
Nodes (24): Enum, main(), qr_ey_target(), Offset vertikal ternormalisasi tempat QR HARUS tampak di kamera bawah.      Grip, St, yaw_from_quaternion(), ey(), Uji qr_ey_target: koreksi offset kamera bawah -> gripper (APPROACH_QR).  Kamera (+16 more)

### Community 17 - "sim.launch.py"
Cohesion: 0.29
Nodes (9): _f(), generate_launch_description(), _launch_setup(), Launch simulasi Gazebo Fortress + spawn ROV HYDROships + ros_gz_bridge.  Argumen, Ambil LaunchConfiguration sbg float; fallback ke default bila kosong/invalid., Kembalikan (x, y, z, yaw) string utk spawn ROV.      rov_random_spawn=true -> ac, RNG utk pose spawn. spawn_seed diisi -> reproducible (replay/debug);     kosong, _rov_spawn_pose() (+1 more)

### Community 18 - "PayloadSpawner"
Cohesion: 0.39
Nodes (3): main(), PayloadSpawner, Node

### Community 32 - "4.7 Sub-Kategori Remotely Operated Underwater Vehicle (ROV)"
Cohesion: 0.18
Nodes (10): 4.7.1 Deskripsi dan Misi, 4.7.2 Ketentuan Teknis Prototipe ROV, 4.7.3 Sistem Kontes dan Lintasan ROV, 4.7.4 Penilaian dan Penentuan Pemenang ROV, 4.7.5 Penilaian Proposal ROV, 4.7.6 Penilaian Laporan Kemajuan ROV, 4.7 Sub-Kategori Remotely Operated Underwater Vehicle (ROV), 4.8 Sistem Penilaian Kategori Prototipe (+2 more)

## Knowledge Gaps
- **47 isolated node(s):** `ros2-ws`, `graphify`, `PROBLEM.md — pindah ke docs/`, `Status Milestone`, `Instalasi Dependensi` (+42 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **8 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `GripperController` connect `QRDetector` to `GripperLogic`?**
  _High betweenness centrality (0.091) - this node is a cross-community bridge._
- **Why does `MissionFSM` connect `MissionFSM` to `hook_servo`, `test_qr_ey_target.py`?**
  _High betweenness centrality (0.081) - this node is a cross-community bridge._
- **Why does `GripperLogic` connect `GripperLogic` to `QRDetector`?**
  _High betweenness centrality (0.077) - this node is a cross-community bridge._
- **Are the 19 inferred relationships involving `GripperLogic` (e.g. with `GripperController` and `.__init__()`) actually correct?**
  _`GripperLogic` has 19 INFERRED edges - model-reasoned connections that need verification._
- **Are the 11 inferred relationships involving `GuiBridgeLogic` (e.g. with `GuiBridge` and `.__init__()`) actually correct?**
  _`GuiBridgeLogic` has 11 INFERRED edges - model-reasoned connections that need verification._
- **Are the 9 inferred relationships involving `PID` (e.g. with `Stabilizer` and `.__init__()`) actually correct?**
  _`PID` has 9 INFERRED edges - model-reasoned connections that need verification._
- **Are the 11 inferred relationships involving `hook_servo()` (e.g. with `test_centered_far_moves_forward_only()` and `test_convergence_reduces_error_over_iterations()`) actually correct?**
  _`hook_servo()` has 11 INFERRED edges - model-reasoned connections that need verification._