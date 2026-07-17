# Graph Report - ros2_ws  (2026-07-18)

## Corpus Check
- 50 files · ~98,467 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 469 nodes · 708 edges · 32 communities (23 shown, 9 thin omitted)
- Extraction: 88% EXTRACTED · 12% INFERRED · 0% AMBIGUOUS · INFERRED: 88 edges (avg confidence: 0.78)
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `0b6822a1`
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
- File yang Harus Diperbaiki/Dibuat
- File yang Harus Diperbaiki
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
- prompt-random-qr-payload.md

## God Nodes (most connected - your core abstractions)
1. `MissionFSM` - 40 edges
2. `GripperLogic` - 28 edges
3. `GuiBridgeLogic` - 18 edges
4. `hook_servo()` - 15 edges
5. `PID` - 15 edges
6. `GripperController` - 12 edges
7. `_fresh()` - 12 edges
8. `QRDetector` - 11 edges
9. `build_allocation_matrix()` - 10 edges
10. `GuiBridge` - 10 edges

## Surprising Connections (you probably didn't know these)
- `test_tam_full_rank()` --calls--> `build_allocation_matrix()`  [INFERRED]
  src/hydroships_control/test/test_allocation.py → src/hydroships_control/hydroships_control/allocation.py
- `GripperController` --uses--> `GripperLogic`  [INFERRED]
  src/hydroships_control/hydroships_control/gripper_controller.py → src/hydroships_control/hydroships_control/gripper_logic.py
- `test_no_offset_not_safe()` --calls--> `GripperLogic`  [INFERRED]
  src/hydroships_control/test/test_gripper.py → src/hydroships_control/hydroships_control/gripper_logic.py
- `test_open_without_attach_no_detach()` --calls--> `GripperLogic`  [INFERRED]
  src/hydroships_control/test/test_gripper.py → src/hydroships_control/hydroships_control/gripper_logic.py
- `test_stale_offset_not_safe()` --calls--> `GripperLogic`  [INFERRED]
  src/hydroships_control/test/test_gripper.py → src/hydroships_control/hydroships_control/gripper_logic.py

## Import Cycles
- None detected.

## Communities (32 total, 9 thin omitted)

### Community 0 - "MissionFSM"
Cohesion: 0.08
Nodes (23): Enum, HookServoGains, Gain PD visual-servo APPROACH_HOOK (holonomik: sway+surge+depth-setpoint)., main(), MissionFSM, Node, Kirim perintah manipulator: close=True -> 'close' (attach payload bila         d, PD posisi HOLONOMIK: dorong ROV ke (tx,ty) dunia via gaya horizontal         bod (+15 more)

### Community 1 - "GuiBridgeLogic"
Cohesion: 0.07
Nodes (28): GuiBridge, clamp(), GuiBridgeLogic, _num(), gui_bridge_logic — inti terjemahan GUI-ROV <-> ROS 2 (murni, tanpa ROS/UDP).  Re, yaw REP-103 (rad, CCW dari +x) -> heading GUI (derajat 0..360)., Susun dict telemetri utk GUI (JSON). Nilai None -> 0 agar GUI aman., Terjemahan stateless-ish GUI<->ROS. Simpan axis manual terakhir & status.      G (+20 more)

### Community 2 - "GripperLogic"
Cohesion: 0.10
Nodes (27): GripperLogic, gripper_logic — inti keputusan manipulator ROV (murni Python, tanpa ROS).  Dipis, Paksa lepas tanpa perintah (mis. saat shutdown/abort)., Aksi auto-detach saat node START.          gz-sim Fortress SELALU meng-attach De, Mesin keputusan gripper. Semua waktu (``now``, ``stamp``) dalam detik.      Para, Simpan sinyal visual servo terbaru (dari /hydroships/qr_offset)., True bila payload ada di jangkauan aman untuk di-attach:         offset kecil (R, Proses perintah semantik. Kembalikan dict aksi tingkat-rendah:             {'jaw (+19 more)

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
Cohesion: 0.14
Nodes (21): _clamp(), hook_servo(), normalize_hook_offset(), hook_logic — helper murni deteksi/servo hook (tanpa ROS/cv2), agar testable.  Di, (center px, area px^2, ukuran frame) -> (ex, ey, size) ternormalisasi.      Konv, PD visual servo hook -> perintah gerak (fungsi MURNI, testable).      Args:, test_hook_offset_centered(), test_hook_offset_left_up() (+13 more)

### Community 7 - "test_qr_logic.py"
Cohesion: 0.07
Nodes (32): CameraInfo, PointStamped, main(), Image, Node, QRDetector, qr_detector — deteksi QR dari kamera → sisi kolam A/B/C/D + offset piksel (M3)., Reshape buffer Image menghormati msg.step (row stride).          sensor_msgs/Ima (+24 more)

### Community 8 - "PROBLEM.md — Catatan Masalah & Verifikasi Tertunda (HYDROships ros2_ws)"
Cohesion: 0.12
Nodes (16): 2026-07-07, 2026-07-08, 2026-07-11, 2026-07-12, 2026-07-14, 2026-07-15 … 07-16, 2026-07-17, 2026-07-18 (+8 more)

### Community 9 - "hook_detector.py"
Cohesion: 0.22
Nodes (10): _best_contour(), detect_hook(), HookDetector, main(), Image, Node, hook_detector — deteksi hook (pipa-U) dari kamera depan -> offset (visual servo), Deteksi hook -> (center, area) atau None. Jenjang: contour/CLAHE lalu Hough. (+2 more)

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

### Community 15 - "File yang Harus Diperbaiki/Dibuat"
Cohesion: 0.09
Nodes (22): 1. **BUAT** `src/hydroships_gazebo/scripts/payload_spawner.py`, 2. **MODIFIKASI** `src/hydroships_gazebo/worlds/kki_arena.sdf`, 3. **MODIFIKASI** `src/hydroships_control/hydroships_control/mission_fsm.py`, 4. **MODIFIKASI** `src/hydroships_gazebo/launch/sim.launch.py`, 5. **MODIFIKASI** `src/hydroships_bringup/launch/hydroships_mission.launch.py`, 6. **MODIFIKASI** `docs/HOW-TO-RUN.txt`, 7. **MODIFIKASI** `docs/STATUS.md`, 8. **MODIFIKASI** `docs/CHANGELOG.md` (+14 more)

### Community 16 - "File yang Harus Diperbaiki"
Cohesion: 0.18
Nodes (10): 1. `src/hydroships_description/urdf/hydroships.urdf.xacro` — Pindah mount ke depan, 2. `src/hydroships_control/hydroships_control/mission_fsm.py` — Konsistenkan mention gripper, 3. `docs/STATUS.md`, 4. `docs/CHANGELOG.md`, 5. `docs/HOW-TO-RUN.txt`, File yang Harus Diperbaiki, Konfirmasi Status Saat Ini, Plan: Mount Gripper Body on Front Face of ROV (+X forward) (+2 more)

### Community 17 - "sim.launch.py"
Cohesion: 0.67
Nodes (3): generate_launch_description(), _launch_setup(), Launch simulasi Gazebo Fortress + spawn ROV HYDROships + ros_gz_bridge.  Argumen

### Community 18 - "PayloadSpawner"
Cohesion: 0.39
Nodes (3): main(), PayloadSpawner, Node

## Knowledge Gaps
- **66 isolated node(s):** `ros2-ws`, `Konfirmasi Status Saat Ini`, `Pendekatan: Payload Spawner Node`, `Aliran setelah perbaikan:`, `1. **BUAT** `src/hydroships_gazebo/scripts/payload_spawner.py`` (+61 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **9 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `normalize_hook_offset()` connect `hook_servo` to `hook_detector.py`?**
  _High betweenness centrality (0.124) - this node is a cross-community bridge._
- **Why does `GripperController` connect `QRDetector` to `GripperLogic`?**
  _High betweenness centrality (0.086) - this node is a cross-community bridge._
- **Are the 18 inferred relationships involving `GripperLogic` (e.g. with `GripperController` and `.__init__()`) actually correct?**
  _`GripperLogic` has 18 INFERRED edges - model-reasoned connections that need verification._
- **Are the 11 inferred relationships involving `GuiBridgeLogic` (e.g. with `GuiBridge` and `.__init__()`) actually correct?**
  _`GuiBridgeLogic` has 11 INFERRED edges - model-reasoned connections that need verification._
- **Are the 12 inferred relationships involving `hook_servo()` (e.g. with `._st_approach_hook()` and `test_centered_far_moves_forward_only()`) actually correct?**
  _`hook_servo()` has 12 INFERRED edges - model-reasoned connections that need verification._
- **Are the 9 inferred relationships involving `PID` (e.g. with `Stabilizer` and `.__init__()`) actually correct?**
  _`PID` has 9 INFERRED edges - model-reasoned connections that need verification._
- **What connects `ros2-ws`, `Konfirmasi Status Saat Ini`, `Pendekatan: Payload Spawner Node` to the rest of the system?**
  _66 weakly-connected nodes found - possible documentation gaps or missing edges._