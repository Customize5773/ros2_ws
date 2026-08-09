# P1-0 Architecture Audit

## 1. Scope

This is a **read-only, cross-repository architecture audit** between:

- `ros2_ws` (this repository) — the ROS 2 / Gazebo Fortress **simulation and reference implementation** of the HYDROships ROV (KKI 2026 competition vehicle).
- `GUI-ROV` (`https://github.com/Customize5773/GUI-ROV`, local checkout at `/home/rasya/GUI-ROV`) — the **target autonomous-system / ground-station application**, which talks to a real Pixhawk/ArduSub vehicle over MAVLink and exposes a browser dashboard.

The objective is **not** to find as many problems as possible. It is to determine **which boundaries and contracts must be made stable** so that behavior validated in `ros2_ws` simulation can become a reliable foundation for the autonomous system in `GUI-ROV`.

This document is produced under the P1 track, which runs **in parallel with and does not modify** the P0 validation track. No source code, configuration, launch files, or P0 experiment state were changed to produce this audit. See §20.

## 2. Audit Baseline

### ros2_ws
```
Current branch: rasya/dev7
Current commit: 8cd30df88719f59e04ea903e1d8373e10b0533fa
                 "feat(tools): add QR instrumentation and smoke test scripts"
Working tree:    1 modified file — graphify-out/cache/last_query_stamp
                 (graphify cache artifact, not part of this audit's evidence
                 or output; left untouched)
```

### GUI-ROV
```
Current branch: main
Current commit: 4dbfeac23ad2b5ef711c8a373f28dd4cf986760f
                 "done 2 panel qr"
Working tree:    clean
```

Both repos are recorded here for reproducibility. `GUI-ROV` was audited entirely read-only; no files in it were created, edited, or deleted.

**Note on existing prior work**: `ros2_ws` already contains `docs/GUI-INTEGRATION.md`, `docs/NODES_REFERENCE.md`, `docs/ARCHITECTURE.md`, and `docs/STATUS.md`, which partially document the GUI-ROV boundary and node topology from inside the P0 track. This audit independently re-verified those claims against current source and GUI-ROV source, and treats prior docs as **evidence to cross-check**, not as ground truth. Where this audit confirms prior docs, that is noted; where it extends or adds nuance, that is also noted.

## 3. Repository Topology

### ros2_ws

Four `ament` packages under `src/`:

| Package | Build type | Purpose |
|---|---|---|
| `hydroships_control` | `ament_python` | All Python control/perception/teleop/bridge nodes + pure logic modules |
| `hydroships_bringup` | `ament_cmake` | Top-level launch files composing the other packages |
| `hydroships_gazebo` | `ament_cmake` | Gazebo world, ros_gz_bridge config, payload spawner, sim launch |
| `hydroships_description` | `ament_cmake` | URDF/xacro robot model, physical parameter YAML |

No custom `.msg`/`.srv`/`.action` files exist anywhere (`find` for these patterns returns zero results). Every interface uses stock `std_msgs`/`geometry_msgs`/`nav_msgs`/`sensor_msgs` types. No services or actions exist anywhere — all coordination is topic pub/sub.

Hygiene note (not a P0 behavior issue, informational): `src/hydroships_control/build/` and `src/hydroships_control/install/` are in-tree colcon build artifacts (unusual — normally these live only at the workspace root); `.claude/worktrees/pid-heading-wraparound-fix-*/` contains a parallel full copy of `src/` for an in-progress fix, not diffed by this audit.

### GUI-ROV

**Confirmed not a ROS 2 workspace** — no `package.xml`, no launch files, no colcon/ament build system, zero `rclpy`/`rospy` hits anywhere in the tree. It is a hand-rolled three-tier stack:

| Layer | Location | Role |
|---|---|---|
| Browser frontend | `public/` (`index.html`, `js/app.js`, `js/core.js`, `js/pages/*.js`, `js/manipulator/*.js`, vendored Three.js/Chart.js) | Dashboard UI, WebSocket client |
| Node bridge server | `server/server.js` (812 lines) | HTTP static server + WebSocket server (port 8080) + UDP JSON bridge; also `server/recording.js`, `server/joystick-config.js`, `server/sim-params.js` |
| Vehicle-side agent (primary/production) | `rov_agent.py` (root, 1540 lines) + pure helper modules `rov_axes.py`, `rov_modes.py`, `rov_heading.py`, `rov_params.py`, `rov_pid.py`, `rov_mavlink.py`, `attitude_filter.py`, `gripper_controller.py` | Runs on the Raspberry Pi; talks to Pixhawk via `pymavlink` over serial |
| Shared schema | `shared/rov-modes.js`, `shared/joystick-profile.js` | Cross-language (Node + browser) single source of truth for mode names and joystick config |
| Competition autonomy subsystem (secondary) | `autonomy/` — `rov_link.py` (422 lines, a **second** MAVLink↔JSON bridge), `fsm/mission5.py` (1066 lines), `vision/`, `control/visual_servo.py`, `config/loader.py`, `tools/`, `sitl_mock.py` | A separate, parallel Python subsystem for the "KKI 2026 Mission 5" competition task |

No Gazebo, no simulation physics anywhere in GUI-ROV (repo-wide grep for "gazebo" returns zero hits). Its own `--sim` mode (`server/server.js`) is a fake sine-wave telemetry generator for UI development, unrelated to `ros2_ws`'s physics simulation.

## 4. Runtime Architecture

### ros2_ws

```
Gazebo Fortress (kki_arena.sdf, Hydrodynamics + graded Buoyancy + Thruster + JointPositionController + DetachableJoint plugins)
  ↓ (ros_gz_bridge, config/bridge.yaml)
Sensors (camera_front, camera_bottom, IMU) + Odometry (/hydroships/odom)
  ↓
Perception (qr_detector, hook_detector — pure-logic modules qr_logic.py/hook_logic.py)
  ↓
State / Feedback (/hydroships/depth, /hydroships/qr_offset, /hydroships/hook_offset, /hydroships/payload_pose)
  ↓
Controllers (stabilizer: 4× PID depth/heading/roll/pitch, config/gains.yaml)
  ↓
Allocator (thruster_allocator: damped pseudo-inverse of 6×6 TAM, allocation.py)
  ↓
Thrusters (6× /hydroships/thruster_N/thrust → Gazebo Thruster plugin)
```

and, layered on top:

```
Mission FSM (mission_fsm.py, 12 states)
  ↓ semantic setpoints only — never touches thrusters directly
/hydroships/setpoint/depth, /hydroships/setpoint/heading, /hydroships/manual/cmd (Fx/Fy), /hydroships/gripper/*
  ↓ consumed by stabilizer + thruster_allocator (must both be running — enforced only by launch-file composition, not code)
Vehicle commands
```

Alternative command sources feeding the same `stabilizer`/`thruster_allocator` chain: `teleop_keyboard`, `teleop_stabilized`, `teleop_gamepad`, and `gui_bridge` (see §6). All of these publish to the same topics as `mission_fsm` with no arbitration — see §9/§16.

### GUI-ROV

```
GUI (public/js, WebSocket client)
 ↓ WebSocket :8080, {type:"cmd", name, value}
Command layer (server/server.js: clamp, tap to recorder, forward)
 ↓ UDP JSON :14550, {name, value, t}
Autonomous/control layer — TWO parallel, non-interoperating implementations:
   (a) rov_agent.py (root) — production path, direct pymavlink/serial to Pixhawk
   (b) autonomy/rov_link.py — competition path, also spawns autonomy/fsm/mission5.py
 ↓ pymavlink (MANUAL_CONTROL / MAV_CMD_DO_SET_SERVO / MAV_CMD_COMPONENT_ARM_DISARM / param_set)
Hardware/communication layer — Pixhawk/ArduSub over serial (rov_agent.py) or UDP MAVLink (rov_link.py, SITL/mock or hardware)
```

Telemetry flows the reverse path (Pixhawk → agent → UDP :14551 → server.js → WebSocket → dashboard). This diagram matches the source; it is not idealized — the duplication in the autonomous/control layer (two independent bridge implementations) is real and documented further in §11.

## 5. ROS Interface Inventory

### Topics

**Core motion/control chain**

| Topic | Type | Publisher(s) | Subscriber(s) | Notes |
|---|---|---|---|---|
| `/hydroships/cmd_vel` | `geometry_msgs/Twist` (used as a **wrench**: linear=N, angular=N·m — NOT a velocity, `thruster_allocator.py:4-5,9-12`) | `stabilizer.py:138`, `teleop_keyboard.py:53`, `teleop_gamepad.py:157,343` (if not routed through stabilizer), `gui_bridge.py:67,115` | `thruster_allocator.py:52-53` | Multiple independent publishers, no arbitration (§9) |
| `/hydroships/manual/cmd` | `geometry_msgs/Twist` (Fx,Fy N; some publishers also set other axes) | `mission_fsm.py:218`, `teleop_stabilized.py:45,97`, `teleop_gamepad.py:158,341` | `stabilizer.py:140` | |
| `/hydroships/setpoint/depth` | `std_msgs/Float64` (m, negative = deeper) | `mission_fsm.py:216`, `teleop_stabilized.py:46,88`, `teleop_gamepad.py:159,204,321` | `stabilizer.py:142` | |
| `/hydroships/setpoint/heading` | `std_msgs/Float64` (**rad**, REP-103) | `mission_fsm.py:217`, `teleop_stabilized.py:47,91`, `teleop_gamepad.py:160,208,325` | `stabilizer.py:144` | |
| `/hydroships/control_mode` | `std_msgs/String` (`manual`/`depth_hold`/`poshold`) | `teleop_gamepad.py:162,211,266` | `stabilizer.py:148-149` | |
| `/hydroships/thruster_{1..6}/thrust` | `std_msgs/Float64` (N, clipped `[-40,50]`) | `thruster_allocator.py:48-51,86-90` | Gazebo Thruster plugin (`bridge.yaml:62-96`) | |

**Perception / navigation feedback**

| Topic | Type | Publisher | Subscriber |
|---|---|---|---|
| `/hydroships/odom` | `nav_msgs/Odometry` | Gazebo `OdometryPublisher` plugin via bridge | `mission_fsm.py:223`, `stabilizer.py:139`, `depth_publisher.py:22-23`, `gui_bridge.py:71`, `teleop_gamepad.py:178` |
| `/hydroships/depth` | `std_msgs/Float64` (m, ≥0, positive-down: `max(0,-z)`) | `depth_publisher.py:21,29` | `mission_fsm.py:222`, `gui_bridge.py:72` |
| `/hydroships/qr_result` | `std_msgs/String` (A/B/C/D) | `qr_detector.py:56,149` | `mission_fsm.py:224` |
| `/hydroships/qr_offset` | `geometry_msgs/PointStamped` (x=ex,y=ey normalized, z=size; `frame_id`=`camera_bottom_link`/`camera_front_link`) | `qr_detector.py:57,154-162` | `mission_fsm.py:225-226` (bottom-camera filtered), `gripper_controller.py:59,83-88` |
| `/hydroships/hook_offset` | `geometry_msgs/PointStamped` (`frame_id`=`camera_front_link`) | `hook_detector.py:119,173-177` | `mission_fsm.py:229-230` |
| `/hydroships/payload_pose` | `geometry_msgs/PointStamped`, QoS **TRANSIENT_LOCAL** (latched), `frame_id`=`world` | `payload_spawner.py:99,180-187` | `mission_fsm.py:234-236` |
| `/hydroships/payload/spawned` | `std_msgs/Empty`, TRANSIENT_LOCAL | `payload_spawner.py:103,174` | `gripper_controller.py:74-75` |
| `/hydroships/camera_front/image_raw`, `/hydroships/camera_bottom/image_raw` | `sensor_msgs/Image` | Gazebo camera sensors via bridge | `qr_detector.py:50-60`, `hook_detector.py:115,120` (front only) |
| `/hydroships/camera_front/camera_info`, `/hydroships/camera_bottom/camera_info` | `sensor_msgs/CameraInfo` | Gazebo via bridge | `qr_detector.py:67-71` (sim-only intrinsics, explicitly not for real distance) |
| `/hydroships/imu` | `sensor_msgs/Imu` | Gazebo IMU via bridge | **No subscriber found anywhere** — bridged but unused |

**Gripper / manipulator**

| Topic | Type | Publisher | Subscriber |
|---|---|---|---|
| `/hydroships/gripper/command` | `std_msgs/String` (`open`/`close`) | `mission_fsm.py:221` (**declared but never actually `.publish()`-ed anywhere in the file — see §16 VERIFIED ISSUE / P0 finding**), `gui_bridge.py:68,117`, `teleop_gamepad.py:161,275` | `gripper_controller.py:58` |
| `/hydroships/gripper/attach` | `std_msgs/Empty` | `gripper_controller.py:56,129` | Gazebo `DetachableJoint` plugin |
| `/hydroships/gripper/detach` | `std_msgs/Empty` | `gripper_controller.py:57,118,131` **and separately** `mission_fsm.py:242,823` | Gazebo `DetachableJoint` plugin |
| `/hydroships/gripper_left/cmd`, `/hydroships/gripper_right/cmd` | `std_msgs/Float64` (rad) | `gripper_controller.py:54-55,92-94` | Gazebo `JointPositionController` plugins |

**Teleop input**: `/joy` (`sensor_msgs/Joy`) subscribed by `teleop_gamepad.py:177`; no publisher inside this workspace (expected external `joy_node`).

### Services

None. No `create_service`/`create_client` anywhere in `hydroships_control`.

### Actions

None. No `ActionServer`/`ActionClient` anywhere. Gripper attach/detach is semantically an action (has a "goal" and eventual physical effect) but is implemented as fire-and-forget `std_msgs/Empty` topic messages with no acknowledgment/feedback path.

### Messages

No custom message/service/action definitions exist in this workspace.

### Parameters

| File | Consumer | Contents |
|---|---|---|
| `src/hydroships_control/config/gains.yaml` | `stabilizer` | PID gains (depth/heading/pitch/roll: kp/ki/kd/integral_limit/out_limit), `buoyancy_ff: -0.3`, `target_depth: -0.1`, `enable_*_hold` flags |
| `src/hydroships_control/config/gamepad.yaml` | `teleop_gamepad` | Axis/button mapping, gains, topic overrides (not wired into any launch file — teleop run standalone) |
| `src/hydroships_description/config/rov_params.yaml` | Loaded at URDF-build time via `xacro.load_yaml`, not a runtime ROS param | Mass 8.3kg `[measured]`, buoyancy geometry, CoG/CoB offsets, 18 hydrodynamic coefficients (`[estimate]`, BlueROV2-scaled placeholder, vs `[measured]`) |
| `src/hydroships_gazebo/config/bridge.yaml` | `ros_gz_bridge parameter_bridge` | Full ROS↔Gazebo topic/type/direction map |

`mission_fsm.py` declares ~45 parameters in-code (`.py:93-171`, timeouts per state, PD gains, geometry offsets) — no external YAML. `thruster_allocator.py:42` declares `alloc_damping` (0.1). `qr_detector.py:50-53`, `hook_detector.py:115-117`, `gripper_controller.py:34-46` each declare a handful of tuning params. `gui_bridge.py:53-59` declares `cmd_port`(14550), `telem_host`, `telem_port`(14551), `telem_hz`, and per-axis gains. `teleop_gamepad.py:66-112` declares ~35 params.

### Frames

No `tf2_ros.TransformBroadcaster` usage anywhere in `hydroships_control` — confirmed by grep. `robot_state_publisher` (`sim.launch.py:176-181`) publishes the static URDF joint tree from `robot_description` (base_link→camera/thruster/gripper links), but **no node publishes an odom→base_link TF** despite the Gazebo `OdometryPublisher` plugin declaring `<odom_frame>odom</odom_frame>`/`<robot_base_frame>base_link</robot_base_frame>` (`hydroships.urdf.xacro:406-407`) — whether gz-sim's own plugin publishes this TF internally was not verified by source reading (classified UNKNOWN, §16).

`frame_id` header strings in use (not a TF chain, just message tagging): `camera_bottom_link`, `camera_front_link` (`qr_detector.py:79-85`, `hook_detector.py:175`), `world` (`payload_spawner.py:185`).

URDF frame tree (`hydroships.urdf.xacro`): `base_link` → `imu_link`, `depth_link` (declared but unused — depth is derived from odom, not this frame), `camera_front_link`, `camera_bottom_link` → `thruster_1..6` → `gripper_base` → `gripper_finger_left`/`gripper_finger_right`.

## 6. Cross-Repository Contracts

**Only one interface qualifies as a genuine `CROSS_REPOSITORY_CONTRACT`**: the UDP-JSON command/telemetry protocol implemented on both sides.

| Side | File | Role |
|---|---|---|
| `ros2_ws` | `src/hydroships_control/hydroships_control/gui_bridge.py` + `gui_bridge_logic.py` | Listens for GUI commands on UDP port `cmd_port` (default **14550**), sends telemetry UDP JSON to `telem_host:telem_port` (default **127.0.0.1:14551**) at `telem_hz` (default 10Hz) |
| `GUI-ROV` | `server/server.js` (env `UDP_OUT=14550`/`UDP_IN=14551`) ↔ `rov_agent.py` (`UDP_CMD_PORT=14550` from `UDP_OUT`, `UDP_TELEM_PORT=14551` from `UDP_IN`, `rov_agent.py:44-45`) | `server.js` sends commands to `:14550`, listens for telemetry on `:14551` |
| `GUI-ROV` (alt) | `autonomy/rov_link.py` | Same port defaults (`--json-rx-port 14550`, `--telem-port 14551`, `.py:387,389`), used when the competition-autonomy path is active instead of `rov_agent.py` |

**Why this is architectural, not coincidental**: `gui_bridge.py`'s own docstring (`.py:1-21`) states its purpose is to make the ROS 2 simulation "look like an ArduSub ROV" to the GUI team's repo (`Customize5773/GUI-ROV`) without touching core sim nodes, and `ros2_ws/docs/GUI-INTEGRATION.md` (pre-existing, cross-checked by this audit) documents the same port numbers and field mapping independently from the GUI-ROV side of the analysis. This is a deliberate, two-sided integration point — not two components that merely happen to share a port number.

**Command mapping** (`gui_bridge_logic.py`, cross-checked against `docs/GUI-INTEGRATION.md` §2 table and GUI-ROV's `rov_axes.py`):

| Field | GUI-ROV semantics | ros2_ws (`gui_bridge`) semantics | Contract status |
|---|---|---|---|
| `name`/`value` JSON envelope | `{"name": str, "value": val}` (`rov_agent.py` `command_listener`) | Same envelope parsed in `gui_bridge.py:_on_cmd` | Matches |
| `surge`/`sway`/`heave`/`yaw` | Percent `-100..100` (or `-1000..1000` depending on layer — GUI-ROV's own axis scale differs between `public/js` (`±1000`) and MAVLink `MANUAL_CONTROL` (`x/y/r: ±1000`, `z: 0..1000`)) | Percent scaled by per-axis gain (`surge_gain=0.40` etc.) into a **force in Newtons** on `/hydroships/cmd_vel` | Units diverge by design: GUI-ROV's percent ultimately becomes a MAVLink joystick axis (mixed by ArduSub firmware into unknown per-motor PWM); `gui_bridge`'s percent becomes an explicit Newton force consumed by a documented allocator. There is **no shared physical unit** — the contract is "percent in, vehicle moves accordingly" not "percent means the same force in both stacks." Gains in `gui_bridge` (`surge_gain`/`sway_gain`/`heave_gain`/`yaw_gain`) are explicitly marked `[VERIFY]` uncalibrated. |
| `gripper` open/close | `{"name":"gripper","value":"open"/"close"}` → PWM via `MAV_CMD_DO_SET_SERVO` | Same string passthrough → `/hydroships/gripper/command` | Matches at the JSON level |
| `arm`/`stop` | Arms/disarms real vehicle via `MAV_CMD_COMPONENT_ARM_DISARM`; `stop` neutralizes + disarms | `gui_bridge_logic.py:93-95` tracks an `armed` flag and zeroes wrench when disarmed — **no ROS-side "vehicle" to actually arm/disarm** (thruster_allocator has no arm concept) | GUI-ROV's arm/disarm is a real hardware safety interlock; `ros2_ws`'s is an adapter-local flag with no downstream enforcement beyond zeroing wrench in `gui_bridge` itself |
| Heading telemetry | `heading`, degrees `0-360` | `/hydroships/odom` yaw, **radians**, REP-103 convention | `gui_bridge.py`'s `_yaw_rpy()` converts rad→deg before sending — confirmed handled, not a live bug, but the unit difference is a real, permanent property of the boundary (see §12) |
| Depth telemetry | `depth`, meters, positive-down | `/hydroships/depth`, meters, ≥0 (positive-down by construction) | Same sign/unit convention — passthrough, no conversion needed |
| Roll/pitch telemetry | degrees | odom quaternion (rad) | `gui_bridge.py:_yaw_rpy()` converts; matches |

**Telemetry field-set comparison**:

| Field | `gui_bridge_logic.build_telemetry()` (`ros2_ws`) | `rov_agent.py` `state` dict (`GUI-ROV`) | Present in both? |
|---|---|---|---|
| `heading` (deg) | Yes | Yes | Yes |
| `depth` (m) | Yes | Yes | Yes |
| `roll`, `pitch` (deg) | Yes | Yes | Yes |
| `temp` | Yes (`gui_bridge.py` sends a value) | Yes (from `SCALED_PRESSURE2.temperature`) | Yes, but `ros2_ws` has no simulated water-temperature source — value is almost certainly a placeholder/constant (not independently re-verified against `gui_bridge_logic.py` internals beyond the field list; flag as UNKNOWN for exact provenance) |
| `voltage` | Yes | Yes (from `SYS_STATUS.voltage_battery`) | Yes; `ros2_ws` has no battery model — placeholder |
| `armed` | Yes (adapter-local flag) | Yes (derived from real `HEARTBEAT` safety bit) | Yes, but semantically different sources (see arm/disarm row above) |
| `light` | Yes | Yes | Yes (both effectively cosmetic/unimplemented-hardware placeholders) |
| `mode` | Yes | Yes (ArduSub firmware mode string) | Yes, but `ros2_ws` has no ArduSub firmware mode concept — likely a static/derived string, not independently re-verified |
| `cmd_link` (stale/ok) | Not found in `gui_bridge_logic.py` | Yes (`rov_agent.py:78-82`, comms-loss indicator) | **Gap**: `ros2_ws` telemetry has no equivalent "is the command link stale" signal |

This field-set table should be read as a starting point for calibration work, not as proof of bugs — see §16 UNKNOWN entries.

## 7. Simulation-Only Interfaces

| Interface | Why simulation-only | Current consumers | Potential target-system replacement | Status |
|---|---|---|---|---|
| `/hydroships/payload_pose` (ground-truth payload position from Gazebo `payload_spawner`) | Comes directly from the simulated payload model's spawn pose — no real-world equivalent (a real ROV has no ground-truth payload location) | `mission_fsm.py:234-236` (gate for `DIVE`→`APPROACH_QR` transition) | None currently — `mission_fsm`'s dependence on this ground-truth topic is a real integration gap if this FSM were ever ported to hardware (see §16 DESIGN GAP) | VERIFIED |
| `/hydroships/camera_*/camera_info` (Gazebo-derived intrinsics `fx=fy=381.4, cx=320, cy=240`) | `qr_detector.py:67-71` comments explicitly mark these as sim-only, not for real distance/pose estimation | `qr_detector.py` (stored but not documented as used for metric distance in the excerpt read) | Real camera calibration via `autonomy/tools/calibrate_camera.py` (GUI-ROV) — that tool exists specifically because GUI-ROV's `autonomy/control/visual_servo.py` PBVS mode needs real intrinsics | VERIFIED |
| `/hydroships/payload/spawned`, Gazebo spawn/reset mechanics (`payload_spawner.py`, `sim.launch.py` spawn randomization/`spawn_seed`) | Pure simulation orchestration, no hardware analog | `gripper_controller.py:74-75` | N/A — this category of interface simply does not exist on hardware | VERIFIED |
| `/hydroships/imu` (bridged, unused) | Not simulation-only in principle, but currently has zero consumers in this workspace | None | N/A until a consumer exists | VERIFIED |
| Gazebo `DetachableJoint` auto-attach-on-load quirk and the forced startup-detach workaround (`gripper_logic.py:115-130`) | A documented Gazebo Fortress bug workaround, not a real vehicle behavior | `gripper_controller.py:65-78` | Real hardware would use `GRIPPER_SERVO_CH` PWM directly — no "auto-attach" concept | VERIFIED |

## 8. Target-System Interfaces

Candidate interfaces expected to remain stable (in role, if not in literal wire format) across simulation → hardware:

| Contract | Producer | Consumers | Why it should be stable | Current implementation | Recommended boundary |
|---|---|---|---|---|---|
| Vehicle command (wrench/axis intent) | `mission_fsm`/teleop nodes (sim) → `stabilizer` → `thruster_allocator`; GUI/joystick (hardware) → ArduSub firmware mixer | `thruster_allocator` (sim) vs. ArduSub motor mixer (hardware) | Both stacks need *some* stable "move the vehicle this way" semantic even though the mixing math differs completely (explicit 6×6 pseudo-inverse vs. firmware black-box mixer) | Sim: explicit Newton-based wrench (`/hydroships/cmd_vel`). Hardware: percent-based joystick axes (`MANUAL_CONTROL`) | Treat as **two intentionally different implementations of the same role**, not a literal shared message. `gui_bridge` is the correct place for the percent↔force conversion; do not attempt to make ArduSub consume Newtons or make the sim consume raw joystick percent directly. |
| Vehicle state (attitude/depth) | Gazebo odom (sim) / MAVLink `ATTITUDE`+`SCALED_PRESSURE2` (hardware) | `stabilizer`, `mission_fsm`, `gui_bridge` (sim); `rov_agent.py`/`rov_link.py` (hardware) | Depth-sign and unit conventions must agree for any FSM logic ported between stacks | Sim: radians internally, `Odometry` message. Hardware: degrees, plain floats in a `state` dict | See §12 — depth convention already matches; heading unit differs but is converted at the one crossing point (`gui_bridge`) |
| Mission state | `mission_fsm.St` enum (sim, 12 states, HYDROships-specific) | `mission_fsm` internal only — **not exposed on any topic** | If GUI-ROV or hardware autonomy is meant to observe/drive mission state, a stable representation is needed | **None exists.** `mission_fsm` has no published state topic; `gui_bridge` does not surface FSM state to GUI-ROV telemetry | DESIGN GAP — see §16 |
| Perception result (QR / hook detection) | `qr_detector`/`hook_detector` (sim, from Gazebo camera images) vs. `autonomy/vision/qr_detect.py`/`hook_detect.py` (GUI-ROV, from real camera) | `mission_fsm` (sim) vs. `autonomy/fsm/mission5.py` (GUI-ROV) | `docs/GUI-INTEGRATION.md` already documents that `hook_detector` was **ported from** GUI-ROV's `autonomy/vision/hook_detect.py` to keep detection semantics aligned | `PointStamped` (ex,ey,size) on both sides by design, per existing prior integration work | Already reasonably aligned — see §11 for nuance |
| Telemetry (aggregate vehicle status) | `gui_bridge_logic.build_telemetry()` (sim) vs. `rov_agent.py`/`rov_link.py` telemetry envelope (hardware) | GUI-ROV `server.js`/dashboard | This is the one interface truly shared cross-repo today | UDP JSON, field-compatible but not identically sourced (see §6 table) | Recommend a single written field spec (name, unit, required/optional, "simulated placeholder" vs "real sensor") shared by reference between repos — currently only implicit in two independently-maintained source files plus one one-sided doc (`GUI-INTEGRATION.md`) |
| Fault state | ArduSub failsafes (hardware: GCS-heartbeat loss, EKF failure, etc.) | GUI-ROV `cmd_link` field, dashboard banners | `mission_fsm`'s `ABORT` state is the closest sim analog but is not exposed as telemetry | No shared representation at all | DESIGN GAP — see §16 |
| Actuator state (gripper) | Gazebo `DetachableJoint`+`JointPositionController` (sim) vs. real servo PWM feedback (hardware, none read back) | `gripper_controller`/`gripper_logic` (sim) vs. `gripper_controller.py`/`rov_link.py` (GUI-ROV) | Both stacks command a 2-position (or continuous) gripper, but neither reads back a real "gripper is closed" signal — both infer state internally | Sim: `attached` bool tracked in `gripper_logic`. Hardware: PWM slew position tracked locally, no closed-loop confirmation | UNKNOWN whether this matters for the audit's goals — flagged, not solved |
| Configuration/parameters | ROS 2 declared params + YAML (sim) vs. ArduSub firmware params (`parameters_ardusub.params`, hardware) | Node-local (sim) vs. Pixhawk (hardware) | Entirely different mechanisms; no expectation of a shared schema | N/A | No recommendation — these are legitimately `SIMULATION_INTERNAL`/`HARDWARE_INTERFACE` respectively |
| Coordinate frames | REP-103 body frame + `world`/link frame_ids (sim) vs. body-frame joystick axes, no global frame (hardware) | See §12 | Neither stack has a shared global frame (GUI-ROV explicitly has no GPS/DVL) — this narrows what "stable" even means here | See §12 | See §12 |
| Safety/arming state | Various partial mechanisms (see §14) | — | Hardware arm/disarm is safety-critical; sim has no equivalent physical risk but a mission run should still respect an analogous gate | Asymmetric — hardware is much richer | DESIGN GAP — see §14/§16 |

## 9. GUI ↔ Autonomy Boundary

Answering the eight mandated questions, evidence-based:

1. **Does GUI know ROS node internals?** No. GUI-ROV has zero ROS awareness; it only knows the UDP-JSON wire protocol (`docs/GUI-INTEGRATION.md` §1, confirmed by grep for `rclpy`/`ros2` in GUI-ROV: zero hits). **HEALTHY.**
2. **Does GUI depend on internal ROS topics?** No — it never sees ROS topics at all; `gui_bridge` is the sole translation point. **HEALTHY.**
3. **Does autonomous logic (`mission_fsm`) depend on GUI?** No — `mission_fsm` has no knowledge of `gui_bridge` or GUI-ROV; it runs identically with or without `gui_bridge` active (`hydroships_mission.launch.py` does not include `gui_bridge`; `hydroships_gui.launch.py` does not include `mission_fsm`, per `hydroships_bringup/launch/*.py`). **HEALTHY.**
4. **Does telemetry have a clear contract?** Partially — see §6 table. Field names match by convention, but there is no single written schema either repo can validate against automatically; the closest thing is `docs/GUI-INTEGRATION.md`, which lives only in `ros2_ws`. **ACCEPTABLE**, with a concrete DESIGN GAP (formalize a shared schema) noted in §17/§18.
5. **Does command interface have clear semantics?** Mostly — percent-in/force-out is documented (`gui_bridge_logic.py` gains) but gains are explicitly `[VERIFY]` uncalibrated, and there's no shared statement of what "percent" physically means across the two backends (ArduSub firmware mixing vs. explicit wrench). **ACCEPTABLE**, calibration is an execution task, not an architecture defect.
6. **Can mission state be represented without Gazebo?** `mission_fsm.St` states are named after mission semantics (DIVE, APPROACH_QR, GRAB, ...), not Gazebo concepts — so *conceptually* yes — but the state is **not exposed anywhere outside the node's own memory** (no topic, no telemetry field), so today the answer is "not currently, though nothing prevents it." **DESIGN GAP**, not a coupling problem per se.
7. **Can GUI be replaced without changing autonomy?** Yes — `mission_fsm` has no dependency on `gui_bridge`/GUI-ROV existing at all (see point 3). **HEALTHY.**
8. **Can autonomy run without GUI?** Yes — same evidence as point 3, and `hydroships_mission.launch.py` is a complete, independent launch stack. **HEALTHY.**

**Overall classification: HEALTHY**, with one **ACCEPTABLE** item (telemetry/command schema exists only informally) worth turning into a written contract before hardware integration — see §18.

## 10. Controller / Perception / FSM Boundaries

Traced in `mission_fsm.py` end-to-end (all 12 state handlers read): the FSM **never** commands actuators (thrusters, gripper PWM) directly except for one path — see below.

```
Mission FSM
 ↓ high-level command (setpoint/depth, setpoint/heading, manual/cmd Fx/Fy)
 ↓
Controller (stabilizer: PID → 6-DOF wrench on /hydroships/cmd_vel)
 ↓
Allocator (thruster_allocator: wrench → 6 per-thruster forces)
 ↓
Actuator (Gazebo Thruster plugin)
```

This layering is clean and matches the "responsibility per layer" model the audit brief asks about — **each layer's responsibility is clear enough to support a simulation-to-real transfer in principle** (the FSM issues setpoints; a different controller/allocator pair on hardware could theoretically consume the same setpoint semantics, modulo the unit/frame notes in §12).

**One direct-coupling exception, evidence-based, not a bug**: `mission_fsm.py:242,823` publishes directly to `/hydroships/gripper/detach` (Gazebo's `DetachableJoint` topic), bypassing `gripper_controller`/`gripper_logic`'s semantic layer and its `is_safe()` gate (`gripper_logic.py:60-70`). The FSM module docstring (`mission_fsm.py:16-18`) explains this is intentional: payload is pre-attached via SDF `DetachableJoint`, not via a service call, so there is no "attach" state machine for the FSM to coordinate with — `AUTO_RELEASE`'s detach is a direct simulation-plugin command. **Classification: DESIGN GAP** — this direct coupling to a Gazebo-specific topic has no defined hardware equivalent (a real gripper release would go through `gripper_controller.py`'s PWM path in GUI-ROV, not a bare `Empty` message), so this specific one line of `mission_fsm` would need rework for a hardware port. Not a bug in the current sim-only context.

**Perception → FSM**: `qr_detector`/`hook_detector` publish `PointStamped` offsets consumed only by `mission_fsm`; `stabilizer`/`thruster_allocator` never see perception data directly. Clean separation.

## 11. Duplicated or Divergent Logic

| Location A | Location B | Functional overlap | Why it exists | Classification | Risk | Recommendation |
|---|---|---|---|---|---|---|
| `GUI-ROV rov_agent.py` (root) | `GUI-ROV autonomy/rov_link.py` | Both are independent MAVLink↔UDP-JSON bridges, both bind the same UDP ports (14550/14551) and both can drive the same vehicle | `rov_agent.py` predates `autonomy/`; `autonomy/rov_link.py` was built for the KKI 2026 competition path including `mission5.py` FSM integration (`rov_link.py:243-287` spawns the FSM as a thread) | DIVERGENT IMPLEMENTATION | If both are run concurrently against real hardware, UDP port binding conflicts and duplicate MAVLink connections would occur (not verified at runtime — a static-code finding) | This is entirely internal to GUI-ROV — outside P1's mandate to fix, but material to this audit because it determines **which GUI-ROV component `ros2_ws`'s `gui_bridge.py` is actually the counterpart of**. Recommend the eventual contract spec (§18) state explicitly which of the two it targets. |
| `GUI-ROV gripper_controller.py` (`GRIPPER_SERVO_CH=7`, PWM open=1550/close=1450) | `GUI-ROV autonomy/rov_link.py` (`GRIPPER_SERVO_CH=7`, PWM open=1900/close=1100) | Same physical actuator (gripper servo), different PWM endpoints | Not explained in source; appears to be independent tuning in each subsystem | INCONSISTENCY | Whichever bridge is actually wired to hardware, the *other* file's constants are simply wrong/unused, but nothing prevents someone from running the wrong one | Internal to GUI-ROV; noted because `ros2_ws`'s `gui_bridge.py` gripper command is a bare `"open"/"close"` string, so this inconsistency doesn't leak across the repo boundary — informational only |
| `GUI-ROV CONTROL-MAPPING.md` (documents channel **10**, `SERVO10_FUNCTION=83`, references a file `rov_gripper.py`) | `GUI-ROV gripper_controller.py`/`autonomy/rov_link.py` (both use channel **7** in current source; `rov_gripper.py` does not exist in this checkout — confirmed via `find`) | Documentation vs. code | Documentation appears stale relative to a refactor | INCONSISTENCY (docs vs. code, not code vs. code) | Low — internal to GUI-ROV, does not affect the `ros2_ws` boundary | Informational only; out of scope to fix per P1.0 mandate |
| `ros2_ws hook_detector.py`/`hook_logic.py` | `GUI-ROV autonomy/vision/hook_detect.py` | Both detect the docking hook from a camera image and produce a normalized offset | **Documented, intentional**: `ros2_ws/docs/GUI-INTEGRATION.md` §3b states `hook_detector` was explicitly ported from GUI-ROV's `hook_detect.py` to align detection semantics before hardware transfer | JUSTIFIED DUPLICATION | Low — this is exactly the kind of intentional logic-sharing the audit is meant to identify as healthy | None — already correctly reasoned about in `GUI-INTEGRATION.md`; this audit confirms it |
| `ros2_ws mission_fsm.py` (12-state ROV mission FSM) | `GUI-ROV autonomy/fsm/mission5.py` (13-state competition FSM) | Both are mission state machines for the same competition, with overlapping state names (DIVE, GRAB/SCAN_QR, NAV_WALL, HANG, SURFACE) | Independently evolved — `mission5.py` targets the specific "Mission 5" scoring rubric and vision pipeline of GUI-ROV's autonomy stack; `ros2_ws`'s FSM targets full sim characterization | UNKNOWN whether one is meant to supersede the other, or whether they are deliberately separate (sim-validation FSM vs. competition-day FSM) — no source-level statement of intent found on either side | Medium — if these are meant to converge into one FSM eventually, divergent per-state timeout/threshold tuning done independently in each will need reconciling; if they're meant to stay separate, no risk | Flag as a `P1` backlog item to get an explicit statement of intent from whoever owns each FSM (§18, `P1-6`) |

## 12. Coordinate / Frame Contract

| Frame/convention | Owner | Convention | Axis | Units | Consumer |
|---|---|---|---|---|---|
| Body-frame wrench (`/hydroships/cmd_vel`) | `ros2_ws` (`thruster_allocator.py:9-12`) | REP-103 body frame | x=surge,y=sway,z=heave; roll/pitch/yaw about same | N (force), N·m (torque) | `thruster_allocator` |
| Odometry yaw | `ros2_ws` (Gazebo `OdometryPublisher`) | REP-103, radians | z-up right-handed | rad | `stabilizer`, `mission_fsm`, `gui_bridge` |
| GUI-ROV heading | `GUI-ROV` (`rov_agent.py`/`rov_link.py`, from MAVLink `ATTITUDE`) | Compass-style, 0-360° | Not REP-103 — clockwise-positive compass convention per `rov_heading.py:50-51` ("positive heading correction = clockwise") | degrees | dashboard, `rov_heading.py` P-controller |
| Depth | Both | Positive = deeper/down | z (vertical) | meters | Both sides — **this one already agrees** |
| GUI-ROV body axes (surge/sway/heave/yaw) | `GUI-ROV` (`rov_axes.py`) | Joystick percent, `-1000..1000` (heave uniquely `0..1000`, 500=neutral) | Same semantic axis names as `ros2_ws` (surge/sway/heave/yaw) but a **percent/PWM-adjacent scale**, not a physical unit | percent / PWM-relative | `gripper`/motor mixing on ArduSub firmware side |
| ArduSub firmware frame | ArduSub (external, not in either repo's source) | Unknown/opaque — GUI-ROV never computes per-motor PWM itself; firmware does the FRAME_CONFIG mixing | N/A | N/A | Not inspectable from either repo |

**Findings**:
- **Heading unit mismatch (rad vs. deg) is real but already bridged** at the one crossing point (`gui_bridge.py:_yaw_rpy()` converts). Classification: **INCONSISTENCY** between the two stacks' native conventions, correctly handled at the boundary today, but any *new* code that talks to both stacks directly (bypassing `gui_bridge`) would need to remember this — worth documenting explicitly rather than relying on institutional knowledge. `docs/GUI-INTEGRATION.md` already calls this out ("Heading telemetri... Adapter konversi rad→deg").
- **Heading sign/rotation-direction convention was not independently re-derived by this audit** (would require reading REP-103's exact sign convention against `rov_heading.py`'s "positive = clockwise" and confirming they match or are correctly inverted) — `docs/GUI-INTEGRATION.md:48-50` itself flags "tanda sumbu" (axis sign) as unverified/`VERIFY`. **Classification: UNKNOWN**, consistent with the existing doc's own admission.
- **No shared global position frame exists on either side** — `ros2_ws` has `world`/`odom` (simulation-only, arbitrary origin at spawn), GUI-ROV has none at all (no GPS/DVL, confirmed `rov_modes.py:33-39`). This is not a bug; it reflects that neither system currently does global localization. Any future "target-system contract" for position would be a **new capability**, not a boundary-alignment fix.

## 13. Command Contract

Traced end-to-end per the mandated pipeline:

```
GUI command → autonomy command → vehicle command → wrench → thruster allocation
```

- **GUI-ROV side**: `public/js` joystick/keyboard → WebSocket `{type:"cmd",name,value}` → `server.js` clamp (±1000) → UDP JSON `{name,value,t}` → `rov_agent.py`/`rov_link.py` re-clamp (`rov_axes.clamp_axis`) → MAVLink `MANUAL_CONTROL` (x/y/r `-1000..1000`, z `0..1000`) → ArduSub firmware mixer → real motors (mixing math not inspectable from either repo).
- **`ros2_ws` side (via `gui_bridge`)**: same UDP JSON in → `gui_bridge_logic.on_command()` → percent × axis gain → `Twist` wrench (N/N·m) on `/hydroships/cmd_vel` → `thruster_allocator` (explicit, inspectable 6×6 damped-pseudo-inverse allocation) → 6 thruster force topics → Gazebo.
- **`ros2_ws` side (native, no GUI)**: `mission_fsm`/teleop nodes → `/hydroships/manual/cmd` + `/hydroships/setpoint/*` → `stabilizer` (PID) → same wrench/allocator path.

**Semantics comparison**:

| Property | GUI-ROV → ArduSub | `ros2_ws` native (`mission_fsm`/teleop → stabilizer → allocator) |
|---|---|---|
| Unit | Percent/PWM-relative, opaque firmware mixing | Explicit Newtons/N·m, explicit linear allocation math |
| Frame | Body-frame joystick axes | REP-103 body-frame wrench |
| Range | `-1000..1000` (heave `0..1000`) | Unbounded at `cmd_vel` level; bounded only after allocation (`MIN_THRUST=-40N`,`MAX_THRUST=50N` per thruster) |
| Rate | Whatever the WebSocket/UDP send rate is (not read in detail) | `stabilizer` publishes at 20Hz (`stabilizer.py:138,151-152`) |
| Saturation | ArduSub firmware-internal (opaque) | Explicit per-thruster clip in `allocation.py:37-38`, consistent with the Gazebo Thruster plugin's own `max_thrust_cmd=50.0`/`min_thrust_cmd=-40.0` (`hydroships.urdf.xacro:415-441`) — **cross-checked and consistent** |
| Arming | Real MAVLink arm/disarm gates all motor output at the firmware level | **No arming concept exists in the native `ros2_ws` command path at all** (only `gui_bridge`'s own local `armed` flag gates its own output — `stabilizer`/`thruster_allocator`/`mission_fsm` publish freely with no arm gate) |
| Failsafe | `IDLE_TIMEOUT=0.5s` → force neutral (`rov_axes.py:70`) | `thruster_allocator`'s `cmd_timeout=0.5s` watchdog zeros thrusters if `/hydroships/cmd_vel` goes stale (`thruster_allocator.py:55-58,81-84`) — **same 0.5s value, likely not coincidental given `gui_bridge`'s explicit design intent to mimic GUI-ROV behavior**, but not confirmed as an intentional shared-constant decision in source comments |

**Question posed by the brief**: *"Can a command from GUI-ROV have a semantic equivalent in simulation without depending on Gazebo?"* — **Yes, via `gui_bridge`**, which is explicitly designed for this and does not require any Gazebo-specific knowledge (it talks to `/hydroships/cmd_vel`/`/hydroships/gripper/command`, both of which are ROS-level abstractions the allocator/gripper controller happen to route to Gazebo, but the GUI-facing contract itself has no Gazebo coupling). **No integration gap here** — this is the one part of the boundary that was explicitly designed and documented (`docs/GUI-INTEGRATION.md`) before this audit.

## 14. Telemetry Contract

| Telemetry | Producer | Consumer | Unit | Frame | Rate | Meaning | Failure behavior |
|---|---|---|---|---|---|---|---|
| Pose/orientation | `ros2_ws`: Gazebo `OdometryPublisher` (30Hz internal) → `/hydroships/odom`. GUI-ROV: MAVLink `ATTITUDE` (requested 10Hz, `rov_agent.py:1249-1302`) | `stabilizer`,`mission_fsm`,`gui_bridge` (sim) / dashboard (GUI-ROV) | rad (sim) / deg (GUI-ROV) | REP-103 (sim) / compass (GUI-ROV) | 30Hz (sim odom) vs. 10Hz (GUI-ROV requested) | Vehicle attitude | No explicit staleness handling found on the `ros2_ws` side for odom (no watchdog on `/hydroships/odom` age in `stabilizer`, beyond what a stale odom would do to PID behavior implicitly) |
| Depth | `depth_publisher.py` (from odom z) / MAVLink `SCALED_PRESSURE2` (10Hz requested) | Same as above | m, positive-down | — | Tied to odom rate (sim) / 10Hz (GUI-ROV) | Vehicle depth | Same as above — no explicit staleness gate found in `ros2_ws` |
| Heading | Derived from odom yaw (sim) / `ATTITUDE.yaw` (GUI-ROV) | — | rad→converted to deg at `gui_bridge` | — | — | — | — |
| Mission state | **Not published anywhere in `ros2_ws`** | N/A | N/A | N/A | N/A | N/A | N/A — this is the DESIGN GAP noted in §8/§16 |
| Perception state (QR/hook) | `qr_detector`/`hook_detector` (5Hz max_rate params) | `mission_fsm` | normalized px offset + size | camera-frame-tagged | ≤5Hz | Visual servo feedback | No consumer-side staleness check found in `mission_fsm` beyond the `_hook_fresh()`/similar freshness checks noted in code (`mission_fsm.py:427` `_hook_fresh`) — **this one does have explicit freshness handling**, contradicting a blanket claim that nothing checks staleness; noted as a positive counter-example |
| Actuator state | Not published for thrusters (only commanded, not read back) in `ros2_ws`; GUI-ROV requests `SERVO_OUTPUT_RAW` telemetry (10Hz) from real hardware but no equivalent exists in `ros2_ws` | — | — | — | — | — | — |
| Fault state | Not published in `ros2_ws` at all. GUI-ROV has `cmd_link` (stale/ok) as its closest fault-adjacent field | — | — | — | — | — | — |
| System status | `gui_bridge`'s `build_telemetry()` includes `voltage`/`temp` (see §6 — provenance UNKNOWN, likely placeholders) | dashboard | — | — | 10Hz (`telem_hz` param) | — | — |

**Overall assessment**: the telemetry contract that exists (heading/depth/roll/pitch via `gui_bridge`) is functional and unit-converted correctly, but it is a **narrow slice** of what GUI-ROV's dashboard actually displays (mode, armed, mission progress, fault/link status are either placeholder values or entirely absent on the sim side). This matches the `gui_bridge.py`/`hydroships_gui.launch.py` files' own `[VERIFY]` markers.

## 15. Safety / Hardware Boundary

| Mechanism | `ros2_ws` (sim) | GUI-ROV (hardware-facing) | Classification |
|---|---|---|---|
| Arming/disarming | No true arm concept — `stabilizer`/`thruster_allocator`/`mission_fsm` run unconditionally once launched. Only `gui_bridge`'s local `armed` flag gates its own wrench output (`gui_bridge_logic.py:93-95`) | Real MAVLink `MAV_CMD_COMPONENT_ARM_DISARM`, derived-not-asserted `armed` state from `HEARTBEAT` | SIMULATION_ONLY (sim has no analog) / HARDWARE_REQUIRED (real arming is meaningless in sim, where there's no real risk) — **not a missing feature, a category mismatch** |
| Command timeout | `thruster_allocator` 0.5s watchdog → zero thrust (`thruster_allocator.py:55-58`) | `rov_axes.IDLE_TIMEOUT=0.5s` → neutral, not full stop (rationale: avoid triggering ArduSub's own pilot-input failsafe unpredictably, `rov_agent.py:357-366`) | TRANSFERABLE in concept (both have a 0.5s watchdog) but the *response* differs (zero-force vs. neutral-hold) — INCONSISTENCY, not necessarily a bug, since "neutral" vs "zero" may be intentionally different given ArduSub's own failsafe interaction |
| Communication loss | Only detected implicitly via the above 0.5s watchdog on `cmd_vel` | Explicit: WebSocket disconnect → client-side E-Stop lock; `cmd_link` telemetry field surfaces staleness to the operator | `ros2_ws` has no operator-facing "comms lost" signal at all — MISSING relative to GUI-ROV's richer handling |
| Actuator failure | Not modeled — Gazebo Thruster plugin has no failure injection found in source | Not modeled beyond firmware-internal ESC feedback (not read back by GUI-ROV) | MISSING on both sides equally — not a boundary issue |
| Sensor failure | Not modeled — no camera/IMU dropout handling found beyond perception freshness checks (`_hook_fresh()` etc.) | Not modeled beyond MAVLink stream absence (implicit) | MISSING on both sides, symmetric |
| Emergency stop | `teleop_gamepad`'s `emergency_stop` toggle is **client-side-only** — zeros its own wrench, does not propagate an emergency signal to `stabilizer`/`mission_fsm`/`thruster_allocator` (`teleop_gamepad.py:28-29,250-252,289-291`) | Explicit `{"name":"stop"}` → neutralize + disarm, reaches the vehicle | SIMULATION_ONLY, and notably **weaker** than GUI-ROV's equivalent — a real architectural asymmetry worth a P1 backlog item (informational; not a P0 fix) |
| Mission abort | `mission_fsm.ABORT` state zeros surge continuously but does **not** retract to surface, does **not** release the gripper, does **not** alert any operator interface (`mission_fsm.py:459-461`) | `Mission5FSM.abort()` (`autonomy/fsm/mission5.py:323-328`) calls `cmd.emergency_stop()`, which does reach the vehicle's stop/disarm path | Both are minimal, but GUI-ROV's abort at least reaches the vehicle-level stop primitive; `ros2_ws`'s abort is a no-op beyond zeroing surge. DESIGN GAP on the `ros2_ws` side (frozen — P0-owned, not touched) |
| Manual override arbitration | **None** — `mission_fsm` and teleop nodes publish to the same topics with no mutex/priority (`teleop_gamepad.py:12-15` code comment warns about this but does not enforce anything) | Kill-switch: real manual-axis input during `autonomous` mode force-switches back to manual and stops `mission5` FSM (`autonomy/rov_link.py:190-198`, `KILL_SWITCH_DEADZONE=15`) | `ros2_ws` MISSING an equivalent; GUI-ROV's autonomy path has one. TRANSFERABLE pattern candidate for the sim side — but this would touch `mission_fsm`/teleop nodes, which are **P0-frozen** (see §19) |

## 16. Findings

### Verified Issues
*(Strong evidence of current, in-scope behavior; all P0-owned and explicitly NOT to be fixed by P1 — see §19.)*

- **V-1**: `mission_fsm.py:221` declares a publisher `pub_grip` for `/hydroships/gripper/command` but never calls `.publish()` on it anywhere in the file — confirmed by both this audit's independent read and by `docs/STATUS.md`'s M5 entry, which documents this as a "regression, blocking" already known to the P0 track (`_st_grab` never sends "close"). This audit did not re-derive it independently from a blank slate; it is cited here because it is directly relevant to §8's "actuator state" contract candidate — payload attach in the current sim FSM relies entirely on Gazebo's SDF auto-attach + a startup-detach workaround, not on a real command path, which matters for any future hardware-transfer discussion of the gripper contract. **P0 impact: N/A (this is P0's own known issue, not a P1 recommendation) — DO NOT FIX under P1.0.**

### Design Gaps

- **D-1**: No mission-state telemetry exists anywhere in `ros2_ws` (§8, §14). If `GUI-ROV`'s dashboard or `autonomy/fsm/mission5.py` is ever meant to observe or coordinate with a simulation-run mission, there is currently no interface for that.
- **D-2**: `mission_fsm`'s dependency on `/hydroships/payload_pose`, a Gazebo-ground-truth-only topic, for the `DIVE`→`APPROACH_QR` transition gate (§7) has no defined replacement for a hardware context, where no ground-truth payload pose would exist.
- **D-3**: No comms-loss / operator-facing fault telemetry exists in `ros2_ws` (§15), unlike GUI-ROV's `cmd_link` field.
- **D-4**: No manual-override arbitration exists between `mission_fsm` and teleop nodes in `ros2_ws` (§15), unlike GUI-ROV's kill-switch pattern in `autonomy/rov_link.py`.
- **D-5**: `mission_fsm.ABORT` does not retract/surface/release/alert (§15) — a narrower gap than V-1, purely about the abort *state's* behavior, not the grab bug.
- **D-6**: No written, versioned schema for the `gui_bridge` UDP-JSON contract exists that both repos could validate against — today it's tribal knowledge plus one one-sided doc (`docs/GUI-INTEGRATION.md`, `ros2_ws`-only) (§6, §9).

### Duplications

- **DUP-1**: `GUI-ROV rov_agent.py` vs. `autonomy/rov_link.py` — two independent MAVLink↔JSON bridges (§11). Internal to GUI-ROV.
- **DUP-2**: `ros2_ws mission_fsm.py` vs. `GUI-ROV autonomy/fsm/mission5.py` — two mission state machines with overlapping semantics and no documented relationship (§11).
- **DUP-3 (JUSTIFIED)**: `ros2_ws hook_detector.py` explicitly ported from `GUI-ROV autonomy/vision/hook_detect.py` (§11) — cited as a positive example, not a problem.

### Inconsistencies

- **I-1**: Heading unit (rad in `ros2_ws`, deg in GUI-ROV) — correctly bridged at `gui_bridge`, but undocumented as a formal contract rule (§12).
- **I-2**: Command-timeout *response* differs (zero-force in `ros2_ws` vs. neutral-hold in GUI-ROV) despite both using a 0.5s window (§15).
- **I-3**: `/hydroships/gripper/detach` has two independent publishers (`gripper_controller.py` and `mission_fsm.py`) — architecturally explained (§10) but still a literal multi-publisher-to-same-topic pattern worth naming explicitly.
- **I-4 (GUI-ROV-internal)**: Gripper PWM channel/value constants disagree across `gripper_controller.py`, `autonomy/rov_link.py`, and `CONTROL-MAPPING.md` (§11) — informational, does not cross the repo boundary.

### Unknowns

- **U-1**: Whether gz-sim Fortress's `OdometryPublisher` plugin internally broadcasts an odom→base_link TF despite no ROS node in this workspace doing so explicitly (§5 Frames) — not resolved by source reading alone.
- **U-2**: Exact provenance/intent of `temp`/`voltage`/`mode` placeholder fields in `gui_bridge_logic.build_telemetry()` (§6) — whether they're meant to be filled in later or are permanently cosmetic.
- **U-3**: Heading sign/rotation-direction convention match between REP-103 and GUI-ROV's compass convention (§12) — flagged UNKNOWN consistent with `docs/GUI-INTEGRATION.md`'s own `[VERIFY]` marker, not independently resolved by this audit.
- **U-4**: Whether `ros2_ws mission_fsm.py` and `GUI-ROV autonomy/fsm/mission5.py` are intended to converge, stay permanently separate, or one is meant to supersede the other (§11 DUP-2).
- **U-5**: Whether GUI-ROV's actual field deployment uses `rov_agent.py` or `autonomy/rov_link.py` as the live bridge (§11 DUP-1) — determines which one `gui_bridge.py`'s design intent should be validated against.

### Deferred Items

- **DF-1**: Gripper PWM constant reconciliation within GUI-ROV (I-4) — internal to GUI-ROV, out of `ros2_ws`-focused P1.0 scope, and not architecturally significant to the cross-repo boundary.
- **DF-2**: `CONTROL-MAPPING.md` stale reference to a non-existent `rov_gripper.py` file (§11) — a GUI-ROV documentation hygiene item, not an architecture question.
- **DF-3**: Detailed runtime verification of odom→base_link TF (U-1) — would require running the sim and inspecting `/tf`, not appropriate for a read-only static audit; deferred to whoever next runs the sim for unrelated reasons.

## 17. Recommended Interface Boundaries

These are **recommendations only** — nothing here is implemented by this task (§19/§20).

1. **Vehicle command**: Keep `gui_bridge`'s percent→wrench conversion as the sole crossing point; do not attempt a literal shared message format between ArduSub `MANUAL_CONTROL` and ROS `Twist`-as-wrench — the physical models are fundamentally different (firmware black-box mixing vs. explicit allocation) and forcing a shared format would hide that reality rather than resolve it.
2. **Vehicle state / telemetry**: Formalize the field list in §6/§14 into a single, versioned schema document (e.g. extend `docs/GUI-INTEGRATION.md` or create a small JSON-schema file) that both repos can reference, listing per field: name, unit, real-vs-placeholder status in `ros2_ws`, and source in GUI-ROV (`rov_agent.py` vs. `autonomy/rov_link.py` — see U-5 first).
3. **Mission state**: If cross-repo mission observability is ever wanted, add a `mission_fsm` state-telemetry topic (e.g. `/hydroships/mission/state`) as a **new, additive** interface — do not repurpose or rename any existing FSM internals. Explicitly **not recommended for immediate implementation** — P0 impact must be assessed first (§19).
4. **Perception result**: Current alignment (`hook_detector` ported from GUI-ROV) is healthy; recommend documenting `qr_detector`'s relationship to GUI-ROV's `autonomy/vision/qr_detect.py` the same way `hook_detector`'s relationship is documented, to close the asymmetry in `docs/GUI-INTEGRATION.md` (which covers hook but not QR provenance).
5. **Fault state**: A comms-loss/fault telemetry field on the `ros2_ws` side, analogous to GUI-ROV's `cmd_link`, would close D-3 — again additive, not a change to existing behavior.
6. **Safety/arming**: Any change here (D-4, override arbitration; I-2, timeout-response alignment) would touch `mission_fsm`, `teleop_gamepad`, or `thruster_allocator` — all P0-frozen. Recommendation is to **document the target design** now and implement only after P0-2.2b/APPROACH_QR work closes, explicitly tagged `WAIT FOR P0`.
7. **Coordinate frames**: No change recommended — the rad/deg split is already correctly handled at the one crossing point; recommend only that this be written down explicitly (as part of item 2's schema doc) rather than left as institutional knowledge.

## 18. P1 Implementation Backlog

| ID | Finding | Classification | Priority | Evidence | Dependency | Recommendation |
|---|---|---|---|---|---|---|
| P1-1 | No written, versioned telemetry/command schema shared between repos | DESIGN GAP (D-6) | P1 | §6, §14 | None — pure documentation | Write a schema doc (extends existing `docs/GUI-INTEGRATION.md`) |
| P1-2 | No mission-state telemetry exposed from `ros2_ws` | DESIGN GAP (D-1) | P2 | §8, §14 | Requires touching `mission_fsm.py` (P0-frozen) | Design only now; implement after P0 unfreezes; `WAIT FOR P0` |
| P1-3 | No comms-loss/fault telemetry field in `ros2_ws` | DESIGN GAP (D-3) | P2 | §15 | Requires touching `gui_bridge.py` and/or `stabilizer.py` | Design only now; `gui_bridge.py` itself is not P0-critical-path but is currently `[VERIFY]`-flagged by P0's own docs, so treat cautiously; `stabilizer.py` changes are P0-frozen |
| P1-4 | No manual-override arbitration between `mission_fsm` and teleop nodes | DESIGN GAP (D-4) | P2 | §15 | Requires touching `mission_fsm.py`/teleop nodes (P0-frozen) | Design only; `WAIT FOR P0` |
| P1-5 | `mission_fsm.ABORT` does not retract/release/alert | DESIGN GAP (D-5) | P3 | §15 | Requires touching `mission_fsm.py` (P0-frozen) | Design only; `WAIT FOR P0`; note this is adjacent to but distinct from the already-known P0 grab-publish bug (V-1) |
| P1-6 | Unclear relationship between `ros2_ws mission_fsm.py` and `GUI-ROV autonomy/fsm/mission5.py` | UNKNOWN (U-4) | P1 | §11 | None — requires a conversation with whoever owns each FSM, not code changes | Get an explicit statement of intent; document it once known |
| P1-7 | `mission_fsm`'s `DIVE→APPROACH_QR` gate depends on Gazebo-ground-truth `/hydroships/payload_pose` | DESIGN GAP (D-2) | P2 | §7, §8 | Requires touching `mission_fsm.py` (P0-frozen) and defining a hardware-side payload-pose source that doesn't exist yet in either repo | Design only; likely needs new perception capability in GUI-ROV's `autonomy/vision/`, not just a `ros2_ws` change; `WAIT FOR P0` |
| P1-8 | Command-timeout response differs (zero-force vs. neutral-hold) between the two stacks | INCONSISTENCY (I-2) | P3 | §15 | Requires touching `thruster_allocator.py` (P0-frozen) if changed on the sim side | Document as an intentional-or-not divergence; do not change without P0 sign-off |
| P1-9 | Determine which GUI-ROV bridge (`rov_agent.py` vs `autonomy/rov_link.py`) is the real integration target | UNKNOWN (U-5) | P1 | §11 | None — requires asking GUI-ROV's owner | Resolve before writing the P1-1 schema doc, since the schema should target the real one |

Priority key: P0 = blocks architecture/integration (none identified — nothing here blocks P0's own work), P1 = important before implementation, P2 = useful engineering improvement, P3 = technical debt / future.

## 19. P0 Compatibility Assessment

| Recommendation | P0 impact | Notes |
|---|---|---|
| P1-1 (schema doc) | **NONE** | Pure documentation, no code touched |
| P1-2 (mission-state telemetry) | **MEDIUM** | Requires adding a publisher to `mission_fsm.py` — even an additive change risks altering timing/behavior of a file under active P0 characterization. `WAIT FOR P0`. |
| P1-3 (fault telemetry) | **LOW–MEDIUM** | `gui_bridge.py` itself is not on the P0 critical path (P0's `NODES_REFERENCE.md`/`STATUS.md` milestones M1-M6 do not depend on it), but it is explicitly `[VERIFY]`-flagged as unfinished by P0's own docs — treat any touch as **MEDIUM** out of caution. `WAIT FOR P0` sign-off before touching, even though `stabilizer.py` (which would need no change) stays untouched. |
| P1-4 (override arbitration) | **HIGH** | Directly touches `mission_fsm.py` and/or teleop nodes — both explicitly frozen. `WAIT FOR P0`. |
| P1-5 (ABORT behavior) | **HIGH** | Directly touches `mission_fsm.py`. `WAIT FOR P0`. |
| P1-6 (FSM relationship clarification) | **NONE** | A conversation/documentation item, not a code change |
| P1-7 (payload-pose gate) | **HIGH** | Directly touches `mission_fsm.py`'s `DIVE`→`APPROACH_QR` transition logic, which is precisely the state pair P0-2.2b (APPROACH_QR acceptance evidence) is currently characterizing. `WAIT FOR P0`. |
| P1-8 (timeout response) | **MEDIUM–HIGH** | Touches `thruster_allocator.py`'s watchdog behavior, which is part of the frozen allocator. `WAIT FOR P0`. |
| P1-9 (bridge target clarification) | **NONE** | GUI-ROV-side investigation only |

No item in this backlog is recommended for implementation under P1.0. All `MEDIUM`/`HIGH` items are explicitly deferred pending P0's own progress (P0-2.2b / APPROACH_QR acceptance evidence).

## 20. Items Explicitly NOT Changed

Per the P1.0 mandate, this audit made **zero** changes to:

- `mission_fsm.py`, `stabilizer.py`, `thruster_allocator.py`/`allocation.py`, `qr_detector.py`/`qr_logic.py`, `recorder_qr.py` (does not exist in this workspace — confirmed by search; `qr_detector.py`/`qr_logic.py` is the actual QR pipeline), any physics/buoyancy/vehicle-dynamics files (`hydroships.urdf.xacro`, `rov_params.yaml`, `kki_arena.sdf`), APPROACH_QR controller logic, or any P0 experiment configuration.
- Any file in `GUI-ROV` — the entire repository was read-only for this audit; `git status --short` in `/home/rasya/GUI-ROV` remains clean.
- Any launch file, parameter YAML, message/service/action definition, or build configuration in `ros2_ws`.
- No topic remapping, renaming, interface migration, or package restructuring was performed or is contained in this document as anything other than a **recommendation** (§17/§18).

The only file created by this task is this document, `docs/P1-0-ARCHITECTURE-AUDIT.md`.
