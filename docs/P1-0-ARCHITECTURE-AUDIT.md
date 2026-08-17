# P1-0: Cross-Repo Architecture Audit — `ros2_ws` ↔ `GUI-ROV`

Read-only architecture/contract survey. No code, config, launch, FSM, controller, or GUI behavior was changed to produce this document. P0-2 (QR candidate ordering) remains frozen and unreviewed here; any P0-adjacent issue found is recorded as a backlog observation only.

Repos audited:
- `ros2_ws` — ROS 2 Humble + Gazebo Fortress simulation (this repo).
- `GUI-ROV` — browser dashboard + Raspberry Pi bridge (`/home/rasya/GUI-ROV`).

---

## 1. Executive architecture summary

The two repositories were built independently toward the same eventual system and share *intent* but not a *verified contract*. `ros2_ws` contains a ROS2 node, `gui_bridge`, explicitly built to speak the same UDP-JSON/port scheme (`14550`/`14551`) that `GUI-ROV` expects — but `GUI-ROV`'s actual production bridge is a separate, independently-evolved Python script (`rov_agent.py`, pymavlink→Pixhawk) that was never reconciled with `gui_bridge`'s implementation. The two were designed against the same informal idea of the protocol (confirmed by matching ports and similar field names) but have since drifted on value ranges, mode vocabulary, and telemetry completeness.

Beyond the bridge, `GUI-ROV` also contains a second, fuller autonomy stack (`autonomy/rov_link.py` + `autonomy/fsm/mission5.py`) that duplicates `ros2_ws`'s `mission_fsm.py` — a five/ten-state docking-and-hook mission FSM — as an entirely separate implementation with its own state names and its own telemetry sub-schema (`mission5`). Neither FSM is aware of the other.

Within `ros2_ws` itself, the architecture is otherwise clean: ROS nodes are consistently split into thin ROS wrappers + pure-logic modules, topics are used exclusively (no services/actions to reconcile), and the ground-truth-vs-hardware boundary is mostly well documented in `docs/HARDWARE.md`. The main structural gaps are internal: no TF tree, two live and incompatible depth-sign conventions on different topics, triplicated quaternion math, and several stale documentation pages.

**Bottom line:** the boundary exists and both sides converge on similar concepts (heading/depth/roll/pitch/arm/light/gripper/mode), but it is currently an **IMPLICIT, unverified contract with concrete DRIFT** on axis scaling and mode strings, and a **DUPLICATED** mission layer. It is not integration-ready as-is; it is a solid foundation for a short, well-scoped reconciliation pass (see §12).

---

## 2. Repository/module responsibility map

### `ros2_ws`

| Package | Responsibility | Notes |
|---|---|---|
| `hydroships_control` | All Python nodes + pure logic: control (`stabilizer`, `pid.py`, `thruster_allocator`, `allocation.py`), perception (`qr_detector`, `hook_detector`, `qr_logic.py`, `hook_logic.py`, `image_util.py`), manipulator (`gripper_controller`, `gripper_logic.py`), autonomy (`mission_fsm.py`), teleop (`teleop_keyboard`, `teleop_stabilized`, `teleop_gamepad`), telemetry bridge (`gui_bridge`, `gui_bridge_logic.py`), sensing (`depth_publisher.py`) | Consistent ROS-wrapper/pure-logic split across all 10 nodes; owns every subsystem except URDF and world definition. |
| `hydroships_description` | URDF/xacro, `rov_params.yaml` (mass/inertia/hydrodynamics), meshes | Feeds Gazebo plugins only, not read at Python runtime. |
| `hydroships_gazebo` | Worlds, Gazebo models/plugins, `bridge.yaml` (ros_gz_bridge topic map), sim launch | Owns simulation-only ground-truth generation (odometry, cameras, thruster forces). |
| `hydroships_bringup` | Top-level launch composition (stabilized/mission/gui/sim) | No logic of its own. |

### `GUI-ROV`

| Component | Responsibility | Notes |
|---|---|---|
| `public/js/app.js` + `public/js/pages/*` | Browser dashboard: telemetry rendering, joystick/keyboard input, tab UI | No build step, vanilla JS/ES modules. |
| `public/js/manipulator/*` | Gripper/rotate command builder (`protocol.js` is the only formal outgoing-packet schema in either repo) | |
| `server/server.js` | Node.js process: static file server, WebSocket server (port 8080), **owns the actual UDP socket** to/from the vehicle, has a `--sim` mode | This is the transport hub; the browser never touches UDP directly. |
| `rov_agent.py` (repo root) | **Real production ROV-side bridge** — pymavlink↔Pixhawk, builds/serializes telemetry, dispatches JSON commands | Functionally the `GUI-ROV`-side counterpart to `ros2_ws`'s `gui_bridge`, but written independently and never validated against it. |
| `autonomy/rov_link.py` + `autonomy/fsm/mission5.py` | A second, separate MAVLink bridge + a full docking/hook mission FSM (`mission5`), plus `autonomy/vision/` | Parallels `ros2_ws`'s `mission_fsm.py` with no shared code or contract. |
| `rov_axes.py`, `rov_modes.py`, `rov_pid.py`, `rov_heading.py`, `attitude_filter.py`, `gripper_controller.py` | Pure-logic support modules for `rov_agent.py`, unit-tested (`test_rov_*.py`) | Same "ROS-wrapper + pure module" instinct as `ros2_ws`, independently arrived at. |

**Ownership overlap / ambiguity:** Both repos claim ownership of "the mission FSM" and "the telemetry bridge." Neither repo's mission FSM is authoritative over the other; there is no single owner of "current mission state" as seen by an operator.

---

## 3. ROS interface inventory (`ros2_ws`)

All interfaces are topic-based; no services or actions exist anywhere in the workspace.

| Topic | Type | Producer(s) | Consumer(s) | Purpose |
|---|---|---|---|---|
| `/hydroships/odom` | `nav_msgs/Odometry` | Gazebo `odometry-publisher-system` plugin (ground truth, bridged) | `stabilizer`, `mission_fsm`, `gui_bridge`, `depth_publisher`, `teleop_gamepad` | Pose/twist, ground truth |
| `/hydroships/depth` | `std_msgs/Float64` | `depth_publisher.py` (`max(0, -z)` from odom) | `mission_fsm`, `gui_bridge` | Positive-down depth (m) |
| `/hydroships/cmd_vel` | `geometry_msgs/Twist` (used as 6-DOF wrench) | `stabilizer`, `teleop_keyboard`, `teleop_gamepad` (conditionally), `gui_bridge` | `thruster_allocator` | Final commanded wrench |
| `/hydroships/manual/cmd` | `Twist` (Fx/Fy only) | `teleop_stabilized`, `teleop_gamepad` (conditionally), `mission_fsm` | `stabilizer` | Manual surge/sway passthrough during stabilized flight |
| `/hydroships/setpoint/depth`, `/hydroships/setpoint/heading` | `Float64` | `teleop_stabilized`, `teleop_gamepad`, `mission_fsm` | `stabilizer` | PID setpoints |
| `/hydroships/control_mode` | `std_msgs/String` (`manual`/`depth_hold`/`poshold`) | `teleop_gamepad` | `stabilizer` | Selects which PID loops are active |
| `/hydroships/thruster_{1..6}/thrust` | `Float64` (N) | `thruster_allocator` | Gazebo `thruster-system` plugin | Per-thruster force |
| `/hydroships/camera_{bottom,front}/image_raw`, `/camera_info` | `sensor_msgs/Image`, `CameraInfo` | Gazebo sensors (bridged) | `qr_detector`, `hook_detector` | Vision input |
| `/hydroships/qr_result` | `String` (A/B/C/D) | `qr_detector` | `mission_fsm` | Decoded payload letter |
| `/hydroships/qr_offset` | `geometry_msgs/PointStamped` (`frame_id=camera_bottom_link`) | `qr_detector` | `mission_fsm`, `gripper_controller` | Normalized pixel offset + size proxy |
| `/hydroships/qr_offset_debug` | `String` | `qr_detector` | (debug/instrumentation only) | CSV corner-point dump |
| `/hydroships/hook_offset` | `PointStamped` (`frame_id=camera_front_link`) | `hook_detector` | `mission_fsm` | Hook visual-servo offset |
| `/hydroships/payload_pose` | `PointStamped` (QoS: depth=1, TRANSIENT_LOCAL) | (external/sim spawner) | `mission_fsm` | Payload ground-truth pose |
| `/hydroships/payload/spawned` | `Empty` (TRANSIENT_LOCAL) | (external/sim spawner) | `gripper_controller` | Payload-ready latch |
| `/hydroships/gripper/command` | `String` (`open`/`close`) | `mission_fsm` (declared, **never published — see §10 finding**), `teleop_gamepad`, `gui_bridge` | `gripper_controller` | Gripper open/close |
| `/hydroships/gripper_{left,right}/cmd` | `Float64` (rad) | `gripper_controller` | Gazebo joint controllers | Cosmetic finger actuation |
| `/hydroships/gripper/attach`, `/gripper/detach` | `Empty` | `gripper_controller` (attach), `mission_fsm` (detach) | Gazebo `DetachableJoint` plugin | Load-bearing attach/detach |
| `/hydroships/mission/start_autonomous` | `Empty` | (operator/launch trigger only — **not wired from `gui_bridge`**) | `mission_fsm` | Advances `WAIT_TRIGGER → APPROACH_HOOK` |
| `/joy` | `sensor_msgs/Joy` | OS joystick driver | `teleop_gamepad` | Raw gamepad input |

No message in this workspace is custom — everything is standard `std_msgs`/`geometry_msgs`/`nav_msgs`/`sensor_msgs`. `PointStamped` is overloaded as a generic 3-tuple channel (offset x/y + size/distance proxy in `z`) rather than a true stamped point — an IMPLICIT convention, not documented as a message contract anywhere.

---

## 4. `gui_bridge` boundary trace

```
ROS topics ─▶ gui_bridge.py (_on_odom/_on_depth) ─▶ GuiBridgeLogic.build_telemetry()
           ─▶ JSON encode ─▶ UDP :14551 ─▶ [GUI-ROV expects: server.js binds here] ─▶ WS ─▶ browser

browser ─▶ WS ─▶ server.js ─▶ UDP :14550 ─▶ [ros2_ws expects: gui_bridge._poll_cmd] 
        ─▶ GuiBridgeLogic.on_command() ─▶ ROS topics (/hydroships/cmd_vel, /gripper/command)
```

**Telemetry direction (ROS → GUI), what `gui_bridge` actually sends** (`gui_bridge_logic.py:110-123`):
```json
{"heading": 0-360, "depth": m, "roll": deg, "pitch": deg,
 "temp": 0.0, "voltage": 0.0, "armed": bool, "light": bool, "mode": "manual"}
```
- `heading`/`roll`/`pitch` derived from `/hydroships/odom` quaternion via a **locally duplicated** `_yaw_rpy()` (`gui_bridge.py:36-47`), not shared with `stabilizer.py`'s equivalent math.
- `depth` passthrough from `/hydroships/depth` — already positive-down, matches GUI-ROV's "pre-tared" assumption. **CONTRACTED.**
- `temp`/`voltage` are hardcoded default-argument zeros — no ROS topic feeds them at all.
- `mode` is a constant string set once in `__init__` and never updated — never reflects `control_mode` or FSM state.

**What GUI-ROV actually expects on this channel** (from `rov_agent.py`'s `state` dict / `app.js:applyTelemetry`): the same four physical fields, **plus** `poshold` (bool), `depth_target` (m), `depth_hold` (bool), and an optional `mission5{state, active_cam, distance_z, offset_x, offset_y}` object. None of the "plus" fields exist in `gui_bridge`'s output — GUI degrades silently (renders `"—"`, no error raised).

**Command direction (GUI → ROS), what `gui_bridge` actually accepts** (`gui_bridge_logic.py:62-89`, `on_command`):
| name | accepted value | effect |
|---|---|---|
| `surge`,`sway`,`yaw`,`heave` | float, **clamped to -100..100** | stored as percent, later scaled by fixed gains into `/hydroships/cmd_vel` |
| `arm` | bool | gates all wrench output (fail-safe zero when disarmed) |
| `stop` | (any) | zeroes axes + disarms |
| `light` | bool | stored in-process only, **no topic publish exists** |
| `gripper` | `open`/`close` | → `/hydroships/gripper/command` |
| `mode`, `pid`, `pool` | (any) | explicitly ignored, `return {}` |

**What GUI-ROV actually sends** (`server.js`/`app.js`, confirmed against `rov_agent.py`'s handler): `surge/sway/yaw/heave` as **-1000..1000** (heave separately remapped 0..1000 by the Pi, not by `gui_bridge`), plus `arm`, `stop`, `light`, `pilot_mode` (lowercase mode names), `control_mode` (manual/autonomous), `gripper` (open/close/**stop**), `gripper_rotate`, `depth_set`, `depth_hold`, `set_surface`, `pool_depth`, `pid`, `motor_test`, `param_get/list/set`, `mavlink_stream`, and several manipulator/light-dimmer commands with no ROS-side counterpart at all.

**Boundary characterization:** the physical/attitude subset (heading, depth, roll, pitch, arm, light-intent, gripper open/close) is **IMPLICIT but broadly compatible**. The axis-range and mode-vocabulary subset is **DRIFT** (see §10). Everything else GUI-ROV sends (`pilot_mode`, `depth_hold`, `pool_depth`, `pid`, `motor_test`, `param_*`, `mavlink_stream`, manipulator extras) is a **GAP** — `gui_bridge` has no handler and silently drops it via the `mode/pid/pool: ignore` branch or falls through unmatched.

---

## 5. Parameter/configuration map

| Source | Authoritative for | Read by | Notes |
|---|---|---|---|
| `hydroships_control/config/gains.yaml` | Stabilizer PID gains (depth/heading/pitch/roll), buoyancy_ff, target setpoints, hold-enable flags | `stabilizer.py` via `declare_parameter` | Current values: depth `kp=35.0/ki=4.0/kd=30.0`. |
| `hydroships_control/config/gamepad.yaml` | Teleop gamepad axis/button mapping, deadzone/expo, rate limits | `teleop_gamepad.py` | 1:1 match between yaml keys and `declare_parameter` calls — no drift found. |
| `hydroships_description/config/rov_params.yaml` | Mass/inertia/hydrodynamic coefficients (mostly BlueROV2-scaled estimates) | Baked into URDF at build time via `xacro.load_yaml`; **not read at Python runtime** | Physical model, not a runtime config. |
| `hydroships_gazebo/config/bridge.yaml` | ros_gz_bridge topic map (15 entries: clock/odom/IMU/cameras/thrusters/gripper) | `ros_gz_bridge` process | Verified topic-name-exact match against Python nodes' subscriptions. |
| `mission_fsm.py` (~50 `declare_parameter` calls, no yaml) | Mission thresholds/timeouts, `start_state`, `start_wall`, `wall_dist` (default `2.15`), score weights | `mission_fsm.py` only; overridable via `hydroships_mission.launch.py` launch args | No yaml backing — code is the sole source of truth. |
| `gui_bridge.py` params | `cmd_port=14550`, `telem_host=127.0.0.1`, `telem_port=14551`, `telem_hz=10.0`, `surge_gain=0.40`, `sway_gain=0.40`, `heave_gain=0.30`, `yaw_gain=0.12` | `gui_bridge.py` | Overridable via `hydroships_gui.launch.py`. Ports match GUI-ROV's `UDP_OUT=14550/UDP_IN=14551` defaults — the one place the two repos' defaults were clearly designed in agreement. |
| `GUI-ROV` env vars | `WS_PORT=8080`, `UDP_IN=14551`, `UDP_OUT=14550`, `RPI_ADDR=192.168.2.2` (`server.js`), `LAPTOP_IP=192.168.2.1` (`rov_agent.py`) | `server.js`, `rov_agent.py` | Port numbers agree with `gui_bridge` defaults; `telem_host=127.0.0.1` in `gui_bridge.py` is loopback-only and would need to be set to the actual GUI-host IP for any real cross-machine test — currently only correct for same-host testing. |

**Documented-vs-actual drift found (independent of GUI-ROV):**
- `docs/CONFIG_REFERENCE.md:47-51` states `depth.kp=60.0/ki=8.0/kd=40.0` as current; actual `gains.yaml:14-18` is `35.0/4.0/30.0` (yaml's own comment says it was lowered after a mass-model correction — doc was never updated). **DRIFT, doc-only, Low severity.**
- `docs/CONFIG_REFERENCE.md:165` and `docs/STATUS.md:40` both state `wall_dist=2.30`; actual code default (`mission_fsm.py:126`) is `2.15`. **DRIFT, doc-only, Low severity.**
- `docs/NODES_REFERENCE.md:154` lists a `/hydroships/qr_request` (`Empty`) publish topic for `mission_fsm` that does not exist anywhere in the code (grep-confirmed). **DRIFT, doc-only phantom interface, Low severity.**

---

## 6. Frame and coordinate convention audit

- **No TF tree exists.** No `tf2`/`TransformBroadcaster`/static transform publisher anywhere in `hydroships_control`. Gazebo's `odometry-publisher-system` plugin (`hydroships.urdf.xacro:402-410`, `odom_frame=odom`, `robot_base_frame=base_link`) is bridged straight into a plain `nav_msgs/Odometry` message — no `/tf` or `/tf_static` is ever published. Any RViz/tf-based consumer would see nothing.
- **REP-103 is explicitly the intended convention** (`rov_params.yaml:19`, `thruster_allocator.py:9-12` docstrings): x-forward, y-left, z-up; wrench mapping `Fx=surge, Fy=sway, Fz=heave, Mx=roll, My=pitch, Mz=yaw`.
- **Quaternion→RPY math is triplicated**, independently implemented in `stabilizer.py:40-56`, `mission_fsm.py:45-48` (yaw-only), and `gui_bridge.py:36-47` (full RPY). `teleop_gamepad.py` imports `stabilizer.py`'s version for yaw only — partial reuse, not full consolidation. **DUPLICATED, Low severity (no observed behavioral divergence, but a maintenance risk if one copy is ever fixed and the others aren't).**
- **Two live, incompatible depth-sign conventions coexist on different topics, by design but undocumented as a system-wide contract:**
  - `/hydroships/depth`: positive-down, `≥0` (`depth_publisher.py:28`, `max(0, -z)`) — consumed by `mission_fsm` state logic and `gui_bridge` telemetry.
  - `/hydroships/setpoint/depth`: negative-down, matches raw odom `z` directly (`mission_fsm.py:367-368`, `m.data = -abs(d_pos)`) — consumed by `stabilizer`'s PID.
  - **IMPLICIT, Medium severity** — a new consumer subscribing to "depth" without reading both producers carefully will get the sign backwards. GUI-ROV's `depth` field (positive-down, pre-tared) matches `/hydroships/depth`'s convention, not the setpoint topic's — so the GUI-facing half of this is fine, but it's a trap for any future ROS-side integration work.
- GUI-ROV's heading (`0-360°, 0=N, clockwise`) is produced by `gui_bridge`'s raw REP-103 yaw with **no compass/north-alignment offset applied** (`gui_bridge_logic.py:105-108`, already flagged as VERIFY/OPEN in `docs/GUI-INTEGRATION.md:48-50`). On real hardware with a real compass, this offset would matter; in sim, "north" is an arbitrary world-frame convention. **IMPLICIT, Medium severity for eventual hardware transition, not a sim bug today.**

---

## 7. Telemetry contract map

| Telemetry concept | `ros2_ws` source | `gui_bridge` output field | GUI-ROV expected field | Status |
|---|---|---|---|---|
| Heading | `/hydroships/odom` yaw → deg | `heading` | `heading` (deg, 0-360, CW) | **CONTRACTED** (format matches; no north-alignment, see §6) |
| Depth | `/hydroships/depth` | `depth` | `depth` (m, positive-down, pre-tared) | **CONTRACTED** |
| Roll/Pitch | `/hydroships/odom` quaternion | `roll`, `pitch` | `roll`, `pitch` (deg) | **CONTRACTED** |
| Armed state | `GuiBridgeLogic.armed` (in-process) | `armed` | `armed` | **CONTRACTED** (echo of last commanded arm, not independently verified against thruster state) |
| Light state | `GuiBridgeLogic.light` (in-process) | `light` | `light` | **IMPLICIT** — no physical light actuator or topic exists in `ros2_ws` at all; field is pure UI-state echo on both sides |
| Pilot mode | hardcoded constant | `mode: "manual"` (always) | `mode` (ArduSub HEARTBEAT strings: `MANUAL`/`STABILIZE`/`ALT_HOLD`/`ACRO`) | **DRIFT** — case and vocabulary mismatch; GUI mode-tab highlighting would stick on whatever it defaults to |
| Pos-hold flag | not produced | (absent) | `poshold` (bool) | **GAP** |
| Depth-hold state | `/hydroships/control_mode` exists but not surfaced via `gui_bridge` | (absent) | `depth_target`, `depth_hold` | **GAP** |
| Battery voltage | not produced | `voltage: 0.0` | `voltage` | **GAP** (stub on both sides — not sim-only, genuinely unimplemented) |
| Temperature | not produced | `temp: 0.0` | `temp` | **GAP** (same as above) |
| Mission/FSM state | `mission_fsm.py` internal `St` enum, never published | (absent) | `mission5.state` (from GUI-ROV's *own* `autonomy/fsm/mission5.py`, unrelated FSM) | **GAP + DUPLICATED** — no path exists from `ros2_ws`'s mission FSM to any GUI telemetry field; the `mission5` field GUI-ROV does receive comes from a wholly separate, ROS-independent FSM |
| Mission score | `mission_fsm.py._print_score()`, logged only | (absent) | (no equivalent field) | **GAP** — score is computed but never exposed anywhere |
| Perception offsets (QR/hook) | `/hydroships/qr_offset`, `/hydroships/hook_offset` | (absent) | `mission5.distance_z/offset_x/offset_y` (from GUI-ROV's own vision stack) | **DUPLICATED** — both repos independently compute and expose visual-servo offsets under different schemas; not reconciled |

---

## 8. Mission interface map

| Aspect | `ros2_ws` (`mission_fsm.py`) | `GUI-ROV` (`autonomy/fsm/mission5.py`) |
|---|---|---|
| States | `IDLE, DIVE, APPROACH_QR, GRAB, NAV_WALL, HANG, SURFACE, WAIT_TRIGGER, APPROACH_HOOK, AUTO_RELEASE, DONE, ABORT` | `IDLE, DIVE, SCAN_QR, GRAB, NAV_WALL, HANG, SURFACE, DOCK, M5_REDIVE, M5_DOCK, M5_ENGAGE, M5_UNHOOK, M5_ASCEND, M5_FALLBACK, DONE, ABORT` |
| Entry/start control | `start_state`/`start_wall` launch params (static, not runtime-triggerable); auto-starts after `start_delay` once depth data arrives | Presumably driven by `autonomy/rov_link.py`'s own lifecycle (not audited in depth here — out of `ros2_ws` scope) |
| Runtime trigger | `/hydroships/mission/start_autonomous` (`Empty`), only honored in `WAIT_TRIGGER`; **not wired from `gui_bridge` at all** | GUI-ROV's `mission5` panel displays state but the audit did not find a corresponding "start/advance" command wired through `server.js`/`rov_agent.py` to `mission5` in the material reviewed |
| Stop/reset | **None** — only `ABORT`/`DONE` natural terminal states; must kill/relaunch process | Not audited in depth (outside `ros2_ws`) |
| Status exposed to operator | **None** — no ROS topic publishes current `St`/score; only logged | `mission5{state, active_cam, distance_z, offset_x, offset_y}` IS exposed to the GUI |
| Separation of intent/autonomy/control/operator command | Mission intent (`mission_fsm`) → autonomy logic (same file) → controller execution (`stabilizer`/`thruster_allocator`) is a clean three-layer split *within* `ros2_ws`. Operator command only enters via `start_state` (pre-mission) and the single mid-mission trigger topic. | GUI has a `manual`/`autonomous` control-mode gate that is meant to suppress joystick input during autonomous operation — this concept has no `ros2_ws` counterpart (nothing in `ros2_ws` gates manual teleop off during an active `mission_fsm` run). |

**Key finding: these are two independently-designed FSMs with overlapping purpose (QR-scan → grab → wall-nav → hang → surface → hook-engage/release) and no shared vocabulary.** State name overlap (`IDLE`, `DIVE`, `GRAB`, `NAV_WALL`, `HANG`, `SURFACE`, `DONE`, `ABORT`) is coincidental/convergent-design, not a real contract — `APPROACH_QR` vs `SCAN_QR`, `WAIT_TRIGGER`/`APPROACH_HOOK`/`AUTO_RELEASE` vs `DOCK`/`M5_*` diverge immediately. **DUPLICATED, High severity** — this is the largest structural integration gap found, since it means "what is the mission doing right now" has two different, non-interoperating answers depending which repo's FSM is actually running against the real vehicle.

---

## 9. Simulation-to-target boundary analysis

| Subsystem | Sim reality (`ros2_ws`) | Target-hardware path | Classification |
|---|---|---|---|
| Odometry | Gazebo `odometry-publisher-system` — pure ground truth, no fusion/EKF | No IMU-fusion or localization node exists in `ros2_ws`; real vehicle would need one | **SIM-ONLY LEAK** — nothing today would replace this on hardware |
| Depth | `depth_publisher.py` derives from ground-truth odom z; `docs/HARDWARE.md` explicitly notes "no pressure sensor in sim" | Needs new `depth_sensor_driver` node (MS5837 I2C) publishing `Float64` to the same topic name — plan already documented in `HARDWARE.md` | **Adapter required, well-scoped** |
| Thrusters | `gz-sim-thruster-system` consumes Newton commands directly | No ESC/PWM driver exists in `ros2_ws` | **Adapter required** |
| Cameras | Gazebo sensor plugin; `qr_detector.py` explicitly warns intrinsics are simulation-only, not physically calibrated | Needs `usb_cam`/`v4l2_camera` + real calibration | **SIM-ONLY LEAK (documented)** |
| Gripper | `DetachableJoint` plugin = rigid attach, not physical jaw force | No servo/PWM driver; grasp mechanics unvalidated per `HARDWARE.md` | **SIM-ONLY LEAK (documented)** |
| IMU | Bridged to `/hydroships/imu` but **not subscribed by any node** — orientation universally comes from ground-truth odom instead | On hardware, odom ground truth disappears; nothing currently falls back to the (already-bridged-but-unused) IMU | **GAP** — the "just swap in hardware drivers" story is incomplete: there's no fusion node to combine a real IMU + depth into anything resembling `/hydroships/odom` |
| Vehicle control mixing | `thruster_allocator.py` (software damped pseudo-inverse) fully replaces PX4-style mixing | No MAVLink/Pixhawk in `ros2_ws` at all | Intended architecture: ROS2 owns control end-to-end, bypassing ArduSub/PX4 mixing entirely |
| GUI-ROV's actual target path | N/A | `rov_agent.py`/`autonomy/rov_link.py` talk pymavlink to a **real Pixhawk running ArduSub**, which does its own mixing | **Architecturally divergent from `ros2_ws`'s intended control path** — see §11 |

**Note the architectural fork:** `ros2_ws` is designed as a ROS2-native control stack that does its own thrust allocation and was never meant to go through ArduSub/Pixhawk mixing. `GUI-ROV`'s actual hardware bridge (`rov_agent.py`) assumes an ArduSub/Pixhawk flight controller in the loop (MANUAL_CONTROL MAVLink messages, ArduSub mode names, `MAV_CMD_DO_MOTOR_TEST`, etc.). **These are two different target-hardware architectures**, not just two bridges with drifted field names — this is the single most consequential finding in this audit (see §11).

---

## 10. Findings table

| # | Finding | Classification | Severity | Evidence |
|---|---|---|---|---|
| 1 | `ros2_ws`'s target control architecture (ROS2-native thrust allocation) and `GUI-ROV`'s actual hardware bridge (ArduSub/Pixhawk MAVLink mixing) are two different, incompatible target-hardware designs, not just drifted field names | GAP | **Critical** | `thruster_allocator.py` (software mixing) vs `rov_agent.py`/`README-WORK.md:210-291` (MANUAL_CONTROL → Pixhawk) |
| 2 | Axis command range mismatch: GUI sends `surge/sway/yaw/heave` as `-1000..1000`; `gui_bridge_logic.on_command` clamps to `-100..100` | DRIFT | **High** | `gui_bridge_logic.py:62-89` vs GUI-ROV `server.js` `clampAxis`/`app.js` axis send code |
| 3 | Mode string vocabulary drift: GUI matches exact ArduSub HEARTBEAT strings (`MANUAL`/`STABILIZE`/`ALT_HOLD`/`ACRO`); `gui_bridge` always reports constant `"manual"` | DRIFT | **High** | `gui_bridge_logic.py` (`mode` never mutated) vs `app.js:296-301`, `rov_modes.py:72-79` |
| 4 | Two independently-designed mission FSMs (`mission_fsm.py` vs `autonomy/fsm/mission5.py`) with divergent state names and no shared status channel | DUPLICATED | **High** | §8 above |
| 5 | `mission_fsm.py`'s mission state/score is never published to any topic; `gui_bridge` has no mission telemetry at all | GAP | Medium | `mission_fsm.py` (`_print_score` logs only); `gui_bridge_logic.build_telemetry` (no mission field) |
| 6 | GUI-ROV's `poshold`, `depth_target`, `depth_hold` telemetry fields have no `gui_bridge` source | GAP | Medium | `gui_bridge_logic.py:110-123` vs `app.js:294,313,326-331` |
| 7 | `voltage`/`temp` are hardcoded zero stubs on the ROS side, with no backing sensor in sim or hardware anywhere in either repo | GAP | Medium | `gui_bridge_logic.py` defaults; GUI-ROV `rov_agent.py` state dict (no real source either) |
| 8 | Dual, incompatible depth-sign conventions live simultaneously on `/hydroships/depth` (positive-down) vs `/hydroships/setpoint/depth` (negative-down) | IMPLICIT | Medium | `depth_publisher.py:28` vs `mission_fsm.py:367-368` |
| 9 | No formal schema (TS interface/JSON-Schema/protobuf/dataclass) defines the wire contract on either side — both sides use plain dict/JSON literals | GAP | Medium | `ros2_ws`: `gui_bridge_logic.py` dict literal; `GUI-ROV`: confirmed no schema artifact exists except `manipulator/protocol.js` (partial) |
| 10 | `mission_fsm.py`'s `pub_grip` topic is declared but `.publish()` is never called — gripper never actually closes during autonomous `GRAB` | Backlog observation (pre-existing, P0-adjacent — **not modified**) | High (functional) | `mission_fsm.py:256`, confirmed by grep across the file; already tracked in `docs/STATUS.md` |
| 11 | `docs/CONFIG_REFERENCE.md` PID gains and `wall_dist` are stale vs actual `gains.yaml`/`mission_fsm.py` defaults | DRIFT (doc-only) | Low | `CONFIG_REFERENCE.md:47-51,165` vs `gains.yaml:14-18`, `mission_fsm.py:126` |
| 12 | `docs/NODES_REFERENCE.md` lists a `/hydroships/qr_request` topic that doesn't exist in code | DRIFT (doc-only) | Low | grep across `mission_fsm.py` — no such publisher |
| 13 | Quaternion→RPY math independently reimplemented in three files (`stabilizer.py`, `mission_fsm.py`, `gui_bridge.py`) | DUPLICATED | Low | file:line citations in §6 |
| 14 | No TF tree exists anywhere in `ros2_ws`; only raw `Odometry` messages carry pose | GAP (by design, not necessarily wrong) | Low | grep for `tf2`/`TransformBroadcaster` — no hits |
| 15 | GUI-ROV sends many commands (`pilot_mode`, `depth_hold`, `pool_depth`, `pid`, `motor_test`, `param_*`, `mavlink_stream`, manipulator extras) that `gui_bridge` has no handler for at all | GAP | Medium | `gui_bridge_logic.on_command` (only handles surge/sway/yaw/heave/arm/stop/light/gripper) vs full GUI-ROV command list in §4 |
| 16 | `gui_bridge`'s heading has no compass/north-alignment offset — fine in sim, an open question for hardware | IMPLICIT | Low (Medium at hardware transition) | `gui_bridge_logic.py:105-108`; already flagged VERIFY/OPEN in `docs/GUI-INTEGRATION.md:48-50` |

---

## 11. Top integration risks

1. **Architectural fork between the two repos' hardware targets (Finding #1).** If the eventual real vehicle is meant to run a Pixhawk/ArduSub flight controller (as `GUI-ROV`'s actual bridge assumes), `ros2_ws`'s ROS2-native `thruster_allocator` path is the wrong control architecture for that hardware — or vice versa, if the real vehicle has no flight controller and `ros2_ws`'s design is authoritative, `GUI-ROV`'s production bridge (`rov_agent.py`) needs to be retired in favor of `gui_bridge`. **This decision has not been made and blocks everything downstream.**
2. **Axis-range and mode-string drift (Findings #2, #3)** would silently break manual control and mode display the moment `gui_bridge` and the real `GUI-ROV` dashboard are pointed at each other — these are concrete, mechanical bugs, not design ambiguity, and are cheap to fix once the architecture question (#1) is resolved.
3. **Two disconnected mission FSMs (Finding #4)** mean "what is the mission doing" has no single source of truth across the boundary — an operator watching the GUI-ROV dashboard's `mission5` panel today would see nothing related to `ros2_ws`'s `mission_fsm` even if both were running.
4. **No formal schema (Finding #9)** on either side means every future field addition/rename risks silent drift exactly like Findings #2/#3 — there is no compiler or validator to catch it.
5. **Missing telemetry (voltage/temp/mission status, Findings #6/#7/#5)** limits operator diagnostic visibility below what a real fielded system needs, independent of the architecture question.

---

## 12. Recommended next steps

**Contract clarification (no code changes, decisions/documentation only):**
- Resolve the architecture fork (Risk #1): decide whether the target vehicle has a Pixhawk/ArduSub flight controller in the loop, and designate exactly one of `gui_bridge` or `rov_agent.py` as the real bridge going forward.
- Write down one formal telemetry schema and one formal command schema (even a plain markdown table or a shared JSON-Schema file) that both repos commit to — starting from the fields already common to both (heading/depth/roll/pitch/arm/light/gripper).
- Decide which mission FSM (`mission_fsm.py` or `mission5.py`) is authoritative, or explicitly scope them as serving different purposes if both are meant to persist.
- Fix stale docs (Findings #11, #12) — pure editing, zero risk.

**Implementation changes (deferred until the above is settled, to avoid building against the wrong target):**
- Align axis scaling and mode-string vocabulary between `gui_bridge_logic.py` and whatever GUI-ROV client is authoritative (Findings #2, #3).
- Wire `mission_fsm.py`'s state into a published status topic and surface it through `gui_bridge`'s telemetry (Finding #5).
- Add missing telemetry fields (`poshold`, `depth_target`, `depth_hold`) or explicitly drop them from the target contract (Finding #6).
- (Separately tracked, not part of this architecture work) fix `mission_fsm.py`'s `pub_grip` publish bug (Finding #10) — record as its own ticket, do not bundle with contract work.

**Proposed P1 sequence:**
- **P1.0** — this audit (complete).
- **P1.1 — Architecture decision: single target hardware/control path.** This is the single most important next task; every other integration fix depends on knowing whether `ros2_ws`'s ROS2-native allocator or `GUI-ROV`'s ArduSub/Pixhawk bridge is the real target.
- **P1.2** — Formal wire-contract document (telemetry + command schema) ratified by both repos, plus stale-doc cleanup (Findings #11, #12).
- **P1.3** — Implementation reconciliation: axis scaling, mode strings, mission-status telemetry, missing fields — scoped strictly to what P1.2's contract specifies.

---

## Final decision

**Integration-ready: No, not yet.** The two repositories converge on the same broad concepts and even the same UDP ports, but currently rest on an unverified, partially-drifted implicit contract, and — more fundamentally — appear to target two different control architectures (ROS2-native thrust allocation vs. ArduSub/Pixhawk mixing). Neither side has bugs so severe that the *sim-only* system (this repo) is broken today; the risk is entirely in what happens when `ros2_ws` and `GUI-ROV` are pointed at each other or at the same real vehicle.

**Top risks (ranked):**
1. Architectural fork over target hardware/control path (Finding #1, Risk #1).
2. Axis-range and mode-string drift (Findings #2–#3).
3. Duplicated, disconnected mission FSMs (Finding #4).
4. No formal schema to prevent future drift (Finding #9).
5. Incomplete telemetry (mission status, voltage/temp) limiting operator visibility (Findings #5–#7).

**Single most important P1.1 task:** Make the architecture decision — designate one control/hardware path (ROS2-native allocator vs. ArduSub/Pixhawk) and one bridge implementation (`gui_bridge` vs. `rov_agent.py`) as authoritative. Every other finding in this report is either downstream of that decision or independent, low-risk cleanup.

**Proposed P1 sequence:** P1.1 (architecture decision) → P1.2 (formal contract + doc cleanup) → P1.3 (implementation reconciliation, scoped to the ratified contract).
