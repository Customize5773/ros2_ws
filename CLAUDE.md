# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## graphify

This project has a knowledge graph at graphify-out/ with god nodes, community structure, and cross-file relationships.

Rules:
- For codebase questions, first run `graphify query "<question>"` when graphify-out/graph.json exists. Use `graphify path "<A>" "<B>"` for relationships and `graphify explain "<concept>"` for focused concepts. These return a scoped subgraph, usually much smaller than GRAPH_REPORT.md or raw grep output.
- If graphify-out/wiki/index.md exists, use it for broad navigation instead of raw source browsing.
- Read graphify-out/GRAPH_REPORT.md only for broad architecture review or when query/path/explain do not surface enough context.
- After modifying code, run `graphify update .` to keep the graph current (AST-only, no API cost).

## Project

HYDROships ROV — autonomous underwater vehicle for KKI 2026 competition. ROS 2 Humble + Gazebo Fortress (gz-sim 6) simulation. Documentation and commit messages are in Indonesian.

**Before making claims about what works, check `docs/STATUS.md` first** — it tracks per-milestone state (✅ verified in sim / 🧪 code exists but runtime-unverified / OPEN gap) and known regressions. `docs/CHANGELOG.md` has the full chronological history including reverted decisions. Don't trust stale in-code comments or old docs over `docs/STATUS.md`.

## Commands

```bash
# Install deps (once)
sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge ros-humble-xacro ros-humble-robot-state-publisher python3-numpy python3-opencv
cd ~/ros2_ws && rosdep install --from-paths src --ignore-src -r -y
# or, if apt opencv fails:
pip install -r requirements.txt

# Build (required after ANY change to code/URDF/world/config — launch reads from install/, not src/)
cd ~/ros2_ws && colcon build

# Source (every new terminal)
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Run tests for a package
colcon test --packages-select hydroships_control
colcon test-result --verbose

# Run a single test file directly (faster iteration, no colcon)
cd src/hydroships_control && python3 -m pytest test/test_pid.py -v
python3 -m pytest test/test_pid.py::test_specific_case -v
```

Sim launch scenarios (see `docs/HOW-TO-RUN.md` for the full menu with all args):

```bash
ros2 launch hydroships_gazebo sim.launch.py world:=kki_arena.sdf          # sim only
ros2 launch hydroships_bringup hydroships_stabilized.launch.py            # + stabilizer, then teleop_stabilized in a 2nd terminal
ros2 launch hydroships_bringup hydroships_mission.launch.py               # full autonomous FSM mission
ros2 launch hydroships_bringup hydroships_mission.launch.py start_state:=NAV_WALL start_wall:=B   # mid-mission start for isolated testing
ros2 launch hydroships_bringup hydroships_gui.launch.py                   # UDP-JSON bridge for team GUI (Customize5773/GUI-ROV)
```

Add `headless:=true` to any of the above for machines without GPU/EGL (CI/cloud). Note: headless camera rendering can break QR detection.

## Architecture

Single ROS 2 workspace (`src/`) with 4 packages:

- **hydroships_control** — all Python nodes and pure logic modules. See `docs/NODES_REFERENCE.md` for the full node-by-node breakdown (10 ROS2 nodes + 7 pure-logic modules). Node entry points are registered in `src/hydroships_control/setup.py`; add new nodes there. Each ROS2 node (e.g. `qr_detector.py`) delegates its actual logic to a pure module (`qr_logic.py`) so logic is unit-testable without ROS — follow this split for new nodes.
- **hydroships_description** — URDF/xacro robot model, meshes.
- **hydroships_gazebo** — worlds (`kki_arena.sdf` = competition arena, `pool_empty.sdf` = default), Gazebo models/plugins, sim launch.
- **hydroships_bringup** — top-level launch files that compose the above (stabilized / mission / gui / sim).

Control chain: `teleop_* or stabilizer` → `/hydroships/cmd_vel` (Twist used as a 6-DOF wrench, not a velocity) → `thruster_allocator` (damped pseudo-inverse, `allocation.py`) → 6× `/hydroships/thruster_N/thrust`. `thruster_allocator` has a 0.5s watchdog that zeroes thrust if `cmd_vel` stops.

`stabilizer` runs 4 independent PID loops (depth/heading/pitch/roll, `pid.py`) reading `/hydroships/odom` and publishing the full wrench to `cmd_vel`; gains live in `config/gains.yaml`.

Autonomous mission (`mission_fsm.py`) drives the full state machine: `IDLE → DIVE → APPROACH_QR → GRAB → NAV_WALL → HANG → SURFACE → WAIT_TRIGGER → APPROACH_HOOK → AUTO_RELEASE → (loop)`. It can be started from any mid-state via `start_state:=` for isolated testing — check `docs/STATUS.md` before assuming any given transition works, several have known regressions (e.g. `_st_grab` historically never published gripper "close").

Perception: `qr_detector` decodes payload QR letter + pixel offset (bottom/front cameras); `hook_detector` finds arena hooks via color/contour for `APPROACH_HOOK` visual servo. Both are thin ROS wrappers around pure-logic modules (`qr_logic.py`, `hook_logic.py`) plus `image_util.py` for `Image`→BGR conversion.

Manipulator: `gripper_controller` (+ `gripper_logic.py`) gates payload attach on QR-offset safety, and drives attach/detach through Gazebo's `DetachableJoint` plugin — the two visible "fingers" are cosmetic only, not load-bearing. See the "Manipulator (M5)" section in `docs/STATUS.md` for the current design (superseding older two-finger-gripper history in CHANGELOG).

`gui_bridge` (+ `gui_bridge_logic.py`) adapts UDP-JSON to/from ROS2 for the team's external GUI (`Customize5773/GUI-ROV`) — not MAVLink despite superficial similarity; see `docs/GUI-INTEGRATION.md`.

Physical-hardware notes (e.g. depth sensor MS5837 replacing sim ground-truth) live in `docs/HARDWARE.md`.

Other docs worth knowing about: `docs/CONFIG_REFERENCE.md` (per-parameter meanings), `docs/TUNING_GUIDE.md`, `docs/TROUBLESHOOTING.md`, `docs/ARCHITECTURE.md`.
