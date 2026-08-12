# P1.2B — Consumer-Integration Review for the P1.2A Orientation Estimator

Status: **read-only review**. No code was modified to produce this document.

## 1. Scope & inputs

P1.2A shipped a standalone orientation estimator — `attitude_filter_logic.py`
(pure complementary filter) + `attitude_estimator.py` (thin ROS2 node) —
subscribing to `/hydroships/imu` and publishing fused orientation to
`/hydroships/imu/filtered` (`sensor_msgs/Imu`, self-computed diagonal
`orientation_covariance`, `angular_velocity`/`linear_acceleration` passed
through unchanged). It was built with **zero consumers wired in**, by
explicit design — both `docs/P1-2-STATE-ESTIMATION-INTEGRATION-AUDIT.md`
(§11 item 5) and `docs/P1-2A-ORIENTATION-ESTIMATION-DESIGN.md` (§11) deferred
"integrate into stabilizer/mission_fsm" to a separate, later, gated task.

This document is that review: which existing nodes currently read
orientation off `/hydroships/odom` (Gazebo ground truth), whether each one
could swap to `/hydroships/imu/filtered`, and what would block it. It changes
no code — same read-only pattern as P1.2 and P1.2A's design/verification
docs.

## 2. Ground truth source being replaced

`/hydroships/odom` (`nav_msgs/Odometry`) is **100% Gazebo ground truth**:

- Bridged from `/model/hydroships/odometry` (gz) via
  `src/hydroships_gazebo/config/bridge.yaml:17-21`.
- Produced by the `gz-sim-odometry-publisher-system` plugin, configured in
  `src/hydroships_description/urdf/hydroships.urdf.xacro:404-410`, which
  reads pose/twist directly from the physics engine's `base_link` state at
  30 Hz — no noise model, no equivalent sensor on real hardware.

The IMU sensor (`/hydroships/imu`, `hydroships.urdf.xacro`, `update_rate=50`)
was, before P1.2A, a dead topic: no node in the codebase subscribed to raw
`sensor_msgs/Imu` at all (confirmed by repo-wide grep).

## 3. Consumer inventory

Four nodes currently extract orientation from `/hydroships/odom`. Each has
its own independently-duplicated quaternion→Euler helper — no shared
utility exists across them.

### 3.1 `stabilizer.py`

- Subscribes: `stabilizer.py:139` — `Odometry` on `/hydroships/odom`.
- Extracts: `on_odom`, `stabilizer.py:159-164`:
  ```python
  def on_odom(self, msg: Odometry):
      self.cur_z = msg.pose.pose.position.z
      self.cur_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
      self.cur_roll, self.cur_pitch = roll_pitch_from_quaternion(
          msg.pose.pose.orientation)
  ```
- Local helpers: `yaw_from_quaternion` (`stabilizer.py:40-46`),
  `roll_pitch_from_quaternion` (`stabilizer.py:47-56`).
- Consumed by PID loops: roll/`Mx` (`stabilizer.py:236-238`), pitch/`My`
  (`stabilizer.py:243-245`), heading/`Mz` (`stabilizer.py:250-252`).
- `cur_z` (depth) is read from the same message but is an independent field,
  unrelated to orientation, feeding a separate depth-hold PID.

### 3.2 `mission_fsm.py`

- Callback: `_on_odom`, `mission_fsm.py:430-436`:
  ```python
  def _on_odom(self, msg):
      self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)
      self.x = msg.pose.pose.position.x
      self.y = msg.pose.pose.position.y
      self.vx = msg.twist.twist.linear.x
      self.vy = msg.twist.twist.linear.y
  ```
- A second, independent local `yaw_from_quaternion` (`mission_fsm.py:52-56`)
  — only **yaw** is read; roll/pitch are never used here.
- `self.yaw` usages: heading-first navigation (`_goto_xy_yaw_first`,
  `mission_fsm.py:367,375,377-380`) for NAV_WALL/APPROACH_HOOK; gripper
  offset math in `_st_grab` (`mission_fsm.py:531,533-534`); heading-lock
  logic in HANG/manipulation states (`mission_fsm.py:552-554,589-590,622`);
  SURFACE heading-alignment check (`mission_fsm.py:779-780,789`); debug
  logging (`mission_fsm.py:632-635,732-737`).

### 3.3 `gui_bridge.py`

- Subscribes: `gui_bridge.py:71` — `Odometry` on `/hydroships/odom`.
- Extracts: `_on_odom`, `gui_bridge.py:92-93`:
  ```python
  def _on_odom(self, msg: Odometry):
      self._rpy = _yaw_rpy(msg.pose.pose.orientation)
  ```
  (third independent quaternion→Euler implementation, `gui_bridge.py:36-47`)
- Sent to GUI via `_send_telem`, `gui_bridge.py:119-122`, feeding
  `gui_bridge_logic.build_telemetry` (`gui_bridge_logic.py:110-123`), which
  is a pure function of already-extracted `yaw_rad`/`roll`/`pitch` — no
  topic dependency of its own. Depth comes separately from
  `/hydroships/depth`; position/twist from Odometry are never read here.

### 3.4 `teleop_gamepad.py`

- Imports `yaw_from_quaternion` directly from `stabilizer.py`
  (`teleop_gamepad.py:47`) — the only cross-node reuse of quaternion math in
  the codebase.
- Extracts: `on_odom`, `teleop_gamepad.py:198-200`:
  ```python
  def on_odom(self, msg: Odometry):
      self.cur_z = msg.pose.pose.position.z
      self.cur_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
  ```
- Used to seed bumpless setpoints on mode switch (`_seed_setpoints`,
  `teleop_gamepad.py:220-227`) and for yaw-relative stick handling
  (`teleop_gamepad.py:294,306,312,322-324`).

### 3.5 Non-consumers (checked, ruled out)

`thruster_allocator.py`/`allocation.py` mention "yaw" only in comments about
allocation-matrix singularity, not runtime state. `hook_logic.py`,
`teleop_keyboard.py` have comment-only references. `joy_teleop.py`'s
`self.heading` is locally dead-reckoned from joystick input, not derived
from any orientation topic. `gripper_logic.py`, `gripper_controller.py`,
`qr_detector.py`, `joy_mission_trigger.py` have no orientation usage.

## 4. Per-consumer swap classification

| Consumer | Fields read from `/hydroships/odom` | Swap classification |
|---|---|---|
| `stabilizer.py` | orientation (r/p/yaw) + `position.z` | **Clean swap** — depth and orientation are independent reads in the same callback; only the orientation extraction needs to move to a new subscription |
| `gui_bridge.py` | orientation (r/p/yaw) only | **Clean swap** — never reads odom position/twist; depth already sourced separately |
| `teleop_gamepad.py` | orientation (yaw only) + `position.z` | **Clean swap** — same pattern as stabilizer |
| `mission_fsm.py` | orientation (**yaw only**) + `position.x/y` + `twist.linear.x/y` | **Blocked** — same callback reads x/y/vx/vy for NAV_WALL/APPROACH_HOOK/GRAB navigation; `sensor_msgs/Imu` carries no position field, so a full topic swap silently breaks localization |

"Clean swap" means *structurally* safe — these three nodes only ever touch
`msg.pose.pose.orientation` from Odometry, so adding a second subscription to
`/hydroships/imu/filtered` and rerouting just the orientation extraction
would not lose any other state they depend on. It does **not** mean
integration is approved or has been runtime-verified (see §6).

`mission_fsm.py` cannot be swapped by simply repointing its subscription —
it needs either (a) a second subscription to `/hydroships/imu/filtered` for
yaw while keeping the odom subscription for x/y/vx/vy, or (b) some other
fusion approach. Which of these is right is an open design question, not
resolved by this review.

## 5. Ground-truth-vs-estimate behavioral delta

`/hydroships/imu/filtered` yaw is gyro-integration-only (complementary
filter corrects roll/pitch from accelerometer gravity vector, but yaw has no
correction source — no magnetometer exists anywhere in this ROV, confirmed
in P1.2/P1.2A). It **drifts unboundedly** over time. Ground-truth odom yaw
never drifts.

This is a property change every consumer would inherit identically once
integrated, but it lands hardest on:
- `mission_fsm.py`'s heading-gated navigation (`_goto_xy_yaw_first`) and its
  long-duration HANG/SURFACE dwell states, where accumulated drift could
  silently invalidate heading-lock behavior the longer a state persists.
- `stabilizer.py`'s heading-hold PID, which would track a drifting reference
  with no way to detect the drift.

## 6. Recommended integration ordering (future task, not started here)

If a future task proceeds with integration, the natural order by risk is:

1. **`gui_bridge.py` first** — telemetry-only (never actuates), and
   `gui_bridge_logic.py` is already topic-agnostic (takes yaw/roll/pitch as
   plain arguments, per P1.2 audit §8). Lowest blast radius if the estimator
   misbehaves.
2. **`stabilizer.py` / `teleop_gamepad.py` next** — both are clean swaps but
   feed actuation (PID loops, bumpless-transfer seeding), so they need
   runtime behavior verification under the estimator's actual yaw-drift
   characteristics before being trusted.
3. **`mission_fsm.py` last** — requires resolving the open design question
   in §4 (dual-subscribe vs. fusion) before any code change is possible;
   this is itself unresolved localization work already flagged as an open
   gap in `docs/P1-2-STATE-ESTIMATION-INTEGRATION-AUDIT.md`.

This ordering is a recommendation for whoever picks up the next task, not an
approval to start it.

## 7. Blockers / what remains open

- **`mission_fsm.py`'s mixed position+orientation read** — cannot swap
  topics wholesale without losing x/y/vx/vy; needs a design decision not
  made here.
- **Unresolved x/y/vx/vy localization gap** — already flagged in the P1.2
  audit as a standing gap (no non-ground-truth position source exists at
  all); still open, unaffected by P1.2A.
- **Yaw-drift consequence** — no consumer has been evaluated against how
  much drift accumulates over a realistic mission duration; that
  measurement hasn't been taken (P1.2A's runtime verification checked IMU
  raw-sample sanity, not estimator drift over time).
- **No consumer has run against the estimator's live output** — only
  `attitude_estimator.py`'s own standalone smoke test has exercised it
  (synthetic messages, isolated topic namespace). None of stabilizer,
  gui_bridge, or teleop_gamepad have been tested subscribing to
  `/hydroships/imu/filtered` in sim.

## 8. Verdict

This review does **not** clear any consumer for integration now. It
confirms three of four consumers (`stabilizer.py`, `gui_bridge.py`,
`teleop_gamepad.py`) are *structurally* clean-swap candidates, and one
(`mission_fsm.py`) is structurally blocked pending a localization design
decision — but "structurally clean" is a necessary, not sufficient,
condition. None of the clean-swap candidates have been runtime-verified
against the estimator's actual published output in sim, and yaw-drift
behavior over mission-length durations is unmeasured.

**Next task, if pursued:** a runtime verification gate for one candidate
consumer at a time (starting with `gui_bridge.py` per §6), following the
same audit → design → runtime-verification → implementation gating pattern
used for P1.2/P1.2A. This review does not authorize skipping that gate for
any consumer, including the "clean swap" ones.

## 9. Note on unrelated tree state

The working tree also contains unrelated, uncommitted M5 gripper/FSM work
(`gripper_controller.py`, `gripper_logic.py`, `mission_fsm.py`,
`qr_logic.py`, `docs/STATUS.md`, `test_gripper.py`,
`tools/p0-experiments/run_mission_cycle.sh`). This review did not open, diff,
or otherwise inspect the substance of those changes — only their presence is
noted here, per standing instruction not to touch that work.
