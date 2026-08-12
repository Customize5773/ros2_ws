# P1-1: Architecture Decision Audit — ROS2-native vs. Pixhawk/ArduSub control authority

Read-only architecture decision audit, building on `docs/P1-0-ARCHITECTURE-AUDIT.md`. No code, config, launch files, messages, FSM, controller, `gui_bridge`, `rov_agent.py`, or the frozen P0 baseline was modified to produce this document. This is a decision problem, not an interface-cleanup problem — no bridge was implemented, no topic renamed, no schema reconciled.

---

## 1. Architecture reconstruction

### Architecture A — ROS2-native control

```
mission_fsm.py (autonomy)
   → stabilizer.py (PID: depth/heading/pitch/roll) [optional — mission_fsm can also
     drive /hydroships/manual/cmd directly, bypassing stabilizer's hold loops]
   → /hydroships/cmd_vel (Twist-as-wrench)
   → thruster_allocator.py (damped pseudo-inverse, allocation.py)
   → /hydroships/thruster_{1..6}/thrust (Float64, Newton)
   → [sim: gz-sim-thruster-system plugin] / [real: needs new ESC/PWM driver node — HARDWARE.md item 1]
```

| Property | Value |
|---|---|
| Authoritative controller | `stabilizer.py` (4 independent PID loops, `pid.py`, gains in `gains.yaml`) |
| Authoritative thruster allocator | `thruster_allocator.py` (software damped pseudo-inverse, `allocation.py`) |
| Stabilization authority | ROS2, entirely — no flight controller in the loop |
| Actuator command interface | `Float64` Newton values per thruster topic; on real hardware, an as-yet-unbuilt ESC/PWM driver node consuming the same topics (`HARDWARE.md` item 1) |
| Sensor/state authority | Gazebo ground-truth `/hydroships/odom` in sim; on hardware, an as-yet-unbuilt fusion path (no IMU consumer exists anywhere in `ros2_ws` today — P1-0 Finding, `HARDWARE.md` IMU row) |
| Safety/failsafe authority | `thruster_allocator.py`'s 0.5s cmd_vel watchdog (zeroes thrust) — the only failsafe mechanism in this architecture |
| Telemetry authority | `gui_bridge.py` (partial — see §3) |
| GUI integration point | `gui_bridge` UDP-JSON on ports 14550/14551 |
| Hardware dependencies | ESC/PWM driver, MS5837 depth driver, USB camera driver, gripper servo driver — **all four listed as "Tidak ada" (does not exist) in `HARDWARE.md`** |
| Simulation compatibility | Full — this is the only architecture currently implemented and running in Gazebo |
| Expected real-ROV deployment path | Per `HARDWARE.md` §3 item 5: **explicitly documented as able to run without a Pixhawk at all**, if ESCs are driven directly from the Raspberry Pi. Pixhawk integration is described as "opsional, tergantung keputusan tim" (optional, pending team decision) |

### Architecture B — Pixhawk/ArduSub-authoritative

```
mission5.py / autonomy stack (GUI-ROV: autonomy/fsm/mission5.py, autonomy/control/visual_servo.py)
   → rov_link.py / rov_agent.py (command interface, pymavlink)
   → MANUAL_CONTROL (+ mode-set, param-set) MAVLink messages, port 14555
   → Pixhawk / ArduSub firmware
   → ArduSub's own thruster mixing (frame-specific mixer, "channel/servo configuration")
   → thrusters (real) / SITL physics (dev/test)
```

| Property | Value |
|---|---|
| Authoritative controller | ArduSub firmware's own control loops — confirmed by `README-WORK.md:519-524`: GUI's "depth PID" fields map directly onto **ArduSub's own `PSC_ACCZ_*` parameters** (a 3-loop cascade internal to ArduSub), not a loop running on the Pi or in `ros2_ws` |
| Authoritative thruster allocator | ArduSub's internal frame mixer, configured via Pixhawk channel/servo parameters — not `thruster_allocator.py` |
| Stabilization authority | Pixhawk/ArduSub, entirely, for any mode beyond raw `MANUAL_CONTROL` passthrough (e.g. `ALT_HOLD`/depth-hold, `STABILIZE`) |
| Actuator command interface | MAVLink `MANUAL_CONTROL` (chosen deliberately over `RC_CHANNELS_OVERRIDE` per `README-WORK.md:255-257`, specifically to coexist safely with Pixhawk's own channel/servo config) |
| Sensor/state authority | Pixhawk's own IMU + externally-fed pressure sensor, reported back as MAVLink `ATTITUDE`/`VFR_HUD`/pressure messages — not `ros2_ws`'s `/hydroships/odom` |
| Safety/failsafe authority | ArduSub's own pilot-input failsafe (triggers if `MANUAL_CONTROL` stream stops — `README-WORK.md:269-270`), plus a Pi-side 20 Hz neutral-streaming failsafe in `rov_axes.resolve_manual_packet` |
| Telemetry authority | `rov_agent.py` (`state` dict → UDP :14551), sourced from MAVLink `ATTITUDE`/pressure/`HEARTBEAT` messages |
| GUI integration point | Same UDP-JSON transport shape as `gui_bridge` (ports 14550/14551) but produced by `rov_agent.py`, not `ros2_ws` |
| Hardware dependencies | A real (or SITL) Pixhawk running ArduSub; no ESC/PWM driver needed in software since Pixhawk drives ESCs directly |
| Simulation compatibility | Supported via ArduSub SITL (`autonomy/SITL_SETUP.md`, `sitl_mock.py`, `run_sitl.sh` — WSL2-hosted), entirely independent of Gazebo/`ros2_ws` |
| Expected real-ROV deployment path | `autonomy/ROADMAP_MISI5.md` describes an explicit phased path: **meja (bench) → SITL → hardware → kolam (pool) → lomba (competition)** — this is a further-along, actively-pursued deployment plan than anything documented for Architecture A |

**Both architectures currently exist as running code**, not as one implemented and one theoretical: Architecture A runs in Gazebo today (`ros2_ws`); Architecture B has a working SITL path and its own mission FSM, visual servo, and QR detection stack (`GUI-ROV/autonomy/`), independently reimplementing perception/autonomy that `ros2_ws` also implements.

---

## 2. Evidence for intended target architecture

**Documented intent (KKI 2026 proposal, as summarized in `HARDWARE.md`):** the physical ROV design lists a **Pixhawk PX4** as "kontroler utama" (main controller) — proposal-level intent leans toward Architecture B or a hybrid.

**Implemented behavior — `ros2_ws`:** `HARDWARE.md`'s own component-mapping table (row "Pixhawk PX4") states plainly: *"Tidak dipakai — mixing dilakukan `thruster_allocator.py`"* (not used — mixing is done by `thruster_allocator.py`), and *"Workspace ini tidak punya kode MAVLink"* (this workspace has no MAVLink code at all). `HARDWARE.md` §3 item 5 goes further and explicitly states the current software architecture **can run without a Pixhawk entirely**, with Pixhawk integration framed as optional/redundant ("untuk redundansi IMU/failsafe") and pending a team decision — and explicitly warns against duplicating GUI-ROV's MAVLink work if the team does want Pixhawk integration.

**Implemented behavior — `GUI-ROV`:** the `autonomy/` subtree is a full, independently-built autonomy stack (mission FSM `mission5.py`, PBVS visual servo, QR detection) that assumes and requires ArduSub/Pixhawk (or its SITL) as the control target — not just a passthrough joystick bridge. `README-WORK.md`'s PID section confirms GUI-side "depth PID" tuning fields are cosmetic UI over ArduSub's own `PSC_ACCZ_*` firmware parameters, i.e. stabilization genuinely runs on the flight controller in this path, not on the Pi or in ROS.

**Legacy vs. active:** Architecture A (`thruster_allocator` + `stabilizer`) is the **only architecture with any real ROS2/Gazebo implementation** and is what all of `ros2_ws`'s existing verified sim milestones (per `docs/STATUS.md`) are built on — it is not legacy, it is the active and only implementation in this repository. Architecture B is **not legacy either** — `GUI-ROV/autonomy/ROADMAP_MISI5.md`'s phased rollout and the presence of working SITL tooling indicate active, ongoing development on that side, independent of and unaware of `ros2_ws`.

**Simulation-only vs. target-hardware behavior:** Architecture A's simulation compatibility is Gazebo-only; nothing in it has been exercised against real thrusters, a real depth sensor, or a real Pixhawk. Architecture B's SITL path is explicitly designed to validate against ArduSub's real mixing/failsafe logic before hardware, and its `ROADMAP_MISI5.md` treats hardware-in-the-loop as the very next phase after SITL — i.e., Architecture B currently has the more concrete, closer-to-real-hardware validation story, even though Architecture A has the more complete Gazebo-side simulation (full 6-DOF hydrodynamics, buoyancy, visual rendering) that Architecture B does not appear to replicate.

**Conclusion of this section:** the proposal document leans toward Pixhawk-as-main-controller (Architecture B or hybrid), `ros2_ws`'s own hardware-gap documentation treats Pixhawk as optional and defers the decision explicitly to the team, and `GUI-ROV` has, independently and without coordinating with `ros2_ws`, already built substantially toward Architecture B. **No single document in either repo asserts Architecture A as the final target with confidence** — `HARDWARE.md` item 5 is the closest thing to a decision, and it is phrased as "if the team decides," not as settled fact.

---

## 3. `gui_bridge` role assessment

**Classification: (C) an intermediate compatibility layer that has not been validated against a real GUI client — closer to (D) abandoned/parallel in practice, but built with (C) intent.**

Evidence:
- `gui_bridge.py`'s own design (ports 14550/14551, JSON telemetry/command shape) closely mirrors GUI-ROV's *wire protocol* — strong evidence it was built with intent (C), to let the Gazebo-side simulation stand in for a real vehicle when talking to the same dashboard.
- However, per `docs/HARDWARE.md`'s own table: *"Ada jalur adapter (`gui_bridge`), tapi belum pernah diuji dengan GUI fisik/live"* (an adapter path exists, but has never been tested against a real/live GUI) — only synthetic-client UDP round-trip testing is confirmed (`docs/VERIFICATION-CHECKLIST.md` P4, per that same table row).
- P1-0's audit found concrete protocol drift against what GUI-ROV's dashboard actually expects (axis range, mode-string vocabulary, missing telemetry fields) — consistent with a bridge that was built once against an assumed contract and never re-verified as GUI-ROV's real implementation (`rov_agent.py`) evolved independently.
- `gui_bridge` has **no MAVLink awareness at all** — it cannot represent Architecture B's mode/telemetry vocabulary (ArduSub HEARTBEAT mode names, `PSC_ACCZ_*`-backed depth-hold, MAVLink param tables) even in principle. It is architecturally scoped as a **simulation-to-GUI bridge for Architecture A only** (role A), not a general-purpose adapter that could also represent Architecture B.

**Net assessment:** `gui_bridge` is best described as **role A (an authoritative-for-Architecture-A simulation-to-GUI bridge) that was designed with a role-C aspiration** (to let the same GUI dashboard drive either sim or real vehicle) **but has drifted, unverified, since GUI-ROV's real bridge (`rov_agent.py`) moved on independently.** It is not abandoned in the sense of being unmaintained dead code — it has recent commits per this session's git log — but it is currently unable to serve as a real compatibility layer without the reconciliation work P1-0 already scoped.

---

## 4. Architecture decision matrix

| Criterion | Architecture A (ROS2-native) | Architecture B (Pixhawk/ArduSub) | Basis |
|---|---|---|---|
| Compatibility with actual target hardware (per proposal) | Partial — proposal names Pixhawk as main controller; A treats it as optional/absent | Direct — proposal's stated hardware list assumes a Pixhawk is present and is its main controller | `HARDWARE.md` proposal summary |
| Compatibility with current simulation (Gazebo, this repo) | Full — this is what exists and passes sim milestones today | None — no ArduSub/Pixhawk/MAVLink code exists in `ros2_ws`; would require a separate SITL environment (WSL2) alongside Gazebo | `HARDWARE.md`, `docs/STATUS.md`, `GUI-ROV/autonomy/SITL_SETUP.md` |
| Autonomy ownership | `ros2_ws`'s `mission_fsm.py` (well-integrated with Gazebo topics, verified states per STATUS.md, though with the `pub_grip` bug tracked separately) | `GUI-ROV`'s `autonomy/fsm/mission5.py` (independently developed, ROADMAP shows active SITL-stage progress) | P1-0 §8, this doc §2 |
| Stabilization ownership | `stabilizer.py` PID loops (4 independent loops, tunable via `gains.yaml`, ROS-native and unit-testable) | ArduSub firmware cascade (e.g. `PSC_ACCZ_*` for depth) — tuned via Pixhawk params, not ROS | `pid.py`, `README-WORK.md:519-524` |
| Thruster allocation ownership | `thruster_allocator.py` (damped pseudo-inverse, ROS-native, unit-testable via `allocation.py`) | ArduSub's internal frame mixer (opaque to ROS, configured via Pixhawk channel/servo params) | `allocation.py`, `HARDWARE.md` Pixhawk row |
| Telemetry quality | Currently incomplete per P1-0 (no mission status, stub voltage/temp) but structurally simple (plain ROS topics, easy to extend) | Richer today in practice — `rov_agent.py` surfaces `poshold`, `depth_target`, `depth_hold`, full MAVLink param/inspector tooling (`README-WORK.md` §9) — but tied to ArduSub's own message set, less flexible for ROS-native additions | P1-0 §7, `README-WORK.md` §9 |
| Safety/failsafe behavior | Single failsafe: `thruster_allocator`'s 0.5s cmd_vel watchdog. Simple, but unproven — never exercised against real actuators or a real "stop everything" hardware fault path | ArduSub's mature, field-tested pilot-input failsafe plus a Pi-side neutral-streaming failsafe; MANUAL_CONTROL was deliberately chosen (over RC_CHANNELS_OVERRIDE) specifically for failsafe compatibility with Pixhawk's own logic | `thruster_allocator.py`, `README-WORK.md:255-270` |
| Determinism/testability | High — pure-logic modules (`pid.py`, `allocation.py`) are unit-tested with no hardware/firmware dependency; behavior is fully inspectable in Python | Lower from `ros2_ws`'s vantage point — stabilization behavior lives inside closed ArduSub firmware; testable via SITL but not unit-testable in the same lightweight way; GUI-ROV does unit-test its *own* pure modules (`test_rov_*.py`) but the firmware loop itself is a black box | Both repos' test suites |
| GUI integration effort (from current state) | Requires fixing `gui_bridge`'s drift (axis range, mode strings, missing telemetry) — P1-0 already scopes this as bounded, mechanical work | Already integration-complete on the GUI-ROV side (that's what `rov_agent.py` + the dashboard already do) — but `ros2_ws` would need an entirely new MAVLink-facing autonomy/perception hand-off (mission_fsm/qr_detector/hook_detector would need to talk to ArduSub instead of `stabilizer`/`thruster_allocator`), which is a much larger rewrite | P1-0 §4, this doc §1 |
| Hardware-in-the-loop potential | Not yet started — no HIL tooling referenced anywhere in `ros2_ws` | Actively developed — `autonomy/ROADMAP_MISI5.md` explicitly sequences meja→SITL→hardware; SITL tooling already exists and is documented | `ROADMAP_MISI5.md`, `SITL_SETUP.md` |
| Future maintainability | Two codebases (Python ROS nodes) to maintain, fully owned by the team, but duplicates functionality ArduSub provides for free (mixing, failsafe, IMU fusion) | Offloads mixing/stabilization/failsafe maintenance to a mature open-source firmware (ArduSub), but couples the team to MAVLink/Pixhawk-specific tuning and firmware upgrade cycles | Qualitative, both repos |
| Migration cost (from current state to each) | Low — this is already the working state; migration cost is fixing `gui_bridge` drift only | High — would require building a MAVLink command/telemetry path in `ros2_ws`, deciding how `mission_fsm`/perception nodes hand off control to ArduSub, and likely retiring or re-scoping `stabilizer.py`/`thruster_allocator.py` | This doc §1, §6 |
| Risk of duplicated control authority | Low in isolation (A alone has one control path) — **risk appears only if A and B are ever connected to the same vehicle simultaneously**, see §5 | Low in isolation — same caveat applies symmetrically | See §5 |

---

## 5. Dual-authority / safety risk analysis

These conditions are assessed as **hypothetical integration risks** — none currently occur in either repo running standalone, because `ros2_ws` never talks MAVLink and `GUI-ROV`'s `rov_agent.py`/`autonomy` never talks to `ros2_ws`'s ROS topics. The risk exists **only if the two are ever wired to the same physical vehicle without an explicit decision**, which is exactly the scenario P1.1 exists to prevent.

| Condition | Currently occurring? | Risk if both stacks were pointed at one real vehicle |
|---|---|---|
| ROS and Pixhawk both believing they own stabilization | No (not currently connected) | **High** — `stabilizer.py`'s depth/heading PID and ArduSub's `PSC_ACCZ_*`-based depth-hold would both attempt to command the same thrusters toward possibly different setpoints, with no arbitration mechanism in either codebase |
| ROS and Pixhawk both applying actuator/thruster mixing | No | **High** — `thruster_allocator.py`'s pseudo-inverse allocation and ArduSub's internal frame mixer are mutually exclusive by design; running both means either double-mixing (allocator output re-mixed by ArduSub) or one silently overridden, depending on wiring |
| Duplicated control loops | No | **High** — two independent PID/control layers (ROS `stabilizer` and ArduSub firmware loops) with no shared state or handshake |
| Conflicting command semantics | No | **Medium** — confirmed distinct today (P1-0 Finding #2: `-100..100` vs `-1000..1000` axis ranges) but would become dangerous rather than just cosmetic if the same numeric command reached the vehicle through two different scaling assumptions |
| Conflicting telemetry authority | No | **Medium** — `/hydroships/odom` (ROS ground truth or a future real-sensor equivalent) vs. ArduSub's own `ATTITUDE`/pressure telemetry could disagree, and no fusion/arbitration exists in either repo to reconcile them |
| Simulation behavior that cannot map to real hardware | **Yes, already true today, independent of dual-authority** | Architecture A's Gazebo ground-truth odometry/depth (P1-0 Finding, §9 of P1-0) has no real-hardware equivalent yet in `ros2_ws`; this is a pre-existing sim-to-real gap, not a new risk from combining architectures, but it means Architecture A's sim-verified behavior cannot be assumed to transfer to hardware without the driver work `HARDWARE.md` already lists |

**Summary:** no dual-authority condition is live today because the two stacks are not connected. The danger is entirely **prospective** — it would be triggered by any well-intentioned attempt to "just wire the two together" without first resolving which one owns stabilization and mixing. This is the central argument for making the P1.1 decision explicit before any P1.2/P1.3 implementation work begins.

---

## 6. Decision

**DECISION A — ROS 2-native control is authoritative.**

Rationale is developed in §11 and the closing statement; the explicit partition is:

- **Stabilization**: `ros2_ws`'s `stabilizer.py` (4-loop PID) remains authoritative. ArduSub's firmware-level stabilization loops are not used.
- **Thrust allocation/mixing**: `thruster_allocator.py` remains authoritative. If a Pixhawk is physically present at all (per `HARDWARE.md` item 5's "redundancy" option), it is used only as a pass-through ESC driver / IMU-redundancy source, never as a mixer.
- **Actuator commands**: ROS2 topics (`/hydroships/thruster_{1..6}/thrust`) remain the canonical actuator interface; a to-be-built hardware driver node translates Newton→PWM, whether that driver talks to raw ESCs or to a Pixhawk's `MAIN OUT` in passthrough mode is an implementation detail deferred to P1.2, not an authority question.
- **State estimation**: `ros2_ws` owns it. On hardware this requires building the currently-missing fusion path (P1-0 Finding: IMU bridged but unused) — this is now explicitly a P1.2 blocking dependency, not an open architecture question.
- **Failsafe**: `thruster_allocator.py`'s cmd_vel watchdog remains authoritative; if a Pixhawk is present for IMU redundancy, its own pilot-input failsafe is a defense-in-depth backstop, not the primary failsafe.
- **Mission/autonomy**: continues to run in `ros2_ws`'s `mission_fsm.py`. `GUI-ROV/autonomy/fsm/mission5.py` becomes a legacy/parallel implementation whose future is a project-owner decision (see §12) — this document does not order its removal.
- **GUI connection point**: `gui_bridge` remains the canonical GUI integration point; `rov_agent.py`'s MAVLink-facing role is retired as the *authoritative* bridge going forward (see below).
- **`gui_bridge`'s ultimate role**: the authoritative production ROS-to-GUI bridge (role B, not merely role A/C as assessed today) — its current drift (P1-0 Findings #2, #3, #6) becomes the direct scope of P1.2/P1.3, not a permanent limitation.
- **`rov_agent.py`'s status**: **retired as the authoritative vehicle-facing bridge.** Its pure-logic modules (`rov_axes.py`, `rov_modes.py`, `rov_pid.py`, unit-tested) may still be referenced for their axis-scaling/mode-mapping design when reconciling `gui_bridge`'s contract in P1.2, but the MAVLink transport itself is not the path forward under this decision.
- **What `ros2_ws` ultimately owns**: the full control path from mission/autonomy through actuator command, end to end — stabilization, allocation, state estimation, failsafe, and GUI telemetry/command bridging.
- **Canonical integration boundary**: `gui_bridge`'s UDP-JSON contract (ports 14550/14551) becomes the one authoritative boundary between the vehicle-side stack (wherever it runs — dev laptop today, Raspberry Pi later) and any GUI/dashboard client, superseding `rov_agent.py`'s wire format as the thing GUI-ROV's dashboard must ultimately speak (or be adapted to speak).

This is not a hybrid decision, and is not recommended as one: the matrix in §4 shows Architecture A already has the working simulation, the ROS-native testability the team has invested in (P0's whole QR/mission verification approach depends on this), and `HARDWARE.md`'s own documentation already leans this way ("bisa berjalan tanpa Pixhawk sama sekali"). A genuine hybrid (e.g., ROS owns autonomy/perception, Pixhawk owns stabilization/mixing) was considered and rejected here because it would require exactly the dual-authority arbitration described in §5 as unsolved and unscoped in either repo today — recommending it without a concrete arbitration design would be recommending "hybrid" to avoid choosing, which this audit's terms explicitly disallow.

---

## 7. Explicit ownership table

| Responsibility | Owner under Decision A |
|---|---|
| Stabilization | `ros2_ws` / `stabilizer.py` |
| Thrust allocation / mixing | `ros2_ws` / `thruster_allocator.py` |
| Actuator commands | `ros2_ws` ROS2 topics → (future) hardware driver node |
| State estimation | `ros2_ws` (new fusion node required for hardware — P1.2 dependency) |
| Failsafe | `ros2_ws` / `thruster_allocator.py` watchdog (primary); Pixhawk pilot-input failsafe, if present, as backstop only |
| Mission/autonomy | `ros2_ws` / `mission_fsm.py` |
| GUI connection | `gui_bridge` (becomes authoritative production bridge) |
| `rov_agent.py` | Retired as authoritative bridge; pure-logic modules may inform P1.2 design only |

---

## 8. Canonical integration boundary proposal

**`gui_bridge`'s UDP-JSON contract (ports 14550 command / 14551 telemetry) is the canonical integration boundary** between the ROS2/vehicle-side stack and any GUI/dashboard consumer. This boundary sits at exactly the same physical location `gui_bridge` already occupies — no new component is proposed. The boundary's *content* (field names, ranges, mode vocabulary) is exactly what P1.2 must now formally define, using P1-0's telemetry/command drift tables (P1-0 §7, §4) as the starting diff between what `gui_bridge` produces today and what a GUI dashboard needs.

---

## 9. P1.2 dependencies (simulation ↔ target integration)

Under Decision A, P1.2 must define/implement (implementation, not this document):
- A hardware driver node translating `thruster_allocator`'s Newton output to real actuator commands (ESC/PWM direct, or Pixhawk `MAIN OUT` passthrough per `HARDWARE.md` item 5's optional-redundancy path) — decide the ESC-vs-Pixhawk-passthrough sub-question, since Decision A does not resolve whether a Pixhawk is present at all, only that it does not do mixing/stabilization if present.
- A state-estimation path for hardware: minimally, wiring the already-bridged-but-unused `/hydroships/imu` into something `stabilizer.py`/`mission_fsm.py` can consume in place of Gazebo's ground-truth odom, or an explicit decision to keep relying on a to-be-built EKF/fusion node.
- The MS5837 depth driver, USB camera driver, and gripper servo driver already scoped in `HARDWARE.md` §3 — none of these are affected by the architecture decision itself and can proceed in parallel.
- Reconciliation of `gui_bridge`'s axis range and mode-vocabulary drift (P1-0 Findings #2, #3) against whatever dashboard client is chosen (GUI-ROV's existing browser frontend, retargeted to speak `gui_bridge`'s contract instead of `rov_agent.py`'s).

## 10. P1.3 dependencies (telemetry/observability)

- Publish `mission_fsm.py`'s internal state/score to a ROS topic and surface it through `gui_bridge`'s telemetry (P1-0 Finding #5) — under Decision A, `mission_fsm` remains the sole mission-state authority, so this is now unambiguous, not blocked on an architecture choice.
- Add the currently-missing telemetry fields GUI-ROV's dashboard expects but `gui_bridge` doesn't provide (`poshold`, `depth_target`, `depth_hold` — P1-0 Finding #6), scoped to whatever subset remains meaningful once `rov_agent.py`'s ArduSub-specific concepts (e.g. `poshold` as an ArduSub mode distinction) are re-mapped onto ROS2-native equivalents (e.g. `control_mode` already existing in `ros2_ws`).
- Resolve the `voltage`/`temp` stub-zero gap (P1-0 Finding #7) — requires a real power/environmental sensor driver decision, independent of control architecture.
- Retire or explicitly re-scope GUI-ROV's `mission5` telemetry sub-object and MAVLink Inspector/param tooling (`README-WORK.md` §9) — these are ArduSub-specific concepts with no meaning under Decision A and should not be silently carried into the reconciled schema.

## Findings from P1-0 now unblocked vs. still blocked

| P1-0 finding | Status after this decision |
|---|---|
| #1 (architectural fork) | **Resolved by this document** — Decision A |
| #2, #3 (axis range, mode string drift) | **Unblocked** — now a well-scoped P1.2 implementation task against a known-authoritative target (ROS2-native) |
| #4 (dual mission FSMs) | **Unblocked for `ros2_ws`'s side** (mission_fsm.py is authoritative); GUI-ROV's `mission5.py` future is a project-owner decision (§12), not resolved here |
| #5 (mission status not published) | **Unblocked** — P1.3 scope |
| #6 (missing telemetry fields) | **Unblocked**, pending the ArduSub-specific-concept re-mapping noted in §10 |
| #7 (voltage/temp stubs) | **Unblocked as a hardware-driver task**, independent of architecture |
| #9 (no formal schema) | **Unblocked** — P1.2/P1.3 should produce one, now that the target it's describing is fixed |
| #11, #12 (stale docs) | **Always independent** — can proceed immediately, no dependency on this decision |
| #10 (`pub_grip` bug) | **Unaffected, remains a separate backlog item**, not part of architecture work |

---

## 11. Migration risks

- **GUI-ROV's dashboard frontend (`public/js/app.js` etc.) currently speaks `rov_agent.py`'s wire format**, including ArduSub-specific fields (`mode` as HEARTBEAT strings, `mission5`, MAVLink param/inspector tooling). Retargeting it to `gui_bridge`'s contract is real integration work, not a relabeling — the mode-vocabulary and telemetry-field gaps in P1-0 §7 are exactly this migration's scope.
- **Loss of ArduSub's mature failsafe/mixing logic**: Decision A means the team takes on full responsibility for `thruster_allocator.py`'s watchdog and `stabilizer.py`'s PID tuning as the *only* line of defense, without ArduSub's field-tested pilot-input failsafe as a primary safety net. This is a real capability gap versus Architecture B that P1.2/P1.3 should account for (e.g., by hardening the existing watchdog, not by silently relying on an absent Pixhawk failsafe).
- **State-estimation gap is the largest unresolved technical risk**: `ros2_ws` has no fusion/EKF node today, and Decision A commits the project to building one (or accepting a degraded dead-reckoning/IMU-only estimate) rather than inheriting ArduSub's existing state estimator. This is flagged as the most consequential open engineering task, not a paperwork item.
- **`GUI-ROV/autonomy/`'s SITL-validated progress is not directly reusable** under Decision A — its mission FSM, visual servo, and QR detection logic are ArduSub/MAVLink-coupled and would need re-implementation against ROS2 topics rather than migration, even though `ros2_ws` already has largely equivalent (if less field-validated) logic of its own (`mission_fsm.py`, `qr_detector.py`, `hook_detector.py`).
- **Proposal-vs-implementation gap remains visible to competition judges/stakeholders**: the KKI 2026 proposal document names Pixhawk as "kontroler utama." Decision A does not contradict having a Pixhawk physically present (per `HARDWARE.md` item 5's redundancy option) but does mean it is not the controller in the software-authority sense the proposal implies — this may need to be communicated/reconciled with whoever owns the proposal document, independent of the engineering decision.

---

## 12. Unresolved questions requiring human/project-owner decision

1. **Is a Pixhawk physically present on the competition vehicle at all**, and if so, is it used purely as an ESC/IMU-redundancy passthrough (compatible with Decision A) or does the team still want ArduSub firmware modes available as a fallback control path? `HARDWARE.md` item 5 explicitly defers this to the team and this audit does not have evidence to resolve it unilaterally.
2. **What happens to `GUI-ROV/autonomy/` (`mission5.py`, `visual_servo.py`, QR detection, SITL tooling)** — is it retired, kept as a fallback/backup control path, or repurposed (e.g., its visual-servo math ported into `ros2_ws`'s `hook_logic.py`/`qr_logic.py` — noting P0's frozen QR logic must not be touched)? This decision has schedule and ownership implications (who maintains which stack) beyond what this audit can determine from code alone.
3. **Who owns retargeting the GUI-ROV browser frontend** to speak `gui_bridge`'s contract instead of `rov_agent.py`'s — is this `ros2_ws` team's responsibility (extend `gui_bridge` to match GUI-ROV's existing fields) or `GUI-ROV` team's responsibility (adapt the frontend to `gui_bridge`'s simpler contract)? This is a cross-repo coordination question, not a technical one.
4. **Timeline/resourcing for the state-estimation gap** (§11) — building a real fusion/EKF node is nontrivial and its absence is the largest technical risk identified; a project-owner needs to weigh this against competition deadlines.
5. **How to reconcile the proposal document's "Pixhawk as kontroler utama" language** with Decision A's software-authority split, for any external-facing documentation/judging requirements.

---

## Final statement

**P1.1 recommendation: Architecture A (ROS 2-native control is authoritative), because** `ros2_ws` already has a complete, working, unit-tested control path (stabilization, allocation, mission autonomy) validated against Gazebo simulation and aligned with the team's existing P0 verification investment, while `ros2_ws`'s own hardware-gap documentation (`HARDWARE.md` item 5) already states this path can run without a Pixhawk and explicitly frames Pixhawk integration as optional and team-decided rather than required; retrofitting `ros2_ws` onto ArduSub/MAVLink (Architecture B) would require rewriting or discarding the ROS-native stabilizer and allocator the team has already built and verified, in favor of a firmware-internal control loop that is currently opaque to this codebase, purely to match a proposal-document phrase that the team's own documentation already treats as negotiable.

**Is the evidence strong enough to approve implementation, or is an explicit project-owner decision still required?** The evidence strongly supports Architecture A as the *software* control authority, and is sufficient to unblock P1.2/P1.3 implementation planning on that basis. However, **an explicit project-owner decision is still required** on the physical-hardware sub-question (§12 item 1: is a Pixhawk present at all, and in what capacity) and on the disposition of `GUI-ROV/autonomy/`'s independently-built mission/vision stack (§12 item 2) before implementation work should begin — these are resourcing/ownership calls this audit cannot make from code evidence alone.
