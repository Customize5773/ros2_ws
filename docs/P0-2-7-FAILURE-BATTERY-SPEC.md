# P0-2.7 SPEC — failure-focused APPROACH_QR battery (design only, NOT run)

**STATUS: SPEC ONLY. Nothing in this document has been executed.** `qr_logic.py`,
`qr_detector.py`, `mission_fsm.py`, controller gains, and `_candidates()` ordering are
unchanged and untouched by this document.

## 0. Why this spec exists

The P0-2.6/P0-2.7 candidate-distribution and candidate/gate-correlation passes (this session)
concluded `INSUFFICIENT DATA`: the one gate-failed run in the 6-run P0-2.6 battery (`V1`) had
only 6 corner-bearing observations with per-candidate cells never exceeding n=2 — far too thin
to distinguish "candidate behavior correlates with the gate outcome" from "this run's spawn
pose happened to differ." P0-2.7's own `NEXT RECOMMENDED EXPERIMENT` called for a battery of
N≥15 runs specifically oversampling failure-prone spawn poses to get V1-class failures at n>1.
This spec defines that battery precisely, before any of it runs, matching this repo's
established spec-first discipline (`docs/P0-2-3-SPEC.md`, `docs/P0-2-4-SPEC.md`).

## 1. Failure-prone spawn pose selection

`_rov_spawn_pose()` (`src/hydroships_gazebo/launch/sim.launch.py:88-112`), used whenever
`rov_random_spawn:=true`, derives `(x, y, z, yaw)` deterministically from `random.Random(seed)`
(`_spawn_rng()`, `sim.launch.py:115-124`, seeded via the existing `spawn_seed` launch arg)
through a **fixed RNG call sequence**:

```text
along = rng.uniform(-lim, lim)              # position along the chosen wall
wall  = rng.choice(('A', 'B', 'C', 'D'))
yaw   = _WALL_INWARD_YAW[wall] + rng.uniform(-_YAW_JITTER, _YAW_JITTER)
```

where `lim = rov_arena_half(2.55) - rov_wall_margin(0.5) = 2.05` and `_YAW_JITTER = 0.35`
(both constants copied verbatim from `sim.launch.py:85,101-102`).

Because this is a pure function of `seed`, candidate seeds are screened **offline, with no
simulator run**, by `tools/p0-experiments/select_failure_seeds.py`, which replicates the exact
call sequence above. A seed qualifies as failure-prone iff **both**:

- `|along| >= 0.85 * lim` (≈1.74m) — spawn in the corner-adjacent 15% of the wall span, since
  a corner puts the payload QR closest to a camera-frame edge.
- `|yaw_offset| >= 0.85 * _YAW_JITTER` (≈0.30 rad) — near-maximal in-plane rotation, directly
  targeting the AABB-inflation/rotation mechanism P0-2.3 already confirmed degrades corner-only
  quality (mean inflation ≈1.35-1.4×).

Both thresholds are reused geometry constants, not invented values. In a 500-seed offline scan
this rule qualifies ~3.6% of seeds (18/500) — enough headroom to draw 30 seeds for the battery
cap below.

## 2. V1-class failure definition

Applied **after** each run, using existing tooling only (no new metric):

A run is a **V1-class failure** iff:

1. `gate_mission.sh` result is `PASS` (i.e. `quality_gate()` in `reduce_approach_qr.py` returns
   no contamination reasons) — this excludes infra crashes/node-liveness failures, which are a
   *different* failure class from what P0-2.6/P0-2.7 diagnose. (V1 itself was actually an
   infra-liveness `FAIL` per `quality_gate()`, not a perception failure — confirmed by re-running
   `quality_gate('V1', ...)` against the existing P0-2.6 battery data while writing this spec.
   "V1-class" here refers to the *perception-failure shape* P0-2.5/2.6/2.7 are chasing, not
   literally V1's own gate reason.)
2. `entered_band_with_dwell == False` for the full episode — `reduce_approach_qr.py`'s existing
   Gate 4 retest (`p0_2_4_gate4_retest['entered_band_with_dwell']`, `reduce_approach_qr.py:301-343,
   370-379`): did the ROV hold `qr_center_tol` (or `dist < approach_tol`) for `DWELL_TICKS`
   consecutive ticks. This is the authoritative "did APPROACH_QR actually converge" signal.

Separately reported (not required for qualification): whether `qr_decode_rate == 0.000` across
the episode — P0-2.5's already-named "Mode 1" zero-decode subclass
(`docs/P0-2-5-ENGINEERING-ANALYSIS.md` §A.1), the most QR-perception-attributable failure shape.

## 3. Minimum qualifying sample

Stop only once **both** hold:

- **≥8** qualifying V1-class-failure runs collected.
- **≥40** independent corner-bearing observations (`analyze_qr_candidates.extract_observations()`,
  P0-2.6, dedup on consecutive identical corner tuples) pooled across those failure runs.

40 is chosen to match the n=41 pass-side pool size from the P0-2.7 aggregate (V2-V6 combined),
so a future failure-vs-pass comparison has comparable statistical power on both sides instead of
a large group against a handful of points, as P0-2.7 was forced to do.

## 4. Stopping rule

Run in batches of 6 (same batch size and one-Gazebo-server-at-a-time constraint as every prior
battery), drawing seeds from the pre-screened failure-prone pool in order. After each batch,
recompute the qualifying-failure count and pooled observation count (both computed by reusing
`reduce_approach_qr.py`'s `quality_gate`/`load_params`/`analyze_run` plus
`analyze_qr_candidates.extract_observations`, not a new metric).

- Stop **as soon as both minimums (§3) are met**.
- **Hard cap: 30 total runs (5 batches).** If the cap is reached without meeting both minimums,
  the battery stops anyway. The result is reported as `INSUFFICIENT DATA` (same discipline
  P0-2.7 already used) — the bar is not lowered and "qualifying" is not redefined after the
  fact to force a different outcome.

## 5. Tooling (written, not executed by this spec)

- `tools/p0-experiments/select_failure_seeds.py` — offline seed screening per §1. No simulator
  run. Verified during spec-writing: scanning seeds 0-499 with the default 0.85/0.85 thresholds
  qualifies 18 seeds (3.6%), e.g. `seed=43` → `wall=B, along=-1.892, yaw_offset=+0.322`, both
  past threshold (`|along|>=1.7425`, `|yaw_offset|>=0.2975`); see the script's own output for
  the full pool.
- `tools/p0-experiments/run_qr_failure_battery.sh` — batch loop per §4, calling
  `run_approach_qr_smoke.sh <tag> rov_random_spawn:=true spawn_seed:=<seed>` unmodified,
  computing progress via the reused functions in §2/§3, enforcing the stop-at-minimums-or-cap
  rule. Neither script has been run as a battery (no `ros2 launch` invoked by this spec task);
  `select_failure_seeds.py` was run standalone (pure Python, no simulator) to verify its seed
  screening produces a usable pool, and the battery script's Python progress-counting logic was
  independently verified against the existing `/tmp/p0-2-6-battery` data (V1/V2) while writing
  this spec, confirming it correctly classifies V1 as infra-contaminated (excluded) and V2 as
  converged (not a failure).

## 6. Explicit non-goals

- No battery execution (no `ros2 launch`, no simulator runs) as part of this spec.
- No changes to `qr_logic.py`, `qr_detector.py`, `mission_fsm.py`, controller gains, or
  `_candidates()` ordering.
- No edits to existing battery scripts (`run_approach_qr_battery.sh`, `run_approach_qr_smoke.sh`)
  — only new files.
- No acceptance verdict — this is a design/spec deliverable only, left for a separately approved
  execution pass.

## 7. Status

```text
P0-2.6                    DIAGNOSIS — candidate-distribution evidence gathered (prior session)
P0-2.7                    candidate/gate correlation — INSUFFICIENT DATA (prior session,
                          this session's earlier turn)
P0-2.7 FAILURE BATTERY    SPEC ONLY — pose-selection rule, failure definition, minimum
  SPEC (this document)    sample, stopping rule all defined; select_failure_seeds.py and
                          run_qr_failure_battery.sh written and independently verified
                          against existing data; battery itself NOT RUN
qr_logic.py / qr_detector.py /  UNCHANGED
  mission_fsm.py / controller
Execution                 NOT STARTED — awaiting separate approval to run
  tools/p0-experiments/run_qr_failure_battery.sh
```
