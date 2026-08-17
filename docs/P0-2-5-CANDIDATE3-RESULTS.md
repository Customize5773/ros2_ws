# P0-2.5 Candidate 3 (`approach_min_fmax_frac=0.15`) Acceptance Report (KKI 2026)

Battery executed per the approved isolated-implementation step: raised the force-taper floor in
`_goto_xy()` from `0.05` to `0.15`, **scoped to `APPROACH_QR` only** — `_goto_xy()` is shared
with `_st_hang` and the `NAV_WALL`-adjacent fine-positioning call, so a global change would have
leaked outside the state under test. Implementation: `_goto_xy()` gained an optional
`min_fmax_frac` parameter (default `None` → falls back to the hardcoded `0.05`, so the two other
callers are byte-identical to before); only `_st_approach_qr`'s call site now passes
`min_fmax_frac=self.approach_min_fmax_frac` (new param, default `0.05`).

**Isolated from Candidates 1 and 2**: confirmed via live `ros2 param get` before the battery —
`qr_servo_range=0.3` (Candidate 1 default) and `qr_offset_ema_alpha=1.0` (Candidate 2 default)
both held at baseline while `approach_min_fmax_frac=0.15` was active.

**Regression check**: re-ran the reducer on `/tmp/p0-2-4-battery` with `approach_min_fmax_frac`
defaulting to `0.05` — reproduced the exact baseline numbers byte-for-byte (5/17, 0 diverged,
sat_frac/jitter/final_dist all identical to prior reports).

**Acceptance criteria unchanged**: `DWELL_TICKS=3`, `entered_band_with_dwell` definition, the
Gate 4 PASS/FAIL/INCONCLUSIVE logic, and the stopping rule (`n≥18` OR `entered≥5`) are exactly
as coded for the baseline and both prior candidates — nothing in `reduce_approach_qr.py`'s
decision logic was touched for this candidate; only measurement/reconstruction params
(`approach_min_fmax_frac` threading through `goto_xy_predict`) were added.

## Mandatory guardrail (this is the highest-risk candidate — checked after every batch)

Per `docs/P0-2-5-ENGINEERING-ANALYSIS.md` §B: raising force authority near the target risks the
ROV chasing a still-noisy target harder, which could inflate `diverged`/`saturation_frac`/jitter
even if `entered_band_with_dwell` ticks up — a false positive that would have triggered an
immediate hard-abort. Checked after batch 1 (5 valid), batch 1+2 (11 valid), and the full battery
(17 valid): **`diverged=0` at every checkpoint, `saturation_frac` stayed in the same 0.0-0.23
range already seen in the baseline.** No abort was triggered.

## A. Comparative Summary Table

| Metric | Baseline P0-2.4 (`frac=0.05`) | Candidate 3 (`frac=0.15`) | Delta / Status |
|---|---|---|---|
| n valid runs | 17 | 17 | Equal — clean comparison, no protocol deviation this time |
| Convergence rate (`entered_band_with_dwell`) | 5/17 (29.4%) | 5/17 (29.4%) | **0 — identical, no change** |
| Divergence rate | 0/17 (0%) | 0/17 (0%) | No change — **guardrail passed** |
| Mean `saturation_frac` | 0.154 | 0.142 | Slightly lower — no increase, guardrail passed |
| Mean `stdev(cmd_fx)` | 7.205 N | 7.330 N | +1.7% — negligible |
| Mean `stdev(cmd_fy)` | 6.621 N | 7.130 N | +7.7% — modest, not flagged (no corresponding rise in `diverged`/`sat_frac`) |
| Mean `final_dist` | 0.078 m | **0.067 m** | **−14% — genuine improvement** |
| Median `final_dist` | 0.068 m | **0.057 m** | **−16.5% — genuine improvement** |
| Mean (`final_dist − min_dist_target`) gap | ~0.008 m | 0.006 m | Smaller — less "close then drift back" |
| Gate 4 verdict | FAIL | **FAIL** | Unchanged |

## B. Physical Convergence vs. Metric Analysis

**This is the first candidate where a continuous physical metric genuinely improved without
tripping any guardrail** — mean and median `final_dist` both dropped materially (14-16%), with
`diverged` still at 0/17 and `saturation_frac` not elevated. The mild rise in `stdev(cmd_fy)`
(+7.7%) is the kind of signal the guardrail was built to catch, but it isn't accompanied by any
rise in `diverged` or `saturation_frac` — read as within-noise, not a red flag, consistent with
more force authority producing slightly punchier corrections without destabilizing anything.

**But `entered_band_with_dwell` did not move — still exactly 5/17.** This is a real and
important dissociation, not a contradiction: `final_dist` is a continuous distance, while
`entered_band_with_dwell` requires clearing a specific, tight, binary threshold
(`approach_tol=0.06m` or the `qr_center_tol=0.12` normalized band) for 3 consecutive ticks.
Getting closer *on average* does not guarantee crossing that specific bar more *often* if the
runs that were already failing were failing by a wide enough margin that a ~14% average
improvement still leaves them short. Per the acceptance criteria (unchanged, per instruction),
Gate 4 remains **FAIL** — this improvement does not change the binary verdict.

**No false positives**: the entered-band runs are the same profile as before (dwell either holds
firmly or doesn't), and the guardrail bundle (`diverged`, `saturation_frac`, jitter proxies) shows
no sign of the ROV clipping through the band via higher-velocity noise-chasing — the specific
risk this candidate was flagged for.

## C. Recommendation & Next Step

**Candidate 3 (`approach_min_fmax_frac=0.15`): REJECTED as sufficient to pass Gate 4** — the
acceptance criterion (`entered_band_with_dwell` majority, unchanged per instruction) is not met,
5/17 both before and after. **But it is the only candidate of the three tested so far that shows
a genuine, guardrail-clean improvement in the underlying continuous distance metric** — this is
evidence, not dismissed as noise, and should inform any future combined-candidate discussion
(explicitly out of scope for isolated single-variable testing, per the roadmap's own rule against
scope creep — not proposed here).

**Do not tune `approach_min_fmax_frac` further in this pass** (e.g. try 0.10 or 0.25) — a new
value would need its own justification/exit criteria, not a reactive re-run.

**Status after three candidates**: all three (EMA filter, wider servo gate, higher force floor)
individually fail to flip the Gate 4 binary verdict. Candidate 3 alone shows a real, if partial,
physical effect; Candidates 1 and 2 showed no net benefit or a regression. This is useful
convergent evidence that no single isolated tweak among the three closes the gap — the remaining
approved candidate (#4, dwell-gating in the FSM itself) is a different kind of change
(measurement/policy, not physical), and per the original design, was always expected to be
evaluated last and only after 1-3 were exhausted.

**No code modification beyond what's already in place, no Candidate 4 work, and no further
battery execution proposed in this turn.** Awaiting explicit sign-off.

## Status

```text
P0-2.5 Candidate #1 (qr_servo_range=0.6)        REJECTED — rate regressed, jitter rose
P0-2.5 Candidate #2 (EMA alpha=0.3)             REJECTED — jitter fell, final_dist unchanged
P0-2.5 Candidate #3 (approach_min_fmax_frac=0.15)  REJECTED for Gate 4 (5/17 unchanged),
                                                 but final_dist improved 14-16% with 0
                                                 guardrail violations -- partial physical
                                                 evidence, not a false positive
approach_min_fmax_frac default                  REMAINS 0.05 -- no change to shipped/
                                                 default behavior recommended
Candidates 1-3 (isolated, single-variable)      ALL THREE TESTED, none individually
                                                 flips Gate 4 FAIL -> PASS
Next candidate (approved order 2->1->3->4)      Candidate 4 (FSM dwell-gating) -- NOT YET
                                                 implemented, awaiting sign-off. Last in
                                                 sequence per original design.
qr_detector.py / qr_logic.py / controller       TIDAK DIUBAH beyond the three already-
                                                 approved, already-tested candidates (all
                                                 gated off by default params)
```
