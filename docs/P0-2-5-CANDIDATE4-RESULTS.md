# P0-2.5 Candidate 4 (`approach_dwell_ticks=3`) Acceptance Report (KKI 2026)

Battery executed per the approved isolated-implementation step: the FSM's own `GRAB` exit
condition (`mission_fsm.py`, formerly a single-tick check at the old L659/L666) now requires the
convergence condition (`centered` OR `dist < approach_tol`) to hold for `approach_dwell_ticks`
consecutive 10 Hz ticks (default `1` = old single-tick behavior, unchanged; this battery used
`3`, matching the dwell threshold `reduce_approach_qr.py` has used to evaluate every candidate so
far). Scoped correctly: `_goto_xy()`'s shared taper logic (Candidate 3) was untouched; this
candidate only touches the transition-gating block in `_st_approach_qr`.

**Isolated from Candidates 1-3**: confirmed via live `ros2 param get` — `qr_servo_range=0.3`,
`approach_min_fmax_frac=0.05`, `qr_offset_ema_alpha=1.0` all held at default while
`approach_dwell_ticks=3` was active.

**Default-behavior verification (this candidate needed a live-mission check, not just a
regression re-run, since it changes runtime FSM transition logic)**: ran one full mission at
default `approach_dwell_ticks=1`. Confirmed identical to pre-Candidate-4 behavior — GRAB fired on
the same tick the condition first held (`Wall A dipilih (+15) ... [FSM] APPROACH_QR -> GRAB`),
mission continued normally through `NAV_WALL → HANG → SURFACE → WAIT_TRIGGER`. (Two earlier
attempts at this check were contaminated/timed out due to transient system memory pressure on
the shared desktop this session runs on — unrelated to the code change; a clean retry confirmed
the result and the session moved on.)

## Why this candidate is evaluated differently

Per `docs/P0-2-5-ENGINEERING-ANALYSIS.md` §A.4: Candidate 4 doesn't fix anything physical — it
makes the live FSM's actual GRAB trigger match the same dwell definition
`reduce_approach_qr.py` already uses to *measure* every other candidate. **This means
`entered_band_with_dwell` is expected to become close to trivially satisfied by construction**:
once the FSM itself won't transition until it has held the band for 3 ticks, almost every
non-timeout exit will, by definition, have satisfied a 3-tick dwell. A high count here is not
independent evidence of anything — it was designed into the mechanism. The metric that actually
carries information for this candidate is **TIMEOUT rate**: does forcing genuine settling before
GRAB cost mission completion time and push any run past `t_scan=45s`?

## A. Results

18 runs (3 batches), `Q25` excluded (infra-level contamination — `odom-publishing`/`stabilizer`/
`mission_fsm` all reported missing, the same launch-failure class seen throughout this session
with `U1`/`E1`/`H1`/`L1`, unrelated to dwell logic). **17 valid.**

| Metric | Baseline P0-2.4 (`dwell=1`) | Candidate 4 (`dwell=3`) | Note |
|---|---|---|---|
| `entered_band_with_dwell` | 5/17 (29.4%) | 13/17 (76.5%) | **Expected by construction — not independent evidence, see above** |
| **TIMEOUT rate** | 0/17 (0%) | **0/17 (0%)** | **The metric that matters — unchanged** |
| Divergence rate | 0/17 | 0/17 | No change |
| Mean `final_dist` | 0.078 m | **0.052 m** | Improved — consistent with runs now exiting only once genuinely settled |
| Median `final_dist` | 0.068 m | **0.047 m** | Improved |
| Exit path mix | 6 GT-fallback / 4 visual / 7 XY-tol | 4 GT-fallback / 4 visual / 9 XY-tol | Broadly similar distribution |

**Reconstruction-fidelity caveat, observed concretely this time**: 4/17 runs (`O3`, `P4`,
`Q22`, `Q26`) reached `GRAB` via a non-timeout exit path but were flagged
`entered_band_with_dwell=False` by the offline reducer — i.e., the live FSM's own dwell
tracking and the reducer's independent reconstruction disagreed. This is the exact caveat
`reduce_approach_qr.py` has carried since P0-2.4 (`locked_yaw` approximation, recorder/FSM tick
alignment) — now visible as a live discrepancy rather than a theoretical concern. It does not
change the TIMEOUT-rate conclusion (all 4 of these runs still exited well before `t_scan`), but
it means the reducer's dwell reconstruction should be read as approximate, not as ground truth
for what the live FSM actually enforced.

## B. Physical Convergence vs. Metric Analysis

**No false positive**: the jump to 13/17 is not being claimed as "Gate 4 now passes" — that
would be circular, since the FSM was changed to make the metric and the mechanism the same
thing. The section immediately above states this explicitly rather than letting the raw number
imply an unearned win.

**The one metric that isn't circular — TIMEOUT rate — shows no regression.** 0/17 before, 0/17
after. Forcing 3-tick settling did not push any run past the 45s budget. Combined with `final_dist`
improving (0.078m → 0.052m, the best of any candidate tested, better even than Candidate 3's
0.067m), this suggests the runs that do converge settle to a noticeably closer point when GRAB
isn't triggered by a single lucky tick — consistent with the very first observation in
`docs/P0-2-5-ENGINEERING-ANALYSIS.md` §0 that many baseline runs got close and then drifted away
again before the state exited.

**No divergence, no guardrail violation**: `diverged=0/17`, consistent with every candidate
tested — dwell-gating doesn't change control authority or servo signal, so no reason to expect
instability, and none was found.

## C. Recommendation & Next Step

**Candidate 4 (`approach_dwell_ticks=3`): the results are genuinely positive, but the
comparison is definitionally weaker than Candidates 1-3** — this candidate changes what "success"
means rather than testing whether an existing definition of success happens more often. The
honest reading:

- **Supports adoption**: TIMEOUT rate unchanged (0/17), `final_dist` improved, no divergence.
  Nothing in this data argues against tightening the live acceptance gate to match the standard
  already used to evaluate the system.
- **Does not, by itself, "solve" Gate 4**: it doesn't make the ROV converge more often or more
  precisely — it makes the system honest about when it has actually converged, which is a
  measurement/policy improvement, not a controller fix. The underlying physical bottleneck
  identified after Candidates 1-3 (systematic bias in QR-derived offsets, not noise, not
  exposure time, partially mitigated but not solved by more force authority) is untouched by
  this candidate and remains the open engineering question.

**This closes the four approved candidates.** Summary across all four, isolated and
single-variable as required throughout:

| Candidate | Mechanism | Verdict |
|---|---|---|
| 1 — wider servo gate | More exposure time | REJECTED — rate regressed, jitter rose |
| 2 — EMA filter | Noise smoothing | REJECTED — jitter fell, `final_dist` unchanged |
| 3 — higher force floor | More authority near target | REJECTED for Gate 4, but `final_dist` improved 14-16%, guardrail-clean |
| 4 — dwell-gating | Honest measurement, not a fix | Definitionally can't fail Gate 4; TIMEOUT-neutral, `final_dist` improved most (33%) |

**No further code modification, no new candidate, and no combined/stacked-variable testing
proposed in this turn** (combining candidates was explicitly out of scope for this single-variable
roadmap throughout). Per the roadmap's own exit condition (`docs/P0-2-5-ENGINEERING-ANALYSIS.md`
§C): since none of the four isolated candidates individually resolves precision convergence, the
next step the roadmap itself points to is escalation toward the Mode-1 failure class (6+/17 runs
with zero QR decodes across the whole episode) via `qr_detector.py`/`qr_logic.py` — explicitly
**not** attempted in this pass, and requiring its own separate review before any such work
begins.

## Status

```text
P0-2.5 Candidate #1 (qr_servo_range=0.6)           REJECTED
P0-2.5 Candidate #2 (EMA alpha=0.3)                REJECTED
P0-2.5 Candidate #3 (approach_min_fmax_frac=0.15)  REJECTED for Gate 4, final_dist -14-16%
P0-2.5 Candidate #4 (approach_dwell_ticks=3)       TIMEOUT-neutral (0/17->0/17), final_dist
                                                    -33% (best of 4), but Gate 4 metric is
                                                    circular by design for this candidate --
                                                    not claimed as "solving" convergence
All 4 approved candidates                          TESTED, isolated, single-variable
Default params (all 4)                             UNCHANGED -- qr_servo_range=0.3,
                                                    approach_min_fmax_frac=0.05,
                                                    qr_offset_ema_alpha=1.0,
                                                    approach_dwell_ticks=1
Next step per roadmap exit condition                Escalation to qr_detector.py/qr_logic.py
                                                    decode-quality work (Mode 1) -- NOT
                                                    attempted, requires separate review
qr_detector.py / qr_logic.py                        TIDAK DIUBAH
```
