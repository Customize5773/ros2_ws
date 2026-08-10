# P0-2.5 Candidate 2 (α=0.3) Acceptance Report (KKI 2026)

Battery executed per the approved isolated-implementation step: EMA smoothing
(`qr_offset_ema_alpha=0.3`) on `qr_ex`/`qr_ey` before they feed the visual-servo target
computation (`mission_fsm.py:559-567`), everything else unchanged. Same protocol as the
P0-2.4 baseline (3 batches × 6 runs = 18, 5 random-spawn + 1 deterministic per batch,
stopping rule `n≥18` OR `entered≥5`). Data: `/tmp/p0-2-5-candidate2-battery/*.csv`, `*.log`,
`*.params.yaml`, `P0-2-2b-results.json` (not committed, ephemeral, same convention as prior
batteries). 18 runs total, `E1` excluded (contamination gate FAIL, same class of transient
failure `U1` had in the baseline) — **17 valid runs**, matching baseline `n` exactly.

## A. Comparative Summary Table

| Metric | Baseline P0-2.4 (α=1.0) | Candidate 2 (α=0.3) | Delta / Status |
|---|---|---|---|
| Convergence rate (`entered_band_with_dwell`) | 5/17 (29.4%) | 5/17 (29.4%) | **0 — no change** |
| Divergence rate | 0/17 (0%) | 0/17 (0%) | No change (expected — Candidate 2 doesn't touch force/timing) |
| Mean `t_x` jitter (stdev of tick-to-tick target diff, servoing ticks) | 0.020 m | 0.005 m | **−75%, large improvement** |
| Mean `t_y` jitter | 0.012 m | 0.004 m | **−67%, large improvement** |
| Mean `final_dist` | 0.078 m | 0.077 m | **−0.001 m — essentially unchanged** |
| Mean `min_dist_target` | ~0.070 m (computed from same 17-run set, see §B) | ~0.075 m | No material improvement |
| Gate 4 verdict | FAIL | **FAIL** | Unchanged |

(Baseline jitter/`final_dist` figures are the exact numbers reported when the updated reducer
was re-run on the original `/tmp/p0-2-4-battery` CSVs at `alpha=1.0` — confirmed identical to
the pre-Candidate-2 Gate 4 result, so this is an apples-to-apples comparison, not a re-derived
estimate.)

## B. Physical Convergence vs. Metric Analysis

**Did lower jitter translate into higher physical convergence? No.** `entered_band_with_dwell`
is identically 5/17 in both batteries — not just similar, the exact same count. Gate 4 verdict
is unchanged: **FAIL**, stopping rule met the same way (`entered≥5`).

**Jitter reduction is real and substantial, not noise.** stdev of tick-to-tick target movement
dropped 75%/67% (tx/ty) — this confirms the causal mechanism identified in
`docs/P0-2-5-ENGINEERING-ANALYSIS.md` §A.1 (unfiltered raw `qr_ex`/`qr_ey` causing the servo
target to move every tick) was correctly diagnosed and the filter does what it was designed to
do at the signal level.

**But per-run trajectory shape changed in a way the headline numbers don't show**: in the
baseline, several runs showed `min_dist_target` well below `final_dist` (e.g. `U2`: min=0.072,
final=0.103; `U3`: min=0.066, final=0.125) — the ROV got close, then drifted back out before the
state exited, consistent with a jittery target being chased. In this battery, **`min_dist_target
== final_dist` in 15/17 runs** — the trajectory now decreases and *stays* rather than
overshooting and re-widening. Mean gap (`final_dist − min_dist_target`) dropped from ~0.008 m
(baseline) to ~0.003 m (Candidate 2). This is a genuine qualitative confirmation of the jitter
hypothesis: **the "get close then drift away" failure mode is largely gone.**

**Guardrail flag (per `docs/P0-2-5-ENGINEERING-ANALYSIS.md` §B, Candidate 2 — exactly the risk
that section predicted)**: the mean `final_dist` did **not** improve (0.078m → 0.077m). Per the
recorded guardrail rule — *"credit this candidate ONLY if jitter AND mean final_dist both
improve — reduced jitter alone can mean a filter that stabilized on a biased value, not a fixed
problem"* — this result does **not** clear the bar. The trajectory now converges *smoothly* but
to a resting point that is, on average, just as far from the true target as before. This is
consistent with the systematic bias documented in P0-2.3 (AABB inflation, corner-only residual
bias up to −0.19m) persisting through the filter — an EMA damps noise, not a mean-shifted bias.

**No phase-lag-induced degradation observed**: `α=0.3` is a fairly aggressive smoothing constant
and could in principle introduce enough lag to make the target chronically trail the true offset,
widening `final_dist`. That didn't happen here (`final_dist` is flat, not worse) — so the lack of
improvement is better explained by "jitter wasn't the dominant driver of the residual distance"
than by "the filter made things worse." Both explanations were live hypotheses in the design
review; this data favors the former.

**No false positives found**: no run shows `entered_band_with_dwell=True` riding on a
still-unstable trajectory — `diverged=0/17` and the min/final convergence confirms genuine
(if incomplete) settling wherever runs did stabilize.

## C. Recommendation & Next Step

**Candidate 2 (α=0.3): REJECTED as a standalone fix for Gate 4.**

Reasoning: it achieves its own stated mechanism (jitter reduction, confirmed both by the target
variance metric and by the min/final trajectory-shape change) but does **not** move the metric
that matters — `entered_band_with_dwell` is unchanged at 5/17, and `final_dist` does not improve.
Per the pre-registered guardrail, jitter reduction alone is insufficient to credit this
candidate. This is not "inconclusive due to underpowered dosage" (§C of the design-hardening
review flagged that risk) — `n_servoing_ticks` and jitter reduction are both clearly present in
this data (15/17 runs had ≥3 servoing ticks, mean jitter dropped 70%+), so the mechanism was
adequately exercised. The correct reading is: **noise/jitter was not the dominant cause of Gate 4
failure** — the systematic bias component (already documented in P0-2.3) appears to dominate.

**Do not tune α further** (e.g. 0.2 or 0.5) as a next step — per the roadmap's own exit
criteria (`docs/P0-2-5-ENGINEERING-ANALYSIS.md` §C, Experiment 2), re-tuning α under an already-
rejected hypothesis is scope creep without new justification; a different α would only be
worth testing if there were evidence of *under-* or *over*-filtering (e.g. lag artifacts), which
this data doesn't show.

**Per the approved sequence (2 → 1 → 3 → 4)**: next candidate up for consideration is
**Candidate 1** (widen servoing gate `dist_raw<0.3`), still not implemented. Per the original
design, Candidate 1 has a weaker causal link and was placed second precisely because Candidate 2
was the strongest hypothesis — its rejection here doesn't promote Candidate 3 or 4 automatically;
the roadmap says proceed in order.

**No code modification, no Candidate 4/3 work, and no further battery execution proposed in this
turn.** Awaiting explicit sign-off before implementing Candidate 1.

## Status

```text
P0-2.5 Candidate #2 (EMA alpha=0.3)   REJECTED — jitter mechanism confirmed, but
                                       entered_band_with_dwell unchanged (5/17=5/17) and
                                       final_dist unchanged (0.078m -> 0.077m); guardrail
                                       from design-hardening review correctly caught this
qr_offset_ema_alpha default            REMAINS 1.0 (filter inactive) -- no change to
                                       shipped/default behavior recommended
Next candidate (approved order 2->1->3->4)   Candidate 1 (widen servoing gate) -- NOT YET
                                       implemented, awaiting sign-off
qr_detector.py / qr_logic.py /         TIDAK DIUBAH (unchanged beyond the already-approved
  controller                          Candidate #2 code, which stays in place but is not
                                       recommended for adoption)
```
