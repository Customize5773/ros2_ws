# P0-2.5 Candidate 1 (`qr_servo_range=0.6`) Acceptance Report (KKI 2026)

Battery executed per the approved isolated-implementation step: widened the visual-servo
activation gate from `dist_raw < 0.3` to `dist_raw < 0.6` (`mission_fsm.py:601`, now the
`qr_servo_range` param, default `0.3` unchanged). **Isolated from Candidate 2**: every run in
this battery used the default `qr_offset_ema_alpha=1.0` (filter inactive) — confirmed via
`ros2 param get` before the battery started and via the reducer's per-run
`qr_offset_ema_alpha_used` field afterward (`[1.0]` across all runs). No other parameter,
detector, or controller code touched.

**Isolation/regression check before running anything**: re-ran the updated reducer on the
original `/tmp/p0-2-4-battery` CSVs with the new `qr_servo_range` defaulting to `0.3` —
reproduced the exact baseline numbers (5/17, 0 diverged, jitter 0.020/0.012, final_dist 0.078)
byte-for-byte, confirming the new code path is a true no-op at default settings.

## Protocol deviation (reported, not hidden)

Per the stopping rule (`n_reached≥18` OR `entered≥5`), batch 3 (18 total, 17 valid after 1
contamination) landed at **4/17 entered, stopping rule NOT met** — one run short of 18 valid
because of a contaminated run (`H1`, same transient-gate-failure class as `U1`/`E1` in prior
batteries). Per the acceptance criteria (unchanged, not loosened to force an early stop), a
4th batch was run. Final: **24 runs total, 23 valid, stopping rule met at `entered=5`.**

This means the Candidate 1 battery has a **larger n (23) than the baseline (17)** — the
comparison below reports both raw counts and rates, and flags this explicitly because raw
counts alone are not comparable across different n.

## A. Comparative Summary Table

| Metric | Baseline P0-2.4 (`range=0.3`) | Candidate 1 (`range=0.6`) | Delta / Status |
|---|---|---|---|
| n valid runs | 17 | 23 | (see protocol deviation above) |
| Convergence rate (`entered_band_with_dwell`) | 5/17 (29.4%) | 5/23 (21.7%) | **−7.7 points — regression, not improvement** |
| Divergence rate | 0/17 (0%) | 0/23 (0%) | No change |
| Mean `servo_frac` (fraction of ticks servoing active) | 0.301 | 0.384 | **+27% relative — mechanism engaged as designed** |
| Mean `t_x` jitter (servoing ticks) | 0.020 m | 0.026 m | **+30% — worse, not better** |
| Mean `t_y` jitter | 0.012 m | 0.014 m | +17% — worse |
| Mean `final_dist` | 0.078 m | 0.070 m | Slightly better, but see §B |
| Gate 4 verdict | FAIL | **FAIL** | Unchanged |

## B. Physical Convergence vs. Metric Analysis

**Did wider exposure translate into higher physical convergence? No — by rate, it went
backward.** The stopping rule triggers on an absolute count (`entered≥5`), which this battery
eventually hit, but only after growing to n=23 vs the baseline's n=17. As a **rate**,
convergence dropped from 29.4% to 21.7%. Reporting only "5/23 vs 5/17, both hit the stopping
threshold" would obscure a real regression — flagged here explicitly per the reporting
requirement to check for exactly this kind of metric-comparability trap.

**The mechanism worked exactly as designed, and that is informative on its own.**
`servo_frac` rose 27% (0.301→0.384) — the widened gate genuinely gave the visual servo more
ticks to operate, confirming the code change does what `docs/P0-2-5-ENGINEERING-ANALYSIS.md`
§A intended. But per the pre-registered exit-criteria note in §C, Experiment 1: *"kalau
`servo_frac` naik tapi `entered_band` tidak, itu sendiri temuan (exposure lebih sering aktif
tidak cukup, noise-nya yang jadi masalah)"* — that is exactly what happened here.

**Jitter got worse, not better** — `t_x` jitter rose 30%, `t_y` 17%. This matches the specific
risk flagged in the design-hardening review (Candidate 1, §A): engaging the servo farther from
the target likely exposes it to less mature/noisier QR corner geometry (larger incidence angle,
smaller apparent size) than the tighter 0.3 m gate did. The extra exposure time didn't give the
controller more time to settle on a stable signal — it gave it more ticks of a noisier signal
to chase.

**No false positives**: `diverged=0/23` — the added exposure and higher jitter did not tip into
physical instability by the existing divergence criterion (trend + saturation). `final_dist`
mean improved slightly (0.078→0.070m), but given the convergence *rate* regressed and jitter
rose, this is more plausibly explained by the larger/differently-composed sample (23 vs 17 runs,
different random spawns — no seed control exists per prior documentation) than by a genuine
precision improvement; it should not be read as evidence for this candidate.

## C. Recommendation & Next Step

**Candidate 1 (`qr_servo_range=0.6`): REJECTED.**

This is a clean, unambiguous rejection per the pre-registered exit criteria — not a "servo_frac
rose so it's inconclusive" case. `servo_frac` rising while `entered_band` rate *fell* and jitter
*rose* is precisely the result the roadmap earmarked as informative evidence that **exposure
time was not the bottleneck** — the noise/bias in the QR signal itself (already isolated as the
likely dominant factor after Candidate 2's rejection) degrades further at greater range, not less.

**Do not tune `qr_servo_range` further** (e.g. try 0.4 or 0.5) — there is no evidence in this
data that a smaller widening would avoid the noise-exposure trade-off; it would just be a
weaker version of the same trade-off, not a new hypothesis.

**Two candidates down (2: EMA filter, 1: wider gate), both rejected on their own terms** —
neither noise-smoothing nor more exposure time helped, and in Candidate 1's case, more exposure
measurably hurt signal quality. This strengthens the reading (already flagged after Candidate 2)
that the dominant driver of Gate 4 failure is the **systematic bias** in QR-derived offsets
(AABB inflation / corner-only degradation, documented in P0-2.3), not noise or insufficient
servo engagement time.

**Per the approved sequence (2 → 1 → 3 → 4)**: Candidate 3 (`min_fmax_frac`) is next, and per
the hardening review it carries the **highest risk** of the four (§B: potential to mask
increased physical oscillation as "more band crossings" — mandatory guardrails on
`diverged`/`saturation_frac`/`sign_changes` apply, hard-abort if any rise). Given that neither
of the two lower-risk candidates addressed the actual bottleneck, Candidate 3 should be
approached with the expectation that it targets a *different* hypothesis (insufficient force
authority against a persistent bias) than what's now been twice ruled out (noise).

**No code modification beyond what's already in place, no Candidate 3/4 work, and no further
battery execution proposed in this turn.** Awaiting explicit sign-off.

## Status

```text
P0-2.5 Candidate #1 (qr_servo_range=0.6)   REJECTED — servo_frac rose (+27%) as designed,
                                            but entered_band RATE regressed (29.4%->21.7%)
                                            and jitter rose (+30%/+17%); exposure was not
                                            the bottleneck
P0-2.5 Candidate #2 (EMA alpha=0.3)        REJECTED (prior turn) — jitter fell but
                                            final_dist unchanged
qr_servo_range default                      REMAINS 0.3 -- no change to shipped/default
                                            behavior recommended
Next candidate (approved order 2->1->3->4)  Candidate 3 (min_fmax_frac) -- NOT YET
                                            implemented, awaiting sign-off. Highest risk of
                                            the four; targets a different (bias-authority)
                                            hypothesis than the two already rejected.
qr_detector.py / qr_logic.py / controller   TIDAK DIUBAH beyond the two already-approved,
                                            already-rejected candidate implementations
```
