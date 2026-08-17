# P0-2.8 — engineering review: does adaptive_thresh_denoised justify an ordering experiment?

**STATUS: READ-ONLY REVIEW. TIDAK ADA KODE YANG DIUBAH.** `qr_logic.py`, `qr_detector.py`,
`mission_fsm.py`, and `_candidates()` ordering are unchanged. This document formulates a
single-variable experiment for future, separate approval — it does not approve or run it.

## 0. Why this review exists

P0-2.6 (baseline-pose battery, n=47 corner-bearing observations, 6 runs) and P0-2.7
(failure-focused battery, n=127, 18 seeds oversampling corner-adjacent/high-rotation spawn
poses) both independently flagged `adaptive_thresh_denoised` (`qr_logic.py:_candidates()` index
3) as 0% decode-success while holding one of the largest corner-only shares. This review traces
the code mechanism and asks: is that pattern strong enough evidence to *justify formulating* a
narrowly scoped ordering experiment?

## 1. Code trace

`_candidates()` (`qr_logic.py:57-95`) yields 7 preprocessing variants in a fixed order:

```text
0: mentah (raw)
1: clahe (grayscale + CLAHE)
2: adaptive_thresh (adaptive threshold over CLAHE)
3: adaptive_thresh_denoised (median-blur(3) + adaptive threshold over CLAHE)   <-- under review
4: otsu (Otsu global threshold)
5: adaptive_thresh_upscaled (2x upscale of #2's binary)
6: otsu_upscaled (2x upscale of #4's binary)
```

Index 3 (`qr_logic.py:81-84`) is `cv2.medianBlur(clahe_eq, 3)` → `cv2.adaptiveThreshold(...)` —
the only candidate combining denoise+threshold without upscale, evaluated *before* `otsu` and
both upscaled variants.

`robust_decode()` (`qr_logic.py:98-140`) stops at the **first** candidate that decodes; `best_pts`
(used for the visual-servo offset when decode fails everywhere, `qr_detector.py:136`) locks to the
**first** candidate that produces *any* corner points at all, independent of decode success
(`qr_logic.py:129-131`, `if has_pts and best_pts is None`). Because `adaptive_thresh_denoised`
sits at index 3 — ahead of `otsu` and both upscaled variants — whenever it is the first candidate
to yield corners on a given frame, it **pre-empts** `best_pts` even if a later candidate would
also have found corners, possibly better ones. This is the concrete code-level mechanism
connecting candidate *order* to which candidate's corners feed the visual servo on corner-only
frames (the general mechanism was already documented in `docs/P0-2-6-DIAGNOSTIC.md` §2; this
review identifies the specific candidate most implicated by it).

## 2. Evidence review (both existing batteries, no new data collected)

| Battery | adaptive_thresh_denoised decode rate | corner-only share | comparison: mentah decode rate |
|---|---|---|---|
| P0-2.6 (n=47, 6 runs) | 0/13 (0.0%) | 35.3% (largest of 7) | 55.6% (10/18) |
| P0-2.7 (n=127, 18 seeds, failure-focused) | 0/19 (0.0%) | 22.1% (tied largest of 7) | 60.4% (29/48) |

Combined: **0/32 decode successes** for this candidate across two independently-drawn batteries
under different spawn-pose distributions (one baseline-random, one deliberately oversampling
corner/high-rotation poses), while it was tied for or held the single largest corner-only share
in both.

`adaptive_thresh` (index 2 — same CLAHE input, no blur) is weak but non-zero in both batteries
(0/6, then 1/11) — suggestive that the blur step specifically, not CLAHE/threshold in general, is
the point of difference. This is descriptive/correlational only: both candidates ran on the same
frames in these batteries, but nothing here isolates the blur step in a controlled way.

## 3. Verdict of this review

The 0%-decode / high-corner-only pattern is repeated, independently replicated across two
batteries with different pose distributions, and has a concrete code-level mechanism
(`best_pts` first-corner-wins) tying candidate order to the already-quantified corner-only
residual bias (`docs/P0-2-3-SPEC.md`). n=32 for this candidate specifically is modest, but an
effect this clean (0% across two independent batteries) is about as strong as observational
(non-controlled) evidence gets. **This does justify formulating** a narrowly scoped ordering
experiment. It does **not** by itself justify implementing the reorder — that requires the
separately-approved battery in §5 below.

## 4. Formulated experiment (single-variable, P0-2.5-style isolation)

**Proposed change** (not made by this document): reorder `_candidates()` so
`adaptive_thresh_denoised` is evaluated **last** (after `otsu_upscaled`, i.e. index 6 instead of
3) instead of demoting or removing it entirely. All other candidates, their internal logic, and
`robust_decode()`'s stop-at-first-decode / first-corner-wins semantics are unchanged — one
variable (this candidate's position) moves.

**Rationale for demote-not-remove**: since it has contributed 0 decodes in 32 observations across
two batteries, demoting it cannot plausibly *reduce* decode-success rate (DSR) — the only
candidate outcome under test is whether demotion changes *which* candidate's corners populate
`best_pts` on corner-only frames, and whether that changes the corner-only residual profile.
Removing it entirely would also remove it as a corner-source fallback on frames where no other
candidate produces any points at all — demotion preserves that fallback while fixing the
pre-emption problem in §1.

## 5. Explicit hypothesis (falsifiable, stated before any run)

- **H1**: demoting `adaptive_thresh_denoised` to last position reduces its share of corner-only
  `best_pts` captures (measured the same way as `analyze_qr_candidates.py` view 4 — corner-only
  share vs. success share, per candidate) **without reducing overall DSR**.
- **H0 (null)**: reordering has no measurable effect on DSR, corner-only capture share, or
  corner-only residual — this candidate's position in the sequence is incidental, not causal, to
  the pattern observed in §2 (e.g. because whichever candidate ends up producing corners first
  after the reorder inherits the same or a similar corner-only bias).

## 6. Acceptance metrics for the (future, separately approved) battery

To be evaluated by running a battery of comparable size to §2 (order of magnitude ~150+
corner-bearing observations, same discipline as P0-2.6/P0-2.7) with the reorder applied, then
comparing against the existing pre-reorder baseline (P0-2.6 + P0-2.7 combined, n=174) using
`analyze_qr_candidates.py` unmodified:

- **Reject the reordering** if aggregate DSR decreases, or total decode count decreases, versus
  the pre-reorder baseline — since this candidate never decoded, any DSR drop would indicate an
  unexpected interaction or bug in the reorder itself, not the intended effect, and would need
  investigation before any further consideration.
- **Support H1** if `adaptive_thresh_denoised`'s corner-only capture share measurably drops
  (report the actual delta — no fixed numeric threshold is pre-committed here, given how noisy
  these shares already are at n~30-50 per battery) **and** DSR is flat or improved.
- **Residual (`dist_diff_raw`) direction is descriptive only, not a pass/fail criterion** — both
  existing batteries already show inconsistent sign across candidates at these sample sizes
  (e.g. `otsu_upscaled` mean +0.028m in P0-2.6 vs +0.044m in P0-2.7 corner-only view; `otsu` mean
  +0.013m in P0-2.6 vs −0.073m in P0-2.7), so no directional residual change should be required
  for acceptance.
- **Low-n discipline unchanged**: only `mentah`, `adaptive_thresh`, `adaptive_thresh_denoised`,
  `adaptive_thresh_upscaled` are treated as having enough n for comparison; `clahe`, `otsu`,
  `otsu_upscaled` remain `[LOW-N]`-flagged and descriptive-only, exactly as in P0-2.6/P0-2.7.
- **Minimum evidence**: no verdict on H1 vs H0 is drawn below ~150 corner-bearing observations
  in the post-reorder battery — matching this review's own combined evidence base, so the
  before/after comparison has comparable statistical power on both sides.

## 7. Explicit non-goals of this document

- No edits to `qr_logic.py`, `qr_detector.py`, `mission_fsm.py`, controller, or any candidate
  order.
- No battery execution.
- No acceptance verdict on H1 — that requires the future, unscheduled, separately-approved
  battery described in §6.

## 8. Status

```text
P0-2.6                     DIAGNOSIS — candidate-distribution evidence, n=47
P0-2.7                     candidate/gate correlation — INSUFFICIENT DATA (small sample);
                            failure-focused battery — evidence criteria MET, n=127
P0-2.8 (this document)     REVIEW ONLY — traced _candidates()/robust_decode(), reviewed
                            combined evidence (n=174 across two batteries), formulated ONE
                            single-variable ordering experiment (demote
                            adaptive_thresh_denoised to last) with explicit hypothesis and
                            acceptance metrics. NOT approved, NOT implemented, NOT run.
qr_logic.py / qr_detector.py /  UNCHANGED
  mission_fsm.py / controller /
  _candidates() ordering
Implementation              NOT STARTED — awaiting separate approval to reorder + battery
```
