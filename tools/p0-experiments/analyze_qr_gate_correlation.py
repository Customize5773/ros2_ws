#!/usr/bin/env python3
"""P0-2.7 -- correlate qr_logic.py candidate behavior with APPROACH_QR gate
outcome. DIAGNOSTIC ONLY: reuses analyze_qr_candidates.py's extraction/metrics
(P0-2.6) plus each run's <tag>.gate.txt, and asks whether the one failed run
(V1) is distinguishable from the passing runs on candidate-level behavior --
does NOT modify qr_logic.py/qr_detector.py/mission_fsm.py/controller, does NOT
reorder candidates, does NOT propose a fix, and reaches no verdict beyond the
required 4-way diagnostic classification. Correlational only -- no causal
claim (P0-2.7 task instructions).

usage: analyze_qr_gate_correlation.py <data_dir> [tag ...]
"""
import csv
import os
import statistics as stats
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from analyze_qr_candidates import (   # noqa: E402  (P0-2.6, reused not copied)
    CANDIDATE_NAMES, CORNER_COLS, quad_metrics, to_float, is_nan,
    h_cam_from_depth, QR_SIDE_M, FX_PX, FRAME_W_PX, candidate_name,
)

LOW_N_CANDIDATES = {'clahe', 'otsu', 'otsu_upscaled'}  # flagged low-n in P0-2.6
LOW_N_THRESHOLD = 5


def load_csv(path):
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


def load_gate(data_dir, tag):
    try:
        with open('%s/%s.gate.txt' % (data_dir, tag)) as f:
            return f.read().strip()
    except FileNotFoundError:
        return 'UNKNOWN'


def extract_run_observations(tag, gate, rows):
    """All corner-bearing APPROACH_QR rows with valid qr_candidate_idx, in
    time order, deduped on consecutive identical corner_tuple (same rule as
    P0-2.6's analyze_qr_candidates.extract_observations). Kept per-run (not
    merged across tags) so V1 is never silently pooled with V2-V6."""
    obs = []
    last_tuple = None
    for r in rows:
        if r.get('fsm_state') != 'APPROACH_QR':
            continue
        if 'qr_candidate_idx' not in r:
            continue
        cand = to_float(r.get('qr_candidate_idx'))
        if is_nan(cand) or cand < 0:
            continue
        decode_success = to_float(r.get('qr_decode_success'))
        corners = [to_float(r.get(c)) for c in CORNER_COLS]
        if is_nan(decode_success) or any(is_nan(v) for v in corners):
            continue
        corner_tuple = tuple(corners)
        if corner_tuple == last_tuple:
            continue
        last_tuple = corner_tuple

        qm = quad_metrics(corners)
        if qm is None:
            continue
        _, _, inflation = qm

        dist_diff_raw = None
        distance_est = None
        qr_size = to_float(r.get('qr_size'))
        qr_ey = to_float(r.get('qr_ey'))
        sp_depth = to_float(r.get('sp_depth'))
        if not is_nan(qr_size) and qr_size > 0 and not is_nan(qr_ey) and not is_nan(sp_depth):
            distance_est = (FX_PX * QR_SIDE_M) / (qr_size * FRAME_W_PX)
            h_cam = h_cam_from_depth(-sp_depth)
            dist_diff_raw = distance_est - h_cam

        obs.append({
            'run': tag,
            'gate': gate,
            't': to_float(r.get('t')),
            'candidate_idx': int(cand),
            'candidate_name': candidate_name(int(cand)),
            'decode_success': int(decode_success),
            'dist_diff_raw': dist_diff_raw,
            'distance_est': distance_est,
            'inflation': inflation,
        })
    return obs


def n_tag(n):
    return '[LOW-N]' if n < LOW_N_THRESHOLD else ''


def resid_stats(vals):
    vals = [v for v in vals if v is not None]
    if not vals:
        return None
    return {'n': len(vals), 'mean': stats.fmean(vals), 'median': stats.median(vals)}


def print_view_a(runs_obs):
    print('\n' + '=' * 78)
    print('VIEW A -- per-run candidate distribution')
    print('=' * 78)
    for tag, gate, obs in runs_obs:
        print('\n[%s] gate=%s n=%d' % (tag, gate, len(obs)))
        for idx, name in enumerate(CANDIDATE_NAMES):
            cand_obs = [o for o in obs if o['candidate_idx'] == idx]
            if not cand_obs:
                continue
            n_s = sum(1 for o in cand_obs if o['decode_success'] == 1)
            n_f = len(cand_obs) - n_s
            tag_lown = n_tag(len(cand_obs)) if name in LOW_N_CANDIDATES else ''
            print('  [%d] %-26s obs=%-3d decode_success=%-3d corner_only=%-3d rate=%5.1f%% %s'
                  % (idx, name, len(cand_obs), n_s, n_f, 100.0 * n_s / len(cand_obs), tag_lown))


def print_view_b(v1_obs, rest_obs):
    print('\n' + '=' * 78)
    print('VIEW B -- V1 (FAILED) vs V2-V6 (PASSED), NOT pooled')
    print('=' * 78)
    groups = [('V1 (FAILED)', v1_obs), ('V2-V6 (PASSED)', rest_obs)]
    for label, obs in groups:
        succ = [o for o in obs if o['decode_success'] == 1]
        fail = [o for o in obs if o['decode_success'] == 0]
        print('\n-- %s -- n=%d (decode_success=%d, corner_only=%d)' % (label, len(obs), len(succ), len(fail)))
        for idx, name in enumerate(CANDIDATE_NAMES):
            cand_obs = [o for o in obs if o['candidate_idx'] == idx]
            if not cand_obs:
                continue
            share = 100.0 * len(cand_obs) / len(obs) if obs else 0.0
            n_s = sum(1 for o in cand_obs if o['decode_success'] == 1)
            n_f = len(cand_obs) - n_s
            succ_share = 100.0 * n_s / len(succ) if succ else 0.0
            fail_share = 100.0 * n_f / len(fail) if fail else 0.0
            rs = resid_stats([o['dist_diff_raw'] for o in cand_obs])
            rs_str = ('mean=%+.3fm median=%+.3fm' % (rs['mean'], rs['median'])) if rs else 'n/a'
            tag_lown = n_tag(len(cand_obs)) if name in LOW_N_CANDIDATES else ''
            print('  [%d] %-26s cand_share=%5.1f%% (n=%-3d)  succ_share=%5.1f%%  '
                  'corner_share=%5.1f%%  dist_diff_raw: %s %s'
                  % (idx, name, share, len(cand_obs), succ_share, fail_share, rs_str, tag_lown))


def print_view_c(runs_obs):
    print('\n' + '=' * 78)
    print('VIEW C -- temporal candidate sequence (APPROACH_QR window, time-ordered)')
    print('=' * 78)
    for tag, gate, obs in runs_obs:
        ordered = sorted(obs, key=lambda o: o['t'])
        print('\n[%s] gate=%s n=%d' % (tag, gate, len(ordered)))
        changes = 0
        prev_idx = None
        for o in ordered:
            marker = ''
            if prev_idx is not None and o['candidate_idx'] != prev_idx:
                changes += 1
                marker = '  <- candidate changed'
            dd = ('%+.3fm' % o['dist_diff_raw']) if o['dist_diff_raw'] is not None else 'n/a'
            print('    t=%9.3f  [%d] %-26s decode_success=%d dist_diff_raw=%s%s'
                  % (o['t'], o['candidate_idx'], o['candidate_name'],
                     o['decode_success'], dd, marker))
            prev_idx = o['candidate_idx']
        print('  candidate-index changes within run: %d (of %d transitions)'
              % (changes, max(0, len(ordered) - 1)))


def print_view_d(runs_obs):
    print('\n' + '=' * 78)
    print('VIEW D -- first vs final observation (reported separately, not collapsed)')
    print('=' * 78)
    for tag, gate, obs in runs_obs:
        ordered = sorted(obs, key=lambda o: o['t'])
        print('\n[%s] gate=%s' % (tag, gate))
        if not ordered:
            print('  no corner-bearing observations')
            continue
        first_usable = ordered[0]
        print('  first usable observation:   t=%.3f  [%d] %s  decode_success=%d'
              % (first_usable['t'], first_usable['candidate_idx'],
                 first_usable['candidate_name'], first_usable['decode_success']))
        first_success = next((o for o in ordered if o['decode_success'] == 1), None)
        if first_success:
            print('  first successful decode:    t=%.3f  [%d] %s'
                  % (first_success['t'], first_success['candidate_idx'], first_success['candidate_name']))
        else:
            print('  first successful decode:    none in this run')
        final_obs = ordered[-1]
        print('  final corner-bearing obs:   t=%.3f  [%d] %s  decode_success=%d'
              % (final_obs['t'], final_obs['candidate_idx'],
                 final_obs['candidate_name'], final_obs['decode_success']))


def print_view_e(v1_obs, rest_obs):
    print('\n' + '=' * 78)
    print('VIEW E -- candidate -> residual -> decode outcome -> gate (descriptive, NOT causal)')
    print('=' * 78)
    for idx, name in enumerate(CANDIDATE_NAMES):
        v1_cand = [o for o in v1_obs if o['candidate_idx'] == idx]
        rest_cand = [o for o in rest_obs if o['candidate_idx'] == idx]
        if not v1_cand and not rest_cand:
            continue
        v1_rs = resid_stats([o['dist_diff_raw'] for o in v1_cand])
        rest_rs = resid_stats([o['dist_diff_raw'] for o in rest_cand])
        v1_rate = (100.0 * sum(1 for o in v1_cand if o['decode_success'] == 1) / len(v1_cand)) if v1_cand else None
        rest_rate = (100.0 * sum(1 for o in rest_cand if o['decode_success'] == 1) / len(rest_cand)) if rest_cand else None
        tag_lown = n_tag(len(v1_cand) + len(rest_cand)) if name in LOW_N_CANDIDATES else ''
        print('  [%d] %-26s V1: n=%-3d rate=%s resid=%s | V2-6: n=%-3d rate=%s resid=%s %s'
              % (idx, name, len(v1_cand),
                 ('%5.1f%%' % v1_rate) if v1_rate is not None else '  n/a',
                 ('%+.3fm' % v1_rs['mean']) if v1_rs else 'n/a',
                 len(rest_cand),
                 ('%5.1f%%' % rest_rate) if rest_rate is not None else '  n/a',
                 ('%+.3fm' % rest_rs['mean']) if rest_rs else 'n/a',
                 tag_lown))
    print('\nNote: n=1 failed run vs n=5 passed runs. Any V1-vs-rest pattern below is')
    print('correlational at best (one failure cannot establish a rate); it is reported')
    print('to describe the data, not to claim any candidate causes the gate outcome.')


def classify(v1_obs, rest_obs):
    """Returns (classification_string, reasoning_lines). Rule-based on the
    concrete comparisons view B/E already computed -- not chosen in advance."""
    reasons = []
    n_v1, n_rest = len(v1_obs), len(rest_obs)
    if n_v1 < 10 or n_rest < 30:
        reasons.append('n=%d (V1) / n=%d (V2-V6) -- below what a 6-run, 1-failure '
                        'battery can use to establish a rate difference.' % (n_v1, n_rest))
        return 'INSUFFICIENT DATA', reasons

    def cand_share(obs, idx):
        return sum(1 for o in obs if o['candidate_idx'] == idx) / len(obs) if obs else 0.0

    adaptive_idxs = [i for i, n in enumerate(CANDIDATE_NAMES) if n.startswith('adaptive_thresh')]
    v1_adaptive_share = sum(cand_share(v1_obs, i) for i in adaptive_idxs)
    rest_adaptive_share = sum(cand_share(rest_obs, i) for i in adaptive_idxs)
    v1_mentah_share = cand_share(v1_obs, 0)
    rest_mentah_share = cand_share(rest_obs, 0)

    v1_resid = resid_stats([o['dist_diff_raw'] for o in v1_obs])
    rest_resid = resid_stats([o['dist_diff_raw'] for o in rest_obs])

    reasons.append('V1 adaptive_thresh* share=%.1f%% vs V2-V6=%.1f%% (delta=%+.1fpp)'
                    % (100 * v1_adaptive_share, 100 * rest_adaptive_share,
                       100 * (v1_adaptive_share - rest_adaptive_share)))
    reasons.append('V1 mentah share=%.1f%% vs V2-V6=%.1f%% (delta=%+.1fpp)'
                    % (100 * v1_mentah_share, 100 * rest_mentah_share,
                       100 * (v1_mentah_share - rest_mentah_share)))
    if v1_resid and rest_resid:
        reasons.append('V1 dist_diff_raw mean=%+.3fm (n=%d) vs V2-V6 mean=%+.3fm (n=%d)'
                        % (v1_resid['mean'], v1_resid['n'], rest_resid['mean'], rest_resid['n']))

    adaptive_delta = v1_adaptive_share - rest_adaptive_share
    mentah_delta = rest_mentah_share - v1_mentah_share
    resid_delta = (abs(v1_resid['mean'] - rest_resid['mean']) if v1_resid and rest_resid else 0.0)

    signals = sum([adaptive_delta > 0.10, mentah_delta > 0.10, resid_delta > 0.10])
    if signals == 0:
        return 'NO EVIDENCE OF CANDIDATE-LEVEL CAUSATION', reasons
    if signals >= 1 and n_v1 < 20:
        reasons.append('Directional signal(s) present but n_v1=%d from a single failed run '
                        '-- cannot rule out run-specific pose/lighting confound instead of '
                        'candidate behavior.' % n_v1)
        return 'CANDIDATE CORRELATION OBSERVED — MORE EVIDENCE REQUIRED', reasons
    return 'CANDIDATE CORRELATION OBSERVED — MORE EVIDENCE REQUIRED', reasons


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    data_dir = sys.argv[1]
    tags = sys.argv[2:] or ['V1', 'V2', 'V3', 'V4', 'V5', 'V6']

    print('=' * 78)
    print('P0-2.7 -- candidate behavior vs APPROACH_QR gate outcome (DIAGNOSTIC ONLY)')
    print('No candidate reordering. No detector/FSM/controller changes. No fix.')
    print('Correlational only -- no causal claim.')
    print('=' * 78)

    runs_obs = []
    for tag in tags:
        gate = load_gate(data_dir, tag)
        try:
            rows = load_csv('%s/%s.csv' % (data_dir, tag))
        except FileNotFoundError:
            print('\n[%s] csv missing, skipped' % tag)
            continue
        obs = extract_run_observations(tag, gate, rows)
        runs_obs.append((tag, gate, obs))

    if not runs_obs:
        print('\nNo runs loaded -- nothing to analyze.')
        return

    all_obs = [o for _, _, obs in runs_obs for o in obs]
    v1_obs = [o for tag, _, obs in runs_obs for o in obs if tag == 'V1']
    rest_obs = [o for tag, _, obs in runs_obs for o in obs if tag != 'V1']

    print('\n[MACHINE-READABLE per-observation records: run,gate,candidate,t,decode_success,'
          'dist_diff_raw,distance_est,corner_bearing]')
    for o in all_obs:
        print('  %s,%s,%s,%.3f,%d,%s,%s,1'
              % (o['run'], o['gate'], o['candidate_name'], o['t'], o['decode_success'],
                 ('%.4f' % o['dist_diff_raw']) if o['dist_diff_raw'] is not None else '',
                 ('%.4f' % o['distance_est']) if o['distance_est'] is not None else ''))

    print_view_a(runs_obs)
    print_view_b(v1_obs, rest_obs)
    print_view_c(runs_obs)
    print_view_d(runs_obs)
    print_view_e(v1_obs, rest_obs)

    print('\n' + '=' * 78)
    print('LIMITATIONS')
    print('=' * 78)
    print('- Battery is 6 runs, 1 failure (V1) -- any V1-specific pattern is n=1 at the')
    print('  run level; cannot separate "this candidate causes failure" from "this run\'s')
    print('  spawn pose/lighting happened to favor different candidates."')
    print('- clahe / otsu / otsu_upscaled cells carry very low n (P0-2.6 finding, unchanged')
    print('  here) -- rates/residuals for those three are descriptive only, flagged [LOW-N].')
    print('- AABB inflation (mean ~1.335x, median ~1.293x, P0-2.6) is NOT reopened here; no')
    print('  new evidence in this pass contradicts it.')
    print('- dist_diff_raw uses the same pinhole-geometry estimate as P0-2.3/P0-2.6 -- same')
    print('  known limitation that qr_offset_debug corner data is not filtered by camera')
    print('  frame_id (reduce_qr_precision.py docstring).')

    verdict, reasons = classify(v1_obs, rest_obs)
    next_experiment = (
        'Battery with N>=15 runs specifically oversampling failure-prone spawn poses '
        '(near-frame-edge / high in-plane rotation), instrumented identically, to get '
        'V1-class failures at n>1 before any candidate-reordering or plausibility-gate '
        'candidate (P0-2.6 S4) is escalated from formulated to approved.'
    )

    print('\n' + '=' * 78)
    print('CLASSIFICATION REASONING')
    print('=' * 78)
    for r in reasons:
        print('  - %s' % r)

    print('\nP0-2.7 VERDICT:')
    print(verdict)
    print('\nCODE CHANGES:')
    print('NONE')
    print('\nDETECTOR CHANGES:')
    print('NONE')
    print('\nFSM CHANGES:')
    print('NONE')
    print('\nCONTROLLER CHANGES:')
    print('NONE')
    print('\nNEXT RECOMMENDED EXPERIMENT:')
    print(next_experiment)


if __name__ == '__main__':
    main()
