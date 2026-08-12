#!/usr/bin/env python3
"""P0-2.6 -- winning-candidate distribution for the qr_logic.py preprocessing
pipeline (_candidates(), qr_logic.py:57-90). DIAGNOSTIC ONLY: reads existing
battery CSVs (qr_candidate_idx column, added P0-2.6, recorder_qr.py:64,88,140-142)
and reports distribution/rate/residual breakdowns. Does NOT reorder candidates,
does NOT touch qr_detector.py/qr_logic.py/mission_fsm.py, and reaches no
acceptance verdict -- see docs/P0-2-6-DIAGNOSTIC.md S4.1.

CORNER_COLS/quad_metrics()/distance_est geometry reused verbatim from
reduce_qr_precision.py so the AABB-inflation number here means the same thing
it does in P0-2.3/P0-2.4/P0-2.6.

usage: analyze_qr_candidates.py <data_dir> [tag ...]
"""
import csv
import statistics as stats
import sys

CANDIDATE_NAMES = [
    'mentah', 'clahe', 'adaptive_thresh', 'otsu',
    'adaptive_thresh_upscaled', 'otsu_upscaled', 'adaptive_thresh_denoised',
]  # qr_logic.py:_candidates() order AFTER P0-2.8 reorder (adaptive_thresh_denoised
   # demoted to last). CSVs from BEFORE P0-2.8 (P0-2.6/P0-2.7 batteries) used the
   # old order (denoised at index 3, otsu/upscaled shifted one earlier) -- do not
   # re-run this script against pre-P0-2.8 CSVs with this list, indices won't match.

CORNER_COLS = ['qr_c1x', 'qr_c1y', 'qr_c2x', 'qr_c2y',
               'qr_c3x', 'qr_c3y', 'qr_c4x', 'qr_c4y']

QR_SIDE_M = 0.12
FX_PX = FY_PX = 381.4
FRAME_W_PX, FRAME_H_PX = 640.0, 480.0
QR_FLOOR_Z = -0.894
CAM_BOTTOM_DZ = 0.18


def to_float(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return float('nan')


def is_nan(v):
    return v != v


def h_cam_from_depth(depth):
    return max(0.05, abs(QR_FLOOR_Z) - depth - CAM_BOTTOM_DZ)


def quad_metrics(corners):
    """Same as reduce_qr_precision.py:94-109 -- AABB-side / mean-edge inflation."""
    import math
    pts = [(corners[2 * i], corners[2 * i + 1]) for i in range(4)]
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    bw, bh = max(xs) - min(xs), max(ys) - min(ys)
    edges = [math.hypot(pts[(i + 1) % 4][0] - pts[i][0], pts[(i + 1) % 4][1] - pts[i][1])
             for i in range(4)]
    mean_edge = sum(edges) / 4
    if mean_edge <= 0:
        return None
    aabb_side = max(bw, bh)
    return aabb_side, mean_edge, aabb_side / mean_edge


def load_csv(path):
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


def candidate_name(idx):
    if idx is None or idx < 0 or idx >= len(CANDIDATE_NAMES):
        return 'unknown(%s)' % idx
    return CANDIDATE_NAMES[idx]


def extract_observations(tag, rows):
    """One record per corner-bearing APPROACH_QR row with a valid
    qr_candidate_idx, deduped on consecutive identical corner_tuple to
    independent detector observations (same dedup rule as
    reduce_qr_precision.py:summarize_corner_rows)."""
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
        qr_size = to_float(r.get('qr_size'))
        qr_ey = to_float(r.get('qr_ey'))
        sp_depth = to_float(r.get('sp_depth'))
        if not is_nan(qr_size) and qr_size > 0 and not is_nan(qr_ey) and not is_nan(sp_depth):
            distance_est = (FX_PX * QR_SIDE_M) / (qr_size * FRAME_W_PX)
            h_cam = h_cam_from_depth(-sp_depth)
            dist_diff_raw = distance_est - h_cam

        obs.append({
            'tag': tag,
            'candidate_idx': int(cand),
            'candidate_name': candidate_name(int(cand)),
            'decode_success': int(decode_success),
            'dist_diff_raw': dist_diff_raw,
            'inflation': inflation,
            'corners': corner_tuple,
        })
    return obs


def resid_stats(vals):
    vals = [v for v in vals if v is not None]
    if not vals:
        return None
    return {'n': len(vals), 'mean': stats.fmean(vals), 'median': stats.median(vals)}


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    data_dir = sys.argv[1]
    tags = sys.argv[2:] or ['V1', 'V2', 'V3', 'V4', 'V5', 'V6']

    print('=' * 78)
    print('P0-2.6 -- winning/first-pts candidate distribution (DIAGNOSTIC ONLY,')
    print('no candidate reordering, no detector/FSM/controller changes).')
    print('=' * 78)

    all_obs = []
    for tag in tags:
        try:
            rows = load_csv('%s/%s.csv' % (data_dir, tag))
        except FileNotFoundError:
            print('\n[%s] csv missing, skipped' % tag)
            continue
        obs = extract_observations(tag, rows)
        print('\n[%s] %d independent corner-bearing observations with qr_candidate_idx'
              % (tag, len(obs)))
        all_obs.extend(obs)

    if not all_obs:
        print('\nNo observations with qr_candidate_idx found -- battery CSVs may '
              'predate the P0-2.6 instrumentation, or all rows failed the corner/'
              'decode_success/candidate_idx validity checks above.')
        return

    n_total = len(all_obs)
    succ = [o for o in all_obs if o['decode_success'] == 1]
    fail = [o for o in all_obs if o['decode_success'] == 0]
    print('\n' + '=' * 78)
    print('TOTAL: n=%d independent observations (decode_success=%d, corner-only=%d)'
          % (n_total, len(succ), len(fail)))

    print('\n-- 1. candidate distribution: decode_success=1 vs decode_success=0 --')
    for idx, name in enumerate(CANDIDATE_NAMES):
        n_s = sum(1 for o in succ if o['candidate_idx'] == idx)
        n_f = sum(1 for o in fail if o['candidate_idx'] == idx)
        pct_s = 100.0 * n_s / len(succ) if succ else 0.0
        pct_f = 100.0 * n_f / len(fail) if fail else 0.0
        print('  [%d] %-26s success: n=%-4d (%5.1f%% of successes)   '
              'corner-only: n=%-4d (%5.1f%% of corner-only)'
              % (idx, name, n_s, pct_s, n_f, pct_f))

    print('\n-- 2. per-candidate decode-success rate --')
    for idx, name in enumerate(CANDIDATE_NAMES):
        cand_obs = [o for o in all_obs if o['candidate_idx'] == idx]
        if not cand_obs:
            print('  [%d] %-26s n=0' % (idx, name))
            continue
        n_s = sum(1 for o in cand_obs if o['decode_success'] == 1)
        print('  [%d] %-26s n=%-4d decode-success rate=%.1f%% (%d/%d)'
              % (idx, name, len(cand_obs), 100.0 * n_s / len(cand_obs), n_s, len(cand_obs)))

    print('\n-- 3. per-candidate corner-only residual (dist_diff_raw, decode_success=0) --')
    for idx, name in enumerate(CANDIDATE_NAMES):
        cand_fail = [o for o in fail if o['candidate_idx'] == idx]
        rs = resid_stats([o['dist_diff_raw'] for o in cand_fail])
        if rs:
            print('  [%d] %-26s n=%-4d mean=%+.3fm median=%+.3fm'
                  % (idx, name, rs['n'], rs['mean'], rs['median']))
        else:
            print('  [%d] %-26s n=0 (no valid dist_diff_raw)' % (idx, name))

    print('\n-- 4. corner-only concentration by candidate (share of corner-only vs '
          'share of decode-success) --')
    for idx, name in enumerate(CANDIDATE_NAMES):
        n_s = sum(1 for o in succ if o['candidate_idx'] == idx)
        n_f = sum(1 for o in fail if o['candidate_idx'] == idx)
        share_s = 100.0 * n_s / len(succ) if succ else 0.0
        share_f = 100.0 * n_f / len(fail) if fail else 0.0
        delta = share_f - share_s
        flag = '  <-- disproportionate corner-only source' if delta > 15.0 else ''
        print('  [%d] %-26s corner-only share=%5.1f%%  success share=%5.1f%%  delta=%+5.1f%%%s'
              % (idx, name, share_f, share_s, delta, flag))

    print('\n-- 5. do successful decodes come from a different candidate than '
          'corner-only detections? (mode / distribution shape) --')
    if succ:
        succ_counts = {idx: sum(1 for o in succ if o['candidate_idx'] == idx)
                       for idx in range(len(CANDIDATE_NAMES))}
        mode_s = max(succ_counts, key=succ_counts.get)
        print('  decode-success mode candidate: [%d] %s (%d/%d = %.1f%%)'
              % (mode_s, CANDIDATE_NAMES[mode_s], succ_counts[mode_s], len(succ),
                 100.0 * succ_counts[mode_s] / len(succ)))
    else:
        print('  no decode-success observations in this battery')
    if fail:
        fail_counts = {idx: sum(1 for o in fail if o['candidate_idx'] == idx)
                       for idx in range(len(CANDIDATE_NAMES))}
        mode_f = max(fail_counts, key=fail_counts.get)
        print('  corner-only mode candidate:    [%d] %s (%d/%d = %.1f%%)'
              % (mode_f, CANDIDATE_NAMES[mode_f], fail_counts[mode_f], len(fail),
                 100.0 * fail_counts[mode_f] / len(fail)))
    else:
        print('  no corner-only observations in this battery')

    print('\n-- overall AABB inflation (all independent observations) --')
    infl = [o['inflation'] for o in all_obs]
    print('  mean=%.3f median=%.3f range=[%.3f, %.3f] n=%d'
          % (stats.fmean(infl), stats.median(infl), min(infl), max(infl), len(infl)))

    print('\n' + '=' * 78)
    print('P0-2.6 candidate-distribution pass: evidence only, no candidate '
          'reordering, no detector/FSM/controller changes, no acceptance verdict.')
    print('=' * 78)


if __name__ == '__main__':
    main()
