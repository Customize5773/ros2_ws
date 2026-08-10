#!/usr/bin/env python3
"""P0-2.3 separation experiment analysis (docs/P0-2-3-SEPARATION-SPEC.md).

Implements the APPROVED, REVISED design exactly:
  - decode_success=1 vs decode_success=0 compared ONLY within inflation bin
    [1.00,1.20) (the only bin with both groups populated in prior data).
  - Higher bins ([1.20,1.50), [1.50+)) used for a corner-only-ONLY dose-response
    check (does residual worsen with inflation), NOT a decode_success comparison.
  - Squareness (edge-length CV) and angle_deviation_90 as secondary geometric
    quality proxies, explicitly caveated as confounded with real off-nadir
    perspective, not pure noise indicators.
  - Outlier handling at the OBSERVATION level: aggregates reported with AND
    without any single extreme observation (e.g. a degenerate quad like the
    confirmed R4 outlier from the R1-R6 battery).
  - Inconclusive criteria applied exactly as specified: <3 per group/bin,
    <15 total decode_success, method disagreement, single-run dominance.

No PASS/FAIL, no P0-2.3 verdict -- evidence only, per the spec's own discipline.

usage: analyze_qr_separation.py <data_dir> [tag ...]
"""
import csv
import json
import math
import statistics as stats
import sys

sys.path.insert(0, __file__.rsplit('/', 1)[0])
from reduce_qr_precision import (QR_SIDE_M, FX_PX, FRAME_W_PX, CORNER_COLS,
                                 to_float, is_nan, load_csv, h_cam_from_depth, quad_metrics)

INFLATION_BINS = [(1.00, 1.20), (1.20, 1.50), (1.50, float('inf'))]
MIN_CELL_N = 3
MIN_TOTAL_DECODE_SUCCESS = 15


def rotation_angle_deg(corners):
    pts = [(corners[2 * i], corners[2 * i + 1]) for i in range(4)]
    return math.degrees(math.atan2(pts[1][1] - pts[0][1], pts[1][0] - pts[0][0])) % 90


def squareness_and_angledev(corners):
    pts = [(corners[2 * i], corners[2 * i + 1]) for i in range(4)]
    edges = [math.hypot(pts[(i + 1) % 4][0] - pts[i][0], pts[(i + 1) % 4][1] - pts[i][1])
             for i in range(4)]
    mean_e = sum(edges) / 4
    squareness = (stats.pstdev(edges) / mean_e) if mean_e > 0 else float('nan')

    def ang(a, b, c):
        v1 = (a[0] - b[0], a[1] - b[1])
        v2 = (c[0] - b[0], c[1] - b[1])
        n1, n2 = math.hypot(*v1), math.hypot(*v2)
        if n1 == 0 or n2 == 0:
            return float('nan')
        cosang = max(-1.0, min(1.0, (v1[0] * v2[0] + v1[1] * v2[1]) / (n1 * n2)))
        return math.degrees(math.acos(cosang))

    angles = [ang(pts[(i - 1) % 4], pts[i], pts[(i + 1) % 4]) for i in range(4)]
    if any(is_nan(a) for a in angles):
        return squareness, float('nan')
    angle_dev = stats.fmean(abs(a - 90.0) for a in angles)
    return squareness, angle_dev


def collect_observations(tag, rows):
    approach = [r for r in rows if r['fsm_state'] == 'APPROACH_QR']
    if not approach or 'qr_decode_success' not in approach[0]:
        return []
    per_row = []
    for r in approach:
        qr_size = to_float(r['qr_size'])
        if is_nan(qr_size) or qr_size <= 0 or qr_size > 1.0:
            continue  # excludes the confirmed-degenerate class of observation up front
        qr_ex, qr_ey = to_float(r['qr_ex']), to_float(r['qr_ey'])
        sp_depth = to_float(r['sp_depth'])
        decode_success = to_float(r.get('qr_decode_success'))
        corners = [to_float(r.get(c)) for c in CORNER_COLS]
        if (is_nan(qr_ex) or is_nan(qr_ey) or is_nan(sp_depth) or is_nan(decode_success)
                or any(is_nan(v) for v in corners)):
            continue
        qm = quad_metrics(corners)
        if qm is None:
            continue
        _, _, inflation = qm
        squareness, angle_dev = squareness_and_angledev(corners)
        distance_est = (FX_PX * QR_SIDE_M) / (qr_size * FRAME_W_PX)
        distance_est_corrected = distance_est * inflation
        h_cam = h_cam_from_depth(-sp_depth)
        per_row.append({
            'tag': tag, 't': to_float(r['t']), 'corner_tuple': tuple(corners),
            'decode_success': decode_success, 'inflation': inflation,
            'squareness': squareness, 'angle_dev': angle_dev,
            'angle_rot': rotation_angle_deg(corners),
            'dist_diff_raw': distance_est - h_cam,
            'dist_diff_corrected': distance_est_corrected - h_cam,
        })
    independent, last = [], None
    for o in per_row:
        if o['corner_tuple'] != last:
            independent.append(o)
            last = o['corner_tuple']
    return independent


def bin_of(inflation):
    for i, (lo, hi) in enumerate(INFLATION_BINS):
        if lo <= inflation < hi:
            return i
    return None


def group_stats(vals):
    if not vals:
        return None
    return {'n': len(vals), 'mean': stats.fmean(vals), 'median': stats.median(vals),
           'stdev': stats.pstdev(vals) if len(vals) > 1 else 0.0}


def pearson(xs, ys):
    pairs = [(a, b) for a, b in zip(xs, ys) if not is_nan(a) and not is_nan(b)]
    if len(pairs) < 3:
        return None, len(pairs)
    xs2, ys2 = zip(*pairs)
    mx, my = stats.fmean(xs2), stats.fmean(ys2)
    cov = sum((a - mx) * (b - my) for a, b in pairs)
    sx = math.sqrt(sum((a - mx) ** 2 for a in xs2))
    sy = math.sqrt(sum((b - my) ** 2 for b in ys2))
    return (cov / (sx * sy) if sx > 0 and sy > 0 else None), len(pairs)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    data_dir = sys.argv[1]
    tags = sys.argv[2:] or ['S1', 'S2', 'S3', 'S4', 'S5', 'S6', 'T1', 'T2', 'T3', 'T4', 'T5', 'T6']

    print('=' * 78)
    print('P0-2.3 SEPARATION ANALYSIS -- per docs/P0-2-3-SEPARATION-SPEC.md (revised).')
    print('Evidence only. No PASS/FAIL, no P0-2.3 verdict. Bin-1-only decode_success')
    print('comparison + corner-only dose-response for higher bins, per the approved design.')
    print('=' * 78)

    all_obs = []
    per_tag_n = {}
    for tag in tags:
        try:
            rows = load_csv('%s/%s.csv' % (data_dir, tag))
        except FileNotFoundError:
            continue
        obs = collect_observations(tag, rows)
        all_obs.extend(obs)
        per_tag_n[tag] = len(obs)

    n_success = sum(1 for o in all_obs if o['decode_success'] == 1.0)
    n_fail = sum(1 for o in all_obs if o['decode_success'] == 0.0)
    print('\nTotal independent observations: %d  (decode_success=1: %d, corner-only: %d)'
          % (len(all_obs), n_success, n_fail))
    print('Per-run: %s' % ', '.join('%s=%d' % (t, n) for t, n in per_tag_n.items()))

    if n_success < MIN_TOTAL_DECODE_SUCCESS:
        print('\n*** INCONCLUSIVE: total decode_success=%d < %d floor (spec S8). ***'
              % (n_success, MIN_TOTAL_DECODE_SUCCESS))
    else:
        print('\nTotal decode_success=%d >= %d floor -- proceeding with stratified analysis.'
              % (n_success, MIN_TOTAL_DECODE_SUCCESS))

    # bin membership
    for o in all_obs:
        o['bin'] = bin_of(o['inflation'])

    print('\n' + '=' * 78)
    print('Bin coverage (spec S1: [1.00,1.20) is the only bin expected to have both groups)')
    print('=' * 78)
    bin_counts = {}
    for i, (lo, hi) in enumerate(INFLATION_BINS):
        in_bin = [o for o in all_obs if o['bin'] == i]
        s = sum(1 for o in in_bin if o['decode_success'] == 1.0)
        f = sum(1 for o in in_bin if o['decode_success'] == 0.0)
        bin_counts[i] = (s, f)
        label = '[%.2f,%s)' % (lo, ('%.2f' % hi) if hi < float('inf') else '+inf')
        print('  bin%d %s: n=%d  decode_success=1: %d   decode_success=0: %d'
              % (i, label, len(in_bin), s, f))

    # === Primary comparison: bin 0 only ===
    print('\n' + '=' * 78)
    print('PRIMARY: decode_success=1 vs 0 WITHIN bin [1.00,1.20) only')
    print('=' * 78)
    bin0 = [o for o in all_obs if o['bin'] == 0]
    succ0 = [o['dist_diff_raw'] for o in bin0 if o['decode_success'] == 1.0]
    fail0 = [o['dist_diff_raw'] for o in bin0 if o['decode_success'] == 0.0]
    succ0_c = [o['dist_diff_corrected'] for o in bin0 if o['decode_success'] == 1.0]
    fail0_c = [o['dist_diff_corrected'] for o in bin0 if o['decode_success'] == 0.0]
    s_stat, f_stat = group_stats(succ0), group_stats(fail0)
    s_stat_c, f_stat_c = group_stats(succ0_c), group_stats(fail0_c)
    if s_stat and f_stat and s_stat['n'] >= MIN_CELL_N and f_stat['n'] >= MIN_CELL_N:
        print('  RAW residual:       decode_success=1: n=%d mean=%+.3f | corner-only: n=%d mean=%+.3f'
              % (s_stat['n'], s_stat['mean'], f_stat['n'], f_stat['mean']))
        print('  CORRECTED residual: decode_success=1: n=%d mean=%+.3f | corner-only: n=%d mean=%+.3f'
              % (s_stat_c['n'], s_stat_c['mean'], f_stat_c['n'], f_stat_c['mean']))
        gap_raw = s_stat['mean'] - f_stat['mean']
        gap_corr = s_stat_c['mean'] - f_stat_c['mean']
        print('  Gap (decode_success - corner-only): RAW=%+.3f  CORRECTED=%+.3f' % (gap_raw, gap_corr))
    else:
        print('  *** INCONCLUSIVE for this bin: n<%d in at least one group ***' % MIN_CELL_N)
        print('  decode_success=1: n=%d, corner-only: n=%d' % (s_stat['n'] if s_stat else 0,
                                                                f_stat['n'] if f_stat else 0))

    # single-run dominance check on bin0 decode_success=1 group
    succ0_tags = [o['tag'] for o in bin0 if o['decode_success'] == 1.0]
    if succ0_tags:
        from collections import Counter
        c = Counter(succ0_tags)
        top_tag, top_n = c.most_common(1)[0]
        print('  decode_success=1 (bin0) run distribution: %s' % dict(c))
        if top_n / len(succ0_tags) > 0.5:
            print('  *** single-run dominance flag: %s supplies %d/%d (%.0f%%) of this group ***'
                  % (top_tag, top_n, len(succ0_tags), 100 * top_n / len(succ0_tags)))
            without = [o['dist_diff_raw'] for o in bin0
                      if o['decode_success'] == 1.0 and o['tag'] != top_tag]
            print('  decode_success=1 mean WITHOUT %s: %s'
                  % (top_tag, ('%+.3f (n=%d)' % (stats.fmean(without), len(without))) if without else 'n/a'))

    # === Secondary: corner-only dose-response across all 3 bins ===
    print('\n' + '=' * 78)
    print('SECONDARY: corner-only (decode_success=0) dose-response across inflation bins')
    print('=' * 78)
    for i, (lo, hi) in enumerate(INFLATION_BINS):
        vals = [o['dist_diff_raw'] for o in all_obs if o['bin'] == i and o['decode_success'] == 0.0]
        st = group_stats(vals)
        label = '[%.2f,%s)' % (lo, ('%.2f' % hi) if hi < float('inf') else '+inf')
        print('  bin%d %s: %s' % (i, label, ('n=%d mean=%+.3f median=%+.3f' %
                                              (st['n'], st['mean'], st['median'])) if st else 'n/a'))

    # === Squareness / angle_deviation_90 vs decode_success and vs residual ===
    print('\n' + '=' * 78)
    print('Squareness / angle_deviation_90 (secondary geometric-quality proxies)')
    print('CAVEAT: confounded with real off-nadir viewing perspective, not pure noise (spec S3)')
    print('=' * 78)
    sq_all = [o['squareness'] for o in all_obs if not is_nan(o['squareness'])]
    ad_all = [o['angle_dev'] for o in all_obs if not is_nan(o['angle_dev'])]
    sq_succ = [o['squareness'] for o in all_obs if o['decode_success'] == 1.0 and not is_nan(o['squareness'])]
    sq_fail = [o['squareness'] for o in all_obs if o['decode_success'] == 0.0 and not is_nan(o['squareness'])]
    ad_succ = [o['angle_dev'] for o in all_obs if o['decode_success'] == 1.0 and not is_nan(o['angle_dev'])]
    ad_fail = [o['angle_dev'] for o in all_obs if o['decode_success'] == 0.0 and not is_nan(o['angle_dev'])]
    print('  squareness: decode_success=1 mean=%.4f (n=%d) | corner-only mean=%.4f (n=%d)'
          % (stats.fmean(sq_succ) if sq_succ else float('nan'), len(sq_succ),
             stats.fmean(sq_fail) if sq_fail else float('nan'), len(sq_fail)))
    print('  angle_dev:  decode_success=1 mean=%.3f (n=%d) | corner-only mean=%.3f (n=%d)'
          % (stats.fmean(ad_succ) if ad_succ else float('nan'), len(ad_succ),
             stats.fmean(ad_fail) if ad_fail else float('nan'), len(ad_fail)))

    r_sq, n_sq = pearson([o['squareness'] for o in all_obs], [o['dist_diff_raw'] for o in all_obs])
    r_ad, n_ad = pearson([o['angle_dev'] for o in all_obs], [o['dist_diff_raw'] for o in all_obs])
    r_ds, n_ds = pearson([o['decode_success'] for o in all_obs], [o['dist_diff_raw'] for o in all_obs])
    print('\n  Correlate strength vs dist_diff_raw (whole dataset, non-tautological check):')
    print('    r(residual, squareness)     = %s (n=%d)' % (('%+.3f' % r_sq) if r_sq is not None else 'n/a', n_sq))
    print('    r(residual, angle_dev)      = %s (n=%d)' % (('%+.3f' % r_ad) if r_ad is not None else 'n/a', n_ad))
    print('    r(residual, decode_success) = %s (n=%d)' % (('%+.3f' % r_ds) if r_ds is not None else 'n/a', n_ds))

    # observation-level outlier check (extreme squareness/angle_dev)
    if sq_all:
        sq_sorted = sorted(all_obs, key=lambda o: (o['squareness'] if not is_nan(o['squareness']) else -1), reverse=True)
        top = sq_sorted[0]
        print('\n  Most extreme squareness observation: tag=%s t=%.3f squareness=%.4f decode_success=%d'
              % (top['tag'], top['t'], top['squareness'], int(top['decode_success'])))
        without_top = [o['dist_diff_raw'] for o in all_obs if o is not top]
        with_top = [o['dist_diff_raw'] for o in all_obs]
        print('  overall mean dist_diff_raw WITH this observation: %+.3f | WITHOUT: %+.3f'
              % (stats.fmean(with_top), stats.fmean(without_top)))

    # write full data
    out = {
        'n_total': len(all_obs), 'n_decode_success': n_success, 'n_corner_only': n_fail,
        'bin_coverage': {str(i): {'decode_success': s, 'corner_only': f} for i, (s, f) in bin_counts.items()},
        'observations': [{k: v for k, v in o.items() if k != 'corner_tuple'} for o in all_obs],
    }
    out_path = '%s/P0-2-3-separation-results.json' % data_dir
    with open(out_path, 'w') as f:
        json.dump(out, f, indent=2, default=str)
    print('\nFull data written to %s' % out_path)
    print('This is evidence only -- no PASS/FAIL, no P0-2.3 verdict.')


if __name__ == '__main__':
    main()
