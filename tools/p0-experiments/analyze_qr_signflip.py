#!/usr/bin/env python3
"""P0-2.3 follow-up: what correlates with the distance_est-h_cam sign-flip?

Read-only analysis over the R1-R6 battery CSVs (docs/P0-2-3-SPEC.md S18) --
no simulator run, no code/detector changes. Reuses the exact per-observation
dedup/inflation logic from reduce_qr_precision.py (imported, not
re-derived) and adds per-observation context (rotation angle, frame
position, distance, decode_success) to look for what distinguishes
sign-flipped observations (dist_diff_raw > 0) from the rest.

Small-n exploratory analysis (42 independent observations total across 6
runs) -- reports group comparisons, not a fitted model. No verdict.

usage: analyze_qr_signflip.py <data_dir> [tag ...]
"""
import csv
import math
import statistics as stats
import sys

sys.path.insert(0, __file__.rsplit('/', 1)[0])
from reduce_qr_precision import (QR_SIDE_M, FX_PX, FY_PX, FRAME_W_PX, FRAME_H_PX,
                                 CORNER_COLS, to_float, is_nan, load_csv,
                                 h_cam_from_depth, quad_metrics)


def rotation_angle_deg(corners):
    pts = [(corners[2 * i], corners[2 * i + 1]) for i in range(4)]
    a = math.degrees(math.atan2(pts[1][1] - pts[0][1], pts[1][0] - pts[0][0])) % 90
    return a


def collect_observations(tag, rows):
    approach = [r for r in rows if r['fsm_state'] == 'APPROACH_QR']
    if not approach or 'qr_decode_success' not in approach[0]:
        return []
    per_row = []
    for r in approach:
        qr_size = to_float(r['qr_size'])
        if is_nan(qr_size) or qr_size <= 0:
            continue
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
        distance_est = (FX_PX * QR_SIDE_M) / (qr_size * FRAME_W_PX)
        h_cam = h_cam_from_depth(-sp_depth)
        dist_diff_raw = distance_est - h_cam
        per_row.append({
            'tag': tag, 't': to_float(r['t']),
            'corner_tuple': tuple(corners),
            'decode_success': decode_success,
            'inflation': inflation,
            'angle_deg': rotation_angle_deg(corners),
            'qr_ex': qr_ex, 'qr_ey': qr_ey, 'qr_size': qr_size,
            'distance_est': distance_est,
            'dist_diff_raw': dist_diff_raw,
            'sign_flip': dist_diff_raw > 0,
        })
    # dedup consecutive identical corner readings -> independent observations
    independent, last = [], None
    for o in per_row:
        if o['corner_tuple'] != last:
            independent.append(o)
            last = o['corner_tuple']
    return independent


def group_compare(obs, key, label):
    flipped = [o[key] for o in obs if o['sign_flip']]
    normal = [o[key] for o in obs if not o['sign_flip']]
    print('  %s: sign-flip group (n=%d) mean=%s | normal group (n=%d) mean=%s'
          % (label, len(flipped), ('%.3f' % stats.fmean(flipped)) if flipped else 'n/a',
             len(normal), ('%.3f' % stats.fmean(normal)) if normal else 'n/a'))


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    data_dir = sys.argv[1]
    tags = sys.argv[2:] or ['R1', 'R2', 'R3', 'R4', 'R5', 'R6']

    print('=' * 78)
    print('P0-2.3 sign-flip correlate analysis -- EXPLORATORY, small-n (n=42 total).')
    print('Group comparisons only, no fitted model, no verdict.')
    print('=' * 78)

    all_obs = []
    for tag in tags:
        try:
            rows = load_csv('%s/%s.csv' % (data_dir, tag))
        except FileNotFoundError:
            continue
        all_obs.extend(collect_observations(tag, rows))

    n_flip = sum(1 for o in all_obs if o['sign_flip'])
    print('\nTotal independent observations: %d  (sign-flip=%d, normal=%d)'
          % (len(all_obs), n_flip, len(all_obs) - n_flip))

    print('\nPer-tag breakdown:')
    for tag in tags:
        tag_obs = [o for o in all_obs if o['tag'] == tag]
        if not tag_obs:
            continue
        tf = sum(1 for o in tag_obs if o['sign_flip'])
        print('  %s: n=%d sign-flip=%d' % (tag, len(tag_obs), tf))

    print('\nGroup comparisons (sign-flip vs normal):')
    group_compare(all_obs, 'inflation', 'AABB inflation factor')
    group_compare(all_obs, 'angle_deg', 'rotation angle (deg, mod 90)')
    group_compare(all_obs, 'qr_ex', 'qr_ex (frame position, lateral)')
    group_compare(all_obs, 'qr_ey', 'qr_ey (frame position, vertical)')
    group_compare(all_obs, 'qr_size', 'qr_size (apparent size fraction)')
    group_compare(all_obs, 'distance_est', 'distance_est (m)')
    group_compare(all_obs, 'decode_success', 'decode_success (0/1)')

    print('\ndecode_success crosstab:')
    for ds in (1.0, 0.0):
        sub = [o for o in all_obs if o['decode_success'] == ds]
        tf = sum(1 for o in sub if o['sign_flip'])
        label = 'decode_success=1' if ds == 1.0 else 'decode_success=0 (corner-only)'
        print('  %s: n=%d, sign-flip=%d (%.0f%%)'
              % (label, len(sub), tf, 100 * tf / len(sub) if sub else 0))

    print('\n(No model fitted, no threshold declared. Small n throughout -- read as')
    print('directional signal only, not a settled causal explanation.)')


if __name__ == '__main__':
    main()
