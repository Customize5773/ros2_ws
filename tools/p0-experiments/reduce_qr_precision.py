#!/usr/bin/env python3
"""P0-2.3 Gate P2/P3 -- QR-as-independent-sensor position estimate accuracy.

Deliberately SEPARATE from reduce_approach_qr.py (P0-2.2) and from gripper_err
(docs/P0-2-3-SPEC.md S1). Those measure "did the controller reach a
ground-truth-anchored target" -- structurally circular for judging QR itself,
since the target IS the ground truth. This script instead reprojects the raw
QR visual signal (qr_ex, qr_ey, qr_size) into an estimated relative position
using pinhole camera geometry and a physically-confirmed QR size, and compares
THAT estimate against ground truth -- independent of whatever the controller
did with the signal.

Confirmed inputs (see docs/P0-2-3-SPEC.md S3, S6.2):
  qr_side_m = 0.12   -- src/hydroships_gazebo/scripts/payload_spawner.py:59-61,
                        PAYLOAD_SDF_TEMPLATE 'qr' visual <size>0.12 0.12</size>.
                        NOT the mission_fsm.py comment -- the actual SDF geometry.
  fx=fy=381.4 px, frame=640x480 -- observed identical in all P0-2.2b run logs
                        ("camera_info camera_bottom_link: fx=381.4 ..."),
                        treated as a known constant for this first pass.

offset_from_points() convention (qr_logic.py:120-134): ex=(cx-w/2)/(w/2),
ey=(cy-h/2)/(h/2), size=max(bw/w,bh/h). qr_ey_target()'s docstring
(mission_fsm.py:64-65) establishes ey>0 == QR behind ROV, ex>0 == QR to the
right -- the same sign convention the visual-servo correction itself uses
(mission_fsm.py:563-564: body_dx=-(ey-ey_target)*k, body_dy=-ex*k). This
script reuses that exact sign convention so the QR-alone estimate and the
system's own body-frame axes agree.

IMPORTANT (user's constraint 1 & 2):
  - payload_pose is SIMULATOR GROUND TRUTH, never available on real hardware.
    It is used here ONLY to score the QR-alone estimate, never as an input to
    the estimate itself.
  - This script reports QR-ESTIMATION accuracy ONLY. It does not compute or
    reference gripper_err/base_err (controller-reaches-target accuracy,
    docs/P0-2-3-SPEC.md S1) anywhere -- the two must never be combined into a
    single number or claim.

P0-2.3 battery extension (active only when a CSV has qr_decode_success/
qr_c1x..qr_c4y -- old Q1-Q6 lack these and are skipped for this part,
everything above still runs on them unchanged): breaks the distance_est-
vs-h_cam residual down by decode_success and by AABB-inflation tertile, to
separate the CONFIRMED rotation-inflation effect (docs/P0-2-3-SPEC.md S16,
measured mean 1.405x) from the still-OPEN corner-only/decode-failure-noise
question and the still-UNEXPLAINED Q4 sign-flip. KNOWN LIMITATION: the
qr_offset_debug corner data is not filtered by camera frame_id the way
mission_fsm's own qr_offset consumption is (mission_fsm.py:408-413 keeps
only camera_bottom_link) -- some corner rows here may originate from the
front camera. Not corrected in this pass; flagged so results aren't
over-read.

usage: reduce_qr_precision.py <data_dir> [tag ...]
  (default tags: Q1 Q2 Q3 Q4 Q5 Q6; reuses EXISTING CSVs, runs no simulator)
"""
import csv
import json
import math
import statistics as stats
import sys

QR_SIDE_M = 0.12          # payload_spawner.py:61 -- confirmed SDF geometry
FX_PX = FY_PX = 381.4     # observed in *.log camera_info lines, all runs
FRAME_W_PX, FRAME_H_PX = 640.0, 480.0

# qr_ey_target() geometry constants (mission_fsm.py:159-171 / params.yaml),
# used only for the free distance_est-vs-h_cam cross-check in S output.
QR_FLOOR_Z = -0.894
CAM_BOTTOM_DZ = 0.18


def to_float(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return float('nan')


def is_nan(v):
    return v != v


def load_csv(path):
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


def h_cam_from_depth(depth):
    return max(0.05, abs(QR_FLOOR_Z) - depth - CAM_BOTTOM_DZ)


CORNER_COLS = ['qr_c1x', 'qr_c1y', 'qr_c2x', 'qr_c2y',
              'qr_c3x', 'qr_c3y', 'qr_c4x', 'qr_c4y']


def quad_metrics(corners):
    """corners: 8 floats x1,y1,...,x4,y4 (original-frame pixel space, same as
    offset_from_points()). Returns (aabb_side, mean_edge, inflation) or None
    if degenerate. inflation = aabb_side/mean_edge -- P0-2.3 S16's measured
    AABB-vs-true-side-length inflation from in-plane QR rotation."""
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


def analyze_run(tag, rows):
    approach = [r for r in rows if r['fsm_state'] == 'APPROACH_QR']
    has_corners = bool(rows) and 'qr_decode_success' in rows[0]
    errs, dist_diffs = [], []
    # per-row corner-linked records: (dist_diff_raw, dist_diff_corrected, inflation,
    # decode_success, corner_tuple) -- only populated when has_corners
    corner_rows = []
    for r in approach:
        qr_size = to_float(r['qr_size'])
        if is_nan(qr_size) or qr_size <= 0:
            continue
        qr_ex, qr_ey = to_float(r['qr_ex']), to_float(r['qr_ey'])
        x, y, yaw = to_float(r['x']), to_float(r['y']), to_float(r['yaw'])
        payload_x, payload_y = to_float(r['payload_x']), to_float(r['payload_y'])
        sp_depth = to_float(r['sp_depth'])
        if is_nan(qr_ex) or is_nan(qr_ey) or is_nan(x) or is_nan(payload_x):
            continue

        # --- QR-alone estimate (no ground truth used here) ---
        distance_est = (FX_PX * QR_SIDE_M) / (qr_size * FRAME_W_PX)
        forward_est = -qr_ey * distance_est * (FRAME_H_PX / 2.0) / FY_PX
        lateral_est = -qr_ex * distance_est * (FRAME_W_PX / 2.0) / FX_PX

        # --- ground truth, rotated into the same body-frame axes _goto_xy uses ---
        ex0, ey0 = payload_x - x, payload_y - y
        c, s = math.cos(yaw), math.sin(yaw)
        forward_true = ex0 * c + ey0 * s
        lateral_true = -ex0 * s + ey0 * c

        err_m = math.hypot(forward_est - forward_true, lateral_est - lateral_true)
        errs.append(err_m)

        h_cam = None
        if not is_nan(sp_depth):
            # sp_depth (/hydroships/setpoint/depth) is published as -abs(depth_target)
            # by _set_depth() (mission_fsm.py:319-320), but qr_ey_target()/h_cam is
            # always called in production with the POSITIVE depth_target
            # (mission_fsm.py:534,540) -- negate here to match that convention.
            h_cam = h_cam_from_depth(-sp_depth)
            dist_diffs.append(distance_est - h_cam)

        if has_corners and h_cam is not None:
            decode_success = to_float(r.get('qr_decode_success'))
            corners = [to_float(r.get(c)) for c in CORNER_COLS]
            if any(is_nan(v) for v in corners) or is_nan(decode_success):
                continue
            qm = quad_metrics(corners)
            if qm is None:
                continue
            _, _, inflation = qm
            dist_diff_raw = distance_est - h_cam
            distance_est_corrected = distance_est * inflation
            dist_diff_corrected = distance_est_corrected - h_cam
            corner_rows.append({
                'dist_diff_raw': dist_diff_raw,
                'dist_diff_corrected': dist_diff_corrected,
                'inflation': inflation,
                'decode_success': decode_success,
                'corner_tuple': tuple(corners),
            })

    result = {'tag': tag, 'n': len(errs)}
    if not errs:
        return result

    result.update({
        'err_m_mean': stats.fmean(errs),
        'err_m_median': stats.median(errs),
        'err_m_min': min(errs),
        'err_m_max': max(errs),
        'err_m_stdev': stats.pstdev(errs) if len(errs) > 1 else 0.0,
        'distance_est_minus_h_cam_mean': stats.fmean(dist_diffs) if dist_diffs else None,
        'distance_est_minus_h_cam_n': len(dist_diffs),
    })

    if corner_rows:
        result['corner_analysis'] = summarize_corner_rows(corner_rows)
    return result


def summarize_corner_rows(corner_rows):
    """P0-2.3 battery extension: dedup to independent detector observations,
    then break down inflation/residual by decode_success and by inflation
    tertile. All derived from already-computed per-row values -- no new
    ground-truth usage beyond what analyze_run already did."""
    # dedup consecutive identical corner readings -> independent observations
    independent = []
    last = None
    for cr in corner_rows:
        if cr['corner_tuple'] != last:
            independent.append(cr)
            last = cr['corner_tuple']

    n_success = sum(1 for cr in independent if cr['decode_success'] == 1.0)
    n_fail = sum(1 for cr in independent if cr['decode_success'] == 0.0)
    inflations = [cr['inflation'] for cr in independent]

    def resid_stats(rows_subset, key):
        vals = [cr[key] for cr in rows_subset]
        if not vals:
            return None
        return {'n': len(vals), 'mean': stats.fmean(vals), 'median': stats.median(vals)}

    succ_rows = [cr for cr in independent if cr['decode_success'] == 1.0]
    fail_rows = [cr for cr in independent if cr['decode_success'] == 0.0]

    tertile_result = None
    if len(independent) >= 3:
        sorted_by_infl = sorted(independent, key=lambda cr: cr['inflation'])
        n = len(sorted_by_infl)
        third = max(1, n // 3)
        low, mid, high = (sorted_by_infl[:third], sorted_by_infl[third:n - third],
                          sorted_by_infl[n - third:])
        tertile_result = {
            'low_inflation': {'n': len(low), 'inflation_mean': stats.fmean([c['inflation'] for c in low]),
                              'dist_diff_raw_mean': stats.fmean([c['dist_diff_raw'] for c in low])},
            'mid_inflation': {'n': len(mid), 'inflation_mean': stats.fmean([c['inflation'] for c in mid]) if mid else None,
                              'dist_diff_raw_mean': stats.fmean([c['dist_diff_raw'] for c in mid]) if mid else None},
            'high_inflation': {'n': len(high), 'inflation_mean': stats.fmean([c['inflation'] for c in high]),
                               'dist_diff_raw_mean': stats.fmean([c['dist_diff_raw'] for c in high])},
        }

    return {
        'n_independent_observations': len(independent),
        'n_decode_success': n_success,
        'n_decode_fail_corner_only': n_fail,
        'inflation_mean': stats.fmean(inflations) if inflations else None,
        'inflation_median': stats.median(inflations) if inflations else None,
        'inflation_min': min(inflations) if inflations else None,
        'inflation_max': max(inflations) if inflations else None,
        'dist_diff_raw_all': resid_stats(independent, 'dist_diff_raw'),
        'dist_diff_corrected_all': resid_stats(independent, 'dist_diff_corrected'),
        'dist_diff_raw_decode_success': resid_stats(succ_rows, 'dist_diff_raw'),
        'dist_diff_raw_decode_fail': resid_stats(fail_rows, 'dist_diff_raw'),
        'by_inflation_tertile': tertile_result,
        'sign_flip_observed': any(cr['dist_diff_raw'] > 0 for cr in independent),
    }


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    data_dir = sys.argv[1]
    tags = sys.argv[2:] or ['Q1', 'Q2', 'Q3', 'Q4', 'Q5', 'Q6']

    print('=' * 78)
    print('P0-2.3 Gate P2/P3 -- QR-ESTIMATE accuracy vs simulator ground truth.')
    print('payload_pose used ONLY to score this estimate, never as an input to it.')
    print('This table is SEPARATE from gripper_err (controller-reaches-target')
    print('accuracy, docs/P0-2-3-SPEC.md S1) -- do not combine the two.')
    print('No new simulator run: reused existing CSVs from the P0-2.2b battery.')
    print('=' * 78)

    results = {}
    for tag in tags:
        try:
            rows = load_csv('%s/%s.csv' % (data_dir, tag))
        except FileNotFoundError:
            print('\n[%s] csv missing, skipped' % tag)
            continue
        r = analyze_run(tag, rows)
        results[tag] = r
        if r['n'] == 0:
            print('\n[%s] no valid APPROACH_QR rows with qr_size>0' % tag)
            continue
        print('\n[%s] n=%d rows | QR-estimate err_m: mean=%.3f median=%.3f '
              'min=%.3f max=%.3f stdev=%.3f'
              % (tag, r['n'], r['err_m_mean'], r['err_m_median'],
                 r['err_m_min'], r['err_m_max'], r['err_m_stdev']))
        if r['distance_est_minus_h_cam_mean'] is not None:
            print('  cross-check distance_est - h_cam: mean=%+.3f m (n=%d) '
                  '(both computed independently; large/consistent offset would '
                  'flag the pinhole/QR-size assumption, not the estimate itself)'
                  % (r['distance_est_minus_h_cam_mean'], r['distance_est_minus_h_cam_n']))
        ca = r.get('corner_analysis')
        if ca:
            print('  [P0-2.3 battery] independent detector observations=%d '
                  '(decode_success=%d, corner-only=%d)'
                  % (ca['n_independent_observations'], ca['n_decode_success'],
                     ca['n_decode_fail_corner_only']))
            print('    AABB inflation factor: mean=%.3f median=%.3f range=[%.3f, %.3f]'
                  % (ca['inflation_mean'], ca['inflation_median'],
                     ca['inflation_min'], ca['inflation_max']))
            raw = ca['dist_diff_raw_all']
            corr = ca['dist_diff_corrected_all']
            print('    distance_est-h_cam RAW: mean=%+.3f (n=%d)   '
                  'INFLATION-CORRECTED: mean=%+.3f (n=%d)'
                  % (raw['mean'], raw['n'], corr['mean'], corr['n']))
            s_ = ca['dist_diff_raw_decode_success']
            f_ = ca['dist_diff_raw_decode_fail']
            print('    residual by decode_success: success=%s  corner-only=%s'
                  % (('mean=%+.3f (n=%d)' % (s_['mean'], s_['n'])) if s_ else 'n/a',
                     ('mean=%+.3f (n=%d)' % (f_['mean'], f_['n'])) if f_ else 'n/a'))
            if ca['by_inflation_tertile']:
                t = ca['by_inflation_tertile']
                print('    residual by inflation tertile: low=%+.3f(n=%d,infl=%.2f)  '
                      'mid=%s  high=%+.3f(n=%d,infl=%.2f)'
                      % (t['low_inflation']['dist_diff_raw_mean'], t['low_inflation']['n'],
                         t['low_inflation']['inflation_mean'],
                         ('%+.3f(n=%d,infl=%.2f)' % (t['mid_inflation']['dist_diff_raw_mean'],
                          t['mid_inflation']['n'], t['mid_inflation']['inflation_mean']))
                         if t['mid_inflation']['n'] else 'n/a',
                         t['high_inflation']['dist_diff_raw_mean'], t['high_inflation']['n'],
                         t['high_inflation']['inflation_mean']))
            print('    sign-flip (positive dist_diff_raw) observed in this run: %s'
                  % ca['sign_flip_observed'])

    valid = {t: r for t, r in results.items() if r.get('n', 0) > 0}
    print('\n' + '=' * 78)
    print('Gate P3: repeatability of QR-estimate accuracy across runs (n_valid=%d)'
          % len(valid))
    print('=' * 78)
    if valid:
        means = [r['err_m_mean'] for r in valid.values()]
        print('  per-run mean err_m: %s' % ', '.join('%s=%.3f' % (t, r['err_m_mean'])
                                                       for t, r in valid.items()))
        print('  across-run spread of mean err_m: min=%.3f max=%.3f median=%.3f'
              % (min(means), stats.median(means), max(means)))

    out_path = '%s/P0-2-3-precision-results.json' % data_dir
    with open(out_path, 'w') as f:
        json.dump(results, f, indent=2, default=str)
    print('\nFull data written to %s' % out_path)
    print('This is QR-estimate accuracy evidence only -- not a PASS/FAIL, and not')
    print('comparable to gripper_err without accounting for what each measures.')


if __name__ == '__main__':
    main()
