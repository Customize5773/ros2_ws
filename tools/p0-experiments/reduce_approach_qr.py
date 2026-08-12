#!/usr/bin/env python3
"""P0-2.2b/P0-2.4 reducer — auditable evidence for the docs/P0-2-2-SPEC.md gates
plus the docs/P0-2-4-SPEC.md Gate 4 retest (convergence/oscillation/divergence).

Reads recorder_qr.py CSVs (+ <tag>.gate.txt, <tag>.rec.log, <tag>.log,
<tag>.params.yaml) produced by run_approach_qr_battery.sh and computes six
numeric evidence blocks per run, plus an aggregate. Prints NO APPROACH_QR
PASS/FAIL: a GRAB transition is not itself evidence QR worked -- that is the
hypothesis under test, not a conclusion. Every number here must be traceable
back to a CSV/log column, not eyeballed.

The Gate 4 block additionally implements docs/P0-2-4-SPEC.md S3: dwell-based
time-to-converge, overshoot, command variance/saturation, and an explicit
divergence flag -- all derived from the same recorder_qr.py columns, no new
instrumentation. The aggregate section prints a Gate 4 PASS/FAIL/INCONCLUSIVE
verdict per docs/P0-2-4-SPEC.md S7 (still evidence for review, not a license
to change qr_detector.py/qr_logic.py/mission_fsm.py/controller/parameters).

P1-0 Fase 0 adds a vision-attribution breakdown. `entered_band_with_dwell` --
the number the Gate 4 verdict is computed from -- is a logical OR of a camera
criterion and a pure-odometry criterion (dist to the simulator's ground-truth
payload_pose). It therefore CANNOT answer "did the camera contribute?", and
must not be quoted as perception evidence on its own. The new block reports the
same dwell test over three disjoint criteria (camera / decoded-QR /
ground-truth) so both populations are visible with their own n. No pre-existing
number is altered.

usage: reduce_approach_qr.py <data_dir> [tag ...]
       reduce_approach_qr.py --selftest     # runs the P1-0 attribution checks
  (default tags: Q1 Q2 Q3 Q4 Q5 Q6)
"""
import csv
import json
import math
import re
import statistics as stats
import sys

GRAB_VISUAL_RE = re.compile(r'QR terpusat \(visual servo\) -> GRAB')
GRAB_XYTOL_RE = re.compile(r'QR terpusat \(jarak XY\) -> GRAB')
GRAB_FALLBACK_RE = re.compile(r'Wall \S+ dipilih \(\+15\) \[urutan ke-\d+\]')
TIMEOUT_RE = re.compile(r'APPROACH_QR timeout')

# Runtime-param fallback if <tag>.params.yaml is missing/unreadable -- source
# defaults from mission_fsm.py (docs/P0-2-AUDIT.md S1.4/1.5). Prefer the
# dumped runtime values whenever available (P0-1 rule: verify runtime, not
# just source).
DEFAULT_PARAMS = {
    'approach_kp': 90.0, 'approach_kd': 140.0, 'approach_fmax': 16.0,
    'approach_tol': 0.06, 'qr_servo_gain': 0.15, 'qr_servo_sign': 1.0,
    'qr_center_tol': 0.12, 'qr_max_age': 1.5, 'gripper_base_dx': 0.18,
    'cam_gripper_dx': 0.16, 'qr_floor_z': -0.894, 'cam_bottom_dz': 0.18,
    'cam_vfov_half_tan': 0.6293, 'ey_target_max': 0.8, 'scan_depth': 0.30,
    'qr_offset_ema_alpha': 1.0,  # P0-2.5 Kandidat #2 -- 1.0 = filter nonaktif (default)
    'qr_servo_range': 0.3,       # P0-2.5 Kandidat #1 -- 0.3 = nilai lama/default
    'approach_min_fmax_frac': 0.05,  # P0-2.5 Kandidat #3 -- 0.05 = nilai lama/default
}

# docs/P0-2-4-SPEC.md S3.1/S3.3 -- design-time constants, not tuned from data.
DWELL_TICKS = 3          # S3.1: min consecutive 10Hz ticks in-band to count as "converged"
SATURATION_FRAC_DIVERGED = 0.8  # S3.3: fraction of ticks at |cmd|>=fmax to flag divergence
DIVERGED_TREND_MIN_LEN = 6      # trend() itself already requires len>=6 (thirds of >=2)


def first_dwell(flags, t_l, dwell):
    """First index where `flags` is True for `dwell` consecutive ticks.

    Returns (index, seconds_since_window_start) or (None, None). Extracted in
    P1-0 Fase 0 because the same dwell scan now runs over four different
    criteria series (combined / camera / decoded / ground-truth) -- one scan,
    four inputs, so the populations cannot drift apart by copy-paste.
    """
    for i in range(len(flags) - dwell + 1):
        if all(flags[i:i + dwell]):
            return i, t_l[i] - t_l[0]
    return None, None


def attribute_convergence(cam_idx, gt_idx):
    """Who got the ROV into the band first: the camera or ground-truth homing?

    VISION / GROUND_TRUTH / NONE are unambiguous. When both criteria qualify,
    the EARLIER dwell entry is credited -- a tie goes to GROUND_TRUTH, because
    ground-truth homing runs unconditionally and is therefore the null
    hypothesis the camera has to beat, not the other way round.
    """
    if cam_idx is None and gt_idx is None:
        return 'NONE'
    if gt_idx is None:
        return 'VISION'
    if cam_idx is None:
        return 'GROUND_TRUTH'
    return 'VISION' if cam_idx < gt_idx else 'GROUND_TRUTH'


def qr_ey_target(depth, cam_gripper_dx, qr_floor_z, cam_bottom_dz, vfov_half_tan, ey_max):
    """Exact copy of mission_fsm.py:55-78 (qr_ey_target). Keep in sync with
    the source if that function ever changes -- duplicated deliberately so
    this reducer stays a self-contained, portable script."""
    h_cam = max(0.05, abs(qr_floor_z) - depth - cam_bottom_dz)
    half_h = max(1e-3, h_cam * vfov_half_tan)
    ey = -cam_gripper_dx / half_h
    return max(-ey_max, min(ey_max, ey))


def goto_xy_predict(tx, ty, x, y, yaw, vx, vy, kp, kd, fmax, min_fmax_frac=0.05):
    """Exact copy of mission_fsm.py:389-411 (_goto_xy), minus the
    _set_surge()/self mutation -- returns predicted (fx, fy). min_fmax_frac
    default matches the hardcoded fallback in _goto_xy() -- P0-2.5 Candidate #3
    passes params['approach_min_fmax_frac'] explicitly (see analyze_run)."""
    ex, ey = tx - x, ty - y
    dist = math.hypot(ex, ey)
    slow_radius = 1.0
    fm = fmax
    if dist < slow_radius:
        fm = fm * max(min_fmax_frac, dist / slow_radius)
    c, s = math.cos(yaw), math.sin(yaw)
    bx = ex * c + ey * s
    by = -ex * s + ey * c
    surge = kp * bx - kd * vx
    sway = kp * by - kd * vy
    clamp = lambda v: max(-fm, min(fm, v))
    return clamp(surge), clamp(sway)


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


def load_params(path):
    params = dict(DEFAULT_PARAMS)
    used_defaults = set(DEFAULT_PARAMS)
    try:
        with open(path) as f:
            for line in f:
                m = re.match(r'^ {4}(\w+): (.+)$', line.rstrip('\n'))
                if not m:
                    continue
                k, v = m.group(1), m.group(2).strip().strip("'\"")
                if k in DEFAULT_PARAMS:
                    try:
                        params[k] = float(v)
                        used_defaults.discard(k)
                    except ValueError:
                        pass
    except FileNotFoundError:
        pass
    return params, used_defaults


def pearson(xs, ys):
    pairs = [(a, b) for a, b in zip(xs, ys) if not is_nan(a) and not is_nan(b)]
    if len(pairs) < 3:
        return None, len(pairs)
    xs2, ys2 = zip(*pairs)
    try:
        mx, my = stats.fmean(xs2), stats.fmean(ys2)
        cov = sum((a - mx) * (b - my) for a, b in pairs)
        sx = math.sqrt(sum((a - mx) ** 2 for a in xs2))
        sy = math.sqrt(sum((b - my) ** 2 for b in ys2))
        if sx == 0 or sy == 0:
            return None, len(pairs)
        return cov / (sx * sy), len(pairs)
    except Exception:
        return None, len(pairs)


def rmse(actual, predicted):
    pairs = [(a, p) for a, p in zip(actual, predicted) if not is_nan(a[0]) and not is_nan(p[0])]
    if not pairs:
        return None, 0
    sq = [((a[0] - p[0]) ** 2 + (a[1] - p[1]) ** 2) for a, p in pairs]
    return math.sqrt(stats.fmean(sq)), len(pairs)


def quality_gate(tag, data_dir):
    reasons = []
    try:
        gate = open('%s/%s.gate.txt' % (data_dir, tag)).read().strip()
    except FileNotFoundError:
        gate = None
        reasons.append('gate.txt missing')
    if gate != 'PASS':
        reasons.append('contamination gate=%s' % gate)
    try:
        rec_text = open('%s/%s.rec.log' % (data_dir, tag)).read()
    except FileNotFoundError:
        rec_text = ''
        reasons.append('rec.log missing')
    if 'RECORDING COMPLETE' not in rec_text:
        reasons.append('recorder did not report RECORDING COMPLETE')
    try:
        rows = load_csv('%s/%s.csv' % (data_dir, tag))
    except FileNotFoundError:
        return None, reasons + ['csv missing']
    if not rows:
        return rows, reasons + ['csv empty']
    first_states = [r['fsm_state'] for r in rows[:10] if r['fsm_state']]
    if first_states and first_states[0] not in ('IDLE', 'DIVE'):
        reasons.append('recorder observed fsm_state=%s in its first rows '
                        '(rule 1: recorder must be running before APPROACH_QR entry)'
                        % first_states[0])
    if all(is_nan(to_float(r['payload_x'])) for r in rows):
        reasons.append('payload_pose never populated (ground truth missing)')
    return rows, reasons


def classify_exit(log_text):
    counts = {
        'QR_SCORED_VISUAL_SERVO': len(GRAB_VISUAL_RE.findall(log_text)),
        'QR_SCORED_XY_TOL': len(GRAB_XYTOL_RE.findall(log_text)),
        'GROUND_TRUTH_FALLBACK': len(GRAB_FALLBACK_RE.findall(log_text)),
        'TIMEOUT': len(TIMEOUT_RE.findall(log_text)),
    }
    order = ['QR_SCORED_VISUAL_SERVO', 'QR_SCORED_XY_TOL', 'GROUND_TRUTH_FALLBACK', 'TIMEOUT']
    first = None
    for pat_name, pat in [('QR_SCORED_VISUAL_SERVO', GRAB_VISUAL_RE),
                          ('QR_SCORED_XY_TOL', GRAB_XYTOL_RE),
                          ('GROUND_TRUTH_FALLBACK', GRAB_FALLBACK_RE),
                          ('TIMEOUT', TIMEOUT_RE)]:
        m = pat.search(log_text)
        if m and (first is None or m.start() < first[1]):
            first = (pat_name, m.start())
    primary = first[0] if first else 'NO_GRAB_UNKNOWN'
    return primary, counts


def analyze_run(tag, data_dir, rows, params, used_defaults):
    approach = [r for r in rows if r['fsm_state'] == 'APPROACH_QR']
    if not approach:
        return {'tag': tag, 'reached_approach_qr': False}

    # locked_yaw approximation: yaw at first APPROACH_QR row (heading held
    # constant in this state, mission_fsm.py:518) -- the one methodological
    # assumption in this whole analysis, everything else uses recorded values.
    locked_yaw = to_float(approach[0]['yaw'])
    c0, s0 = math.cos(locked_yaw), math.sin(locked_yaw)

    payload_x = to_float(approach[0]['payload_x'])
    payload_y = to_float(approach[0]['payload_y'])
    tx0 = payload_x - params['gripper_base_dx'] * c0
    ty0 = payload_y - params['gripper_base_dx'] * s0

    qr_ex_l, qr_ey_l, qr_size_l, bx0_l, by0_l, dist0_l = [], [], [], [], [], []
    actual_cmd, pred_without, pred_with, servoing_mask = [], [], [], []
    ey_target_l = []
    t_l, dist_l, off_fresh_l, decode_l = [], [], [], []  # P0-2.4 S3: dist_l uses the (possibly
    # servo-shifted) target tx,ty -- the same value mission_fsm.py:569 checks
    # against approach_tol, not the unshifted tx0,ty0 used for dist0_l above.
    tx_l, ty_l = [], []  # P0-2.5 Candidate #2 guardrail: reconstructed servo
    # target per tick, used to measure tick-to-tick jitter independent of the
    # approach trend (see tick_jitter_stdev below).
    qr_ex_filt, qr_ey_filt = None, None  # P0-2.5 Candidate #2: EMA state, reset
    # per run (mirrors mission_fsm.py's reset on APPROACH_QR entry, _to()).

    for r in approach:
        x, y, yaw = to_float(r['x']), to_float(r['y']), to_float(r['yaw'])
        vx, vy = to_float(r['vx']), to_float(r['vy'])
        qr_ex, qr_ey, qr_size = to_float(r['qr_ex']), to_float(r['qr_ey']), to_float(r['qr_size'])
        qr_age = to_float(r['qr_age'])
        sp_depth = to_float(r['sp_depth'])
        depth_target = sp_depth if not is_nan(sp_depth) else params['scan_depth']

        ey_target = qr_ey_target(depth_target, params['cam_gripper_dx'], params['qr_floor_z'],
                                 params['cam_bottom_dz'], params['cam_vfov_half_tan'],
                                 params['ey_target_max'])
        ey_target_l.append(ey_target)

        ex0, eyy0 = tx0 - x, ty0 - y
        bx0 = ex0 * c0 + eyy0 * s0
        by0 = -ex0 * s0 + eyy0 * c0
        bx0_l.append(bx0); by0_l.append(by0); dist0_l.append(math.hypot(ex0, eyy0))
        qr_ex_l.append(qr_ex); qr_ey_l.append(qr_ey); qr_size_l.append(qr_size)

        dist_raw = math.hypot(x - tx0, y - ty0)
        off_fresh = (not is_nan(qr_age)) and qr_age < params['qr_max_age'] and not is_nan(qr_ex)

        # P0-2.5 Candidate #2: same EMA update as mission_fsm.py -- runs on
        # every off_fresh tick, NOT gated by dist_raw<0.3 (that gate is
        # `servoing` below, unchanged). alpha=1.0 (default) reproduces the
        # pre-Candidate-#2 raw-passthrough behavior exactly.
        if off_fresh:
            if qr_ex_filt is None:
                qr_ex_filt, qr_ey_filt = qr_ex, qr_ey
            else:
                a = params['qr_offset_ema_alpha']
                qr_ex_filt = a * qr_ex + (1.0 - a) * qr_ex_filt
                qr_ey_filt = a * qr_ey + (1.0 - a) * qr_ey_filt

        servoing = off_fresh and dist_raw < params['qr_servo_range']
        servoing_mask.append(servoing)

        tx, ty = tx0, ty0
        if servoing:
            k = params['qr_servo_gain'] * params['qr_servo_sign']
            body_dx = -(qr_ey_filt - ey_target) * k
            body_dy = -qr_ex_filt * k
            tx += body_dx * c0 - body_dy * s0
            ty += body_dx * s0 + body_dy * c0
        tx_l.append(tx); ty_l.append(ty)

        fx_w, fy_w = goto_xy_predict(tx0, ty0, x, y, yaw, vx, vy,
                                      params['approach_kp'], params['approach_kd'],
                                      params['approach_fmax'], params['approach_min_fmax_frac'])
        fx_q, fy_q = goto_xy_predict(tx, ty, x, y, yaw, vx, vy,
                                      params['approach_kp'], params['approach_kd'],
                                      params['approach_fmax'], params['approach_min_fmax_frac'])
        pred_without.append((fx_w, fy_w))
        pred_with.append((fx_q, fy_q))
        actual_cmd.append((to_float(r['cmd_fx']), to_float(r['cmd_fy'])))

        t_l.append(to_float(r['t']))
        dist_l.append(math.hypot(x - tx, y - ty))
        off_fresh_l.append(off_fresh)
        # P1-0 Fase 0: per-tick decode flag, needed to separate "camera produced
        # an offset" (corner points survive a failed decode -- qr_logic.py
        # best_pts) from "the QR was actually read". Recorded by recorder_qr.py
        # since P0-2.3; previously only aggregated into qr_decode_rate.
        decode_l.append(to_float(r['qr_decode_success']) == 1.0)

    # Gate 2
    r_size_dist, n_size = pearson(qr_size_l, dist0_l)
    r_ex_lat, n_ex = pearson(qr_ex_l, by0_l)
    r_ey_fwd, n_ey = pearson(qr_ey_l, bx0_l)

    # Gate 3
    rmse_all_without, n_all = rmse(actual_cmd, pred_without)
    rmse_all_with, _ = rmse(actual_cmd, pred_with)
    serv_actual = [a for a, m in zip(actual_cmd, servoing_mask) if m]
    serv_without = [p for p, m in zip(pred_without, servoing_mask) if m]
    serv_with = [p for p, m in zip(pred_with, servoing_mask) if m]
    rmse_serv_without, n_serv = rmse(serv_actual, serv_without)
    rmse_serv_with, _ = rmse(serv_actual, serv_with)

    # Gate 4
    err_ex = [abs(v) for v in qr_ex_l if not is_nan(v)]
    err_ey = [abs(qr_ey_l[i] - ey_target_l[i]) for i in range(len(qr_ey_l)) if not is_nan(qr_ey_l[i])]

    def trend(series):
        if len(series) < 6:
            return None
        n = len(series) // 3
        first_mean = stats.fmean(series[:n])
        last_mean = stats.fmean(series[-n:])
        return {'first_third_mean': first_mean, 'last_third_mean': last_mean,
                'net_decrease': last_mean < first_mean}

    def sign_changes(series):
        signs = [1 if v > 0 else (-1 if v < 0 else 0) for v in series if v == v]
        return sum(1 for i in range(1, len(signs)) if signs[i] and signs[i - 1] and signs[i] != signs[i - 1])

    entered_band = any(
        abs(qr_ex_l[i]) < params['qr_center_tol'] and abs(qr_ey_l[i] - ey_target_l[i]) < params['qr_center_tol']
        for i in range(len(qr_ex_l)) if not is_nan(qr_ex_l[i]) and not is_nan(qr_ey_l[i])
    )

    # docs/P0-2-4-SPEC.md S3 -- Gate 4 retest: dwell-based time-to-converge,
    # residual trajectory summary, oscillation/divergence. Uses the same
    # centered/dist<approach_tol condition mission_fsm.py:593-596 checks per
    # tick, not just "ever touched the band once" (that's the pre-existing
    # entered_band above, kept unchanged for P0-2.2b comparability).
    n_rows = len(approach)
    # P1-0 Fase 0: the three criteria are now built SEPARATELY and only then
    # OR-ed into `combined_entered`. The combined series is what P0-2.4/2.5
    # already reported; keeping it byte-identical preserves comparability with
    # docs/P0-2-4-RESULTS.md, while the separate series make it visible how
    # much of that number the camera actually earned.
    camera_entered = []    # centering from a camera-derived offset (decode OR corner-only)
    decoded_entered = []   # same, but restricted to ticks where the QR really decoded
    gt_entered = []        # pure odometry vs ground-truth payload_pose -- no camera at all
    for i in range(n_rows):
        centered_i = (off_fresh_l[i] and not is_nan(qr_ex_l[i]) and not is_nan(qr_ey_l[i])
                      and abs(qr_ex_l[i]) < params['qr_center_tol']
                      and abs(qr_ey_l[i] - ey_target_l[i]) < params['qr_center_tol'])
        camera_entered.append(centered_i)
        decoded_entered.append(centered_i and decode_l[i])
        gt_entered.append(dist_l[i] < params['approach_tol'])
    combined_entered = [camera_entered[i] or gt_entered[i] for i in range(n_rows)]

    _, t_conv_s = first_dwell(combined_entered, t_l, DWELL_TICKS)
    entered_band_with_dwell = t_conv_s is not None
    cam_idx, t_conv_camera_s = first_dwell(camera_entered, t_l, DWELL_TICKS)
    dec_idx, t_conv_decoded_s = first_dwell(decoded_entered, t_l, DWELL_TICKS)
    gt_idx, t_conv_gt_s = first_dwell(gt_entered, t_l, DWELL_TICKS)

    # Overshoot: after the first tick dist<approach_tol (not necessarily
    # dwell-qualified), how far does dist excurse above approach_tol again.
    below_tol_idxs = [i for i in range(n_rows) if dist_l[i] < params['approach_tol']]
    overshoot_dist = None
    if below_tol_idxs:
        first_below = below_tol_idxs[0]
        after = dist_l[first_below:]
        overshoot_dist = max(0.0, max(after) - params['approach_tol'])

    cmd_fx_l = [a for a, _ in actual_cmd if not is_nan(a)]
    cmd_fy_l = [b for _, b in actual_cmd if not is_nan(b)]
    stdev_cmd_fx = stats.pstdev(cmd_fx_l) if len(cmd_fx_l) >= 2 else None
    stdev_cmd_fy = stats.pstdev(cmd_fy_l) if len(cmd_fy_l) >= 2 else None
    fmax = params['approach_fmax']
    sat_hits = sum(1 for a, b in actual_cmd
                    if not is_nan(a) and not is_nan(b)
                    and (abs(a) >= 0.99 * fmax or abs(b) >= 0.99 * fmax))
    saturation_frac = (sat_hits / len(actual_cmd)) if actual_cmd else None

    dist_trend = trend(dist_l)
    diverged = bool(
        dist_trend is not None and not dist_trend['net_decrease']
        and saturation_frac is not None and saturation_frac >= SATURATION_FRAC_DIVERGED
    )

    p0_2_4_gate4_retest = {
        'dwell_ticks_required': DWELL_TICKS,
        'time_to_converge_s': t_conv_s,
        'entered_band_with_dwell': entered_band_with_dwell,
        'dist_trend': dist_trend,
        'overshoot_dist_m': overshoot_dist,
        'stdev_cmd_fx_N': stdev_cmd_fx,
        'stdev_cmd_fy_N': stdev_cmd_fy,
        'saturation_frac': saturation_frac,
        'diverged': diverged,
        'qr_decode_rate': (
            sum(1 for r in approach if r.get('qr_decode_success') == '1')
            / n_rows) if n_rows else None,
        'window_duration_s': (t_l[-1] - t_l[0]) if n_rows else None,
    }

    # P1-0 Fase 0 -- vision attribution. `entered_band_with_dwell` above ORs a
    # camera criterion with a ground-truth-odometry criterion, so it cannot
    # answer "did the camera do anything?". These three disjoint views can.
    # Nothing here changes any pre-existing number; it only says which
    # population each run belongs to.
    p1_0_vision_attribution = {
        'converged_via': attribute_convergence(cam_idx, gt_idx),
        'entered_band_with_dwell_camera': cam_idx is not None,
        'entered_band_with_dwell_decoded': dec_idx is not None,
        'entered_band_with_dwell_ground_truth': gt_idx is not None,
        'time_to_converge_camera_s': t_conv_camera_s,
        'time_to_converge_decoded_s': t_conv_decoded_s,
        'time_to_converge_ground_truth_s': t_conv_gt_s,
        # Provable non-contribution: the QR was never read in the whole
        # episode, so any convergence in this run is ground-truth homing.
        'qr_never_decoded': (p0_2_4_gate4_retest['qr_decode_rate'] == 0.0
                             if p0_2_4_gate4_retest['qr_decode_rate'] is not None else None),
        'n_ticks_camera_offset_fresh': sum(off_fresh_l),
        'n_ticks_decoded': sum(decode_l),
        'n_approach_qr_ticks': n_rows,
    }

    # P0-2.5 Candidate #2 guardrails (design-hardening review, approved
    # sequence 2->1->3->4): jitter must go down AND final_dist must improve
    # before this candidate is credited -- variance reduction alone is not
    # sufficient (an EMA can hide a persistent bias, not just noise).
    def tick_jitter_stdev(series, mask):
        vals = [series[i] for i in range(len(series)) if mask[i]]
        if len(vals) < 3:
            return None
        diffs = [vals[i + 1] - vals[i] for i in range(len(vals) - 1)]
        return stats.pstdev(diffs) if len(diffs) >= 2 else None

    p0_2_5_candidate2_guardrail = {
        'qr_offset_ema_alpha_used': params['qr_offset_ema_alpha'],
        'stdev_diff_tx_servoing_m': tick_jitter_stdev(tx_l, servoing_mask),
        'stdev_diff_ty_servoing_m': tick_jitter_stdev(ty_l, servoing_mask),
        'n_servoing_ticks': sum(servoing_mask),
        'final_dist_m': dist_l[-1] if dist_l else None,
        'min_dist_target_m': min(dist_l) if dist_l else None,
    }

    # Gate 5
    log_text = ''
    try:
        log_text = open('%s/%s.log' % (data_dir, tag)).read()
    except FileNotFoundError:
        pass
    exit_path, exit_counts = classify_exit(log_text)

    return {
        'tag': tag,
        'reached_approach_qr': True,
        'n_approach_qr_rows': len(approach),
        'locked_yaw_assumption_rad': locked_yaw,
        'params_from_defaults': sorted(used_defaults),
        'gate2_qr_offset_tracks_relative_pose': {
            'r_qr_size_vs_dist_to_target': r_size_dist, 'n': n_size,
            'r_qr_ex_vs_lateral_offset': r_ex_lat, 'n': n_ex,
            'r_qr_ey_vs_forward_offset': r_ey_fwd, 'n': n_ey,
        },
        'gate3_command_follows_qr_offset': {
            'rmse_all_rows_ground_truth_only': rmse_all_without,
            'rmse_all_rows_with_qr_correction': rmse_all_with,
            'n_all_rows': n_all,
            'rmse_servoing_window_ground_truth_only': rmse_serv_without,
            'rmse_servoing_window_with_qr_correction': rmse_serv_with,
            'n_servoing_rows': n_serv,
            'with_qr_fits_better': (rmse_serv_with < rmse_serv_without)
                if (rmse_serv_with is not None and rmse_serv_without is not None) else None,
        },
        'gate4_error_converges': {
            'abs_qr_ex_trend': trend(err_ex),
            'abs_qr_ey_minus_ey_target_trend': trend(err_ey),
            'sign_changes_qr_ex': sign_changes(qr_ex_l),
            'sign_changes_qr_ey_minus_ey_target': sign_changes(
                [qr_ey_l[i] - ey_target_l[i] for i in range(len(qr_ey_l))]),
            'entered_qr_center_tol_band': entered_band,
        },
        'p0_2_4_gate4_retest': p0_2_4_gate4_retest,
        'p1_0_vision_attribution': p1_0_vision_attribution,
        'p0_2_5_candidate2_guardrail': p0_2_5_candidate2_guardrail,
        'gate5_exit_path': {'primary': exit_path, 'occurrence_counts': exit_counts},
    }


def selftest():
    """P1-0 Fase 0 checks: the split must not change the combined number, and
    must actually separate a ground-truth-only run from a camera-led one."""
    t = [i * 0.1 for i in range(10)]
    F, T = False, True

    # dwell scan
    assert first_dwell([F] * 10, t, 3) == (None, None)
    assert first_dwell([F, F, T, T, T, F, F, F, F, F], t, 3)[0] == 2
    assert first_dwell([T, T, F, T, T, F, F, F, F, F], t, 3) == (None, None), \
        'must require CONSECUTIVE ticks, not a total count'
    idx, secs = first_dwell([F, T, T, T, T, F, F, F, F, F], t, 3)
    assert idx == 1 and abs(secs - 0.1) < 1e-9, 'time is measured from window start'

    # attribution
    assert attribute_convergence(None, None) == 'NONE'
    assert attribute_convergence(3, None) == 'VISION'
    assert attribute_convergence(None, 3) == 'GROUND_TRUTH'
    assert attribute_convergence(2, 5) == 'VISION'
    assert attribute_convergence(5, 2) == 'GROUND_TRUTH'
    assert attribute_convergence(4, 4) == 'GROUND_TRUTH', \
        'tie goes to ground truth -- it is the null hypothesis vision must beat'

    # the contamination this whole block exists to expose: a run where the
    # camera never contributed still counts as converged in the combined view.
    cam = [F] * 10
    gt = [F, F, F, T, T, T, T, F, F, F]
    combined = [cam[i] or gt[i] for i in range(10)]
    assert first_dwell(combined, t, 3)[0] is not None, 'combined says converged'
    assert first_dwell(cam, t, 3)[0] is None, 'camera says it did nothing'
    assert attribute_convergence(first_dwell(cam, t, 3)[0],
                                 first_dwell(gt, t, 3)[0]) == 'GROUND_TRUTH'

    # decoded is a strict subset of camera: centring on corner points that
    # never decoded must not be credited as a decoded-QR convergence.
    cam2 = [F, T, T, T, T, F, F, F, F, F]
    decode = [F, T, T, F, T, F, F, F, F, F]
    decoded = [cam2[i] and decode[i] for i in range(10)]
    assert first_dwell(cam2, t, 3)[0] is not None
    assert first_dwell(decoded, t, 3)[0] is None
    print('selftest OK')


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    if sys.argv[1] == '--selftest':
        selftest()
        return
    data_dir = sys.argv[1]
    tags = sys.argv[2:] or ['Q1', 'Q2', 'Q3', 'Q4', 'Q5', 'Q6']

    print('=' * 78)
    print('P0-2.2b evidence report -- NOT a PASS/FAIL verdict for APPROACH_QR.')
    print('A GRAB transition is not itself evidence QR worked; that is the')
    print('hypothesis these six gates test. See docs/P0-2-2-SPEC.md.')
    print('=' * 78)

    results = {}
    inconclusive = {}
    for tag in tags:
        rows, reasons = quality_gate(tag, data_dir)
        if reasons:
            inconclusive[tag] = reasons
            print('\n[%s] INCONCLUSIVE: %s' % (tag, '; '.join(reasons)))
            continue
        params, used_defaults = load_params('%s/%s.params.yaml' % (data_dir, tag))
        result = analyze_run(tag, data_dir, rows, params, used_defaults)
        results[tag] = result
        print('\n[%s] Gate 1: QR is a control input -- confirmed by static code trace '
              '(docs/P0-2-2-SPEC.md S1), cross-checked below by Gate 3.' % tag)
        if not result['reached_approach_qr']:
            print('  Did not observe fsm_state==APPROACH_QR in this run.')
            continue
        g2 = result['gate2_qr_offset_tracks_relative_pose']
        print('  Gate 2 (qr_offset vs relative pose): r(size,dist)=%s (n=%d) '
              'r(ex,lateral)=%s (n=%d) r(ey,forward)=%s (n=%d)'
              % (fmt(g2['r_qr_size_vs_dist_to_target']), g2['n'],
                 fmt(g2['r_qr_ex_vs_lateral_offset']), g2['n'],
                 fmt(g2['r_qr_ey_vs_forward_offset']), g2['n']))
        g3 = result['gate3_command_follows_qr_offset']
        print('  Gate 3 (command follows qr_offset, servoing window n=%d): '
              'RMSE ground-truth-only=%s  RMSE with-QR-correction=%s  '
              'with-QR fits better=%s'
              % (g3['n_servoing_rows'], fmt(g3['rmse_servoing_window_ground_truth_only']),
                 fmt(g3['rmse_servoing_window_with_qr_correction']), g3['with_qr_fits_better']))
        g4 = result['gate4_error_converges']
        print('  Gate 4 (error converges): |qr_ex| trend=%s  |qr_ey-ey_target| trend=%s  '
              'entered qr_center_tol band=%s'
              % (g4['abs_qr_ex_trend'], g4['abs_qr_ey_minus_ey_target_trend'],
                 g4['entered_qr_center_tol_band']))
        p24 = result['p0_2_4_gate4_retest']
        print('  P0-2.4 Gate 4 retest: time_to_converge=%ss (dwell=%d ticks)  '
              'overshoot=%sm  stdev(cmd_fx,cmd_fy)=(%s,%s)N  sat_frac=%s  diverged=%s  '
              'decode_rate=%s'
              % (fmt(p24['time_to_converge_s']), p24['dwell_ticks_required'],
                 fmt(p24['overshoot_dist_m']), fmt(p24['stdev_cmd_fx_N']),
                 fmt(p24['stdev_cmd_fy_N']), fmt(p24['saturation_frac']),
                 p24['diverged'], fmt(p24['qr_decode_rate'])))
        pva = result['p1_0_vision_attribution']
        print('  P1-0 vision attribution: converged_via=%s  dwell entry: camera=%s '
              'decoded=%s ground_truth=%s  t_conv(camera/decoded/gt)=(%s/%s/%s)s  '
              'qr_never_decoded=%s  ticks decoded/fresh/total=%d/%d/%d'
              % (pva['converged_via'], pva['entered_band_with_dwell_camera'],
                 pva['entered_band_with_dwell_decoded'],
                 pva['entered_band_with_dwell_ground_truth'],
                 fmt(pva['time_to_converge_camera_s']),
                 fmt(pva['time_to_converge_decoded_s']),
                 fmt(pva['time_to_converge_ground_truth_s']),
                 pva['qr_never_decoded'], pva['n_ticks_decoded'],
                 pva['n_ticks_camera_offset_fresh'], pva['n_approach_qr_ticks']))
        p25 = result['p0_2_5_candidate2_guardrail']
        print('  P0-2.5 Candidate #2 guardrail: alpha=%s  stdev(diff tx,ty | servoing)=(%s,%s)m '
              '(n_servo_ticks=%d)  final_dist=%sm  min_dist_target=%sm'
              % (p25['qr_offset_ema_alpha_used'], fmt(p25['stdev_diff_tx_servoing_m']),
                 fmt(p25['stdev_diff_ty_servoing_m']), p25['n_servoing_ticks'],
                 fmt(p25['final_dist_m']), fmt(p25['min_dist_target_m'])))
        g5 = result['gate5_exit_path']
        print('  Gate 5 (exit path, from FSM\'s own log lines): %s  (occurrence counts: %s)'
              % (g5['primary'], g5['occurrence_counts']))
        if result['params_from_defaults']:
            print('  NOTE: params.yaml missing/incomplete, used source defaults for: %s'
                  % result['params_from_defaults'])

    # Gate 6: aggregate
    print('\n' + '=' * 78)
    print('Gate 6: repeatability aggregate (n_valid=%d, n_inconclusive=%d)'
          % (len(results), len(inconclusive)))
    print('=' * 78)
    reached = {t: r for t, r in results.items() if r['reached_approach_qr']}
    exit_paths = [r['gate5_exit_path']['primary'] for r in reached.values()]
    for path in sorted(set(exit_paths)):
        print('  exit path %s: %d/%d runs' % (path, exit_paths.count(path), len(exit_paths)))
    fits_better = [r['gate3_command_follows_qr_offset']['with_qr_fits_better'] for r in reached.values()]
    fits_better = [v for v in fits_better if v is not None]
    if fits_better:
        print('  Gate 3 "with-QR fits better": %d/%d runs' % (sum(fits_better), len(fits_better)))
    converged = [r['gate4_error_converges']['entered_qr_center_tol_band'] for r in reached.values()]
    if converged:
        print('  Gate 4 "entered qr_center_tol band": %d/%d runs' % (sum(converged), len(converged)))

    # docs/P0-2-4-SPEC.md S6/S7 -- Gate 4 retest verdict (dwell-qualified
    # convergence + divergence check). This is a verdict over the runs
    # actually present in this data_dir/tag list -- if that's fewer than the
    # S6 stopping-rule thresholds, it MUST read INCONCLUSIVE, not be forced.
    print('\n' + '-' * 78)
    print('P0-2.4 Gate 4 retest verdict (docs/P0-2-4-SPEC.md S6-S7)')
    print('-' * 78)
    p24_all = [r['p0_2_4_gate4_retest'] for r in reached.values()]
    n_reached = len(p24_all)
    entered_dwell = [p['entered_band_with_dwell'] for p in p24_all]
    diverged_flags = [p['diverged'] for p in p24_all]
    n_entered = sum(entered_dwell)
    n_diverged = sum(diverged_flags)
    stopping_rule_met = n_reached >= 18 or n_entered >= 5
    majority = n_reached > 0 and n_entered > n_reached / 2
    print('  n_reached_approach_qr=%d  entered_band_with_dwell=%d/%d  diverged=%d/%d  '
          'stopping_rule_met=%s (S6: n>=18 OR entered>=5)'
          % (n_reached, n_entered, n_reached, n_diverged, n_reached, stopping_rule_met))
    if not stopping_rule_met:
        verdict = 'INCONCLUSIVE'
        reason = 'stopping rule (S6) not yet met -- more runs needed before a verdict'
    elif majority and n_diverged == 0:
        verdict = 'PASS'
        reason = 'majority of runs entered+held the band, no run diverged'
    elif not majority:
        verdict = 'FAIL'
        reason = 'minority of runs entered+held the band, stopping rule already met'
    else:
        verdict = 'INCONCLUSIVE'
        reason = 'majority entered the band but >=1 run diverged -- mixed result, not resolved by picking one signal'
    print('  VERDICT: %s (%s)' % (verdict, reason))
    print('  This verdict answers Gate 4 (precision convergence) only -- it does not')
    print('  authorize any qr_detector.py/qr_logic.py/mission_fsm.py/controller/param change.')
    print('  SCOPE: entered_band_with_dwell is camera-OR-ground-truth. Read the P1-0')
    print('  breakdown below before quoting this number as evidence the camera works.')

    # ------------------------------------------------------------------
    # P1-0 Fase 0 -- vision attribution breakdown.
    # The verdict above cannot distinguish "the visual servo centred the ROV"
    # from "PD homing to the simulator's exact payload coordinates centred the
    # ROV". mission_fsm.py navigates to /hydroships/payload_pose, which is the
    # spawner's ground truth, and its APPROACH_QR exit also scores +15 through
    # a fallback branch that picks a wall from a shuffled list. So the combined
    # number is an upper bound on vision performance, never a measurement of it.
    # ------------------------------------------------------------------
    print('\n' + '-' * 78)
    print('P1-0 Fase 0: vision attribution (two populations, reported separately)')
    print('-' * 78)
    pva_all = [r['p1_0_vision_attribution'] for r in reached.values()]
    n_cam = sum(p['entered_band_with_dwell_camera'] for p in pva_all)
    n_dec = sum(p['entered_band_with_dwell_decoded'] for p in pva_all)
    n_gt = sum(p['entered_band_with_dwell_ground_truth'] for p in pva_all)
    n_never = sum(1 for p in pva_all if p['qr_never_decoded'])
    by_attr = {k: sum(1 for p in pva_all if p['converged_via'] == k)
               for k in ('VISION', 'GROUND_TRUTH', 'NONE')}
    print('  n_reached_approach_qr = %d' % n_reached)
    print('    combined (camera OR ground truth) : %d/%d   <- the P0-2.4 number above'
          % (n_entered, n_reached))
    print('    camera-derived offset centred     : %d/%d' % (n_cam, n_reached))
    print('    decoded-QR centred (strictest)    : %d/%d' % (n_dec, n_reached))
    print('    ground-truth odometry alone       : %d/%d' % (n_gt, n_reached))
    print('  first to reach the band: VISION=%d  GROUND_TRUTH=%d  NONE=%d'
          % (by_attr['VISION'], by_attr['GROUND_TRUTH'], by_attr['NONE']))
    print('  runs where the QR was NEVER decoded: %d/%d -- vision provably contributed'
          % (n_never, n_reached))
    print('    nothing in these; any convergence they show is ground-truth homing.')
    if n_reached:
        print('  Vision-only Gate 4 rate: %d/%d = %.0f%%   (combined rate: %d/%d = %.0f%%)'
              % (n_dec, n_reached, 100.0 * n_dec / n_reached,
                 n_entered, n_reached, 100.0 * n_entered / n_reached))
    print('  Interpretation rule: quote the CAMERA or DECODED row as evidence about')
    print('  perception. The COMBINED row is only valid as an end-to-end mission')
    print('  statistic, and only while payload_pose ground truth remains wired in.')

    # docs/P0-2-5-ENGINEERING-ANALYSIS.md Candidate #2 guardrail (design-
    # hardening review): variance reduction alone does NOT credit this
    # candidate -- final_dist must also improve vs the P0-2.4 baseline
    # (mean final_dist reported here; baseline values are in
    # docs/P0-2-4-RESULTS.md S3, not recomputed here to avoid conflating
    # two different data_dirs in one run of this script).
    print('\n' + '-' * 78)
    print('P0-2.5 Candidate #2 guardrail summary (docs/P0-2-5 hardening review)')
    print('-' * 78)
    p25_all = [r['p0_2_5_candidate2_guardrail'] for r in reached.values()]
    alphas = sorted(set(p['qr_offset_ema_alpha_used'] for p in p25_all))
    jitter_tx = [p['stdev_diff_tx_servoing_m'] for p in p25_all if p['stdev_diff_tx_servoing_m'] is not None]
    jitter_ty = [p['stdev_diff_ty_servoing_m'] for p in p25_all if p['stdev_diff_ty_servoing_m'] is not None]
    final_dists = [p['final_dist_m'] for p in p25_all if p['final_dist_m'] is not None]
    print('  qr_offset_ema_alpha used across runs: %s (1.0 = filter inactive/baseline)' % alphas)
    print('  mean stdev(diff tx | servoing)=%s m (n=%d runs w/ >=3 servoing ticks)'
          % (fmt(stats.fmean(jitter_tx)) if jitter_tx else 'n/a', len(jitter_tx)))
    print('  mean stdev(diff ty | servoing)=%s m (n=%d runs w/ >=3 servoing ticks)'
          % (fmt(stats.fmean(jitter_ty)) if jitter_ty else 'n/a', len(jitter_ty)))
    print('  mean final_dist=%s m (n=%d) -- compare against P0-2.4 baseline mean before crediting'
          % (fmt(stats.fmean(final_dists)) if final_dists else 'n/a', len(final_dists)))
    print('  Per docs/P0-2-5-ENGINEERING-ANALYSIS.md SB: credit this candidate ONLY if jitter AND')
    print('  mean final_dist both improve vs baseline -- reduced jitter alone can mean a filter')
    print('  that stabilized on a biased value, not a fixed problem.')

    if inconclusive:
        print('\n  INCONCLUSIVE runs (excluded from statistics above, re-run manually):')
        for t, reasons in inconclusive.items():
            print('    %s: %s' % (t, '; '.join(reasons)))

    out = {
        'results': results,
        'inconclusive': inconclusive,
        'p0_2_4_gate4_verdict': {
            'n_reached_approach_qr': n_reached,
            'entered_band_with_dwell': n_entered,
            'diverged': n_diverged,
            'stopping_rule_met': stopping_rule_met,
            'verdict': verdict,
            'reason': reason,
        },
        'p1_0_vision_attribution_summary': {
            'n_reached_approach_qr': n_reached,
            'entered_band_combined': n_entered,
            'entered_band_camera': n_cam,
            'entered_band_decoded': n_dec,
            'entered_band_ground_truth': n_gt,
            'converged_via_counts': by_attr,
            'runs_qr_never_decoded': n_never,
        },
        'p0_2_5_candidate2_guardrail_summary': {
            'qr_offset_ema_alpha_used': alphas,
            'mean_stdev_diff_tx_m': stats.fmean(jitter_tx) if jitter_tx else None,
            'mean_stdev_diff_ty_m': stats.fmean(jitter_ty) if jitter_ty else None,
            'mean_final_dist_m': stats.fmean(final_dists) if final_dists else None,
            'n_runs': len(p25_all),
        },
    }
    out_path = '%s/P0-2-2b-results.json' % data_dir
    with open(out_path, 'w') as f:
        json.dump(out, f, indent=2, default=str)
    print('\nFull machine-readable evidence written to %s' % out_path)
    print('Acceptance matrix in docs/P0-2-AUDIT.md S2 is NOT updated by this script.')


def fmt(v):
    return 'n/a' if v is None else ('%.3f' % v)


if __name__ == '__main__':
    main()
