#!/usr/bin/env python3
"""R-10 trajectory reducer — parse recorder_qr.py CSV and extract DESCEND
window metrics for alt_gap overshoot investigation.

Usage:
    # Single file:
    python3 reduce_r10_trajectory.py /path/to/R10-after-3001.csv

    # Directory (battery summary):
    python3 reduce_r10_trajectory.py /path/to/p0-data-dir/

Output: table with depth at DESCEND entry, depth_ok trigger, GRAB trigger,
max depth (overshoot), DESCEND duration, attach result, and inferred
alt_gap at GRAB.
"""
import csv
import math
import os
import sys


QR_FLOOR_Z = 0.894       # world z of QR payload top surface (m, negative in world)
GRIPPER_BOTTOM_DZ = 0.16 # gripper_base joint z offset below base_link (m)
GRAB_DEPTH = 0.70        # target depth for GRAB (m, positive down)


def parse_csv(path):
    rows = []
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    return rows


def find_transitions(rows):
    """Return list of (t, from_state, to_state) transitions."""
    transitions = []
    prev = None
    for row in rows:
        cur = row['fsm_state'].strip()
        if cur and cur != prev:
            transitions.append((float(row['t']), prev, cur))
            prev = cur
    return transitions


def read_meta(csv_path):
    """Read .meta file next to the CSV, if present."""
    meta_path = csv_path.replace('.csv', '.meta')
    if not os.path.exists(meta_path):
        return {}
    d = {}
    with open(meta_path) as f:
        for line in f:
            line = line.strip()
            if '=' in line:
                k, v = line.split('=', 1)
                d[k.strip()] = v.strip()
    return d


def extract_descend_window(rows, descend_depth_tol=0.02):
    """Extract DESCEND window metrics from a single run CSV.

    descend_depth_tol: tolerance used for depth_ok detection (default 0.02).
    """
    tag = os.path.basename(rows[0].get('_source', 'unknown')) if rows else 'unknown'
    transitions = find_transitions(rows)

    desc_entry = None   # row where fsm_state first becomes DESCEND
    depth_ok = None     # first row in DESCEND where depth >= grab_depth - tol
    grab = None         # first row where fsm_state becomes GRAB
    max_depth = None
    min_depth = None
    vz_at_grab = None
    gripper_at_grab = None

    in_descend = False
    for row in rows:
        t = float(row['t'])
        depth = float(row['depth']) if row['depth'] else None
        vz = float(row['vz']) if row['vz'] else None
        state = row['fsm_state'].strip()
        gs = row['gripper_status'].strip()

        if depth is not None:
            if max_depth is None or depth > max_depth:
                max_depth = depth
            if min_depth is None or depth < min_depth:
                min_depth = depth

        if state == 'DESCEND' and desc_entry is None:
            desc_entry = {'t': t, 'depth': depth, 'z': float(row['z']) if row['z'] else None, 'vz': vz}
            in_descend = True
        elif state != 'DESCEND' and in_descend:
            in_descend = False

        if in_descend and depth_ok is None and depth is not None:
            # Use a slightly looser threshold because the CSV samples every 0.1s
            # and the actual depth_ok trigger may fall between samples (just before
            # state transition to GRAB). The deepest DESCEND row is the best proxy.
            if depth >= GRAB_DEPTH - descend_depth_tol - 0.005:
                depth_ok = {'t': t, 'depth': depth, 'vz': vz}

        if state == 'GRAB' and grab is None:
            grab = {'t': t, 'depth': depth, 'z': float(row['z']) if row['z'] else None, 'vz': vz}
            vz_at_grab = vz
            gripper_at_grab = gs

    # alt_gap at GRAB: positive = gripper above floor, negative = below
    alt_gap = None
    if grab and grab['depth'] is not None:
        alt_gap = QR_FLOOR_Z - grab['depth'] - GRIPPER_BOTTOM_DZ

    # Attach result: look for 'attached' status during GRAB window
    attach_result = 'no_grab'
    if grab:
        grab_t = grab['t']
        for row in rows:
            if float(row['t']) >= grab_t and row['gripper_status'].strip() == 'attached':
                attach_result = 'attached'
                break
        else:
            attach_result = 'not_attached'

    notes = []
    if depth_ok and grab:
        dwell = grab['t'] - depth_ok['t']
        if dwell > 6.0:
            notes.append('recenter_timeout_likely')
    if alt_gap is not None and alt_gap < -0.005:
        notes.append('negative_alt_gap_overshoot')
    if alt_gap is not None and alt_gap > 0.10:
        notes.append('alt_gap_very_loose')
    if desc_entry and grab:
        descend_dur = grab['t'] - desc_entry['t']
        if descend_dur > 12.0:
            notes.append('descend_slow_%.1fs' % descend_dur)

    return {
        'transitions': transitions,
        'desc_entry': desc_entry,
        'depth_ok': depth_ok,
        'grab': grab,
        'max_depth': max_depth,
        'min_depth': min_depth,
        'vz_at_grab': vz_at_grab,
        'gripper_at_grab': gripper_at_grab,
        'attach_result': attach_result,
        'alt_gap_at_grab': alt_gap,
        'notes': notes,
    }


def format_row(metrics, tag):
    t = metrics['transitions']
    de = metrics['desc_entry']
    dok = metrics['depth_ok']
    g = metrics['grab']

    trans_str = ' -> '.join(
        '%s@%.1f' % (to, ti) for ti, _, to in t if to
    )

    desc = 'no_descend'
    if de:
        desc = 'DESCEND@%.1f depth=%.3f' % (de['t'], de['depth'] if de['depth'] is not None else -1)

    dok_str = 'no_depth_ok'
    if dok:
        dwell = (g['t'] - dok['t']) if g else None
        dok_str = 'depth_ok@%.1f depth=%.3f dwell_to_grab=%s' % (
            dok['t'], dok['depth'],
            '%.1fs' % dwell if dwell is not None else 'n/a')

    grab_str = 'no_grab'
    if g:
        grab_str = 'GRAB@%.1f depth=%.3f vz=%.3f alt_gap=%s' % (
            g['t'], g['depth'] if g['depth'] is not None else -1,
            g['vz'] if g['vz'] is not None else float('nan'),
            '%.3f' % metrics['alt_gap_at_grab'] if metrics['alt_gap_at_grab'] is not None else 'n/a')

    attach = metrics['attach_result']
    notes = ','.join(metrics['notes']) if metrics['notes'] else '-'

    return (tag, trans_str, desc, dok_str, grab_str,
            '%.3f' % metrics['max_depth'] if metrics['max_depth'] is not None else 'n/a',
            '%.3f' % metrics['min_depth'] if metrics['min_depth'] is not None else 'n/a',
            '%.3f' % metrics['vz_at_grab'] if metrics['vz_at_grab'] is not None else 'n/a',
            metrics['gripper_at_grab'] or '-', attach,
            '%.3f' % metrics['alt_gap_at_grab'] if metrics['alt_gap_at_grab'] is not None else 'n/a',
            notes)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    target = sys.argv[1]
    if os.path.isdir(target):
        csvs = sorted(
            os.path.join(target, f)
            for f in os.listdir(target)
            if f.endswith('.csv') and os.path.getsize(os.path.join(target, f)) > 100
        )
    else:
        csvs = [target]

    if not csvs:
        print('No CSV files found.', file=sys.stderr)
        sys.exit(1)

    rows_out = []
    for csv_path in csvs:
        try:
            rows = parse_csv(csv_path)
            if not rows:
                continue
            tag = os.path.basename(csv_path).replace('.csv', '')
            meta = read_meta(csv_path)
            tol = float(meta.get('descend_depth_tol', '0.02'))
            metrics = extract_descend_window(rows, descend_depth_tol=tol)
            rows_out.append(format_row(metrics, tag))
        except Exception as e:
            print('ERROR parsing %s: %s' % (csv_path, e), file=sys.stderr)

    if not rows_out:
        print('No data extracted.', file=sys.stderr)
        sys.exit(1)

    # Header
    hdr = ('tag', 'transitions', 'descend_entry', 'depth_ok', 'grab_trigger',
           'max_depth', 'min_depth', 'vz_at_grab', 'gripper_at_grab',
           'attach', 'alt_gap_at_grab', 'notes')
    fmt = '%-28s %-50s %-22s %-28s %-28s %-10s %-10s %-10s %-14s %-12s %-14s %-20s'
    print(fmt % hdr)
    print('-' * 280)
    for r in rows_out:
        print(fmt % r)


if __name__ == '__main__':
    main()
