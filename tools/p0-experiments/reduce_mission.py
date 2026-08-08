#!/usr/bin/env python3
"""P0-1e reduction — closed-loop DIVE regression."""
import csv
import re
import sys

import numpy as np

import os
_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(_REPO, 'src', 'hydroships_control'))
from hydroships_control.allocation import THRUSTERS

AXZ = np.array([a[2] for _, a in THRUSTERS])
SCAN_DEPTH, DEPTH_TOL = 0.30, 0.06
THRESH = SCAN_DEPTH - DEPTH_TOL          # 0.24 m
FLOOR_Z = -0.809                          # kki_arena floor contact
OUT_LIMIT = 60.0                          # stabilizer depth.out_limit


def load(p):
    return [{k: float(v) for k, v in r.items()} for r in csv.DictReader(open(p))]


def fsm_times(logpath):
    """sim-time is not in the log, so use wall-clock deltas between transitions."""
    ev = []
    for line in open(logpath):
        m = re.search(r'\[(\d+\.\d+)\] \[mission_fsm\]: \[FSM\] (\S+) -> (\S+)', line)
        if m:
            ev.append((float(m.group(1)), m.group(2), m.group(3)))
        if 'DIVE timeout' in line:
            ev.append((0.0, 'DIVE', 'TIMEOUT'))
    return ev


def dive_window(rows):
    """DIVE = from first sample where setpoint == -0.30 until depth first >= 0.24."""
    sp = np.array([r['sp_depth'] for r in rows])
    on = np.where(np.abs(sp + 0.30) < 1e-6)[0]
    if len(on) == 0:
        return None
    i0 = on[0]
    depth = np.array([r['depth'] for r in rows])
    hit = np.where(depth[i0:] >= THRESH)[0]
    i1 = i0 + hit[0] if len(hit) else len(rows) - 1
    return i0, i1, len(hit) > 0


def report(tag, csvp, logp):
    rows = load(csvp)
    t = np.array([r['t'] for r in rows])
    z = np.array([r['z'] for r in rows])
    depth = np.array([r['depth'] for r in rows])
    pitch = np.degrees([r['pitch'] for r in rows])
    roll = np.degrees([r['roll'] for r in rows])
    fz = np.array([r['fz'] for r in rows])
    thr = np.array([[r['thr%d' % i] for i in range(1, 7)] for r in rows])
    sumfz = thr @ AXZ

    w = dive_window(rows)
    ev = fsm_times(logp)
    trans = [e for e in ev if e[1] == 'DIVE']
    print('--- %s ---' % tag)
    if not w:
        print('  no DIVE setpoint seen; CONTAMINATED')
        return None
    i0, i1, reached = w
    dt = t[i1] - t[i0]
    sl = slice(i0, i1 + 1)
    sat = np.abs(fz[sl]) >= OUT_LIMIT - 1e-6
    print('  FSM               : %s' % (', '.join('%s->%s' % (a, b) for _, a, b in ev[:2]) or 'none'))
    print('  DIVE timeout      : %s' % ('YES' if any(e[2] == 'TIMEOUT' for e in ev) else 'no'))
    print('  setpoint          : %.3f m' % rows[i0]['sp_depth'])
    print('  depth at entry    : %.3f m -> threshold 0.24 m' % depth[i0])
    print('  time to threshold : %.2f s sim   (budget 20 s)  reached=%s' % (dt, reached))
    print('  |pitch| max /end  : %.2f / %+.2f deg' % (np.abs(pitch[sl]).max(), pitch[i1]))
    print('  |roll|  max /end  : %.2f / %+.2f deg' % (np.abs(roll[sl]).max(), roll[i1]))
    print('  cmd_vel.z range   : %.2f .. %.2f N   saturated %.0f%% of samples'
          % (fz[sl].min(), fz[sl].max(), 100.0 * sat.mean()))
    ok = np.abs(fz[sl]) > 1e-6
    fid = (sumfz[sl][ok] / fz[sl][ok])
    print('  allocator fidelity: %.1f%% (median Sum(thrust.axis_z)/cmd_vel.z)'
          % (100 * np.median(fid)))
    print('  max |thrust| cmd  : %.2f N  (limits +50/-40)' % np.abs(thr[sl]).max())
    print('  floor contact     : %s' % ('YES' if (z[sl] <= FLOOR_Z).any() else 'no'))
    print('  post-DIVE (raw, no verdict): z %.3f -> %.3f, |pitch|max %.1f deg'
          % (z[i1], z[-1], np.abs(pitch[i1:]).max()))
    print()
    return dt, np.abs(pitch[sl]).max(), np.abs(roll[sl]).max(), 100 * np.median(fid)


if __name__ == '__main__':
    res = {}
    for tag, c, l in (('R1 random', 'R1.csv', 'R1.log'),
                      ('R2 random', 'R2.csv', 'R2.log'),
                      ('R3 random', 'R3.csv', 'R3.log'),
                      ('R4 fixed ', 'R4.csv', 'R4.log')):
        res[tag] = report(tag, c, l)
    print('=' * 66)
    print('%-11s %12s %11s %10s %10s' % ('run', 't_thresh(s)', '|pitch|max', '|roll|max', 'alloc_fid'))
    for k, v in res.items():
        if v:
            print('%-11s %12.2f %11.2f %10.2f %9.1f%%' % (k, v[0], v[1], v[2], v[3]))
    ts = [v[0] for v in res.values() if v]
    print('\nspread across runs: %.2f .. %.2f s  (budget 20 s, open-loop P0-1d 0.55-0.57 s)'
          % (min(ts), max(ts)))
