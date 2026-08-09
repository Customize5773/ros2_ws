#!/usr/bin/env python3
"""P0-1d reduction v2 — truncates every segment at first floor contact.

Floor contact measured empirically at z = -4.829 m (vehicle rests there and
stops). Any sample at or below FLOOR_Z, and everything after it, is discarded;
segments with too little pre-contact data are reported CONTAMINATED rather than
interpreted.
"""
import csv
import sys

import numpy as np

import os
_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(_REPO, 'src', 'hydroships_control'))
from hydroships_control.allocation import THRUSTERS

COG = np.array([0.00237, 0.0, 0.00024])
FLOOR_Z = -4.82
SURF_Z = -0.111
MIN_CLEAN_S = 3.0


def load(p):
    return [{k: (int(v) if k == 'step' else float(v)) for k, v in r.items()}
            for r in csv.DictReader(open(p))]


def segs(rows):
    out, cur, idx = [], [], rows[0]['step']
    for r in rows:
        if r['step'] != idx:
            out.append((idx, cur)); cur = []; idx = r['step']
        cur.append(r)
    out.append((idx, cur))
    return out


def truncate(g):
    """Keep only samples strictly before the first floor contact."""
    z = np.array([r['z'] for r in g])
    hit = np.argmax(z <= FLOOR_Z) if (z <= FLOOR_Z).any() else len(g)
    return g[:hit], (z <= FLOOR_Z).any()


def wrench(f):
    W = np.zeros(6)
    for i, (pos, axis) in enumerate(THRUSTERS):
        W[0:3] += f[i] * axis
        W[3:6] += f[i] * np.cross(np.asarray(pos) - COG, axis)
    return W


def report(path, label):
    print('=== %s ===' % label)
    print('  %-20s %7s %8s %7s %8s %9s %9s  %s' % (
        'cmd T1/T2/T6', 'Fz_cog', 'My_cog', 'clean_s', 'vz(m/s)', 'pitch_max',
        'wy_max', 'status'))
    for idx, g in segs(load(path)):
        if len(g) < 10:
            continue
        f = np.array([g[0]['cmd%d' % i] for i in range(1, 7)])
        if abs(f).sum() == 0:
            continue
        W = wrench(f)
        clean, touched = truncate(g)
        tag = '%.2f/%.2f/%.2f' % (f[0], f[1], f[5])
        if len(clean) < 10:
            print('  %-20s %7.2f %8.3f %7s %8s %9s %9s  CONTAMINATED (started at floor)'
                  % (tag, W[2], W[4], '-', '-', '-', '-'))
            continue
        t = np.array([r['t'] for r in clean]); z = np.array([r['z'] for r in clean])
        p = np.degrees([r['pitch'] for r in clean])
        wy = np.degrees([r['wy'] for r in clean])
        dur = t[-1] - t[0]
        if dur < MIN_CLEAN_S:
            print('  %-20s %7.2f %8.3f %7.1f %8s %9s %9s  CONTAMINATED (too short)'
                  % (tag, W[2], W[4], dur, '-', '-', '-'))
            continue
        n = max(5, len(clean) // 4)
        vz = (z[-1] - z[-n]) / (t[-1] - t[-n])
        status = 'clean, truncated at contact' if touched else 'clean'
        print('  %-20s %7.2f %8.3f %7.1f %8.4f %9.2f %9.1f  %s'
              % (tag, W[2], W[4], dur, vz, np.abs(p).max(), np.abs(wy).max(), status))
    print()


def dive(path, label):
    rows = load(path)
    g = [r for r in rows if r['step'] == 1]
    z = np.array([r['z'] for r in g]); t = np.array([r['t'] for r in g])
    p = np.degrees([r['pitch'] for r in g])
    x = np.array([r['x'] for r in g]); y = np.array([r['y'] for r in g])
    depth = -z
    hit = np.argmax(depth >= 0.24) if (depth >= 0.24).any() else None
    contact = np.argmax(z <= FLOOR_Z) if (z <= FLOOR_Z).any() else None
    print('=== %s ===' % label)
    print('  start z / depth      : %.4f m / %.4f m' % (z[0], depth[0]))
    print('  depth >= 0.24 m at   : %s'
          % ('t+%.2f s' % (t[hit] - t[0]) if hit is not None else 'NEVER'))
    if contact is not None:
        print('  floor contact at     : t+%.2f s  (data after this is discarded)'
              % (t[contact] - t[0]))
        k = contact
    else:
        print('  floor contact        : none')
        k = len(g) - 1
    print('  clean window         : %.1f s of the 20 s budget' % (t[k] - t[0]))
    print('  depth at end of clean: %.3f m' % depth[k])
    print('  pitch max (clean)    : %.2f deg   final %.2f deg' % (np.abs(p[:k + 1]).max(), p[k]))
    print('  lateral excursion    : %.3f m' % np.hypot(x[:k + 1] - x[0], y[:k + 1] - y[0]).max())
    ok = hit is not None and (contact is None or hit < contact)
    print('  VERDICT              : %s'
          % ('PASS - threshold crossed %s floor contact'
             % ('well before' if ok else 'AFTER') if hit is not None else 'FAIL'))
    print()


if __name__ == '__main__':
    print('=' * 74)
    print('B  equal split (raw geometry, worst-case coupling)')
    print('=' * 74)
    report('B_equal.csv', 'B')
    print('=' * 74)
    print("B' damped-pinv split (what thruster_allocator actually emits)")
    print('=' * 74)
    report('B_pinv.csv', "B'")
    print('=' * 74)
    print('C  individual vertical thrusters at -5 N')
    print('=' * 74)
    report('C_all.csv', 'C')
    print('=' * 74)
    print('E  DIVE-equivalent, 20 s sim from shallowest submerged start')
    print('=' * 74)
    dive('E_pinv14.csv', 'E1 allocator split Fz=-14 N')
    dive('E_pinv14_rep.csv', 'E3 allocator split Fz=-14 N (repeat)')
    dive('E_equal14.csv', 'E2 equal split Fz=-14 N (worst-case coupling)')
