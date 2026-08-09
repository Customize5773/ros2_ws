#!/usr/bin/env python3
"""P0-2.3 preliminary: extract GRAB-time positioning accuracy from EXISTING
run logs -- no new simulator run needed.

mission_fsm.py's _gripper_align_txt() (L488-503) already computes and logs
gripper_err/base_err (ground-truth XY distance from gripper/base_link to the
true payload position) at every GRAB decision (both exit paths log it: L600
and L614). This script just parses that existing text out of <tag>.log
files already produced by run_approach_qr_smoke.sh / run_approach_qr_battery.sh
-- it does not run anything, does not touch mission_fsm.py, and needs no new
data collection to produce its numbers.

usage: extract_gripper_err.py <data_dir> [tag ...]
  (default tags: Q1 Q2 Q3 Q4 Q5 Q6)
"""
import json
import re
import sys

GRAB_LOG_RE = re.compile(
    r'(QR terpusat \((?:visual servo|jarak XY)\) -> GRAB|Wall \S+ dipilih \(\+15\) \[urutan ke-\d+\])'
    r' \(gripper_err=([\d.]+) m base_err=([\d.]+) m '
    r'\(gripper@([-\d.]+),([-\d.]+) payload@([-\d.]+),([-\d.]+)\)\)'
)


def parse_log(path):
    events = []
    try:
        text = open(path).read()
    except FileNotFoundError:
        return events
    for m in GRAB_LOG_RE.finditer(text):
        trigger, gripper_err, base_err, gx, gy, px, py = m.groups()
        events.append({
            'trigger': trigger,
            'gripper_err_m': float(gripper_err),
            'base_err_m': float(base_err),
            'gripper_xy': [float(gx), float(gy)],
            'payload_xy': [float(px), float(py)],
        })
    return events


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    data_dir = sys.argv[1]
    tags = sys.argv[2:] or ['Q1', 'Q2', 'Q3', 'Q4', 'Q5', 'Q6']

    print('P0-2.3 preliminary: GRAB-time positioning accuracy vs ground truth,')
    print('parsed from EXISTING mission_fsm log lines (_gripper_align_txt()).')
    print('No new simulator run. Source: <tag>.log from the P0-2.2b battery.\n')

    all_events = {}
    errs = []
    for tag in tags:
        events = parse_log('%s/%s.log' % (data_dir, tag))
        all_events[tag] = events
        if not events:
            print('[%s] no GRAB event found in log' % tag)
            continue
        for i, e in enumerate(events):
            errs.append(e['gripper_err_m'])
            print('[%s] GRAB #%d: gripper_err=%.3f m  base_err=%.3f m  (%s)'
                  % (tag, i + 1, e['gripper_err_m'], e['base_err_m'], e['trigger']))

    if errs:
        print('\nAcross %d GRAB events: gripper_err min=%.3f max=%.3f mean=%.3f m'
              % (len(errs), min(errs), max(errs), sum(errs) / len(errs)))
        print('approach_tol (source default) = 0.06 m -- gripper_err is EXPECTED to be')
        print('near/under this by construction whenever the XY-tolerance exit path fires,')
        print('since the tolerance gate is computed against the same gripper-corrected')
        print('target. This number describes ground-truth-referenced placement, not')
        print('QR-only precision -- see docs/P0-2-3-SPEC.md S3 for that distinction.')

    with open('%s/P0-2-3-gripper-err.json' % data_dir, 'w') as f:
        json.dump(all_events, f, indent=2)
    print('\nFull data written to %s/P0-2-3-gripper-err.json' % data_dir)


if __name__ == '__main__':
    main()
