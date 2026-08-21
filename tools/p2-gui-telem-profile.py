#!/usr/bin/env python3
"""P2 GUI telemetry rate profiler — receive UDP telemetry on port 14551 and
compute rate statistics.

Usage:
  # Profile for 15s with default target rate 10Hz
  python3 p2-gui-telem-profile.py --duration 15

  # Specify custom port and expected rate
  python3 p2-gui-telem-profile.py --port 14551 --expected-hz 10 --duration 15

Output: JSON with packet count, intervals, min/median/max Hz, and percentage
of 1-second windows that meet the rate target.

Does NOT require ROS — pure UDP receiver.
"""

import argparse
import json
import socket
import statistics
import time


TELEM_PORT_DEFAULT = 14551
EXPECTED_HZ_DEFAULT = 10.0
DURATION_DEFAULT = 15.0


def main():
    ap = argparse.ArgumentParser(description='P2 GUI telemetry rate profiler')
    ap.add_argument('--port', type=int, default=TELEM_PORT_DEFAULT)
    ap.add_argument('--expected-hz', type=float, default=EXPECTED_HZ_DEFAULT)
    ap.add_argument('--duration', type=float, default=DURATION_DEFAULT)
    ap.add_argument('--output', type=str, default=None,
                    help='Write results JSON to this file')
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('0.0.0.0', args.port))
    sock.settimeout(0.1)

    timestamps = []
    start = time.monotonic()
    deadline = start + args.duration

    print(f'Listening on port {args.port} for {args.duration}s '
          f'(expected {args.expected_hz} Hz)...')

    while time.monotonic() < deadline:
        try:
            data, _ = sock.recvfrom(4096)
            now = time.monotonic()
            timestamps.append(now)
            parsed = json.loads(data)
            if len(timestamps) % 10 == 0:
                print(f'  {len(timestamps)} packets received', flush=True)
        except socket.timeout:
            continue

    sock.close()

    if len(timestamps) < 2:
        result = {
            'packets': len(timestamps),
            'duration_s': args.duration,
            'effective_hz': 0.0 if len(timestamps) == 0 else None,
            'error': 'insufficient packets for analysis',
        }
    else:
        intervals = [t2 - t1 for t1, t2 in zip(timestamps, timestamps[1:])]
        hz_values = [1.0 / dt if dt > 0 else float('inf') for dt in intervals]
        effective_hz = len(timestamps) / (timestamps[-1] - timestamps[0])

        # Per-second window analysis
        windows = {}
        for ts in timestamps:
            sec = int(ts - start)
            windows.setdefault(sec, 0)
            windows[sec] += 1

        under_threshold = sum(1 for c in windows.values() if c < args.expected_hz * 0.7)
        window_count = len(windows)

        result = {
            'packets': len(timestamps),
            'duration_s': round(timestamps[-1] - timestamps[0], 3),
            'effective_hz': round(effective_hz, 2),
            'expected_hz': args.expected_hz,
            'interval_ms': {
                'min': round(min(intervals) * 1000, 2),
                'median': round(statistics.median(intervals) * 1000, 2),
                'max': round(max(intervals) * 1000, 2),
                'mean': round(statistics.mean(intervals) * 1000, 2),
                'stdev': round(statistics.stdev(intervals) * 1000, 2)
                                  if len(intervals) > 1 else 0.0,
            },
            'windows_1s': dict(sorted(windows.items())),
            'windows_under_70pct_target': under_threshold,
            'window_count': window_count,
            'pct_windows_meet_target': round(
                (window_count - under_threshold) / window_count * 100, 1)
                                       if window_count > 0 else 0.0,
        }

    print(json.dumps(result, indent=2))
    if args.output:
        with open(args.output, 'w') as f:
            json.dump(result, f, indent=2)
        print(f'Results saved to {args.output}')


if __name__ == '__main__':
    main()
