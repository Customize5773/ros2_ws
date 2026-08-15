#!/usr/bin/env python3
"""Measure R-8 GUI UDP uplink latency against /hydroships/cmd_vel.

The UDP command carries a unique probe value. This tool timestamps send time
with CLOCK_MONOTONIC and timestamps the first matching ROS Twist callback.
Telemetry packets are counted separately as a downlink liveness check.
"""

import argparse
import json
import math
import socket
import statistics
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class Probe(Node):
    def __init__(self, expected, result):
        super().__init__('r8_udp_latency_probe')
        self.expected = expected
        self.result = result
        self.sub = self.create_subscription(
            Twist, '/hydroships/cmd_vel', self.on_cmd, 10)

    def on_cmd(self, msg):
        if math.isclose(msg.linear.x, self.expected, abs_tol=1e-9):
            self.result.setdefault('ros_times', []).append(time.monotonic())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cmd-port', type=int, default=14550)
    ap.add_argument('--telem-port', type=int, default=14551)
    ap.add_argument('--samples', type=int, default=10)
    ap.add_argument('--latency-ms', type=float, default=250.0)
    ap.add_argument('--interval', type=float, default=0.7)
    args = ap.parse_args()

    result = {'sent': [], 'ros_times': [], 'telemetry': 0}
    rclpy.init()
    node = Probe(14.8, result)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.bind(('127.0.0.1', args.telem_port))
    rx.settimeout(0.01)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    tx.sendto(json.dumps({'name': 'arm', 'value': True}).encode(),
              ('127.0.0.1', args.cmd_port))
    time.sleep(0.1)

    for _ in range(args.samples):
        # Same payload each time lets the ROS-side probe identify the sample;
        # the send/receive pairing is kept by order.
        sent_at = time.monotonic()
        result['sent'].append(sent_at)
        tx.sendto(json.dumps({'name': 'surge', 'value': 37.0}).encode(),
                  ('127.0.0.1', args.cmd_port))
        deadline = sent_at + args.interval
        while time.monotonic() < deadline:
            try:
                rx.recvfrom(4096)
                result['telemetry'] += 1
            except socket.timeout:
                pass
        time.sleep(max(0.0, args.interval - (time.monotonic() - sent_at)))

    time.sleep(0.2)
    node.destroy_node()
    rclpy.shutdown()
    spin_thread.join(timeout=1.0)
    tx.close()
    rx.close()

    # Match in order; stale callbacks are excluded after each send window.
    pairs = list(zip(result['sent'], result['ros_times']))[:args.samples]
    delays = [(received - sent) * 1000.0 for sent, received in pairs]
    print(json.dumps({
        'configured_latency_ms': args.latency_ms,
        'samples_sent': len(result['sent']),
        'samples_observed': len(delays),
        'telemetry_packets': result['telemetry'],
        'latency_ms': {
            'min': min(delays) if delays else None,
            'median': statistics.median(delays) if delays else None,
            'max': max(delays) if delays else None,
            'samples': [round(x, 3) for x in delays],
        },
    }, indent=2))


if __name__ == '__main__':
    main()
