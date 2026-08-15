#!/usr/bin/env python3
"""P2 combined experiment runner — single process that subscribes to sensors,
sends UDP commands, and records everything to CSV. Avoids cross-process
DDS discovery timing issues.

Usage:
  python3 p2-experiment.py --mode sustained --axis yaw --value 100 --duration 10 --output /tmp/p2-data/exp1.csv

  python3 p2-experiment.py --mode step --axis surge --value 50 --duration 5 --output /tmp/p2-data/exp2.csv

  python3 p2-experiment.py --mode pulsed --axis yaw --value 100 --on 0.5 --off 0.5 --cycles 10 --output /tmp/p2-data/exp3.csv
"""
import argparse
import csv
import json
import math
import os
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from rosgraph_msgs.msg import Clock


def _yaw_rpy(q):
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


class ExperimentNode(Node):
    def __init__(self, output_path):
        super().__init__('p2_experiment')
        self.f = open(output_path, 'w', newline='')
        self.writer = csv.writer(self.f)
        self.writer.writerow([
            't', 'roll_deg', 'pitch_deg', 'yaw_deg',
            'x', 'y', 'z', 'vx', 'vy', 'vz',
            'cmd_fx', 'cmd_fy', 'cmd_fz', 'cmd_mz',
            't1', 't2', 't3', 't4', 't5', 't6',
        ])

        self.odom = None
        self.cmd = Twist()
        self.thr = [0.0] * 7  # index 1-6
        self.sim_t = 0.0
        self.wall_t0 = time.monotonic()

        self.create_subscription(Clock, '/clock', self._on_clock, 50)
        self.create_subscription(Odometry, '/hydroships/odom', self._on_odom, 50)
        self.create_subscription(Twist, '/hydroships/cmd_vel', self._on_cmd, 50)
        for i in range(1, 7):
            self.create_subscription(Float64, f'/hydroships/thruster_{i}/thrust',
                                     self._make_thr_cb(i), 10)

        self.create_timer(0.01, self._tick)  # 100 Hz record rate

    def _on_clock(self, m):
        self.sim_t = m.clock.sec + m.clock.nanosec * 1e-9

    def _on_odom(self, m):
        self.odom = m

    def _on_cmd(self, m):
        self.cmd = m

    def _make_thr_cb(self, k):
        def cb(msg):
            self.thr[k] = msg.data
        return cb

    def _tick(self):
        if self.odom is None:
            return
        p = self.odom.pose.pose.position
        q = self.odom.pose.pose.orientation
        roll, pitch, yaw = _yaw_rpy(q)
        v = self.odom.twist.twist.linear
        c = self.cmd
        self.writer.writerow([
            f'{self.sim_t:.4f}',
            f'{math.degrees(roll):.4f}', f'{math.degrees(pitch):.4f}', f'{math.degrees(yaw):.4f}',
            f'{p.x:.5f}', f'{p.y:.5f}', f'{p.z:.5f}',
            f'{v.x:.5f}', f'{v.y:.5f}', f'{v.z:.5f}',
            f'{c.linear.x:.4f}', f'{c.linear.y:.4f}', f'{c.linear.z:.4f}', f'{c.angular.z:.4f}',
            f'{self.thr[1]:.4f}', f'{self.thr[2]:.4f}', f'{self.thr[3]:.4f}',
            f'{self.thr[4]:.4f}', f'{self.thr[5]:.4f}', f'{self.thr[6]:.4f}',
        ])
        self.f.flush()


def send_udp(sock, target, name, value):
    msg = json.dumps({'name': name, 'value': value}).encode('utf-8')
    sock.sendto(msg, target)


def run_experiment(args):
    rclpy.init()
    node = ExperimentNode(args.output)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    # Wait for DDS discovery and sim to settle
    time.sleep(2.0)
    print(f'Experiment: sim_t={node.sim_t:.1f}s, armed=False')

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = ('127.0.0.1', args.cmd_port)

    # Arm
    send_udp(sock, target, 'arm', True)
    time.sleep(0.5)
    print(f'After arm: sim_t={node.sim_t:.1f}s, armed=True')

    # Wait for baseline
    baseline_t = node.sim_t
    wall_start = time.monotonic()
    print(f'Baseline time: {baseline_t:.1f}s. Waiting {args.pre_wait}s before command...')
    while node.sim_t - baseline_t < args.pre_wait:
        time.sleep(0.1)

    cmd_start_t = node.sim_t
    cmd_start_wall = time.monotonic()

    if args.mode == 'sustained':
        print(f'Sending {args.axis}={args.value} (sustained {args.duration}s)')
        send_udp(sock, target, args.axis, args.value)
        while node.sim_t - cmd_start_t < args.duration:
            time.sleep(0.1)
        send_udp(sock, target, args.axis, 0.0)
        print(f'Sent {args.axis}=0')

    elif args.mode == 'step':
        print(f'Sending {args.axis}={args.value} (step, {args.duration}s)')
        send_udp(sock, target, args.axis, args.value)
        while node.sim_t - cmd_start_t < args.duration:
            time.sleep(0.1)
        send_udp(sock, target, args.axis, 0.0)
        print(f'Sent {args.axis}=0')

    elif args.mode == 'pulsed':
        print(f'Sending {args.axis} pulses ({args.cycles}x {args.on}s on / {args.off}s off)')
        for i in range(args.cycles):
            send_udp(sock, target, args.axis, args.value)
            while node.sim_t - cmd_start_t < (i + 1) * (args.on + args.off):
                time.sleep(0.05)
            send_udp(sock, target, args.axis, 0.0)

    elif args.mode == 'yaw_ramp':
        print(f'Sending {args.axis} ramp 0→{args.value}')
        for pct in range(0, int(args.value) + 1, 10):
            send_udp(sock, target, args.axis, float(pct))
            while node.sim_t - cmd_start_t < (pct / 10.0):
                time.sleep(0.05)
        time.sleep(2.0)
        send_udp(sock, target, args.axis, 0.0)

    # Post-command decay
    while node.sim_t - cmd_start_t < args.duration + args.post_wait:
        time.sleep(0.1)

    print(f'Experiment done. sim_t={node.sim_t:.1f}s, '
          f'elapsed_wall={time.monotonic() - cmd_start_wall:.1f}s')

    # Also send stop
    send_udp(sock, target, 'stop', None)
    time.sleep(0.2)
    sock.close()

    node.f.close()
    rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser(description='P2 experiment runner')
    ap.add_argument('--mode', choices=['sustained', 'step', 'pulsed', 'yaw_ramp'],
                    required=True)
    ap.add_argument('--axis', choices=['surge', 'sway', 'yaw', 'heave'],
                    default='yaw')
    ap.add_argument('--value', type=float, default=100.0)
    ap.add_argument('--duration', type=float, default=10.0)
    ap.add_argument('--pre-wait', type=float, default=2.0,
                    help='Seconds of baseline recording before command (sim time)')
    ap.add_argument('--post-wait', type=float, default=5.0,
                    help='Seconds of decay recording after command (sim time)')
    ap.add_argument('--on', type=float, default=0.5)
    ap.add_argument('--off', type=float, default=0.5)
    ap.add_argument('--cycles', type=int, default=10)
    ap.add_argument('--cmd-port', type=int, default=14550)
    ap.add_argument('--output', type=str, required=True)
    args = ap.parse_args()

    os.makedirs(os.path.dirname(args.output) or '.', exist_ok=True)
    run_experiment(args)
    print(f'Data saved to {args.output}')


if __name__ == '__main__':
    main()
