#!/usr/bin/env python3
"""Auxiliary M7 odom trace recorder — subscribes to /hydroships/odom and writes
roll/pitch/yaw (degrees) + altitude (m) over BOTH wall clock and sim time to CSV.

This is NOT one of the P2-GUI verification tools (those are pure UDP and do not
record odom content); it exists only to quantify the sustained-yaw roll/pitch
spike that p2-gui-probe.py triggers, because /hydroships/odom is the source of
truth for ROV attitude. Run until killed (or --duration N seconds)."""
import argparse, csv, math, time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def _rpy(q):
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


class OdomTrace(Node):
    def __init__(self, args):
        super().__init__('m7_odom_trace')
        self.t0 = time.monotonic()
        self.sim0 = None
        self.rows = []
        self.duration = args.duration
        self.out = args.output
        self.sub = self.create_subscription(Odometry, '/hydroships/odom',
                                            self._on_odom, 50)

    def _on_odom(self, msg):
        now_sim = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.sim0 is None:
            self.sim0 = now_sim
        roll, pitch, yaw = _rpy(msg.pose.pose.orientation)
        row = {
            'wall_t': round(time.monotonic() - self.t0, 3),
            'sim_t': round(now_sim - self.sim0, 3),
            'roll_deg': round(math.degrees(roll), 3),
            'pitch_deg': round(math.degrees(pitch), 3),
            'yaw_deg': round(math.degrees(yaw), 3),
            'depth_m': round(msg.pose.pose.position.z, 3),
            'vx': round(msg.twist.twist.linear.x, 4),
            'vy': round(msg.twist.twist.linear.y, 4),
            'vz': round(msg.twist.twist.linear.z, 4),
        }
        self.rows.append(row)
        if self.duration and row['wall_t'] >= self.duration:
            self._flush_and_stop()

    def _flush_and_stop(self):
        self._write()
        self.get_logger().info(f'm7_odom_trace: wrote {len(self.rows)} rows to {self.out}')
        rclpy.shutdown()

    def _write(self):
        with open(self.out, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=[
                'wall_t','sim_t','roll_deg','pitch_deg','yaw_deg',
                'depth_m','vx','vy','vz'])
            w.writeheader()
            w.writerows(self.rows)


def main():
    ap = argparse.ArgumentParser(description='M7 odom trace recorder')
    ap.add_argument('--duration', type=float, default=0.0,
                    help='Stop after this many wall seconds (0 = run until killed).')
    ap.add_argument('--output', default='/tmp/m7_odom_trace.csv')
    args = ap.parse_args()
    rclpy.init()
    node = OdomTrace(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._flush_and_stop()
    finally:
        try:
            node._write()
        except Exception:
            pass
        node.destroy_node()


if __name__ == '__main__':
    main()
