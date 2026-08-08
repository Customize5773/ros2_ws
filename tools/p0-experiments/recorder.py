#!/usr/bin/env python3
"""P0-1e control-chain recorder — SUBSCRIBE ONLY.

Publishes nothing. Logs the full closed-loop DIVE chain in sim time so the
integration run is observed, not perturbed.

usage: recorder.py <out.csv> <duration_sim_s>
"""
import math
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64


class Recorder(Node):
    def __init__(self, path, duration):
        super().__init__('p0_1e_recorder')
        self.duration = float(duration)
        self.t0 = None
        self.t = None
        self.done = False

        self.odom = None
        self.cmd = Twist()
        self.sp = float('nan')
        self.depth = float('nan')
        self.thr = [float('nan')] * 6

        self.create_subscription(Clock, '/clock', self.on_clock, 50)
        self.create_subscription(Odometry, '/hydroships/odom', self.on_odom, 50)
        self.create_subscription(Twist, '/hydroships/cmd_vel', self.on_cmd, 50)
        self.create_subscription(Float64, '/hydroships/setpoint/depth', self.on_sp, 10)
        self.create_subscription(Float64, '/hydroships/depth', self.on_depth, 10)
        for i in range(1, 7):
            self.create_subscription(
                Float64, '/hydroships/thruster_%d/thrust' % i,
                (lambda k: (lambda m: self.on_thr(k, m)))(i - 1), 10)

        self.f = open(path, 'w')
        self.f.write('t,sp_depth,depth,x,y,z,roll,pitch,yaw,'
                     'fx,fy,fz,mx,my,mz,'
                     'thr1,thr2,thr3,thr4,thr5,thr6\n')
        self.create_timer(0.02, self.tick)

    def on_clock(self, m):
        self.t = m.clock.sec + m.clock.nanosec * 1e-9

    def on_odom(self, m):
        self.odom = m

    def on_cmd(self, m):
        self.cmd = m

    def on_sp(self, m):
        self.sp = m.data

    def on_depth(self, m):
        self.depth = m.data

    def on_thr(self, k, m):
        self.thr[k] = m.data

    def tick(self):
        if self.t is None or self.odom is None or self.done:
            return
        if self.t0 is None:
            self.t0 = self.t
        if self.t - self.t0 >= self.duration:
            self.f.flush()
            self.done = True
            self.get_logger().info('RECORDING COMPLETE (%.1f s sim)' % (self.t - self.t0))
            return
        p = self.odom.pose.pose.position
        q = self.odom.pose.pose.orientation
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1, min(1, 2 * (q.w * q.y - q.z * q.x))))
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        c = self.cmd
        self.f.write('%.4f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,'
                     '%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%s\n' % (
                         self.t, self.sp, self.depth, p.x, p.y, p.z, roll, pitch, yaw,
                         c.linear.x, c.linear.y, c.linear.z,
                         c.angular.x, c.angular.y, c.angular.z,
                         ','.join('%.4f' % v for v in self.thr)))


def main():
    rclpy.init()
    n = Recorder(sys.argv[1], sys.argv[2])
    while rclpy.ok() and not n.done:
        rclpy.spin_once(n, timeout_sec=0.1)
    n.f.close()
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
