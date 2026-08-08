#!/usr/bin/env python3
"""P0-1d open-loop characterization driver (post buoyancy/trim correction).

Publishes a step schedule to /hydroships/thruster_N/thrust and logs pose AND
twist in sim time. Lives in the scratchpad; touches no repository file.

usage: driver.py <out.csv> <schedule-name>
"""
import math
import sys

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64


# B: equal thrust on the three vertical thrusters (raw geometry coupling).
#    sum of their x positions = -0.1929 m -> parasitic My = 0.1929 * f
def _equal(fz):
    f = fz / 3.0
    return {1: round(f, 3), 2: round(f, 3), 6: round(f, 3)}


# B': damped pseudo-inverse split actually used by thruster_allocator
#     (build_damped_pinv(tam, 0.1) @ [0,0,Fz,0,0,0]).
def _pinv(fz):
    return {1: round(0.4607 * fz, 3), 2: round(0.4607 * fz, 3),
            6: round(0.0728 * fz, 3)}


def _steps(levels, split, settle=6.0, on=16.0, rest=8.0):
    s = [(settle, {})]
    for L in levels:
        s.append((on, split(-L)))
        s.append((rest, {}))
    return s


SCHEDULES = {
    'A_trim': [(40.0, {})],
    'B_equal': _steps([5.0, 10.0, 14.0], _equal),
    'B_pinv': _steps([5.0, 10.0, 14.0], _pinv),
    'C_t1': _steps([5.0], lambda f: {1: f}, on=14.0),
    'C_t2': _steps([5.0], lambda f: {2: f}, on=14.0),
    'C_t6': _steps([5.0], lambda f: {6: f}, on=14.0),
    'C_all': (_steps([5.0], lambda f: {1: f}, on=14.0)
              + _steps([5.0], lambda f: {2: f}, on=14.0, settle=2.0)
              + _steps([5.0], lambda f: {6: f}, on=14.0, settle=2.0)),
    'E_dive_equal_14': [(3.0, {}), (20.0, _equal(-14.0)), (4.0, {})],
    'E_dive_pinv_14': [(3.0, {}), (20.0, _pinv(-14.0)), (4.0, {})],
    'E_dive_pinv_7': [(3.0, {}), (20.0, _pinv(-7.0)), (4.0, {})],
}


class Driver(Node):
    def __init__(self, path, schedule):
        super().__init__('p0_1d_driver')
        self.pubs = {i: self.create_publisher(
            Float64, '/hydroships/thruster_%d/thrust' % i, 10) for i in range(1, 7)}
        self.create_subscription(Odometry, '/hydroships/odom', self.on_odom, 50)
        self.create_subscription(Clock, '/clock', self.on_clock, 50)
        self.sched = list(schedule)
        self.f = open(path, 'w')
        self.f.write('t,step,cmd1,cmd2,cmd3,cmd4,cmd5,cmd6,'
                     'x,y,z,roll,pitch,yaw,vx,vy,vz,wx,wy,wz\n')
        self.t = None
        self.t_step = None
        self.idx = 0
        self.cmd = {}
        self.odom = None
        self.done = False
        self.create_timer(0.02, self.tick)

    def on_clock(self, msg):
        self.t = msg.clock.sec + msg.clock.nanosec * 1e-9

    def on_odom(self, msg):
        self.odom = msg

    def tick(self):
        if self.t is None or self.odom is None or self.done:
            return
        if self.t_step is None:
            self.t_step = self.t
            self.cmd = self.sched[0][1]
            self.get_logger().info('step 0: %s' % self.cmd)
        if self.t - self.t_step >= self.sched[self.idx][0]:
            self.idx += 1
            if self.idx >= len(self.sched):
                for i in range(1, 7):
                    self.pubs[i].publish(Float64(data=0.0))
                self.f.flush()
                self.done = True
                self.get_logger().info('SCHEDULE COMPLETE')
                return
            self.t_step = self.t
            self.cmd = self.sched[self.idx][1]
            self.get_logger().info('step %d: %s' % (self.idx, self.cmd))

        for i in range(1, 7):
            self.pubs[i].publish(Float64(data=float(self.cmd.get(i, 0.0))))

        p = self.odom.pose.pose.position
        q = self.odom.pose.pose.orientation
        tw = self.odom.twist.twist
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1, min(1, 2 * (q.w * q.y - q.z * q.x))))
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.f.write('%.4f,%d,%s,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,'
                     '%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n' % (
                         self.t, self.idx,
                         ','.join('%.3f' % self.cmd.get(i, 0.0) for i in range(1, 7)),
                         p.x, p.y, p.z, roll, pitch, yaw,
                         tw.linear.x, tw.linear.y, tw.linear.z,
                         tw.angular.x, tw.angular.y, tw.angular.z))


def main():
    rclpy.init()
    n = Driver(sys.argv[1], SCHEDULES[sys.argv[2]])
    while rclpy.ok() and not n.done:
        rclpy.spin_once(n, timeout_sec=0.1)
    n.f.close()
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
