#!/usr/bin/env python3
"""P0-2.1/P0-2.2a/P0-2.2b APPROACH_QR instrumentation recorder — SUBSCRIBE ONLY.

Fork of recorder.py (P0-1e). Publishes nothing. Adds the QR detection chain
(qr_result, qr_offset), the FSM's own commanded output (manual/cmd), FSM
state (parsed from /rosout, since mission_fsm.py has no dedicated state
topic), the payload_pose ground truth (P0-2.2a — needed as the
counterfactual reference for the P0-2.2 causal gates, see
docs/P0-2-2-SPEC.md), and odom twist velocity vx/vy (P0-2.2b — _goto_xy()
damps using self.vx/self.vy read from odom.twist, not a position
finite-difference; reduce_approach_qr.py's Gate-3 counterfactual must use
the same velocity source the real controller consumed) on top of the
odom/depth signals already proven by P0-1e.

recorder.py itself is left untouched (frozen P0-1 evidence, tag
p0-1-baseline) — this is a fork, not an extension in place.

usage: recorder_qr.py <out.csv> <duration_sim_s>
"""
import math
import re
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64, String

FSM_TRANSITION_RE = re.compile(r'^\[FSM\]\s+(\S+)\s+->\s+(\S+)$')


class RecorderQR(Node):
    def __init__(self, path, duration):
        super().__init__('p0_2_1_recorder_qr')
        self.duration = float(duration)
        self.t0 = None
        self.t = None
        self.done = False

        self.odom = None
        self.cmd = Twist()
        self.sp = float('nan')
        self.depth = float('nan')
        self.fsm_state = ''
        self.qr_result = ''
        self.qr_ex = float('nan')
        self.qr_ey = float('nan')
        self.qr_size = float('nan')
        self.qr_frame = ''
        self.qr_t = None
        self.payload_x = float('nan')
        self.payload_y = float('nan')
        self.payload_z = float('nan')

        self.create_subscription(Clock, '/clock', self.on_clock, 50)
        self.create_subscription(Odometry, '/hydroships/odom', self.on_odom, 50)
        self.create_subscription(Twist, '/hydroships/manual/cmd', self.on_cmd, 50)
        self.create_subscription(Float64, '/hydroships/setpoint/depth', self.on_sp, 10)
        self.create_subscription(Float64, '/hydroships/depth', self.on_depth, 10)
        self.create_subscription(String, '/hydroships/qr_result', self.on_qr_result, 10)
        self.create_subscription(PointStamped, '/hydroships/qr_offset', self.on_qr_offset, 10)
        self.create_subscription(Log, '/rosout', self.on_rosout, 50)
        # payload_spawner menerbitkan SEKALI dengan QoS latched (TRANSIENT_LOCAL) —
        # subscriber harus pakai durability sama, persis seperti mission_fsm.py:234-236,
        # supaya tetap dapat pesan walau recorder start belakangan (recorder.py pattern:
        # start setelah delay tetap, bisa lebih lambat dari waktu spawn payload).
        self.create_subscription(
            PointStamped, '/hydroships/payload_pose', self.on_payload_pose,
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

        self.f = open(path, 'w')
        self.f.write('t,fsm_state,sp_depth,depth,x,y,z,yaw,vx,vy,cmd_fx,cmd_fy,'
                     'qr_result,qr_ex,qr_ey,qr_size,qr_frame,qr_age,'
                     'payload_x,payload_y,payload_z\n')
        self.create_timer(0.1, self.tick)

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

    def on_qr_result(self, m):
        self.qr_result = m.data

    def on_qr_offset(self, m):
        self.qr_ex = m.point.x
        self.qr_ey = m.point.y
        self.qr_size = m.point.z
        self.qr_frame = m.header.frame_id
        self.qr_t = self.t

    def on_rosout(self, m):
        if m.name != 'mission_fsm':
            return
        match = FSM_TRANSITION_RE.match(m.msg)
        if match:
            self.fsm_state = match.group(2)

    def on_payload_pose(self, m):
        self.payload_x = m.point.x
        self.payload_y = m.point.y
        self.payload_z = m.point.z

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
        v = self.odom.twist.twist.linear
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        c = self.cmd
        qr_age = (self.t - self.qr_t) if self.qr_t is not None else float('nan')
        qr_result = self.qr_result.replace(',', ';').replace('\n', ' ')
        self.f.write('%.4f,%s,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.4f,%.4f,'
                     '%s,%.5f,%.5f,%.5f,%s,%.4f,'
                     '%.5f,%.5f,%.5f\n' % (
                         self.t, self.fsm_state, self.sp, self.depth, p.x, p.y, p.z, yaw,
                         v.x, v.y,
                         c.linear.x, c.linear.y,
                         qr_result, self.qr_ex, self.qr_ey, self.qr_size,
                         self.qr_frame, qr_age,
                         self.payload_x, self.payload_y, self.payload_z))


def main():
    rclpy.init()
    n = RecorderQR(sys.argv[1], sys.argv[2])
    while rclpy.ok() and not n.done:
        rclpy.spin_once(n, timeout_sec=0.1)
    n.f.close()
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
