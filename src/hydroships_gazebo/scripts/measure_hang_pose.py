#!/usr/bin/env python3
"""measure_hang_pose — ukur pose payload SAAT MENG GANTUNG setelah AUTO_RELEASE.

Subscribe /hydroships/world_pose_tf (frame 'payload') + /hydroships/gripper/state.
Tiap kali status berubah closed -> open (detach), catat pose payload selama
berikutnya (payload lepas & bersandar di hook), lalu hitung offset terhadap
tip hook utk wall yg aktif. Log juga ROLL PLAT (quaternion -> rpy) utk
menilai apakah plat menggantung VERTIKAL (benar) atau miring (inaccurate).
Pasif: hanya subscribe, tak publish apa pun.

usage: ros2 run hydroships_gazebo measure_hang_pose  (jalankan bareng mission)
"""
import math
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

WALL_TIP = {  # posisi dunia tip hook (wall_face 2.5 - hang_tip_d 0.14)
    'A': (0.0, -2.36), 'B': (0.0, 2.36), 'C': (2.36, 0.0), 'D': (-2.36, 0.0),
}

# Pusat lubang plat dalam frame payload (dari payload_body.obj / slot collision
# z 0.0865..0.10 -> tengah ~0.0933; pakai 0.0933, nilai hang_hole_dx di FSM).
HOLE_LOCAL = (0.0, 0.0, 0.0933)


def quat_to_rpy(q):
    phi = math.atan2(2*(q.w*q.x + q.y*q.z), 1 - 2*(q.x*q.x + q.y*q.y))
    theta = math.asin(max(-1.0, min(1.0, 2*(q.w*q.y - q.z*q.x))))
    psi = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
    return math.degrees(phi), math.degrees(theta), math.degrees(psi)


def quat_rot(q, v):
    """Rotasi vektor v oleh quaternion q (formula v' = v + 2q×(q×v + wv))."""
    u = (q.x, q.y, q.z)
    s = q.w
    def cross(a, b):
        return (a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2],
                a[0]*b[1] - a[1]*b[0])
    def axpy(k, a, b):
        return (k*a[0]+b[0], k*a[1]+b[1], k*a[2]+b[2])
    t = cross(u, v)
    t = axpy(s, v, t)
    t = cross(u, t)
    return axpy(2.0, t, v)


def quat_to_rpy(q):
    w, x, y, z = q.w, q.x, q.y, q.z
    phi = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    theta = math.asin(max(-1.0, min(1.0, 2*(w*y - z*x))))
    psi = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return math.degrees(phi), math.degrees(theta), math.degrees(psi)


class HangPoseMeasurer(Node):
    def __init__(self):
        super().__init__('measure_hang_pose')
        self.payload = None      # (x, y, z, q) pose payload dunia
        self.grip = None
        self.samples = []        # (t, x, y, z, roll, pitch, yaw)
        self.attach_ts = None
        self.detach_ts = None
        self.create_subscription(TFMessage, '/hydroships/world_pose_tf',
                                 self._on_tf, 10)
        self.create_subscription(String, '/hydroships/gripper/state',
                                 self._on_grip, 10)
        self.create_timer(0.5, self._tick)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_tf(self, msg):
        for t in msg.transforms:
            if t.child_frame_id == 'payload':
                tr = t.transform.translation
                qt = t.transform.rotation
                self.payload = (tr.x, tr.y, tr.z, qt)
                return

    def _on_grip(self, msg):
        # /hydroships/gripper/state: 'closed' = attached, 'open' = detached
        if msg.data == 'closed' and self.grip != 'closed':
            self.attach_ts = self._now()
        elif msg.data == 'open' and self.grip == 'closed':
            self.detach_ts = self._now()
        self.grip = msg.data

    def _tick(self):
        if self.detach_ts is None or self.payload is None:
            return
        since = self._now() - self.detach_ts
        if 1.0 <= since <= 3.0:      # window stelah detach, plat sudah settle
            x, y, z, q = self.payload
            hx, hy, hz = quat_rot(q, HOLE_LOCAL)
            # lubang dunia = origin payload + rotasi offset lubang
            self.samples.append((since, x, y, z, x+hx, y+hy, z+hz)
                                + quat_to_rpy(q))
        elif since > 3.0 and self.samples:
            self._report()
            self.detach_ts = None
            self.samples = []

    def _report(self):
        n = len(self.samples)
        x = sum(s[1] for s in self.samples) / n
        y = sum(s[2] for s in self.samples) / n
        z = sum(s[3] for s in self.samples) / n
        hx = sum(s[4] for s in self.samples) / n
        hy = sum(s[5] for s in self.samples) / n
        roll = max(s[7] for s in self.samples)
        pitch = max(s[8] for s in self.samples)
        # tip terdekat = argmin jarak planar dari LUBANG
        wall, tip = min(WALL_TIP.items(),
                        key=lambda kv: (kv[1][0]-hx)**2 + (kv[1][1]-hy)**2)
        off_origin = math.hypot(x - tip[0], y - tip[1])
        off_hole = math.hypot(hx - tip[0], hy - tip[1])
        print('HANG-POSE wall=%s payload=(%.3f, %.3f, %.3f) '
              'hole=(%.3f, %.3f, %.3f) tip=(%.2f, %.2f) '
              'hole_off_xy=%.1fmm origin_off_xy=%.1fmm '
              'roll_max=%.1f° pitch_max=%.1f° (n=%d)'
              % (wall, x, y, z, hx, hy, z, tip[0], tip[1],
                 off_hole*1000.0, off_origin*1000.0, roll, pitch, n),
              flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = HangPoseMeasurer()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
