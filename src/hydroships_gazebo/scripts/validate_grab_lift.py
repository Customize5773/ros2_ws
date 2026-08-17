#!/usr/bin/env python3
"""validate_grab_lift — validasi runtime M5: payload benar-benar terangkat
& ketahanan grip (DetachableJoint tak lepas saat gangguan gaya).

Jalankan SETELAH sim + mission FSM sudah hidup (mis.
`ros2 launch hydroships_bringup hydroships_mission.launch.py
start_state:=GRAB headless:=true`). Skrip ini pasif: hanya subscribe,
sekali kirim satu gangguan gaya singkat via /hydroships/cmd_vel, lalu cetak
PASS/FAIL. Bukan pytest -- lihat docs/STATUS.md untuk hasil run.

Kriteria:
  1. TERANGKAT   : setelah gripper_status=='attached', selama ROV naik
                   (delta_z odom > lift_min_m), pose payload (dari TF
                   /hydroships/world_pose_tf, frame 'payload') ikut naik
                   dengan delta (odom.z - payload.z) tetap dlm toleransi.
  2. TAK SLIP    : delta ROV<->payload tetap ~konstan (dlm toleransi yg sama)
                   sesaat setelah gangguan gaya singkat dikirim, dan
                   gripper_status tetap 'attached'.
"""

import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


class GrabLiftValidator(Node):
    def __init__(self):
        super().__init__('validate_grab_lift')
        p = self.declare_parameter
        p('delta_tol_m', 0.03)       # toleransi delta ROV<->payload (m)
        p('lift_min_m', 0.05)        # kenaikan odom.z minimum utk cek "terangkat"
        p('timeout_s', 60.0)         # batas waktu tunggu attached+lift
        p('disturb_after_s', 3.0)    # jeda setelah attached sebelum gangguan gaya
        p('disturb_duration_s', 1.0)
        p('disturb_force', 6.0)      # N (Twist.linear.x sesaat, dipakai sbg wrench)

        self.delta_tol = float(self.get_parameter('delta_tol_m').value)
        self.lift_min = float(self.get_parameter('lift_min_m').value)
        self.timeout_s = float(self.get_parameter('timeout_s').value)
        self.disturb_after = float(self.get_parameter('disturb_after_s').value)
        self.disturb_duration = float(self.get_parameter('disturb_duration_s').value)
        self.disturb_force = float(self.get_parameter('disturb_force').value)

        self.odom_z = None
        self.payload_z = None
        self.gripper_status = None
        self.attached_at = None
        self.z0_odom = None
        self.z0_payload = None
        self.max_delta_drift = 0.0
        self.disturb_sent = False
        self.disturb_done_at = None
        self.result_delta_after_disturb = None
        self.done = False

        self.create_subscription(Odometry, '/hydroships/odom', self._on_odom, 10)
        self.create_subscription(TFMessage, '/hydroships/world_pose_tf',
                                  self._on_world_tf, 10)
        self.create_subscription(String, '/hydroships/gripper/status',
                                  self._on_status, 10)
        self.pub_cmd = self.create_publisher(Twist, '/hydroships/cmd_vel', 10)

        self.t0 = self._now()
        self.create_timer(0.2, self._tick)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_odom(self, msg):
        self.odom_z = msg.pose.pose.position.z

    def _on_world_tf(self, msg):
        for t in msg.transforms:
            if t.child_frame_id == 'payload':
                self.payload_z = t.transform.translation.z
                return

    def _on_status(self, msg):
        if msg.data == 'attached' and self.gripper_status != 'attached':
            self.attached_at = self._now()
            self.z0_odom = self.odom_z
            self.z0_payload = self.payload_z
        self.gripper_status = msg.data

    def _tick(self):
        elapsed = self._now() - self.t0

        if self.attached_at is not None and self.odom_z is not None \
                and self.payload_z is not None:
            delta = abs(self.odom_z - self.payload_z)
            base_delta = abs(self.z0_odom - self.z0_payload) if self.z0_odom is not None else delta
            self.max_delta_drift = max(self.max_delta_drift, abs(delta - base_delta))

            since_attach = self._now() - self.attached_at
            if not self.disturb_sent and since_attach >= self.disturb_after:
                self._send_disturbance()
            if self.disturb_done_at is not None and self.result_delta_after_disturb is None \
                    and self._now() - self.disturb_done_at >= 0.5:
                self.result_delta_after_disturb = abs(delta - base_delta)
                self._finish()
                return

        if elapsed >= self.timeout_s:
            self.get_logger().error('TIMEOUT: gripper tak pernah "attached" atau data tak lengkap.')
            self._report(lifted=False, no_slip=False, timed_out=True)
            self.done = True

    def _send_disturbance(self):
        self.disturb_sent = True
        self.get_logger().info('Mengirim gangguan gaya singkat (%.1f N, %.1fs)...'
                                % (self.disturb_force, self.disturb_duration))
        msg = Twist()
        msg.linear.x = self.disturb_force
        self.pub_cmd.publish(msg)
        self.create_timer(self.disturb_duration, self._stop_disturbance)

    def _stop_disturbance(self):
        self.pub_cmd.publish(Twist())
        self.disturb_done_at = self._now()

    def _finish(self):
        lift_delta_z = (self.odom_z - self.z0_odom) if self.z0_odom is not None else 0.0
        lifted = lift_delta_z >= self.lift_min and self.max_delta_drift <= self.delta_tol
        no_slip = (self.gripper_status == 'attached'
                   and self.result_delta_after_disturb is not None
                   and self.result_delta_after_disturb <= self.delta_tol)
        self._report(lifted=lifted, no_slip=no_slip, timed_out=False,
                     lift_delta_z=lift_delta_z)
        self.done = True

    def _report(self, lifted, no_slip, timed_out, lift_delta_z=None):
        print('--- validate_grab_lift hasil ---', flush=True)
        if timed_out:
            print('TERANGKAT : FAIL (timeout)', flush=True)
            print('TAK SLIP  : FAIL (timeout)', flush=True)
        else:
            print('TERANGKAT : %s (delta_z odom=%.3fm, max drift delta=%.3fm, tol=%.3fm)'
                  % ('PASS' if lifted else 'FAIL', lift_delta_z, self.max_delta_drift, self.delta_tol),
                  flush=True)
            print('TAK SLIP  : %s (delta setelah gangguan=%s, status=%s)'
                  % ('PASS' if no_slip else 'FAIL',
                     ('%.3fm' % self.result_delta_after_disturb)
                     if self.result_delta_after_disturb is not None else 'n/a',
                     self.gripper_status),
                  flush=True)
        self._exit_ok = bool(lifted and no_slip and not timed_out)


def main(args=None):
    rclpy.init(args=args)
    node = GrabLiftValidator()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    ok = getattr(node, '_exit_ok', False)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
