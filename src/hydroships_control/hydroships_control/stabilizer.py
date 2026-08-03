#!/usr/bin/env python3
"""Stabilizer HYDROships (Milestone 2): depth-hold + heading-hold + safety fail-safe.

Membaca odometry, menahan KEDALAMAN (sumbu z) dan HEADING (yaw) memakai PID,
lalu menggabung dengan perintah manual dari pilot/autonomy. Output berupa wrench 
body ke /hydroships/cmd_vel (dikonsumsi thruster_allocator).

Fitur & Proteksi Utama:
- Fail-Safe E-STOP: Default NON-AKTIF untuk kemudahan pengujian simulasi.
- Sanitasi Input/Output: Validasi math.isfinite() ketat untuk cegah NaN/inf merusak PID.
- Watchdog & Jump-Reset: Fallback ke manual jika sensor stale, serta auto-reset PID 
  jika terjadi lonjakan setpoint drastis.
- Telemetri Real-time: Mempublikasikan status diagnostik internal sistem.

Aliran Topik ROS 2:
    /hydroships/odom                  (Odometry)          -> Pengukuran z & yaw
    /hydroships/manual/cmd            (Twist)             -> Perintah manual dari pilot (pass-through)
    /hydroships/setpoint/depth        (Float64)           -> Target kedalaman (m, negatif = dalam)
    /hydroships/setpoint/heading      (Float64)           -> Target yaw (rad)
    /hydroships/estop                 (Bool)              -> Sinyal E-STOP (True = output dipaksa 0)
    => /hydroships/cmd_vel            (Twist)             -> Output wrench ke thruster allocator
    => /hydroships/stabilizer/diag    (Float64MultiArray) -> Diagnostik runtime PID & status watchdog

Fz & Mz berasal dari PID (saat hold aktif & odom valid); komponen lain diambil dari manual.
Jika E-STOP aktif atau data sensor/manual stale, sistem otomatis fallback ke mode aman (output 0 / manual).
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Float64, Float64MultiArray

from hydroships_control.pid import PID, wrap_to_pi


def yaw_from_quaternion(q) -> float:
    """Ekstrak yaw (rad) dari geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _twist_is_finite(t: Twist) -> bool:
    """True jika seluruh komponen Twist valid (bukan NaN/inf)."""
    vals = (
        t.linear.x, t.linear.y, t.linear.z,
        t.angular.x, t.angular.y, t.angular.z,
    )
    return all(math.isfinite(v) for v in vals)


DEPTH_SETPOINT_MIN = -5.0          # m
DEPTH_SETPOINT_MAX = 0.0           # m
DEPTH_JUMP_RESET_THRESHOLD = 0.3   # m
HEADING_JUMP_RESET_THRESHOLD = 0.5  # rad
DT_MAX_SANE = 1.0                  # detik

# Parameter numerik yang WAJIB > 0 dan WAJIB finite (dipakai on_param_change).
_POSITIVE_NUMERIC_PARAMS = (
    'manual_timeout', 'odom_timeout',
    'depth.out_limit', 'heading.out_limit',
    'depth.integral_limit', 'heading.integral_limit',
    'rate',
)
# Parameter numerik lain yang boleh negatif/nol tapi tetap WAJIB finite.
_FINITE_ONLY_NUMERIC_PARAMS = (
    'depth.kp', 'depth.ki', 'depth.kd', 'depth.d_filter_alpha',
    'heading.kp', 'heading.ki', 'heading.kd', 'heading.d_filter_alpha',
    'buoyancy_ff', 'target_depth', 'target_heading',
)


class Stabilizer(Node):
    def __init__(self):
        super().__init__('stabilizer')

        # ---- Declaration Parameter ----
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('depth.kp', 60.0)
        self.declare_parameter('depth.ki', 8.0)
        self.declare_parameter('depth.kd', 40.0)
        self.declare_parameter('depth.integral_limit', 30.0)
        self.declare_parameter('depth.out_limit', 60.0)
        self.declare_parameter('depth.d_filter_alpha', 0.3)

        self.declare_parameter('heading.kp', 8.0)
        self.declare_parameter('heading.ki', 0.5)
        self.declare_parameter('heading.kd', 4.0)
        self.declare_parameter('heading.integral_limit', 5.0)
        self.declare_parameter('heading.out_limit', 15.0)
        self.declare_parameter('heading.d_filter_alpha', 0.3)

        self.declare_parameter('buoyancy_ff', -1.45)
        self.declare_parameter('target_depth', -1.0)
        self.declare_parameter('target_heading', 0.0)
        
        # FIX: Diubah ke True agar mode hold langsung aktif
        self.declare_parameter('enable_depth_hold', True)
        self.declare_parameter('enable_heading_hold', True)
        
        self.declare_parameter('manual_timeout', 0.5)
        self.declare_parameter('odom_timeout', 0.5)

        gp = self.get_parameter
        self.depth_pid = PID(
            gp('depth.kp').value, gp('depth.ki').value, gp('depth.kd').value,
            out_min=-gp('depth.out_limit').value,
            out_max=gp('depth.out_limit').value,
            integral_limit=gp('depth.integral_limit').value,
            angular=False,
            d_filter_alpha=gp('depth.d_filter_alpha').value)

        self.heading_pid = PID(
            gp('heading.kp').value, gp('heading.ki').value, gp('heading.kd').value,
            out_min=-gp('heading.out_limit').value,
            out_max=gp('heading.out_limit').value,
            integral_limit=gp('heading.integral_limit').value,
            angular=True,
            d_filter_alpha=gp('heading.d_filter_alpha').value)

        self.buoyancy_ff = float(gp('buoyancy_ff').value)
        self.target_depth = float(gp('target_depth').value)
        self.target_heading = wrap_to_pi(float(gp('target_heading').value))
        self.enable_depth = gp('enable_depth_hold').value
        self.enable_heading = gp('enable_heading_hold').value
        self.manual_timeout = float(gp('manual_timeout').value)
        self.odom_timeout = float(gp('odom_timeout').value)

        # Sinkronkan bound PID depth dengan buoyancy feedforward awal
        self._update_depth_pid_bounds()

        # ---- Internal State ----
        self.cur_z = None
        self.cur_yaw = None
        self.manual = Twist()
        self.last_time = None
        self.last_manual_time = None
        self.last_odom_time = None

        # FIX: E-STOP non-aktif secara default untuk simulasi
        self.estop = False

        # ---- Publisher & Subscriber ----
        self.pub = self.create_publisher(Twist, '/hydroships/cmd_vel', 10)
        self.diag_pub = self.create_publisher(
            Float64MultiArray, '/hydroships/stabilizer/diag', 10)

        self.create_subscription(
            Odometry, '/hydroships/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Twist, '/hydroships/manual/cmd', self.on_manual, 10)
        self.create_subscription(
            Float64, '/hydroships/setpoint/depth', self.on_depth_sp, 10)
        self.create_subscription(
            Float64, '/hydroships/setpoint/heading', self.on_heading_sp, 10)
        self.create_subscription(Bool, '/hydroships/estop', self.on_estop, 10)

        self.add_on_set_parameters_callback(self.on_param_change)

        rate = gp('rate').value
        self.timer = self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().info('Stabilizer siap dan sistem dalam kondisi ARMED (E-STOP non-aktif).')

    # ---- Helper Functions ----
    def _update_depth_pid_bounds(self, new_limit=None, new_ff=None):
        """Sinkronkan out_min/out_max PID depth dengan out_limit DAN buoyancy_ff.

        `new_limit`/`new_ff` dioper eksplisit saat dipanggil dari dalam
        on_param_change untuk parameter yang SEDANG diproses -- get_parameter()
        untuk parameter itu masih mengembalikan nilai LAMA selama callback
        berjalan (rclpy baru menyimpan nilai baru setelah callback sukses).
        """
        limit = (new_limit if new_limit is not None
                 else float(self.get_parameter('depth.out_limit').value))
        ff = new_ff if new_ff is not None else self.buoyancy_ff
        self.depth_pid.out_min = -limit - ff
        self.depth_pid.out_max = limit - ff

    def _is_stale(self, last_time, now, timeout):
        if last_time is None:
            return True
        elapsed = (now - last_time).nanoseconds * 1e-9
        return elapsed < 0.0 or elapsed > timeout

    # ---- Callbacks ----
    def on_odom(self, msg: Odometry):
        self.cur_z = msg.pose.pose.position.z
        self.cur_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.last_odom_time = self.get_clock().now()

    def on_manual(self, msg: Twist):
        if not _twist_is_finite(msg):
            self.get_logger().warn(
                'Manual command mengandung NaN/inf, diabaikan.',
                throttle_duration_sec=2.0)
            return
        self.manual = msg
        self.last_manual_time = self.get_clock().now()

    def on_estop(self, msg: Bool):
        if msg.data and not self.estop:
            self.get_logger().warn('E-STOP diaktifkan, semua output dipaksa nol.')
        elif not msg.data and self.estop:
            self.get_logger().info('E-stop dilepas, sistem di-arm.')
            self.depth_pid.reset()
            self.heading_pid.reset()
        self.estop = msg.data

    def on_depth_sp(self, msg: Float64):
        val = float(msg.data)
        if not math.isfinite(val):
            self.get_logger().warn('Setpoint depth tidak valid (nan/inf), diabaikan.')
            return
        clamped = max(DEPTH_SETPOINT_MIN, min(DEPTH_SETPOINT_MAX, val))
        if abs(clamped - self.target_depth) > DEPTH_JUMP_RESET_THRESHOLD:
            self.depth_pid.reset()
        self.target_depth = clamped

    def on_heading_sp(self, msg: Float64):
        val = float(msg.data)
        if not math.isfinite(val):
            self.get_logger().warn('Setpoint heading tidak valid (nan/inf), diabaikan.')
            return
        new_target = wrap_to_pi(val)
        if abs(wrap_to_pi(new_target - self.target_heading)) > HEADING_JUMP_RESET_THRESHOLD:
            self.heading_pid.reset()
        self.target_heading = new_target

    def on_param_change(self, params):
        try:
            # Validasi SEMUA parameter numerik: wajib finite (bukan NaN/inf),
            # dan untuk subset tertentu wajib > 0.
            for p in params:
                name, val = p.name, p.value
                if name in _POSITIVE_NUMERIC_PARAMS or name in _FINITE_ONLY_NUMERIC_PARAMS:
                    fval = float(val)
                    if not math.isfinite(fval):
                        return SetParametersResult(
                            successful=False,
                            reason=f'{name} tidak boleh NaN/inf (dapat {val}).')
                    if name in _POSITIVE_NUMERIC_PARAMS and fval <= 0.0:
                        return SetParametersResult(
                            successful=False,
                            reason=f'{name} harus bernilai > 0 (dapat {val}).')

            # Terapkan perubahan parameter
            for p in params:
                name, val = p.name, p.value
                if name == 'depth.kp':
                    self.depth_pid.kp = float(val)
                elif name == 'depth.ki':
                    self.depth_pid.ki = float(val)
                elif name == 'depth.kd':
                    self.depth_pid.kd = float(val)
                elif name == 'depth.d_filter_alpha':
                    self.depth_pid.d_filter_alpha = float(val)
                elif name == 'depth.out_limit':
                    self._update_depth_pid_bounds(new_limit=float(val))
                elif name == 'depth.integral_limit':
                    self.depth_pid.integral_limit = float(val)
                elif name == 'heading.kp':
                    self.heading_pid.kp = float(val)
                elif name == 'heading.ki':
                    self.heading_pid.ki = float(val)
                elif name == 'heading.kd':
                    self.heading_pid.kd = float(val)
                elif name == 'heading.d_filter_alpha':
                    self.heading_pid.d_filter_alpha = float(val)
                elif name == 'heading.out_limit':
                    limit = float(val)
                    self.heading_pid.out_min = -limit
                    self.heading_pid.out_max = limit
                elif name == 'heading.integral_limit':
                    self.heading_pid.integral_limit = float(val)
                elif name == 'buoyancy_ff':
                    self.buoyancy_ff = float(val)
                    self._update_depth_pid_bounds(new_ff=float(val))
                elif name == 'target_depth':
                    self.target_depth = max(DEPTH_SETPOINT_MIN,
                                             min(DEPTH_SETPOINT_MAX, float(val)))
                elif name == 'target_heading':
                    self.target_heading = wrap_to_pi(float(val))
                elif name == 'enable_depth_hold':
                    if bool(val) and not self.enable_depth:
                        self.depth_pid.reset()
                    self.enable_depth = bool(val)
                elif name == 'enable_heading_hold':
                    if bool(val) and not self.enable_heading:
                        self.heading_pid.reset()
                    self.enable_heading = bool(val)
                elif name == 'manual_timeout':
                    self.manual_timeout = float(val)
                elif name == 'odom_timeout':
                    self.odom_timeout = float(val)
                elif name == 'rate':
                    self.get_logger().warn(
                        "Parameter 'rate' tidak bisa diubah realtime tanpa "
                        'restart node (timer sudah dibuat dgn periode tetap).')
        except (TypeError, ValueError) as exc:
            return SetParametersResult(successful=False, reason=f'Gagal konversi tipe: {exc}')

        return SetParametersResult(successful=True)

    def on_timer(self):
        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            return

        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt <= 0.0 or dt > DT_MAX_SANE:
            self.get_logger().warn(f'dt tidak wajar ({dt:.3f}s), langkah dilewati.')
            return

        # 1. Jika E-STOP Aktif
        if self.estop:
            self.pub.publish(Twist())
            self._publish_diag(0.0, 0.0, 0.0, 0.0, False, False,
                                manual_stale=True, odom_stale=True)
            return

        # 2. Periksa Status Watchdog
        manual_stale = self._is_stale(self.last_manual_time, now, self.manual_timeout)
        manual = Twist() if manual_stale else self.manual
        odom_stale = self._is_stale(self.last_odom_time, now, self.odom_timeout)

        out = Twist()
        out.linear.x = manual.linear.x
        out.linear.y = manual.linear.y
        out.angular.x = manual.angular.x
        out.angular.y = manual.angular.y

        err_depth = 0.0
        err_heading = 0.0

        # 3. Mode Depth Hold
        depth_hold_active = self.enable_depth and self.cur_z is not None and not odom_stale
        if depth_hold_active:
            err_depth = self.target_depth - self.cur_z
            pid_out = self.depth_pid.update(err_depth, self.cur_z, dt)
            out_limit_z = float(self.get_parameter('depth.out_limit').value)
            out.linear.z = max(-out_limit_z, min(out_limit_z, pid_out + self.buoyancy_ff))
        else:
            out.linear.z = manual.linear.z
            if self.enable_depth and odom_stale:
                self.get_logger().warn(
                    'Odom stale, depth-hold sementara nonaktif, fallback manual.',
                    throttle_duration_sec=2.0)

        # 4. Mode Heading Hold
        heading_hold_active = self.enable_heading and self.cur_yaw is not None and not odom_stale
        if heading_hold_active:
            err_heading = wrap_to_pi(self.target_heading - self.cur_yaw)
            out.angular.z = self.heading_pid.update(err_heading, self.cur_yaw, dt)
        else:
            out.angular.z = manual.angular.z
            if self.enable_heading and odom_stale:
                self.get_logger().warn(
                    'Odom stale, heading-hold sementara nonaktif, fallback manual.',
                    throttle_duration_sec=2.0)

        # 5. Sanity Check Output Akhir
        if not _twist_is_finite(out):
            self.get_logger().error('Output mengandung NaN/inf! Output dipaksa 0, PID direset.')
            self.depth_pid.reset()
            self.heading_pid.reset()
            out = Twist()

        self.pub.publish(out)
        self._publish_diag(err_depth, err_heading, out.linear.z, out.angular.z,
                            depth_hold_active, heading_hold_active,
                            manual_stale, odom_stale)

    def _publish_diag(self, err_depth, err_heading, cmd_z, cmd_yaw,
                       depth_active, heading_active, manual_stale, odom_stale):
        diag = Float64MultiArray()
        diag.data = [
            float(err_depth), float(err_heading),
            float(cmd_z), float(cmd_yaw),
            1.0 if depth_active else 0.0,
            1.0 if heading_active else 0.0,
            1.0 if manual_stale else 0.0,
            1.0 if odom_stale else 0.0,
            1.0 if self.estop else 0.0,
        ]
        self.diag_pub.publish(diag)


def main(args=None):
    rclpy.init(args=args)
    node = Stabilizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()