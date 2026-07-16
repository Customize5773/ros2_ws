"""gui_bridge — node adapter GUI-ROV (UDP-JSON/MAVLink) <-> ROS 2 hydroships.

Membuat sim ROS 2 tampak seperti ROV ArduSub bagi GUI tim (Customize5773/GUI-ROV)
TANPA menyentuh node inti (stabilizer, mission_fsm, thruster_allocator). Karena
GUI memakai UDP-JSON (bukan ROS), remap topik tak cukup — node ini menerjemahkan:

  GUI (UDP command JSON {name,value}) -> ROS:
    surge/sway/yaw/heave (persen) -> /hydroships/cmd_vel (Twist wrench N, N·m)
    gripper "open"/"close"        -> /hydroships/gripper/command (String)
  ROS -> GUI (UDP telemetry JSON):
    /hydroships/odom  -> heading(deg), roll, pitch
    /hydroships/depth -> depth(m)

Logika terjemahan (gain, konvensi) ada di gui_bridge_logic (teruji headless).

Parameter:
  cmd_port   (14550) : UDP port DENGAR perintah GUI (sesuai UDP_CMD_PORT GUI).
  telem_host (127.0.0.1), telem_port (14551) : tujuan telemetri (server.js GUI).
  telem_hz   (10)    : laju kirim telemetri.

CATATAN [VERIFY]: belum diuji dgn GUI live end-to-end; gain/tanda perlu kalibrasi.
"""

import json
import socket

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, String

from hydroships_control.gui_bridge_logic import GuiBridgeLogic


def _yaw_rpy(q):
    """Quaternion -> (roll, pitch, yaw) rad."""
    import math
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


class GuiBridge(Node):
    def __init__(self):
        super().__init__('gui_bridge')
        p = self.declare_parameter
        p('cmd_port', 14550)
        p('telem_host', '127.0.0.1')
        p('telem_port', 14551)
        p('telem_hz', 10.0)
        p('surge_gain', 0.40); p('sway_gain', 0.40)
        p('heave_gain', 0.30); p('yaw_gain', 0.12)
        g = lambda n: self.get_parameter(n).value

        self.logic = GuiBridgeLogic(
            surge_gain=float(g('surge_gain')), sway_gain=float(g('sway_gain')),
            heave_gain=float(g('heave_gain')), yaw_gain=float(g('yaw_gain')))

        # ROS keluar (ke sim) & masuk (telemetri).
        self.pub_cmd = self.create_publisher(Twist, '/hydroships/cmd_vel', 10)
        self.pub_grip = self.create_publisher(String, '/hydroships/gripper/command', 10)
        self._depth = 0.0
        self._rpy = (0.0, 0.0, 0.0)
        self.create_subscription(Odometry, '/hydroships/odom', self._on_odom, 10)
        self.create_subscription(Float64, '/hydroships/depth', self._on_depth, 10)

        # UDP: dengar perintah GUI (non-blocking) + kirim telemetri.
        self._telem_dest = (str(g('telem_host')), int(g('telem_port')))
        self._rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._rx.bind(('0.0.0.0', int(g('cmd_port'))))
        self._rx.setblocking(False)
        self._tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.create_timer(0.02, self._poll_cmd)                 # 50 Hz drain UDP
        hz = max(1.0, float(g('telem_hz')))
        self.create_timer(1.0 / hz, self._send_telem)
        self.get_logger().info(
            'gui_bridge siap — dengar UDP cmd :%d, telemetri -> %s:%d' % (
                int(g('cmd_port')), self._telem_dest[0], self._telem_dest[1]))

    def _on_depth(self, msg: Float64):
        self._depth = float(msg.data)

    def _on_odom(self, msg: Odometry):
        self._rpy = _yaw_rpy(msg.pose.pose.orientation)

    def _poll_cmd(self):
        # Kuras semua datagram tertunda tiap tick.
        for _ in range(50):
            try:
                data, _addr = self._rx.recvfrom(2048)
            except (BlockingIOError, socket.error):
                break
            try:
                msg = json.loads(data.decode('utf-8'))
            except (ValueError, UnicodeDecodeError):
                continue
            self._handle(msg)

    def _handle(self, msg):
        action = self.logic.on_command(msg.get('name'), msg.get('value'))
        if 'wrench' in action:
            fx, fy, fz, mz = action['wrench']
            t = Twist()
            t.linear.x = float(fx); t.linear.y = float(fy); t.linear.z = float(fz)
            t.angular.z = float(mz)
            self.pub_cmd.publish(t)
        if 'gripper' in action:
            s = String(); s.data = action['gripper']; self.pub_grip.publish(s)

    def _send_telem(self):
        roll, pitch, yaw = self._rpy
        telem = self.logic.build_telemetry(
            yaw_rad=yaw, depth_m=self._depth, roll=roll, pitch=pitch)
        try:
            self._tx.sendto(json.dumps(telem).encode('utf-8'), self._telem_dest)
        except socket.error as e:
            self.get_logger().warn('kirim telemetri gagal: %s' % e,
                                   throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = GuiBridge()
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
