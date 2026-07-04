#!/usr/bin/env python3
"""Teleop mode STABILIZED (Milestone 2).

Pilot mengemudi horizontal, sementara kedalaman & heading ditahan otomatis oleh
node `stabilizer`. Tombol i/k dan j/l MENGUBAH SETPOINT (bukan gaya langsung).

Publikasi:
  /hydroships/manual/cmd       (Twist)   -> Fx (surge), Fy (sway)
  /hydroships/setpoint/depth   (Float64) -> target kedalaman
  /hydroships/setpoint/heading (Float64) -> target heading

Tombol:
  w / s : surge maju / mundur     (Fx)
  a / d : sway kiri / kanan        (Fy)
  i / k : setpoint kedalaman naik / turun (k = lebih dalam)
  j / l : setpoint heading kiri / kanan
  spasi : hentikan gerak horizontal (Fx=Fy=0)
  x     : keluar
"""

import math
import sys
import termios
import threading
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64

HELP = __doc__


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class TeleopStabilized(Node):
    def __init__(self):
        super().__init__('teleop_stabilized')
        self.manual_pub = self.create_publisher(Twist, '/hydroships/manual/cmd', 10)
        self.depth_pub = self.create_publisher(Float64, '/hydroships/setpoint/depth', 10)
        self.heading_pub = self.create_publisher(Float64, '/hydroships/setpoint/heading', 10)

        self.fx = 0.0
        self.fy = 0.0
        self.surge_step = 20.0
        self.depth_sp = -1.0
        self.heading_sp = 0.0
        self.depth_step = 0.1      # m
        self.heading_step = math.radians(10.0)

        self.timer = self.create_timer(0.1, self.publish_manual)

    def apply_key(self, key):
        if key == 'w':
            self.fx += self.surge_step
        elif key == 's':
            self.fx -= self.surge_step
        elif key == 'a':
            self.fy += self.surge_step
        elif key == 'd':
            self.fy -= self.surge_step
        elif key == ' ':
            self.fx = self.fy = 0.0
        elif key == 'i':
            self.depth_sp += self.depth_step        # naik (mendekati permukaan)
            self._pub_depth()
        elif key == 'k':
            self.depth_sp -= self.depth_step        # turun (lebih dalam)
            self._pub_depth()
        elif key == 'j':
            self.heading_sp = self._wrap(self.heading_sp + self.heading_step)
            self._pub_heading()
        elif key == 'l':
            self.heading_sp = self._wrap(self.heading_sp - self.heading_step)
            self._pub_heading()

    @staticmethod
    def _wrap(a):
        return math.atan2(math.sin(a), math.cos(a))

    def _pub_depth(self):
        self.depth_pub.publish(Float64(data=float(self.depth_sp)))

    def _pub_heading(self):
        self.heading_pub.publish(Float64(data=float(self.heading_sp)))

    def publish_manual(self):
        msg = Twist()
        msg.linear.x = self.fx
        msg.linear.y = self.fy
        self.manual_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopStabilized()
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    settings = termios.tcgetattr(sys.stdin)
    print(HELP)
    # Kirim setpoint awal sekali.
    node._pub_depth()
    node._pub_heading()
    try:
        while rclpy.ok():
            key = get_key(settings)
            if key == '\x03' or key == 'x':
                break
            node.apply_key(key)
            print(f'\rFx {node.fx:+.1f} Fy {node.fy:+.1f} | depth_sp '
                  f'{node.depth_sp:+.2f} m  heading_sp '
                  f'{math.degrees(node.heading_sp):+.0f} deg    ',
                  end='', flush=True)
    finally:
        node.fx = node.fy = 0.0
        node.publish_manual()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
