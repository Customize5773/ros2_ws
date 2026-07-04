#!/usr/bin/env python3
"""Teleop keyboard ROV HYDROships -> /hydroships/cmd_vel (wrench body, N & N*m).

Tombol:
  w / s : surge maju / mundur   (Fx)
  a / d : sway kiri / kanan      (Fy)
  i / k : heave naik / turun     (Fz)  -> untuk MENYELAM tekan 'k'
  j / l : yaw kiri / kanan       (Mz)
  u / o : roll                   (Mx)
  t / g : pitch                  (My)
  + / - : naik / turun besaran perintah
  spasi : STOP (semua nol)
  Ctrl-C atau x : keluar

Perintah bersifat "sticky": nilai bertahan sampai diubah. Node allocator punya
watchdog, jadi bila teleop mati thruster otomatis nol.
"""

import sys
import termios
import threading
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

HELP = __doc__

# tombol -> (indeks komponen wrench, tanda)
#  0=Fx 1=Fy 2=Fz 3=Mx 4=My 5=Mz
KEY_BINDINGS = {
    'w': (0, +1), 's': (0, -1),
    'a': (1, +1), 'd': (1, -1),
    'i': (2, +1), 'k': (2, -1),
    'u': (3, +1), 'o': (3, -1),
    't': (4, +1), 'g': (4, -1),
    'j': (5, +1), 'l': (5, -1),
}


def get_key(settings):
    """Baca satu karakter dari stdin (non-canonical)."""
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.pub = self.create_publisher(Twist, '/hydroships/cmd_vel', 10)
        self.wrench = [0.0] * 6
        self.linear_step = 20.0   # N per tekan
        self.angular_step = 5.0   # N*m per tekan
        # Publikasi ulang berkala supaya watchdog allocator tidak reset.
        self.timer = self.create_timer(0.1, self.publish_cmd)

    def step_for(self, idx):
        return self.linear_step if idx < 3 else self.angular_step

    def apply_key(self, key):
        if key in KEY_BINDINGS:
            idx, sign = KEY_BINDINGS[key]
            self.wrench[idx] += sign * self.step_for(idx)
        elif key == ' ':
            self.wrench = [0.0] * 6
        elif key == '+':
            self.linear_step *= 1.25
            self.angular_step *= 1.25
        elif key == '-':
            self.linear_step *= 0.8
            self.angular_step *= 0.8

    def publish_cmd(self):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z = self.wrench[0:3]
        msg.angular.x, msg.angular.y, msg.angular.z = self.wrench[3:6]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()

    # Spin node di thread terpisah supaya loop keyboard tidak memblok timer.
    executor_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    executor_thread.start()

    settings = termios.tcgetattr(sys.stdin)
    print(HELP)
    try:
        while rclpy.ok():
            key = get_key(settings)
            if key == '\x03' or key == 'x':  # Ctrl-C atau x
                break
            node.apply_key(key)
            print(f'\rwrench = [Fx {node.wrench[0]:+.1f} Fy {node.wrench[1]:+.1f} '
                  f'Fz {node.wrench[2]:+.1f} | Mx {node.wrench[3]:+.1f} '
                  f'My {node.wrench[4]:+.1f} Mz {node.wrench[5]:+.1f}] '
                  f'step(lin {node.linear_step:.1f}/ang {node.angular_step:.1f})   ',
                  end='', flush=True)
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(f'teleop error: {exc}')
    finally:
        node.wrench = [0.0] * 6
        node.publish_cmd()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
