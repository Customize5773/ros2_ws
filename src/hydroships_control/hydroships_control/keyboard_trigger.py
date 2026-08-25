import sys, termios, tty, select, threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

HELP = "keyboard_trigger: N/Enter=trigger  q/ESC=quit  (focus terminal ini — jangan focus Gazebo)"

class KeyboardTrigger(Node):
    def __init__(self):
        super().__init__('keyboard_trigger')
        self.declare_parameter('trigger_topic', '/hydroships/mission/start_autonomous')
        topic = self.get_parameter('trigger_topic').value
        self.pub = self.create_publisher(Empty, topic, 10)
        self.get_logger().info('Keyboard trigger siap: SPACE/T -> %s  (%s)' % (topic, HELP))
        print(HELP, flush=True)
        self._stop = False
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while not self._stop and rclpy.ok():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    ch = sys.stdin.read(1)
                    if ch in ('n', 'N', '\r', '\n'):
                        self.get_logger().info('Keyboard N/Enter -> publish trigger')
                        print('>> trigger', flush=True)
                        self.pub.publish(Empty())
                    elif ch in ('q', '\x1b', '\x03'):
                        print('quit', flush=True); self._stop = True; break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
            if rclpy.ok(): rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTrigger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop = True
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
