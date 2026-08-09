import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

LINEAR_SCALE = 3.0  # tune this
YAW_RATE = 0.02
DEPTH_RATE = 0.02
DEPTH_MIN = -3.0   # tune to pool depth
DEPTH_MAX = 0.0    # surface


class JoyTeleop(Node):
    def __init__(self):
        super().__init__('joy_teleop')
        self.sub = self.create_subscription(Joy, '/joy', self.cb, 10)
        self.manual_pub = self.create_publisher(Twist, '/hydroships/manual/cmd', 10)
        self.heading_pub = self.create_publisher(Float64, '/hydroships/setpoint/heading', 10)
        self.depth_pub = self.create_publisher(Float64, '/hydroships/setpoint/depth', 10)
        self.heading = 0.0
        self.depth = 0.0

    def cb(self, msg: Joy):
        t = Twist()
        lx = msg.axes[1] if abs(msg.axes[1]) > 0.1 else 0.0
        ly = msg.axes[0] if abs(msg.axes[0]) > 0.1 else 0.0
        t.linear.x = lx * LINEAR_SCALE
        t.linear.y = ly * LINEAR_SCALE
        self.manual_pub.publish(t)

        yaw_input = msg.axes[3]
        if abs(yaw_input) > 0.1:
            self.heading += yaw_input * YAW_RATE
            h = Float64(); h.data = self.heading
            self.heading_pub.publish(h)

        depth_input = msg.axes[4]  # right stick up/down, adjust index if used elsewhere
        if abs(depth_input) > 0.1:
            self.depth += depth_input * DEPTH_RATE
            self.depth = max(DEPTH_MIN, min(DEPTH_MAX, self.depth))
            d = Float64(); d.data = self.depth
            self.depth_pub.publish(d)

def main():
    rclpy.init()
    node = JoyTeleop()
    rclpy.spin(node)

if __name__ == '__main__':
    main()