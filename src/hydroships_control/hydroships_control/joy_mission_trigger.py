import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty


class JoyMissionTrigger(Node):
    """Tombol joystick sebagai trigger pilot utk melewati WAIT_TRIGGER.

    Misi FSM parkir di state WAIT_TRIGGER setelah SURFACE dan menunggu
    pesan Empty di /hydroships/mission/start_autonomous (lihat
    mission_fsm.py::_on_trigger). Node ini meneruskan satu pesan Empty
    ke topic itu saat tombol ditekan (rising edge 0->1), jadi pilot bisa
    melewati WAIT_TRIGGER dari joystick tanpa terminal tambahan.

    Default tombol = index 0 (A / Cross pada XInput / Logitech F310).
    Ganti lewat param `button_index`; nama topic lewat `trigger_topic`.
    """

    def __init__(self):
        super().__init__('joy_mission_trigger')

        # --- PARAMETERS ---
        self.declare_parameter('button_index', 0)
        self.declare_parameter(
            'trigger_topic', '/hydroships/mission/start_autonomous')

        self.button_index = self.get_parameter('button_index').value
        self.trigger_topic = self.get_parameter('trigger_topic').value

        # Subscriber & Publisher
        self.subscription = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)
        self.publisher_ = self.create_publisher(Empty, self.trigger_topic, 10)

        self.last_button_state = 0
        self.get_logger().info(
            'Joy trigger siap: tombol index [%d] -> %s '
            '(tekan utk lewati WAIT_TRIGGER)' % (self.button_index,
                                                 self.trigger_topic))

    def joy_callback(self, msg: Joy):
        if len(msg.buttons) <= self.button_index:
            return
        current_button_state = msg.buttons[self.button_index]

        # Rising edge: tombol berubah 0 -> 1 (tekan baru, bukan tahan).
        if current_button_state == 1 and self.last_button_state == 0:
            self.get_logger().info(
                'Trigger joystick ditekan (button %d) -> publish %s'
                % (self.button_index, self.trigger_topic))
            self.publisher_.publish(Empty())

        self.last_button_state = current_button_state


def main(args=None):
    rclpy.init(args=args)
    node = JoyMissionTrigger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
