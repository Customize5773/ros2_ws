import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

class JoyMissionTrigger(Node):
    def __init__(self):
        super().__init__('joy_mission_trigger')
        
        # --- PARAMETERS ---
        self.button_index = 0  # 0 is usually 'A' / 'Cross' on XInput controllers
        self.trigger_topic = '/mission/trigger'
        
        # Subscriber & Publisher
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        self.publisher_ = self.create_publisher(Bool, self.trigger_topic, 10)
        
        self.last_button_state = 0
        self.get_logger().info(f"Joy Trigger Node active. Press button index [{self.button_index}] to start mission.")

    def joy_callback(self, msg: Joy):
        # Check if the button index exists in the message array
        if len(msg.buttons) > self.button_index:
            current_button_state = msg.buttons[self.button_index]

            # Detect Rising Edge (Button transition from 0 to 1)
            if current_button_state == 1 and self.last_button_state == 0:
                self.get_logger().info('Joystick trigger pressed! Sending state machine bypass signal...')
                
                trigger_msg = Bool()
                trigger_msg.data = True
                self.publisher_.publish(trigger_msg)

            self.last_button_state = current_button_state

def main(args=None):
    rclpy.init(args=args)
    node = JoyMissionTrigger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()