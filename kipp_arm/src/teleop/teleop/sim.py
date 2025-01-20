import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class SimulateJoy(Node):

    def __init__(self):
        super().__init__('simulate_joy')
        self.publisher = self.create_publisher(Joy, '/SPEAR_Arm/Joy_Topic', 10)

        # Create a timer to call publish_joy_data every 0.1 seconds
        self.timer = self.create_timer(0.1, self.publish_joy_data)

    def publish_joy_data(self):
        joy_msg = Joy()

        # Simulating joystick axes (Xbox controller has 10 axes)
        joy_msg.axes = [0.0] * 10  # Default 10 axes values
        joy_msg.axes[0] = 0  # Simulate left stick X-axis movement
        joy_msg.axes[1] = 0  # Simulate left stick Y-axis movement
        joy_msg.axes[3] = 0  # Simulate right stick X-axis movement
        joy_msg.axes[4] = 0  # Simulate right stick Y-axis movement

        # Simulating button presses (buttons A, B, X, Y, etc.)
        joy_msg.buttons = [0] * 12  # Default 12 buttons
        joy_msg.buttons[0] = 0  # Simulate button A not pressed
        joy_msg.buttons[1] = 0  # Button B not pressed
        joy_msg.buttons[2] = 0  # Simulate button X being pressed

        # Publish the simulated joystick data
        self.publisher.publish(joy_msg)
        self.get_logger().info('Publishing simulated joy data')

def main(args=None):
    rclpy.init(args=args)
    joy_simulator = SimulateJoy()

    # Spin to keep the node alive and handling the timer
    rclpy.spin(joy_simulator)

    joy_simulator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
