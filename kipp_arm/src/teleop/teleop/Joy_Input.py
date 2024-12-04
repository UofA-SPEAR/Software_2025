import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import pygame
from rclpy.qos import QoSProfile

class JoyPublisher(Node):

    def __init__(self):
        super().__init__('joy_publisher')

        # Initialize pygame for joystick input
        pygame.init()
        pygame.joystick.init()

        # Initialize publishers
        self.publisher_spear = self.create_publisher(msg_type = Joy, topic = '/SPEAR_Arm/Joy_Topic', qos_profile = QoSProfile(depth=10))
        #self.publisher_rover = self.create_publisher(Joy, '/Rover/Joy_Topic', 10)
        self.threshold = 0.08 #Control threshold for stick 
        self.timer = self.create_timer(0.05, self.publish_joystick_input)

    def get_joystick_input(self, joystick_id):
        # Get the specific joystick
        joystick = pygame.joystick.Joystick(joystick_id)
        joystick.init()

        # Get axes and buttons from the joystick
        axes = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
        buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]
        hats = joystick.get_hat(0)
        hats_float = tuple(float(x) for x in hats )
        axes.extend(hats_float)

        axes = [0.00 if abs(x) < self.threshold else x for x in axes]

        # Create Joy message
        joystick_input = Joy()
        joystick_input.header.stamp = self.get_clock().now().to_msg()
        joystick_input.axes = axes
        joystick_input.buttons = buttons

        return joystick_input

    def publish_joystick_input(self):
        # Get joystick inputs for two controllers
        joystick_input_spear = self.get_joystick_input(0)  # Assuming ID 0 for the SPEAR_Arm controller
        #joystick_input_rover = self.get_joystick_input(1)  # Assuming ID 1 for the Rover_Arm controller

        # Publish joystick inpu
        self.publisher_spear.publish(joystick_input_spear)
        #self.publisher_rover.publish(joystick_input_rover)

def main(args=None):
    rclpy.init(args=args)

    joy_publisher = JoyPublisher()

    rclpy.spin(joy_publisher)

    joy_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

