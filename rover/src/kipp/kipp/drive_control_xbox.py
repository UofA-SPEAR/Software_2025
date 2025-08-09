import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import time

class XboxControllerNode(Node):
    def __init__(self):
        super().__init__('xbox_controller_node', namespace='manual')
        self.callback_group = ReentrantCallbackGroup()
        self.subscription = self.create_subscription(Joy, 'joy1', self.joy_callback, 10, callback_group=self.callback_group)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10, callback_group=self.callback_group)
        self.last_received_time = self.get_clock().now()
        self.connected = False
        #self.timer = self.create_timer(1.0, self.check_connection, callback_group=self.callback_group)  # Check connection every 1 second

        self.max_linear_speed = 1.0  # Adjust as needed
        self.max_angular_speed = 1.0  # Adjust as needed
        self.current_linear_speed = 0.0  # Track current linear speed
        self.ramp_rate = 0.05

    def joy_callback(self, msg):
        self.connected = True
        self.last_received_time = self.get_clock().now()
        
        twist = Twist()
        left_trigger = 1 - msg.axes[2]  # Assuming axis 3 is the left trigger
        right_trigger = 1 - msg.axes[5]  # Assuming axis 6 is the right trigger
        steering = msg.axes[0]

        linear_speed = (right_trigger - left_trigger) /2  * self.max_linear_speed
        angular_speed = -steering
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed

        self.publisher.publish(twist)

    def check_connection(self):
        current_time = self.get_clock().now()
        if self.connected and (current_time - self.last_received_time).nanoseconds() > 5.0 / 1e9:  # No message for 5 seconds
            self.get_logger().warn('No controller input received recently. Is the controller connected?')
            self.connected = False
        elif not self.connected:
            self.get_logger().warn('Waiting for controller input...')

def main(args=None):
    rclpy.init(args=args)
    xbox_controller_node = XboxControllerNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(xbox_controller_node, executor=executor)
    xbox_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()