import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import can
import struct

class GripperCANControl(Node):
    def __init__(self):
        super().__init__('gripper_can_control')

        # Create a CAN bus interface
        self.bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=1000000)

        # Create a subscriber to listen to the gripper velocity commands
        self.gripper_velocity_sub = self.create_subscription(Float64, '/gripper_controller/velocity', self.gripper_callback, 10)

        # Initialize a timer to periodically check and publish the current value
        self.timer = self.create_timer(0.1, self.check_and_publish_zero)

        # Variable to store the last received velocity
        self.current_velocity = 0.0

    def gripper_callback(self, msg):
        # Update the current velocity with the received value
        self.current_velocity = msg.data

        # Send the CAN command based on the current velocity
        self.send_can_command(self.current_velocity)

    def check_and_publish_zero(self):
        # If no new value is received, publish a zero command
        if self.current_velocity == 0.0:
            self.send_can_command(0.0)

    def send_can_command(self, value):
        # Send a CAN command to motor 26 based on the gripper state

        # Construct arbitration ID
        priority = 0x0
        command_id = 0x03  # Assuming 0x03 for drive command
        receiver_node_id = 0x21  # Motor 26
        sender_node_id = 1
        arbitration_id = priority << 24 | command_id << 16 | receiver_node_id << 8 | sender_node_id

        # Pack data (value)
        data = struct.pack(">f", value)  # 'f' for float32

        # Create CAN message
        message = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=True)

        # Send the CAN message
        self.bus.send(message)
        self.get_logger().info(f"Sent CAN command to motor 26 with value: {value:.2f}")

def main(args=None):
    rclpy.init(args=args)

    gripper_can_control = GripperCANControl()

    rclpy.spin(gripper_can_control)

    gripper_can_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
