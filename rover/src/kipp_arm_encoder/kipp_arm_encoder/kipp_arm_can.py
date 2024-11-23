import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import can
import struct

class CanInterface(Node):
    def __init__(self):
        super().__init__('can_interface')
        
        # Initialize the CAN bus interface
        self.bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=1000000)
        
        # Publisher for joint states
        self.joint_state_publisher = self.create_publisher(JointState, '/raw/joint_states', 10)
        
        # Subscriber for joint trajectory
        self.joint_trajectory_subscriber = self.create_subscription(JointTrajectory, '/arm_controller/joint_trajectory', self.joint_trajectory_callback, 10)
        
        # Timer for receiving CAN data
        self.create_timer(0.1, self.receive_can_data)
        
        # Actuator IDs for the arm joints
        self.actuator_ids = {
            'joint_1': 0x30,
            'joint_2': 0x31,
            'joint_3': 0x32,
            'joint_4': 0x33,
            'joint_5': 0x34,
            'joint_6': 0x35
        }

    def joint_trajectory_callback(self, msg: JointTrajectory):

      for point in msg.points:
        # Loop over each position and its corresponding joint name
        for idx, position in enumerate(point.positions):
            joint_name = msg.joint_names[idx]
            self.get_logger().info(f'Joint Name: {joint_name}, Position: {position}')

            # Retrieve actuator ID using the joint name (if applicable)
            actuator_id = self.actuator_ids.get(joint_name, None)
            self.get_logger().info(f'Actuator ID from message: {actuator_id}')
            
            # Send CAN data if actuator ID is available
            if actuator_id is not None:
                self.send_can_data(actuator_id, position)

    def send_can_data(self, actuator_id, position):
        command_id = 0x02
        priority = 0x01
        receiver_node_id = actuator_id
        sender_node_id = 1
        arbitration_id = (priority << 24) | (command_id << 16) | (receiver_node_id << 8) | sender_node_id
        
        data = struct.pack(">f", position)
        message = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=True)
        
        try:
            self.bus.send(message)
            self.get_logger().info(f'Sent CAN message to actuator {actuator_id} with position {position}')
        except can.CanError as e:
            self.get_logger().error(f'Failed to send CAN message: {e}')


#ENCODER 
    def receive_can_data(self):
        try:
            message = self.bus.recv(timeout=0.1)
            if message is not None and message.arbitration_id >> 16 == 0x00:
                self.process_can_data(message)
        except can.CanError as e:
            self.get_logger().error(f'Failed to receive CAN message: {e}')

    def process_can_data(self, message):
        receiver_node_id = (message.arbitration_id >> 8) & 0xFF
        if receiver_node_id in self.actuator_ids.values():
            position = struct.unpack(">f", message.data)[0]
            joint_name = f'joint_{list(self.actuator_ids.values()).index(receiver_node_id) + 1}'
            
            joint_state = JointState()
            joint_state.name = [joint_name]
            joint_state.position = [position]
            joint_state.header.stamp = self.get_clock().now().to_msg()
            
            self.joint_state_publisher.publish(joint_state)
            self.get_logger().info(f'Received CAN message from actuator {receiver_node_id} with position {position}')

def main(args=None):
    rclpy.init(args=args)
    can_interface_node = CanInterface()
    rclpy.spin(can_interface_node)
    
    can_interface_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
