#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import can
import struct
from can.message import Message
import time

class ArmCanVelocityNode(Node):
    def __init__(self):
        super().__init__('arm_can_velocity_node')
        
        # Initialize CAN bus
        try:
            self.bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=1000000)
            self.get_logger().info("CAN bus initialized successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize CAN bus: {e}")
            self.bus = None
        
        # Subscribe to motor commands from IK node
        self.motor_commands_sub = self.create_subscription(
            Float64MultiArray, 
            'motor_commands', 
            self.motor_commands_callback, 
            10
        )
        
        # Arm actuator IDs mapping (from your CAN docs)
        self.arm_actuator_ids = [
            0x31,  # Joint 0: Shoulder Yaw
            0x32,  # Joint 1: Shoulder Pitch  
            0x34,  # Joint 2: Elbow Pitch
            0x35,  # Joint 3: Elbow Roll
            0x36,  # Joint 4: Wrist Pitch
            0x16,  # Joint 5: Wrist Roll
            0x15   # Joint 6: End Effector
        ]
        
        # CAN message parameters
        self.priority = 0x01
        self.velocity_command_id = 0x03
        self.sender_node_id = 0x01  # Jetson node ID
        
        # Rate limiting
        self.last_send_time = 0
        self.send_interval = 0.1  # 10Hz = 0.1 seconds
        self.latest_commands = None
        
        # Timer to send commands at 10Hz
        self.send_timer = self.create_timer(self.send_interval, self.send_commands)
        
        self.get_logger().info("Arm CAN Velocity Node initialized")
        self.get_logger().info(f"Subscribed to: motor_commands")
        self.get_logger().info(f"Arm actuator IDs: {[hex(id) for id in self.arm_actuator_ids]}")
        self.get_logger().info("Rate limited to 10Hz")

    def motor_commands_callback(self, msg):
        """
        Store latest motor commands without sending immediately.
        """
        if len(msg.data) != 7:
            self.get_logger().warn(f"Expected 7 joint velocities, got {len(msg.data)}")
            return
        
        self.latest_commands = msg.data

    def send_commands(self):
        """
        Send the latest motor commands at 10Hz rate.
        """
        if not self.bus or self.latest_commands is None:
            return
        
        # Send velocity command for each arm joint
        for joint_index, velocity in enumerate(self.latest_commands):
            if joint_index >= len(self.arm_actuator_ids):
                break
                
            actuator_id = self.arm_actuator_ids[joint_index]
            
            # Clamp velocity to valid range (-1.0 to 1.0)
            clamped_velocity = max(-1.0, min(1.0, velocity))
            
            # Create and send CAN message
            try:
                message = self.create_velocity_command(actuator_id, clamped_velocity)
                self.bus.send(message)
                    
            except Exception as e:
                self.get_logger().error(f"Failed to send velocity command for joint {joint_index}: {e}")

    def create_velocity_command(self, actuator_id, velocity):
        """
        Create a CAN velocity command message.
        """
        arbitration_id = (
            self.priority << 24 | 
            self.velocity_command_id << 16 | 
            actuator_id << 8 | 
            self.sender_node_id
        )
        
        data = struct.pack(">f", velocity)
        
        message = can.Message(
            arbitration_id=arbitration_id,
            data=data,
            is_extended_id=True
        )
        
        return message

    def send_stop_command(self):
        """
        Send stop command to all arm actuators.
        """
        if not self.bus:
            return
            
        self.get_logger().info("Sending stop commands to all arm actuators")
        
        for actuator_id in self.arm_actuator_ids:
            try:
                message = self.create_velocity_command(actuator_id, 0.0)
                self.bus.send(message)
            except Exception as e:
                self.get_logger().error(f"Failed to send stop command to actuator {hex(actuator_id)}: {e}")

    def destroy_node(self):
        """
        Clean shutdown - send stop commands before destroying node.
        """
        self.get_logger().info("Shutting down Arm CAN Velocity Node")
        self.send_stop_command()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ArmCanVelocityNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()