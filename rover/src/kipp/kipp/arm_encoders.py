#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import can
import struct
import threading
import time

class ArmEncoderReaderNode(Node):
    def __init__(self):
        super().__init__('arm_encoder_reader_node')
        
        try:
            self.bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=1000000)
            self.get_logger().info("CAN bus initialized successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize CAN bus: {e}")
            self.bus = None
            return
        
        self.joint_angles_pub = self.create_publisher(Float64MultiArray, 'arm_joint_angles', 10)
        
        # DEBUG: Check if these IDs match what you expect from the CAN docs
        self.arm_actuator_ids = [0x31, 0x32, 0x34, 0x35, 0x36, 0x16]
        
        self.joint_angles = [0.0] * len(self.arm_actuator_ids)
        self.offset_angles = [None] * len(self.arm_actuator_ids)
        self.angles_lock = threading.Lock()
        
        self.position_feedback_command_id = 0x01
        self.jetson_node_id = 0x01
        
        # DEBUG: Add message counters
        self.total_messages_received = 0
        self.position_feedback_messages = 0
        self.valid_actuator_messages = 0
        
        self.can_thread = threading.Thread(target=self.can_listener_thread, daemon=True)
        self.can_thread.start()
        
        self.publish_timer = self.create_timer(0.1, self.publish_joint_angles)
        
        # DEBUG: Add debug timer to print statistics
        self.debug_timer = self.create_timer(2.0, self.print_debug_stats)

    def can_listener_thread(self):
        self.get_logger().info("CAN listener thread started")
        
        while rclpy.ok() and self.bus:
            try:
                message = self.bus.recv(timeout=1.0)
                if message:
                    self.total_messages_received += 1
                    self.process_can_message(message)
            except Exception as e:
                if rclpy.ok():
                    self.get_logger().error(f"CAN listener error: {e}")
                break

    def process_can_message(self, message):
        try:
            arbitration_id = message.arbitration_id
            sender_node_id = arbitration_id & 0xFF
            receiver_node_id = (arbitration_id >> 8) & 0xFF
            command_id = (arbitration_id >> 16) & 0xFF
            priority = (arbitration_id >> 24) & 0x1F
            
            # DEBUG: Log every message for debugging
            self.get_logger().info(f"CAN MSG: ID=0x{arbitration_id:08X}, "
                                 f"Sender=0x{sender_node_id:02X}, "
                                 f"Receiver=0x{receiver_node_id:02X}, "
                                 f"Command=0x{command_id:02X}, "
                                 f"Data={message.data.hex()}")
            
            # Check if this is a position feedback message to us
            if (command_id == self.position_feedback_command_id and 
                receiver_node_id == self.jetson_node_id):
                
                self.position_feedback_messages += 1
                self.get_logger().info(f"Position feedback message from 0x{sender_node_id:02X}")
                
                # Check if sender is one of our arm actuators
                if sender_node_id in self.arm_actuator_ids:
                    self.valid_actuator_messages += 1
                    joint_index = self.arm_actuator_ids.index(sender_node_id)
                    
                    if len(message.data) >= 4:
                        angle = struct.unpack(">f", message.data[:4])[0]
                        
                        self.get_logger().info(f"Raw angle from joint {joint_index}: {angle}")
                        
                        with self.angles_lock:
                            if self.offset_angles[joint_index] is None:
                                self.offset_angles[joint_index] = angle
                                self.get_logger().info(f"Set offset for joint {joint_index}: {angle}")
                            
                            self.joint_angles[joint_index] = angle - self.offset_angles[joint_index]
                            self.get_logger().info(f"Joint {joint_index} final angle: {self.joint_angles[joint_index]}")
                    else:
                        self.get_logger().warn(f"Insufficient data from 0x{sender_node_id:02X}: {len(message.data)} bytes")
                else:
                    self.get_logger().warn(f"Unknown actuator ID: 0x{sender_node_id:02X}")
            else:
                self.get_logger().debug(f"Not position feedback: cmd=0x{command_id:02X}, recv=0x{receiver_node_id:02X}")
                        
        except Exception as e:
            self.get_logger().error(f"Error processing CAN message: {e}")

    def print_debug_stats(self):
        self.get_logger().info(f"DEBUG STATS - Total CAN messages: {self.total_messages_received}, "
                             f"Position feedback: {self.position_feedback_messages}, "
                             f"Valid actuator messages: {self.valid_actuator_messages}")
        
        with self.angles_lock:
            for i, (angle, offset) in enumerate(zip(self.joint_angles, self.offset_angles)):
                self.get_logger().info(f"Joint {i}: angle={angle}, offset={offset}")

    def publish_joint_angles(self):
        msg = Float64MultiArray()
        with self.angles_lock:
            msg.data = self.joint_angles.copy()
        self.joint_angles_pub.publish(msg)

    def destroy_node(self):
        if self.bus:
            self.bus.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ArmEncoderReaderNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()