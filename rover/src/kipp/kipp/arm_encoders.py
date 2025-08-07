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
        
        # Initialize CAN bus
        try:
            self.bus = can.interface.Bus(interface='socketcan', channel='vcan0', bitrate=1000000)
            self.get_logger().info("CAN bus initialized successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize CAN bus: {e}")
            self.bus = None
            return
        
        # Publisher for joint angles
        self.joint_angles_pub = self.create_publisher(
            Float64MultiArray, 
            'arm_joint_angles', 
            10
        )
        
        # Arm actuator IDs mapping (from your CAN docs)
        self.arm_actuator_ids = [
            0x30,  # Joint 0: Shoulder Yaw
            0x31,  # Joint 1: Shoulder Pitch  
            0x32,  # Joint 2: Elbow Pitch
            0x33,  # Joint 3: Elbow Roll
            0x34,  # Joint 4: Wrist Pitch
            0x35,  # Joint 5: Wrist Roll
            0x36   # Joint 6: End Effector
        ]
        
        # Store latest joint angles (initialize to 0.0)
        self.joint_angles = [0.0] * len(self.arm_actuator_ids)
        self.angle_timestamps = [0.0] * len(self.arm_actuator_ids)
        
        # CAN message parameters
        self.position_feedback_command_id = 0x00
        self.jetson_node_id = 0x01
        
        # Thread safety
        self.angles_lock = threading.Lock()
        
        # Start CAN listener thread
        self.can_thread = threading.Thread(target=self.can_listener_thread, daemon=True)
        self.can_thread.start()
        
        # Timer to publish joint angles periodically
        self.publish_timer = self.create_timer(0.1, self.publish_joint_angles)  # 10 Hz
        
        # Timer to check for stale data
        self.staleness_timer = self.create_timer(1.0, self.check_data_staleness)  # 1 Hz
        
        self.get_logger().info("Arm Encoder Reader Node initialized")
        self.get_logger().info(f"Publishing joint angles on: arm_joint_angles")
        self.get_logger().info(f"Listening for position feedback from actuator IDs: {[hex(id) for id in self.arm_actuator_ids]}")

    def can_listener_thread(self):
        """
        Background thread to continuously listen for CAN messages.
        """
        self.get_logger().info("CAN listener thread started")
        
        while rclpy.ok() and self.bus:
            try:
                # Wait for CAN message (timeout after 1 second)
                message = self.bus.recv(timeout=1.0)
                if message:
                    self.process_can_message(message)
                    
            except Exception as e:
                if rclpy.ok():  # Only log if we're not shutting down
                    self.get_logger().error(f"CAN listener error: {e}")
                break
                
        self.get_logger().info("CAN listener thread stopped")

    def process_can_message(self, message):
        """
        Process incoming CAN message and extract position feedback.
        
        Args:
            message (can.Message): Received CAN message
        """
        try:
            # Decode 29-bit arbitration ID
            # Format: Priority (5 bits) | Command ID (8 bits) | Receiver Node ID (8 bits) | Sender Node ID (8 bits)
            arbitration_id = message.arbitration_id
            
            # Extract fields
            sender_node_id = arbitration_id & 0xFF
            receiver_node_id = (arbitration_id >> 8) & 0xFF
            command_id = (arbitration_id >> 16) & 0xFF
            priority = (arbitration_id >> 24) & 0x1F
            
            # Check if this is a position feedback message to us
            if (command_id == self.position_feedback_command_id and 
                receiver_node_id == self.jetson_node_id):
                
                # Check if sender is one of our arm actuators
                if sender_node_id in self.arm_actuator_ids:
                    joint_index = self.arm_actuator_ids.index(sender_node_id)
                    
                    # Decode angle data (float32, big-endian to match your other code)
                    if len(message.data) >= 4:
                        angle = struct.unpack(">f", message.data[:4])[0]
                        
                        # Update stored angle with thread safety
                        with self.angles_lock:
                            self.joint_angles[joint_index] = angle
                            self.angle_timestamps[joint_index] = time.time()
                        
                        self.get_logger().debug(f"Joint {joint_index} (ID: {hex(sender_node_id)}): angle = {angle:.3f} rad ({angle*180/3.14159:.1f}°)")
                    else:
                        self.get_logger().warn(f"Position feedback from {hex(sender_node_id)} has insufficient data: {len(message.data)} bytes")
                        
        except Exception as e:
            self.get_logger().error(f"Error processing CAN message: {e}")

    def publish_joint_angles(self):
        """
        Publish current joint angles as Float64MultiArray.
        """
        msg = Float64MultiArray()
        
        with self.angles_lock:
            msg.data = self.joint_angles.copy()
        
        self.joint_angles_pub.publish(msg)
        
        # Log angles occasionally (every 50 publishes = ~5 seconds at 10Hz)
        if hasattr(self, '_publish_count'):
            self._publish_count += 1
        else:
            self._publish_count = 1
            
        if self._publish_count % 50 == 0:
            angle_str = ", ".join([f"{angle:.2f}" for angle in msg.data])
            self.get_logger().info(f"Current joint angles (rad): [{angle_str}]")

    def check_data_staleness(self):
        """
        Check for stale encoder data and warn if data is old.
        """
        current_time = time.time()
        stale_threshold = 5.0  # seconds
        
        with self.angles_lock:
            for i, timestamp in enumerate(self.angle_timestamps):
                if timestamp > 0 and (current_time - timestamp) > stale_threshold:
                    actuator_id = self.arm_actuator_ids[i]
                    self.get_logger().warn(f"Joint {i} (ID: {hex(actuator_id)}) data is stale (last update: {current_time - timestamp:.1f}s ago)")

    def get_current_joint_angles(self):
        """
        Get current joint angles (thread-safe).
        
        Returns:
            list: Current joint angles in radians
        """
        with self.angles_lock:
            return self.joint_angles.copy()

    def destroy_node(self):
        """
        Clean shutdown.
        """
        self.get_logger().info("Shutting down Arm Encoder Reader Node")
        
        # Close CAN bus
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