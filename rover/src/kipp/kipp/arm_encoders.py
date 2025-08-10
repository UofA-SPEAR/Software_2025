#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import can
import struct
import threading

class ArmEncoderReaderNode(Node):
    def __init__(self):
        super().__init__('arm_encoder_reader_node')
        
        self.bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=1000000)
        self.joint_angles_pub = self.create_publisher(Float64MultiArray, 'arm_joint_angles', 10)
        
        self.arm_actuator_ids = [0x31, 0x32, 0x34, 0x35, 0x36, 0x16]
        self.initial_values = [0, 0, 0, 0, 0, 0]
        self.joint_multipliers = [0.01, 0.005, 0.0125, 0.02, 0.02, 1]
        self.joint_angles = self.initial_values.copy()
        self.offset_angles = [None] * len(self.arm_actuator_ids)
        self.angles_lock = threading.Lock()
        
        self.can_thread = threading.Thread(target=self.can_listener_thread, daemon=True)
        self.can_thread.start()
        
        self.publish_timer = self.create_timer(0.1, self.publish_joint_angles)

    def can_listener_thread(self):
        while rclpy.ok():
            message = self.bus.recv(timeout=1.0)
            if message:
                self.process_can_message(message)

    def process_can_message(self, message):
        arbitration_id = message.arbitration_id
        sender_node_id = arbitration_id & 0xFF
        receiver_node_id = (arbitration_id >> 8) & 0xFF
        command_id = (arbitration_id >> 16) & 0xFF
        
        print(f"CAN: ID=0x{arbitration_id:08X}, Sender=0x{sender_node_id:02X}, Receiver=0x{receiver_node_id:02X}, Command=0x{command_id:02X}")
        
        if command_id == 0x01 and receiver_node_id == 0x01 and sender_node_id in self.arm_actuator_ids:
            print(f"Processing arm actuator 0x{sender_node_id:02X}")
            joint_index = self.arm_actuator_ids.index(sender_node_id)
            
            if len(message.data) >= 4:
                angle = struct.unpack(">f", message.data[:4])[0]
                print(f"Raw angle from joint {joint_index}: {angle}")
                
                with self.angles_lock:
                    if self.offset_angles[joint_index] is None:
                        self.offset_angles[joint_index] = angle - self.initial_values[joint_index]
                        print(f"Set offset for joint {joint_index}: {self.offset_angles[joint_index]}")
                    
                    offset_angle = angle - self.offset_angles[joint_index]
                    self.joint_angles[joint_index] = offset_angle * self.joint_multipliers[joint_index]
                    print(f"Joint {joint_index} final: {self.joint_angles[joint_index]}")
            else:
                print(f"Insufficient data: {len(message.data)} bytes")

    def publish_joint_angles(self):
        msg = Float64MultiArray()
        with self.angles_lock:
            msg.data = self.joint_angles.copy()
        self.joint_angles_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmEncoderReaderNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()