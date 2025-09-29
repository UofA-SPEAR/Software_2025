#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import serial
import pynmea2

class SimpleGPSNode(Node):
    def __init__(self):
        super().__init__('gps_node')
        
        # Setup serial and publisher
        self.serial_port = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        self.pub = self.create_publisher(NavSatFix, '/fix', 10)
        
        # Read GPS data every 100ms
        self.timer = self.create_timer(0.1, self.read_gps)
        
    def read_gps(self):
        if self.serial_port.in_waiting > 0:
            line = self.serial_port.readline().decode('ascii', errors='replace').strip()
            
            if line.startswith('$GPGGA') or line.startswith('$GPRMC'):
                try:
                    msg = pynmea2.parse(line)
                    
                    if hasattr(msg, 'latitude') and msg.latitude:
                        fix = NavSatFix()
                        fix.header.stamp = self.get_clock().now().to_msg()
                        fix.header.frame_id = 'gps'
                        fix.latitude = float(msg.latitude)
                        fix.longitude = float(msg.longitude)
                        
                        if hasattr(msg, 'altitude') and msg.altitude:
                            fix.altitude = float(msg.altitude)
                        
                        self.pub.publish(fix)
                        
                except:
                    pass

def main():
    rclpy.init()
    node = SimpleGPSNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()