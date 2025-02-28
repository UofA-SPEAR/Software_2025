#For testing purposes only

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
import random
import time

class DummyGps(Node):

    def __init__(self):
        super().__init__('dummy_gps')
        self.navsat_publisher = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.speed_publisher = self.create_publisher(Float64, '/gps/speed', 10)
        self.heading_publisher = self.create_publisher(Float64, '/gps/heading', 10)

        self.timer = self.create_timer(2.0, self.timer_callback)  # Periodic callback every 2 seconds

    def timer_callback(self):
        msg = NavSatFix()
        msg.latitude = 51.0447#random.uniform(-90.0, 90.0)
        msg.longitude = -114.0719#random.uniform(-180.0, 180.0)
        msg.altitude = 1045.0#random.uniform(0, 10000)

        msg.position_covariance = [
            0.0005, 0, 0,
            0, 0.0005, 0,
            0, 0, 0.002
        ]

        msg.position_covariance_type = 2

        speed_msg = Float64()
        speed_msg.data = 10.0 # random speed between 0 and 120 m/s

        heading_msg = Float64()
        heading_msg.data = 60.0 # random heading between 0 and 360 degrees

        self.navsat_publisher.publish(msg)
        self.speed_publisher.publish(speed_msg)
        self.heading_publisher.publish(heading_msg)

        self.get_logger().info(f'Published GPS fix: {msg.latitude}, {msg.longitude}')
        self.get_logger().info(f'Published speed: {speed_msg.data} m/s')
        self.get_logger().info(f'Published heading: {heading_msg.data} degrees')


def main(args=None):
    rclpy.init(args=args)
    dummy_gps = DummyGps()
    try:
        rclpy.spin(dummy_gps)
    except KeyboardInterrupt:
        pass
    dummy_gps.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
