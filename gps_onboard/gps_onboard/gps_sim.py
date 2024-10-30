import rclpy # Import ros2 library for python
from rclpy.node import Node # A Node Class in the ROS graph.
from sensor_msgs.msg import NavSatFix # Data type for GPS location
from std_msgs.msg import Float64 # Another useful data type 
import random # generate random numbers to fake gps coordinates
import time # allows code to time events

class DummyGps(Node): # Generate a class for the object type Node

    def __init__(self):
        super().__init__('dummy_gps') # calls the constructor of the parent class
        self.navsat_publisher = self.create_publisher(NavSatFix, '/gps/fix', 10) # publish NavsatFix data type
        self.speed_publisher = self.create_publisher(Float64, '/gps/speed', 10) # publish float64 data type
        self.heading_publisher = self.create_publisher(Float64, '/gps/heading', 10) # publish float64 data type

        self.timer = self.create_timer(2.0, self.timer_callback)  # Periodic callback every 2 seconds

    def timer_callback(self):
        msg = NavSatFix() # Creating a NavSatFix message:
        msg.latitude = random.uniform(-90.0, 90.0) # add latitude to msg
        msg.longitude = random.uniform(-180.0, 180.0) # add longitude to msg
        msg.altitude = random.uniform(0, 10000) # add altitude to msg

        speed_msg = Float64() # create a Float64 msg
        speed_msg.data = random.uniform(0, 120)  # random speed between 0 and 120 m/s

        heading_msg = Float64() # create a Float64 msg
        heading_msg.data = random.uniform(0, 360)  # random heading between 0 and 360 degrees

        self.navsat_publisher.publish(msg) # Publish data
        self.speed_publisher.publish(speed_msg) # Publish data
        self.heading_publisher.publish(heading_msg) # Publish data

        self.get_logger().info(f'Published GPS fix: {msg.latitude}, {msg.longitude}') # Output data
        self.get_logger().info(f'Published speed: {speed_msg.data} m/s') # Output data
        self.get_logger().info(f'Published heading: {heading_msg.data} degrees') # Output data


def main(args=None): # create an instance of this class
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