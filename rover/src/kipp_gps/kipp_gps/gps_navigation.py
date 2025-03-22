import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
import math
import threading

class GpsNavigator(Node):

    def __init__(self):
        super().__init__('gps_navigator')
        self.navsat_subscriber = self.create_subscription(NavSatFix, '/gps/fix', self.navsat_callback, 10)
        self.heading_subscriber = self.create_subscription(Float64, '/gps/heading', self.heading_callback, 10)
        
        self.current_lat = None
        self.current_lon = None
        self.current_heading = None

        self.timer = self.create_timer(2.0, self.timer_callback)  # Periodic callback every 2 seconds

        self.destination_lat = None
        self.destination_lon = None

        self.print_instructions()

        # Start a new thread for user input
        self.user_input_thread = threading.Thread(target=self.get_user_input)
        self.user_input_thread.start()

    def print_instructions(self):
        instructions = """
        Welcome to the GPS Navigator!

        Instructions:
        1. Enter destination coordinates in the format: latitude,longitude
           Example: 34.052235,-118.243683
        2. Type 'stop' to stop the current navigation.
        3. You can enter new coordinates at any time to update the destination.

        Please enter your destination coordinates:
        """
        print(instructions)

    def navsat_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    def heading_callback(self, msg):
        self.current_heading = msg.data

    def timer_callback(self):
        if self.current_lat is not None and self.current_lon is not None and self.destination_lat is not None and self.destination_lon is not None:
            distance, turn_angle = self.calculate_distance_and_turn(self.current_lat, self.current_lon, self.destination_lat, self.destination_lon)
            # Format the output as required
            self.get_logger().info(f"Distance to destination: {distance:.2f} meters | Adjust heading by: {turn_angle:.2f} degrees")

    def get_user_input(self):
        while True:
            user_input = input("Enter destination coordinates (latitude, longitude) or 'stop' to stop navigation: ")
            if user_input.lower() == 'stop':
                self.destination_lat = None
                self.destination_lon = None
                self.get_logger().info('Navigation stopped.')
                continue

            try:
                lat, lon = map(float, user_input.split(','))
                if self.validate_coordinates(lat, lon):
                    self.destination_lat = lat
                    self.destination_lon = lon
                    self.get_logger().info(f'Navigating to: {self.destination_lat}, {self.destination_lon}')
                else:
                    self.get_logger().error('Invalid coordinates format. Please enter valid latitude and longitude.')
            except ValueError:
                self.get_logger().error('Invalid input. Please enter coordinates in the format: latitude, longitude.')

    def validate_coordinates(self, lat, lon):
        return -90 <= lat <= 90 and -180 <= lon <= 180

    def calculate_distance_and_turn(self, lat1, lon1, lat2, lon2):
        R = 6371000  # Radius of the Earth in meters
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)

        a = math.sin(delta_phi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2.0) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        distance = R * c

        y = math.sin(delta_lambda) * math.cos(phi2)
        x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(delta_lambda)
        destination_heading = (math.degrees(math.atan2(y, x)) + 360) % 360

        if self.current_heading is not None:
            turn_angle = (destination_heading - self.current_heading + 360) % 360
            if turn_angle > 180:
                turn_angle -= 360  # Normalize to range [-180, 180]
        else:
            turn_angle = destination_heading

        return distance, turn_angle

def main(args=None):
    rclpy.init(args=args)
    gps_navigator = GpsNavigator()
    try:
        rclpy.spin(gps_navigator)
    except KeyboardInterrupt:
        pass
    gps_navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
