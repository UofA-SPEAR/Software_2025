import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class SaveGpsNode(Node):

    def __init__(self):
        super().__init__('save_gps_node')
        self.subscription = self.create_subscription(NavSatFix, 'gps/fix', self.gps_callback, 10)
        self.gps_data = None
        self.file_path = 'src/kipp_gps/kipp_gps/gps_coordinates.txt'  # File to save GPS coordinates
        self.timer = self.create_timer(2.0, self.save_gps_callback)  # Save every 2 minutes (120 seconds)
        self.get_logger().info('GPS saving node has been started.')

    def gps_callback(self, msg):
        self.gps_data = msg
        self.get_logger().info(f'Received GPS data: lat={msg.latitude}, lon={msg.longitude}, alt={msg.altitude}')

    def save_gps_callback(self):
        if self.gps_data is not None:
            position = {
                'latitude': self.gps_data.latitude,
                'longitude': self.gps_data.longitude,
                'altitude': self.gps_data.altitude
            }
            self.get_logger().info(f'Saved GPS position: {position}')
            self.write_to_file(position)
        else:
            self.get_logger().warn('No GPS data available to save.')

    def write_to_file(self, position):
        try:
            with open(self.file_path, 'a') as file:
                file.write(f"{position['latitude']}, {position['longitude']}, {position['altitude']}\n")
            self.get_logger().info('GPS position written to file.')
        except Exception as e:
            self.get_logger().error(f'Failed to write to file: {e}')

def main(args=None):
    rclpy.init(args=args)
    save_gps_node = SaveGpsNode()
    rclpy.spin(save_gps_node)
    save_gps_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()