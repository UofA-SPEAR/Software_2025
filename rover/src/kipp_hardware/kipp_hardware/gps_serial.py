import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64
import serial
import pynmea2
from pyudev import Context, Devices

class GPSNode(Node):
    def __init__(self):
        super().__init__('gps_node')
        self.declare_parameter('vendor_id', '1546')  # Example VID
        self.declare_parameter('product_id', '01a9')  # Example PID
        self.declare_parameter('baud_rate', 38400)
        
        vendor_id = self.get_parameter('vendor_id').get_parameter_value().string_value
        product_id = self.get_parameter('product_id').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        
        serial_port = self.find_serial_port(vendor_id, product_id)
        if serial_port:
            self.serial_connection = serial.Serial(serial_port, baud_rate, timeout=1)
        else:
            self.get_logger().error('GPS device not found')
            rclpy.shutdown()

        self.navsat_publisher = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.speed_publisher = self.create_publisher(Float64, '/gps/speed', 10)
        self.heading_publisher = self.create_publisher(Float64, '/gps/heading', 10)

        self.timer = self.create_timer(0.02, self.read_and_publish)
    
    def find_serial_port(self, vendor_id, product_id):
        context = Context()
        for device in context.list_devices(subsystem='tty'):
            if 'ID_VENDOR_ID' in device and 'ID_MODEL_ID' in device:
                if device['ID_VENDOR_ID'] == vendor_id and device['ID_MODEL_ID'] == product_id:
                    return device.device_node
        return None

    def read_and_publish(self):
        if not self.serial_connection:
            return
        line = self.serial_connection.readline().decode('ascii', errors='replace')
        if line.startswith('$'):
            try:
                msg = pynmea2.parse(line)
                #print(msg)
                if isinstance(msg, pynmea2.types.talker.GGA):
                    navsat_msg = NavSatFix()
                    navsat_msg.header.stamp = self.get_clock().now().to_msg()
                    navsat_msg.header.frame_id = 'gps'
                    if msg.gps_qual == 0:  # Check for no fix
                        self.get_logger().warn('No GPS fix available.')
                        navsat_msg.status.status = NavSatStatus.STATUS_NO_FIX
                        navsat_msg.latitude = float('nan')
                        navsat_msg.longitude = float('nan')
                        navsat_msg.altitude = float('nan')
                    else:
                        navsat_msg.status.status = NavSatStatus.STATUS_FIX  # Assuming a fix
                        navsat_msg.latitude = msg.latitude
                        navsat_msg.longitude = msg.longitude
                        navsat_msg.altitude = msg.altitude if msg.altitude != '' else float('nan')
                    navsat_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
                    self.navsat_publisher.publish(navsat_msg)
                elif isinstance(msg, pynmea2.types.talker.RMC):
                    if msg.status == 'V':  # Check for invalid (void) fix status
                        self.get_logger().warn('GPS fix not valid.')
                        speed = -1.0  # Indicate error/invalid state
                        heading = -1.0  # Indicate error/invalid state
                    else:
                        try:
                            speed = float(msg.spd_over_grnd) if msg.spd_over_grnd else -1.0
                        except ValueError:
                            speed = -1.0  # Default to error value if conversion fails
                        try:
                            heading = float(msg.true_course) if msg.true_course else -1.0
                        except ValueError:
                            heading = -1.0  # Default to error value if conversion fails

                    speed_msg = Float64()
                    speed_msg.data = speed
                    self.speed_publisher.publish(speed_msg)
                    
                    heading_msg = Float64()
                    heading_msg.data = heading
                    self.heading_publisher.publish(heading_msg)
            except pynmea2.ParseError as e:
                self.get_logger().warn(f'Failed to parse NMEA sentence: {e}')


def main(args=None):
    rclpy.init(args=args)

    gps_node = GPSNode()

    rclpy.spin(gps_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gps_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()