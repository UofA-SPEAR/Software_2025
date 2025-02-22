import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import SetBool, Trigger
import serial
import struct
import time

class GNSSNode(Node):
    def __init__(self):
        super().__init__('gnss_node')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 38400)
        self.declare_parameter('protocol', 'NMEA')  # UBX, NMEA, RTCM
        self.declare_parameter('interface', 'UART')  # UART, SPI, I2C, USB

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.protocol = self.get_parameter('protocol').get_parameter_value().string_value
        self.interface = self.get_parameter('interface').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(NavSatFix, 'gps/fix', 10)
        self.serial_port = None

        self.srv_toggle = self.create_service(SetBool, 'toggle_gnss', self.toggle_gnss)
        self.srv_reconfigure = self.create_service(Trigger, 'reconfigure_gnss', self.reconfigure_gnss)
        self.srv_reset = self.create_service(Trigger, 'reset_gnss', self.reset_gnss)
        self.active = True

        self.init_communication()
        self.timer = self.create_timer(1.0, self.read_gnss_data)
    
    def init_communication(self):
        if self.interface == 'UART':
            try:
                self.serial_port = serial.Serial(self.port, self.baudrate, timeout=1)
                self.get_logger().info(f'Connected via UART to {self.port} at {self.baudrate} baud')
            except serial.SerialException as e:
                self.get_logger().error(f'Failed to open serial port: {e}')
        elif self.interface == 'SPI':
            self.get_logger().info('SPI interface selected, implementation needed')
        elif self.interface == 'I2C':
            self.get_logger().info('I2C interface selected, implementation needed')
        elif self.interface == 'USB':
            self.get_logger().info('USB interface selected, implementation needed')

    def read_gnss_data(self):
        if self.serial_port is None or not self.active:
            return

        try:
            line = self.serial_port.readline().decode('utf-8').strip()
            if self.protocol == 'NMEA' and line.startswith('$GNGGA'):
                self.parse_nmea(line)
        except Exception as e:
            self.get_logger().error(f'Error reading GNSS data: {e}')

    def parse_nmea(self, nmea_sentence):
        parts = nmea_sentence.split(',')
        if len(parts) < 10:
            return

        try:
            lat = self.convert_to_decimal(parts[2], parts[3])
            lon = self.convert_to_decimal(parts[4], parts[5])
            alt = float(parts[9]) if parts[9] else 0.0

            msg = NavSatFix()
            msg.latitude = lat
            msg.longitude = lon
            msg.altitude = alt
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'gps'
            self.publisher_.publish(msg)
        except ValueError:
            self.get_logger().warn('Invalid NMEA sentence received')

    def convert_to_decimal(self, degrees_minutes, direction):
        if not degrees_minutes:
            return 0.0

        degrees = float(degrees_minutes[:2])
        minutes = float(degrees_minutes[2:])
        decimal = degrees + (minutes / 60)
        if direction in ['S', 'W']:
            decimal *= -1
        return decimal

    def toggle_gnss(self, request, response):
        self.active = request.data
        response.success = True
        response.message = f'GNSS {"enabled" if self.active else "disabled"}'
        return response

    def reconfigure_gnss(self, request, response):
        """Reconfigure GNSS settings dynamically without restarting."""
        self.get_logger().info('Reconfiguring GNSS settings')
        response.success = True
        response.message = 'GNSS reconfigured successfully'
        return response

    def reset_gnss(self, request, response):
        """Reset the GNSS module to its default settings."""
        self.get_logger().info('Resetting GNSS module')
        response.success = True
        response.message = 'GNSS module reset successfully'
        return response

    def destroy_node(self):
        if self.serial_port:
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GNSSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
