import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class PosePublisher(Node):

    def __init__(self):
        super().__init__('tf2_frame_publisher')
        self.br = TransformBroadcaster(self)

        # Create a timer to publish transforms regularly
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.broadcast_timer_callback)

    def broadcast_timer_callback(self):
        # Now get the transformations.  They'll change/be new
        # due to the cartographer publisher
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'zed_camera_link'

        t.transform.translation.x = 0.0  # Replace with known values
        t.transform.translation.y = 0.0 # Replace with known values
        t.transform.translation.z = 0.0   # Replace with known values

        t.transform.rotation.x = 0.0 # Replace with known values
        t.transform.rotation.y = 0.0 # Replace with known values
        t.transform.rotation.z = 0.0 # Replace with known values
        t.transform.rotation.w = 1.0   # Replace with known values


        # Send the transformation
        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    
    node = PosePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()