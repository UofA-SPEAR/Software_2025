#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct
import numpy as np

class RandomPointCloudPublisher(Node):
    def __init__(self):
        super().__init__('random_point_cloud_publisher')

        # Publisher for the point cloud topic
        self.publisher = self.create_publisher(PointCloud2, '/point_cloud_topic', 10)

        # Timer to publish at a fixed interval (e.g., 1 Hz)
        self.timer = self.create_timer(1.0, self.publish_point_cloud)
        self.get_logger().info('Random Point Cloud Publisher Initialized.')

    def publish_point_cloud(self):
        """
        Publish random 3D points as a PointCloud2 message.
        """
        # Generate random points (100 points in this example)
        num_points = 100
        points = np.random.rand(num_points, 3)  # Random (x, y, z) in [0, 1]

        # Log the points being published
        self.get_logger().info(f'Generated points:\n{points}')

        # Convert to PointCloud2 format
        point_cloud_msg = self.create_pointcloud2(points)
        self.publisher.publish(point_cloud_msg)

        self.get_logger().info(f'Published {num_points} random points.')

    def create_pointcloud2(self, points):
        """
        Create a PointCloud2 message from an array of points.
        Each point is a (x, y, z) tuple.
        """
        # Create a proper Header
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'  # Set the coordinate frame ID

        # Define the PointCloud2 fields
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        # Flatten the points array and convert to bytes
        point_data = []
        for point in points:
            point_data.append(struct.pack('fff', *point))
        point_data = b''.join(point_data)

        # Create the PointCloud2 message
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(points)
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 12  # Size of each point in bytes (3 floats * 4 bytes)
        msg.row_step = msg.point_step * len(points)
        msg.data = point_data
        msg.is_dense = True

        return msg

def main(args=None):
    rclpy.init(args=args)
    node = RandomPointCloudPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()