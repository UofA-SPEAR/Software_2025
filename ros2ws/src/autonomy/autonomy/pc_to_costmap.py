#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
import numpy as np

class PointCloudToCostmap(Node):
    def __init__(self):
        super().__init__('pc_to_costmap')
        
        # Subscriber: PointCloud2 topic (e.g., from a ZED camera)
        self.pointcloud_subscriber = self.create_subscription(
            PointCloud2,
            '/point_cloud_topic',  # Replace with actual topic name
            self.pointcloud_callback,
            10
        )

        # Publisher: OccupancyGrid topic (e.g., for costmap)
        self.costmap_publisher = self.create_publisher(
            OccupancyGrid,
            '/costmap',
            10
        )

        self.get_logger().info('PointCloudToCostmap Node Initialized.')

    def pointcloud_callback(self, msg):
        """
        Callback function to process incoming PointCloud2 message.
        """
        self.get_logger().info('Received PointCloud2 message.')

        # Example: Convert point cloud to a numpy array
        point_cloud_np = self.process_point_cloud(msg)

        # Example: Generate a simple costmap from the point cloud
        costmap = self.generate_costmap(point_cloud_np)

        # Publish the costmap
        self.costmap_publisher.publish(costmap)
        self.get_logger().info('Published costmap.')

    def process_point_cloud(self, msg):
        """
        Converts PointCloud2 to a numpy array for processing.
        """
        # NOTE: Implement actual conversion logic depending on the PointCloud2 format.
        # This is a placeholder example.
        return np.random.rand(100, 3)  # Example: 100 random 3D points (x, y, z)

    def generate_costmap(self, point_cloud):
        """
        Converts a numpy array of points into an OccupancyGrid.
        """
        # Define grid parameters
        grid_width = 100  # Number of cells in X
        grid_height = 100  # Number of cells in Y
        resolution = 0.1  # Each cell represents 10cm

        # Initialize an empty OccupancyGrid
        costmap = OccupancyGrid()
        costmap.header.stamp = self.get_clock().now().to_msg()
        costmap.header.frame_id = 'map'
        costmap.info.resolution = resolution
        costmap.info.width = grid_width
        costmap.info.height = grid_height
        costmap.info.origin.position.x = -5.0  # Set origin appropriately
        costmap.info.origin.position.y = -5.0
        costmap.info.origin.orientation.w = 1.0

        # Example: Generate costmap data (fill with dummy values for now)
        data = [0] * (grid_width * grid_height)  # Initialize as free space
        for point in point_cloud:
            # Convert each (x, y) point into grid indices
            grid_x = int((point[0] + 5.0) / resolution)
            grid_y = int((point[1] + 5.0) / resolution)

            # Mark the cell as occupied if it's within the grid bounds
            if 0 <= grid_x < grid_width and 0 <= grid_y < grid_height:
                index = grid_y * grid_width + grid_x
                data[index] = 100  # Mark as fully occupied (100)

        costmap.data = data
        return costmap

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToCostmap()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()