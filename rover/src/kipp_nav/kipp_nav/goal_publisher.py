import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PointStamped
from nav2_gps_waypoint_follower_demo.utils.gps_utils import latLonYaw2Geopose


class GoalPublisher(Node):
    """
    ROS2 node to send gps waypoints to nav2 every second.
    """
    def __init__(self):
        super().__init__(node_name="gps_wp_commander")
        self.navigator = BasicNavigator("basic_navigator")
        
        # Timer to call the callback every 1 second
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Simulated GPS point for demonstration (You can adjust it based on real-time requirements)
        self.current_point = PointStamped()
        self.current_point.header.frame_id = "wgs84"
        self.current_point.point.x = 0.0  # Longitude
        self.current_point.point.y = 0.0  # Latitude

    def timer_callback(self):
        """
        Callback that is called every 1 second by the timer.
        """
        msg = self.current_point  # Use the current simulated GPS point

        # Check if the point is in the correct frame
        if msg.header.frame_id != "wgs84":
            self.get_logger().warning(
                "Received point is not in wgs84 frame. This is not a GPS point and won't be followed.")
            return

        self.navigator.waitUntilNav2Active(localizer='robot_localization')

        # Convert the point to a geopose and follow the waypoint
        wp = [latLonYaw2Geopose(msg.point.y, msg.point.x)]
        self.navigator.followGpsWaypoints(wp)

        if self.navigator.isTaskComplete():
            self.get_logger().info("Waypoints completed successfully")

def main():
    rclpy.init()
    gps_wpf = GoalPublisher()
    rclpy.spin(gps_wpf)

if __name__ == "__main__":
    main()
