#!/usr/bin/env python3
import math
import threading
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

def haversine(lat1, lon1, lat2, lon2):
    R = 6371000.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    return 2*R*math.atan2(math.sqrt(a), math.sqrt(1-a))

def bearing(lat1, lon1, lat2, lon2):
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dlambda = math.radians(lon2 - lon1)
    y = math.sin(dlambda) * math.cos(phi2)
    x = math.cos(phi1)*math.sin(phi2) - math.sin(phi1)*math.cos(phi2)*math.cos(dlambda)
    brng = math.degrees(math.atan2(y, x))
    return (brng + 360) % 360

class GpsNavigator(Node):
    def __init__(self):
        super().__init__('gps_navigator')
        self.target_lat = None
        self.target_lon = None
        self.current_lat = None
        self.current_lon = None
        self.running = True

        self.subscription = self.create_subscription(NavSatFix, 'fix', self.gps_callback, 10)
        self.timer = self.create_timer(1.0, self.print_status)  # Print every 1 sec

        threading.Thread(target=self.input_thread, daemon=True).start()

    def gps_callback(self, msg):
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    def print_status(self):
        if not self.running:
            return
        if self.current_lat is None or self.current_lon is None:
            self.get_logger().info("Waiting for GPS fix...")
            return
        if self.target_lat is not None and self.target_lon is not None:
            dist = haversine(self.current_lat, self.current_lon, self.target_lat, self.target_lon)
            brng = bearing(self.current_lat, self.current_lon, self.target_lat, self.target_lon)
            direction = self.bearing_to_direction(brng)
            self.get_logger().info(
                f"Current: {self.current_lat:.6f}, {self.current_lon:.6f} | "
                f"Target: {self.target_lat:.6f}, {self.target_lon:.6f} | "
                f"Dist: {dist:.1f} m | Bearing: {brng:.1f}° ({direction})"
            )
        else:
            self.get_logger().info(f"Current: {self.current_lat:.6f}, {self.current_lon:.6f} | No target set")

    def bearing_to_direction(self, brng):
        dirs = ['N','NE','E','SE','S','SW','W','NW']
        idx = round(brng / 45) % 8
        return dirs[idx]

    def input_thread(self):
        while True:
            input("Press Enter to set new target...")
            self.running = False
            try:
                line = input("Enter target lat lon: ").strip()
                lat_str, lon_str = line.split()
                self.target_lat = float(lat_str)
                self.target_lon = float(lon_str)
                self.get_logger().info(f"Updated target to: {self.target_lat}, {self.target_lon}")
            except Exception as e:
                self.get_logger().warn(f"Invalid input: {e}")
            self.running = True

def main():
    rclpy.init()
    node = GpsNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
