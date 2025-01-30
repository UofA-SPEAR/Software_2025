#!/bin/bash

# Call the ROS2 service
ros2 service call /arm_servo/start_servo std_srvs/srv/Trigger {}
