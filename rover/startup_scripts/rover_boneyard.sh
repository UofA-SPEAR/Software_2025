#!/bin/bash

gnome-terminal -- bash -c "export ROS_DOMAIN_ID=10; source /home/prjadmin/Desktop/Software_2025/install/setup.bash; ros2 launch kipp rover_launch.py"

# gnome-terminal -- bash -c "/home/spear1/Desktop/Software_2025/rover/camera_launch/rover_arm_cam.sh"

gnome-terminal -- bash -c "/home/spear1/Desktop/Software_2025/rover/camera_launch/rover_pov_cam.sh"

gnome-terminal -- bash -c "/home/spear1/Desktop/Software_2025/rover/camera_launch/rover_zed_cam.sh"
