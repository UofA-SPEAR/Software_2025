#!/bin/bash

# Kill existing screen sessions
screen -S rover_launch -X quit 2>/dev/null
screen -S night_cam1 -X quit 2>/dev/null
screen -S night_cam2 -X quit 2>/dev/null
screen -S arm_cam -X quit 2>/dev/null
screen -S zed_cam -X quit 2>/dev/null

# Start ROS launch in screen
screen -dmS rover_launch bash -c "export ROS_DOMAIN_ID=10; source /home/spear1/Desktop/Software_2025/install/setup.bash; ros2 launch kipp rover_launch.py"

# Start night vision cameras in screen
screen -dmS night_cam1 bash -c "/home/spear1/Desktop/Software_2025/rover/camera_launch/rover_night_cam1.sh"
screen -dmS night_cam2 bash -c "/home/spear1/Desktop/Software_2025/rover/camera_launch/rover_night_cam2.sh"
screen -dmS arm_cam bash -c "/home/spear1/Desktop/Software_2025/rover/camera_launch/rover_arm_cam.sh"
screen -dmS zed_cam bash -c "/home/spear1/Desktop/Software_2025/rover/camera_launch/rover_zed_cam.sh"

echo "Rover processes started in screen sessions!"