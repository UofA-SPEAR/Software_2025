#!/bin/bash

# Kill existing screen sessions
screen -S rover_launch -X quit 2>/dev/null
screen -S night_cam1 -X quit 2>/dev/null
screen -S night_cam2 -X quit 2>/dev/null

# Start ROS launch in screen
screen -dmS rover_launch bash -c "export ROS_DOMAIN_ID=10; source /home/spear1/Desktop/Software_2025/install/setup.bash; ros2 launch kipp rover_launch.py"

# Start night vision cameras in screen
screen -dmS night_cam1 bash -c "/home/spear1/Desktop/Software_2025/rover/camera_launch/rover_night_cam1.sh"
screen -dmS night_cam2 bash -c "/home/spear1/Desktop/Software_2025/rover/camera_launch/rover_night_cam2.sh"

echo "Rover processes started in screen sessions!"
echo "Available sessions:"
echo "  - rover_launch: Main ROS launch"
echo "  - night_cam1: Night vision camera 1"
echo "  - night_cam2: Night vision camera 2"
echo ""
echo "Commands:"
echo "  screen -list                    # List all sessions"
echo "  screen -r rover_launch          # Attach to ROS session"
echo "  screen -r night_cam1            # Attach to camera 1"
echo "  screen -r night_cam2            # Attach to camera 2"
echo "  screen -S rover_launch -X quit  # Stop ROS session"