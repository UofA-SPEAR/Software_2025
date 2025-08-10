#!/bin/bash

SESSION_NAME="rover_session"

# Kill existing session if it exists
tmux kill-session -t "$SESSION_NAME" 2>/dev/null

# Create new session with first command (ROS launch)
# tmux new-session -d -s "$SESSION_NAME" -n "rover_launch"
# tmux send-keys -t "$SESSION_NAME:rover_launch" "export ROS_DOMAIN_ID=10; source /home/spear1/Desktop/Software_2025/install/setup.bash; ros2 launch kipp rover_launch.py" Enter

# Create new window for POV camera
tmux new-window -t "$SESSION_NAME" -n "pov_cam"
tmux send-keys -t "$SESSION_NAME:pov_cam" "/home/spear1/Desktop/Software_2025/rover/camera_launch/rover_pov_cam.sh" Enter

# Create new window for ZED camera
# tmux new-window -t "$SESSION_NAME" -n "zed_cam"
# tmux send-keys -t "$SESSION_NAME:zed_cam" "/home/spear1/Desktop/Software_2025/rover/camera_launch/rover_zed_cam.sh" Enter

# Optional: Create window for the commented arm camera (uncomment if needed)
# tmux new-window -t "$SESSION_NAME" -n "arm_cam"
# tmux send-keys -t "$SESSION_NAME:arm_cam" "/home/spear1/Desktop/Software_2025/rover/camera_launch/rover_arm_cam.sh" Enter

echo "Rover session started! Use 'tmux attach -t $SESSION_NAME' to view/interact with the processes."
echo "Commands:"
echo "  tmux attach -t $SESSION_NAME    # Attach to session"
echo "  tmux list-windows -t $SESSION_NAME  # List all windows"
echo "  tmux kill-session -t $SESSION_NAME  # Stop all processes"