from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction

def generate_launch_description():


    joystick_conv = TimerAction(
        period = 3.0,
        actions = [Node(
                package = 'teleop_control',
                executable = 'kipps_arm_node',
        )]
    )

    joy_input = Node(
                package = 'teleop_control',
                executable ='joy_input',
    )

    

    return LaunchDescription([
           
        
        joy_input,
        joystick_conv

    ])