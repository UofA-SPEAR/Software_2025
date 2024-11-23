from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    return LaunchDescription([
       
        Node(
            package='kipp_arm_encoder',
            executable='kipp_arm_can',
            name='kipp_arm_can',
            output='screen',
            emulate_tty=True,
        ),
    
    ])