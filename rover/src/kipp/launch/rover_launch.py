import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    
    arm_control_canbus = Node(
            package='kipp',
            executable='arm_control_canbus',
            name='arm_control_canbus',
        )

    arm_control_xbox = Node(
            package='kipp',
            executable='arm_control_xbox',
            name='arm_control_xbox',
            output='screen',
            emulate_tty=True,
        )
    
    arm_simple_xbox = Node(
            package='kipp',
            executable='arm_simple_xbox',
            name='arm_simple_xbox',
            output='screen',
            emulate_tty=True,
        )
    
    drive_control_canbus = Node(
            package='kipp',
            executable='drive_control_canbus',
            name='drive_control_canbus',
        )
    
    drive_control_xbox = Node(
            package='kipp',
            executable='drive_control_xbox',
            name='drive_control_xbox',
            output='screen',
            emulate_tty=True,
        )
    

    arm_encoders = Node(
            package='kipp',
            executable='arm_encoders',
            name='arm_encoders',
        )

    return launch.LaunchDescription([
        SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),
        arm_control_canbus,
        # arm_control_xbox,
        arm_simple_xbox,
        drive_control_canbus,
        drive_control_xbox,

        # arm_encoders,
    ])


if __name__ == '__main__':
    generate_launch_description()