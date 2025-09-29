import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    joy_node1 = Node(
            package='joy',
            executable='joy_node',
            name='joy_node_1',
            parameters=[{'device_id': 0}],
            remappings=[('joy', 'joy1')],
            namespace='manual',
        )
    
    joy_node2 = Node(
            package='joy',
            executable='joy_node',
            name='joy_node_2',
            parameters=[{'device_id': 1}],
            remappings=[('joy', 'joy2')],
            namespace='manual',
        )

    return launch.LaunchDescription([
        SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),
        joy_node1,
        joy_node2,
    ])


if __name__ == '__main__':
    generate_launch_description()