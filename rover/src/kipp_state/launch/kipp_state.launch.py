import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command  # Import Command for xacro processing


def generate_launch_description():
    # Path to the URDF file (Xacro file)
    urdf_file = os.path.join(
        get_package_share_directory('kipp_description'),  # Ensure this is the correct package name
        'urdf',
        'kipp.urdf.xacro'  # Ensure this is the correct Xacro file name
    )

    # Use xacro to preprocess the URDF file
    robot_description = Command(['xacro ', urdf_file])

    # Define the joint_state_publisher node
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'source_list':["wheel_encoders/"]}]
    )
    # Define the robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )
    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the nodes to the launch description
    ld.add_action(joint_state_publisher_node)
    ld.add_action(robot_state_publisher_node)
    return ld


    

   

