from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.logging import launch_config
from launch_ros.substitutions import FindPackageShare

import os

def generate_launch_description():
    # Declare arguments with absolute paths
    declared_arguments = [
        DeclareLaunchArgument(
            "world_path",
            default_value="rover/src/kipp_sim/world/heightmap_world.world",
            description="Absolute path to Gazebo world file",
        ),
        
        DeclareLaunchArgument(
            "xacro_path",
            default_value=[FindPackageShare("kipp_description"), "/urdf/kipp.urdf.xacro"],
            description="Path to robot urdf.xacro file",
        ),

    ]

    # Get the values of the arguments
    world_path = LaunchConfiguration("world_path")
    xacro_path = LaunchConfiguration("xacro_path")

    # Generate robot description from xacro file
    robot_description_config = Command(
        [
            "xacro",
            " ",
            xacro_path,
        ]
    )

    # Print to the console (debugging)
    print("XACRO Command: ", ["xacro", " ", xacro_path])  # Print the command itself
    #os.system(f"xacro {LaunchConfiguration('xacro_path').perform(context=None)}") # Another debug method
    #os.system("xacro /home/ayden/Projects/Software_2025/rover/src/kipp_description/urdf/rover.urdf.xacro") # test command


    # Include Gazebo launch file with the absolute world path
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            ["/opt/ros/humble/share/gazebo_ros/launch/gazebo.launch.py"]  # Replace this if you have it in a different location
        ),
        launch_arguments={"world": world_path}.items(),
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_config}],
    )

    # Spawn robot
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "rover",  # You can change this entity name
            "-x",
            "0",  # The initial position of the robot
            "-y",
            "0",
            "-z",
            "0",
        ],
    )

    return LaunchDescription(
        declared_arguments + [gazebo, robot_state_publisher, spawn_robot]
    )