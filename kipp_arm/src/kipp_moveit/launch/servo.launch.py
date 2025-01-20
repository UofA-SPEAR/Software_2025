from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from moveit_configs_utils import MoveItConfigsBuilder


import os

param_file = os.path.join(get_package_share_directory("kipp_moveit"), "config", "moveit_servo_config.yaml")

def generate_launch_description():

    moveit_config = (
         MoveItConfigsBuilder(robot_name = "", package_name ="kipp_moveit")
         .robot_description(file_path="config/kipps_arm.urdf.xacro")
         .to_moveit_configs()
     )

    ld = LaunchDescription()
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name='arm_servo',
        parameters= [
        param_file,
        moveit_config.robot_description_kinematics,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description
        ],
        arguments=['--ros-args', '--log-level', 'INFO'],
        output = 'screen',
        
    )
 
    ld.add_action(servo_node)
    return ld