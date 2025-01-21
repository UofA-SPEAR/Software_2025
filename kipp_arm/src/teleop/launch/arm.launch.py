from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction, DeclareLaunchArgument
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
import subprocess


def generate_launch_description():
    
    param_file = os.path.join(get_package_share_directory("moveit_spear"), "config", "moveit_servo_config.yaml")

    joystick_conv = TimerAction(
        period=3.0,
        actions=[Node(
            package='teleop',
            executable='SPEAR_Arm_Node',
        )]
    )

    joy_input = Node(
        package='teleop',
        executable='Joystick_Input',
    )

    # Integrate the small launch file functionality
    moveit_config = MoveItConfigsBuilder("SPEAR_Arm", package_name="moveit_spear").to_moveit_configs()
    demo_launch = generate_demo_launch(moveit_config)  # This should handle RViz implicitly
    
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name='arm_servo',
        parameters=[
            param_file,
            moveit_config.robot_description_kinematics,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description
        ],
        arguments=['--ros-args', '--log-level', 'INFO'],
        output='screen',
    )

    # Initialize the LaunchDescription object
    ld = LaunchDescription()
    ld.add_action(joy_input)
    ld.add_action(joystick_conv)
    ld.add_action(demo_launch)  # Adding the demo launch, which sets up RViz
    ld.add_action(servo_node)

    # Call the ROS2 service to start the servo
    try:
        subprocess.run(['ros2', 'service', 'call', '/arm_servo/start_servo', 'std_srvs/srv/Trigger', '{}'], check=True)
    except subprocess.CalledProcessError as e:
        print(f"Service call failed: {e}")

    return ld
