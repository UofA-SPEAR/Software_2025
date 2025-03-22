import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import Node

def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')
    gps_wpf_dir = get_package_share_directory("kipp_nav")
    
    launch_dir = os.path.join(gps_wpf_dir, 'launch')
    params_dir = os.path.join(gps_wpf_dir, "config")
    nav2_params = os.path.join(params_dir, "kipp_nomap_nav.yaml")
    
    configured_params = RewrittenYaml(
        source_file=nav2_params, root_key="", param_rewrites="", convert_types=True
    )

    navigation2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "false",
            "params_file": configured_params,
            "autostart": "True",
        }.items(),
    )
    
    robot_description_path = os.path.join(
        get_package_share_directory('kipp_description'), 'urdf', 'kipp.urdf.xacro'
    )
    robot_description_content = Command(['xacro ', robot_description_path])
    robot_description = {'robot_description': robot_description_content}
    
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    
    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
        'use_gui': False,
        'source_list': ['/joint_states'],
        'rate': 30,
        'publish_default_positions': True
    }]
    )
    
    ld = LaunchDescription()
    ld.add_action(navigation2_cmd)
    ld.add_action(rsp_node)
    ld.add_action(jsp_node)
    
    return ld
