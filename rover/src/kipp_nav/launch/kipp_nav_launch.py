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
    nav_dir = get_package_share_directory('kipp_nav')
    gps_dir = get_package_share_directory("kipp_gps")
    zed_dir = get_package_share_directory("kipp_zed")
    

    nav_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_dir, "launch", "kipp_autonomy.launch.py")
        )
    )
    
    gps_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gps_dir, "launch", "_________.launch.py")
        )
    )
    
    zed_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(zed_dir, "launch", "kipp_zed.launch.py")
        ),
        launch_arguments={
            "camera_model": "zed2i",
        }.items(),
    )
    
    ld = LaunchDescription()
    ld.add_action(zed_cmd)
    ld.add_action(nav_cmd)
    # ld.add_action(gps_node)
    
    return ld
