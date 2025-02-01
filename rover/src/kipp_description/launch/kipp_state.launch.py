import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='kipp_description').find('kipp_description')
    default_model_path = os.path.join(pkg_share, 'urdf/kipp.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/kipp.rviz')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )

    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'kipp', '-topic', '/robot_description'],
        output='screen'
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='model', default_value=default_model_path,
            description='Absolute path to robot URDF file'
        ),
        launch.actions.DeclareLaunchArgument(
            name='rvizconfig', default_value=default_rviz_config_path,
            description='Absolute path to RViz config file'
        ),
        launch.actions.DeclareLaunchArgument(
            name='gui', default_value='true',  
            description='Set to "true" to launch the joint state publisher GUI'
        ),
        launch.actions.ExecuteProcess(
        cmd=['gazebo', '--verbose','/opt/ros/humble/share/gazebo_ros/worlds/empty.world','-s', 'libgazebo_ros_init.so'],
        output='screen'
        ),
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        spawn_entity,
        rviz_node
    ])