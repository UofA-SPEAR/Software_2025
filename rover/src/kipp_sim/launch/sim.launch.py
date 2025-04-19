import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # --- Package Paths ---
    pkg_kipp_description = FindPackageShare('kipp_description').find('kipp_description')
    pkg_kipp_sim = get_package_share_directory('kipp_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # --- Construct Default World Path ---
    default_world_path = os.path.join(pkg_kipp_sim, 'worlds', 'world.sdf')

    # --- Declare Launch Arguments ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    model_name = LaunchConfiguration('model_name', default='kipp')
    xacro_file_name = LaunchConfiguration('xacro_file_name', default='kipp.urdf.xacro')
    urdf_package_name = LaunchConfiguration('urdf_package_name', default='kipp_description')
    gz_world_file = LaunchConfiguration('gz_world_file', default=default_world_path)

    # *** Add Launch Arguments for Initial Pose ***
    initial_x = LaunchConfiguration('initial_x', default='0.0')
    initial_y = LaunchConfiguration('initial_y', default='0.0')
    initial_z = LaunchConfiguration('initial_z', default='0.5') # Default slightly above ground
    initial_roll = LaunchConfiguration('initial_roll', default='0.0')
    initial_pitch = LaunchConfiguration('initial_pitch', default='0.0')
    initial_yaw = LaunchConfiguration('initial_yaw', default='0.0')

    # --- Construct Xacro Path using Substitutions ---
    xacro_file_path = PathJoinSubstitution([
        FindPackageShare(urdf_package_name),
        'urdf',
        xacro_file_name
    ])

    # --- Process the xacro file ---
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', xacro_file_path
    ])

    # --- Define Nodes ---

    # 1. Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(robot_description_content, value_type=str)
        }]
    )

    # 2. Gazebo Sim Spawner Node
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', robot_description_content,
            '-name', model_name,
            '-allow_renaming', 'true',
            # *** Add Pose Arguments Here ***
            '-x', initial_x,
            '-y', initial_y,
            '-z', initial_z,
            '-R', initial_roll,
            '-P', initial_pitch,
            '-Y', initial_yaw,
        ]
    )

    # 3. Gazebo Sim Launch
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r ', gz_world_file]}.items(),
    )

    # --- Create Launch Description ---
    ld = LaunchDescription()

    # Add launch arguments (Declarations)
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock?'))
    ld.add_action(DeclareLaunchArgument('model_name', default_value='kipp', description='Name of the robot model in Gazebo Sim'))
    ld.add_action(DeclareLaunchArgument('xacro_file_name', default_value='kipp.urdf.xacro', description='Name of the .urdf.xacro file (relative to urdf/ folder)'))
    ld.add_action(DeclareLaunchArgument('urdf_package_name', default_value='kipp_description', description='Package containing the URDF file'))
    ld.add_action(DeclareLaunchArgument('gz_world_file', default_value=default_world_path, description='Gazebo Sim world file to load'))

    # *** Declare Pose Arguments ***
    ld.add_action(DeclareLaunchArgument('initial_x', default_value='0.0', description='Initial X position'))
    ld.add_action(DeclareLaunchArgument('initial_y', default_value='10', description='Initial Y position'))
    ld.add_action(DeclareLaunchArgument('initial_z', default_value='16.45', description='Initial Z position')) # Start slightly above ground
    ld.add_action(DeclareLaunchArgument('initial_roll', default_value='0.0', description='Initial Roll angle (radians)'))
    ld.add_action(DeclareLaunchArgument('initial_pitch', default_value='0.0', description='Initial Pitch angle (radians)'))
    ld.add_action(DeclareLaunchArgument('initial_yaw', default_value='0.0', description='Initial Yaw angle (radians)'))


    # Add nodes and includes
    ld.add_action(gazebo_sim)                  # Start Gazebo Sim first
    ld.add_action(robot_state_publisher_node)  # Then Robot State Publisher
    ld.add_action(spawn_entity_node)           # Finally spawn the entity

    return ld