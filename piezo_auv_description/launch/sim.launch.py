import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_piezo_auv_desc = get_package_share_directory('piezo_auv_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_piezo_auv_control = get_package_share_directory('piezo_auv_control')

    # Path to URDF xacro
    urdf_path = os.path.join(pkg_piezo_auv_desc, 'urdf', 'Centroid_body.urdf.xacro')
    robot_description_config = xacro.process_file(urdf_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # World configuration argument
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='underwater.sdf',
        description='World file to load from worlds/ directory'
    )
    world_file = PathJoinSubstitution([pkg_piezo_auv_desc, 'worlds', LaunchConfiguration('world')])

    # Configure GZ_SIM_RESOURCE_PATH so Gazebo resolves URIs
    install_share_parent = os.path.dirname(pkg_piezo_auv_desc)
    current_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    new_resource_path = f"{install_share_parent}:{pkg_piezo_auv_desc}:{current_resource_path}"

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=new_resource_path
    )

    # Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # Gazebo Sim launch (-r flag starts simulation immediately upon load)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': [PathJoinSubstitution(['-r', world_file])]}.items(),
    )

    # Spawn robot entity from /robot_description
    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'Centroid_body',
            '-topic', '/robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5'
        ]
    )

    # Parameter Bridge Node
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/world/underwater_world/wrench/persistent@ros_gz_interfaces/msg/EntityWrench]gz.msgs.EntityWrench',
            '/world/underwater_world/wrench/clear@ros_gz_interfaces/msg/Entity]gz.msgs.Entity',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        remappings=[
            ('/model/Centroid_body/imu', '/imu/data'),
        ],
        output='screen'
    )

    # PJA Allocation Controller Node
    pja_controller_node = Node(
        package='piezo_auv_control',
        executable='pja_controller',
        output='screen'
    )

    # Depth Hold PID Node
    depth_hold_node = Node(
        package='piezo_auv_control',
        executable='depth_hold_pid',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        gz_resource_path,
        rsp_node,
        gazebo,
        spawn_node,
        bridge_node,
        pja_controller_node,
        depth_hold_node,
        rviz_node
    ])