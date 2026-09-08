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

    # Path to URDF xacro
    urdf_path = os.path.join(pkg_piezo_auv_desc, 'urdf', 'Centroid_body.urdf.xacro')
    robot_description_config = xacro.process_file(urdf_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Path to empty world
    world_file = os.path.join(pkg_piezo_auv_desc, 'worlds', 'empty.sdf')

    # Configure GZ_SIM_RESOURCE_PATH so Gazebo resolves "package://piezo_auv_description/..."
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

    # Gazebo Sim launch
    gz_args = LaunchConfiguration('gz_args')
    declare_gz_args = DeclareLaunchArgument(
        'gz_args',
        default_value=f'-r {world_file}',
        description='Arguments for Gazebo Sim'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args}.items(),
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
            '-z', '0.3'  # 30 cm drop above ground plane
        ]
    )

    # Clock bridge: bridge /clock from Gazebo to ROS
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen'
    )

    return LaunchDescription([
        gz_resource_path,
        declare_gz_args,
        rsp_node,
        gazebo,
        spawn_node,
        bridge_node
    ])
