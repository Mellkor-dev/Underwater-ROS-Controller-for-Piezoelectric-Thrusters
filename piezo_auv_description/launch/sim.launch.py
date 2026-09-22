import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_piezo_auv_desc = get_package_share_directory('piezo_auv_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_piezo_auv_control = get_package_share_directory('piezo_auv_control')

    urdf_path = os.path.join(pkg_piezo_auv_desc, 'urdf', 'Centroid_body.urdf.xacro')
    robot_description_config = xacro.process_file(urdf_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='underwater.sdf',
        description='World file to load from worlds/ directory'
    )

    install_share_parent = os.path.dirname(pkg_piezo_auv_desc)
    current_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    new_resource_path = f"{install_share_parent}:{pkg_piezo_auv_desc}:{current_resource_path}"

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=new_resource_path
    )

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [
                '-r -v 4 ',
                pkg_piezo_auv_desc, '/worlds/', LaunchConfiguration('world')
            ]
        }.items(),
    )

    # Spawn robot entity at water level (z = 0.5)
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

    # Directional clock bridge (GZ -> ROS ONLY using '[') prevents WorldControl GUI flickering
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/world/underwater_world/wrench@ros_gz_interfaces/msg/EntityWrench]gz.msgs.EntityWrench',
            '/world/underwater_world/wrench/clear@ros_gz_interfaces/msg/Entity]gz.msgs.Entity',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        remappings=[
            ('/model/Centroid_body/imu', '/imu/data'),
        ],
        output='screen'
    )

    # Delayed instantiation prevents duplicate action execution error
    delayed_pja_controller = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='piezo_auv_control',
                executable='pja_controller',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    delayed_depth_hold = TimerAction(
        period=3.5,
        actions=[
            Node(
                package='piezo_auv_control',
                executable='depth_hold_pid',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    rviz_config_file = os.path.join(pkg_piezo_auv_desc, 'config', 'auv.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        world_arg,
        gz_resource_path,
        rsp_node,
        gazebo,
        spawn_node,
        bridge_node,
        delayed_pja_controller,
        delayed_depth_hold,
        rviz_node
    ])