import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_description = get_package_share_directory('piezo_auv_description')
    pkg_control = get_package_share_directory('piezo_auv_control')

    # Path to Xacro / URDF
    xacro_file = os.path.join(pkg_description, 'urdf', 'robot.urdf.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    # 1. Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # 2. Gazebo Sim Launch
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r underwater_world.sdf'}.items(),
    )

    # 3. Spawn Robot Entity in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'Centroid_body', '-z', '0.0'],
        output='screen'
    )

    # 4. Bidirectional Parameter Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            
            # IMU Data (Gazebo -> ROS 2)
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            
            # Persistent Wrench & Clear (ROS 2 -> Gazebo)
            '/world/underwater_world/wrench/persistent@ros_gz_interfaces/msg/EntityWrench]gz.msgs.EntityWrench',
            '/world/underwater_world/wrench/clear@ros_gz_interfaces/msg/Entity]gz.msgs.Entity',
        ],
        remappings=[
            ('/imu', '/imu/data'),
        ],
        output='screen'
    )

    # 5. PJA Controller Node
    pja_controller_node = Node(
        package='piezo_auv_control',
        executable='pja_controller',
        output='screen'
    )

    return LaunchDescription([
        node_robot_state_publisher,
        gz_sim,
        spawn_entity,
        bridge,
        pja_controller_node
    ])