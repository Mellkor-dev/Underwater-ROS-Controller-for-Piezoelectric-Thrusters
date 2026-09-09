import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_description = get_package_share_directory('piezo_auv_description')
    pkg_control = get_package_share_directory('piezo_auv_control')

    # Parent directory (install/share) allows model://piezo_auv_description URI lookup
    install_share_dir = os.path.dirname(pkg_description)
    world_path = os.path.join(pkg_description, 'worlds', 'underwater.sdf')
    worlds_dir = os.path.join(pkg_description, 'worlds')

    # Xacro parsing
    xacro_file = os.path.join(pkg_description, 'urdf', 'Centroid_body.urdf.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    # 1. Register parent share path so Gazebo finds model:// URIs & SDF files
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f"{install_share_dir}:{worlds_dir}:" + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    )

    # 2. Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # 3. Gazebo Sim Launch
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r "{world_path}"'}.items(),
    )

    # 4. Spawn Robot Entity
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'Centroid_body', '-z', '0.0'],
        output='screen'
    )

    # 5. Parameter Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/world/underwater_world/wrench/persistent@ros_gz_interfaces/msg/EntityWrench]gz.msgs.EntityWrench',
            '/world/underwater_world/wrench/clear@ros_gz_interfaces/msg/Entity]gz.msgs.Entity',
        ],
        remappings=[
            ('/imu', '/imu/data'),
        ],
        output='screen'
    )

    # 6. PJA Controller Node
    pja_controller_node = Node(
        package='piezo_auv_control',
        executable='pja_controller',
        output='screen'
    )

    return LaunchDescription([
        set_gz_resource_path,
        node_robot_state_publisher,
        gz_sim,
        spawn_entity,
        bridge,
        pja_controller_node
    ])