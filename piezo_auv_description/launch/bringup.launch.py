import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_piezo_auv_desc = get_package_share_directory('piezo_auv_description')

    # Pass through world arguments (default: underwater.sdf)
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='underwater.sdf',
        description='World file to load from worlds/ directory'
    )

    # Delegate directly to sim.launch.py
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_piezo_auv_desc, 'launch', 'sim.launch.py')
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    return LaunchDescription([
        world_arg,
        sim_launch
    ])