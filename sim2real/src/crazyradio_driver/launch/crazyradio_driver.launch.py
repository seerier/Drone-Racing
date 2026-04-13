import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """Builds a LaunchDescription for the CrazyradioDriver app"""
    ld = LaunchDescription()

    # Build config file path
    config = os.path.join(
        get_package_share_directory('crazyradio_driver'), 'config', 'crazyradio_driver.yaml')

    # Declare launch arguments
    config_file = LaunchConfiguration('config_file')
    config_file_launch_arg = DeclareLaunchArgument('config_file', default_value=config)
    ld.add_action(config_file_launch_arg)

    # Create node launch description
    node = Node(
        package='crazyradio_driver',
        executable='crazyradio_driver',
        shell=False,
        emulate_tty=True,
        output='both',
        log_cmd=True,
        parameters=[config_file],
    )
    ld.add_action(node)

    return ld
