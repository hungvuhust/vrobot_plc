import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('vrobot_plc')

    # Path to config file
    config_file = os.path.join(pkg_dir, 'config', 'vrobot_plc_params.yaml')

    # Define the node
    vrobot_plc_node = Node(
        package='vrobot_plc',
        executable='vrobot_plc_node',
        name='vrobot_plc_node',
        parameters=[config_file],
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        vrobot_plc_node
    ])
