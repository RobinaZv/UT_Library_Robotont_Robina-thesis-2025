from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to the YAML configuration file
    config_file = os.path.join(
        get_package_share_directory('robina_thesis'),
        'config',
        'laser_filter_config.yaml'
    )

    return LaunchDescription([
        # Laser Filter Node
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='laser_filter',
            output='screen',
            parameters=[config_file],  # Load the YAML file
            remappings=[
                ('scan', '/scan_raw'),  # Input topic (raw scan data from RPLidar)
                ('scan_filtered', '/scan')  # Output topic (filtered scan data)
            ]
        ),
    ])
