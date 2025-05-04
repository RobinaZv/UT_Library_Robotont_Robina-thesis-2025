import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Path to your map file
    map_file = os.path.join(os.getcwd(), 's92.yaml')

    return LaunchDescription([

        # Map server node
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_file}]
        ),

        # Map server lifecycle bringup
        Node(
            package='nav2_util',
            executable='lifecycle_bringup',
            name='lifecycle_bringup_map_server',
            output='screen',
            arguments=['map_server']
        ),

        # AMCL node
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen'
        ),

        # AMCL lifecycle bringup
        Node(
            package='nav2_util',
            executable='lifecycle_bringup',
            name='lifecycle_bringup_amcl',
            output='screen',
            arguments=['amcl']
        )
    ])

