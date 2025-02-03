import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Launch the map_server with the YAML file
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': 's11.yaml'}]
        ),
        
        # Bring up lifecycle for map_server
        Node(
            package='nav2_util',
            executable='lifecycle_bringup',
            name='map_server_lifecycle',
            output='screen',
            parameters=[{'node_name': 'map_server'}]
        ),
        
        # Launch the amcl
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen'
        ),
        
        # Bring up lifecycle for amcl
        Node(
            package='nav2_util',
            executable='lifecycle_bringup',
            name='amcl_lifecycle',
            output='screen',
            parameters=[{'node_name': 'amcl'}]
        ),
    ])
