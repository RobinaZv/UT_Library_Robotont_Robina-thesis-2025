from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Include the inventory launch file
    inventory = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('robina_thesis'), 'launch', 'inventory_launch.py'
            ])
        )
    )

    # Include the first launch file (robotont_driver/example_launch.py)
    example1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('robotont_driver'), 'launch', 'example_launch.py'
            ])
        )
    )

    # Include the second launch file (robotont_driver/example2.py)
    example2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('robotont_driver'), 'launch', 'example2.py'
            ])
        )
    )

    # Include the navigation launch file (nav2_bringup/navigation_launch.py)
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'
            ])
        )
    )

    # Start rosbag to record odom and other topics (e.g., acl)
    rosbag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '/odom', '/amcl_pose'],  # Add other topics as needed
        output='screen'
    )

    return LaunchDescription([
        inventory,
        example1,
        example2,
        #nav,
        rosbag_record,
    ])

