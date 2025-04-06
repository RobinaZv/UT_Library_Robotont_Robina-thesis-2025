from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    robina_thesis_launch_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robina_thesis'),
                'launch',
                'example_launch.py'
            )
        )
    )

    robina_thesis_launch_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robina_thesis'),
                'launch',
                'example2.py'
            )
        )
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            )
        )
    )

    return LaunchDescription([
        robina_thesis_launch_1,
        robina_thesis_launch_2,
        nav2_launch
    ])
