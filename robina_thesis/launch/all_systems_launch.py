from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess

def generate_launch_description():
    # Step 1: Launch navigation immediately
    nav_launch = ExecuteProcess(
        cmd=['ros2', 'launch', 'robina_thesis', 'navigation_launch.py'],
        output='screen'
    )

    # Step 2: Wait 5 seconds, then launch height_changer_node and point_follower
    height_changer = Node(
        package='robina_rfid',
        executable='height_changer_node',
        name='height_changer_node',
        output='screen'
    )

    point_follower = Node(
        package='robina_rfid',
        executable='point_follower',
        name='point_follower',
        output='screen'
    )

    delay_5s_group = TimerAction(
        period=5.0,
        actions=[height_changer, point_follower]
    )

    # Step 3: Wait another 4 seconds, then launch tag_location_processor
    tag_location_processor = Node(
        package='robina_rfid',
        executable='tag_location_processor',
        name='tag_location_processor',
        output='screen'
    )

    delay_9s_group = TimerAction(
        period=9.0,  # 5 + 4 seconds after start
        actions=[tag_location_processor]
    )

    return LaunchDescription([
        nav_launch,
        delay_5s_group,
        delay_9s_group
    ])
