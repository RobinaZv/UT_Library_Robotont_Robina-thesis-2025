import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
	pkg_name = 'bringup'
	pkg_dir = os.popen('/bin/bash -c "source /usr/share/colcon_cd/function/colcon_cd.sh && \ colcon_cd %s && pwd"' % pkg_name).read().strip()
    
	rviz_config_dir = os.path.join(
            get_package_share_directory('sllidar_ros2'),
            'rviz',
            'myrviz.rviz')
    
    
	sllidar_launch = IncludeLaunchDescription(
    	PythonLaunchDescriptionSource(
        	['/home/peko/colcon_ws/src/sllidar_ros2/launch/view_sllidar_a3_launch.py']
    	),
    	launch_arguments={}.items()
	)
	
	
	filter_launch = IncludeLaunchDescription(
    	PythonLaunchDescriptionSource(
        	['/home/peko/colcon_ws/src/laser_filters/examples/box_filter_example.launch.py']
    	),
    	launch_arguments={}.items()
	)


    
	static_transform2 = Node(
    	package='tf2_ros',
    	executable='static_transform_publisher',
    	name='static_transform_publisher',
    	arguments=['0', '0', '0.15', '3.14', '0', '0', 'base_link', 'laser'],
    	output='screen'
	)
 

	description_launch = IncludeLaunchDescription(
    	PythonLaunchDescriptionSource(
        	os.path.join(get_package_share_directory('robotont_description'), 'launch/display_simulated_robot.launch.py')
    	),
    	launch_arguments={'rviz_fixed_frame': 'odom'}.items()
	)
	return LaunchDescription([
    	sllidar_launch,
    	static_transform2,
    	description_launch,
    	filter_launch,
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'),

	])
