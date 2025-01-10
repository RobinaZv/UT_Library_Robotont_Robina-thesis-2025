from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # Correct working directory path
    working_dir = '/home/peko/colcon_ws/src/robina_thesis/launch'

    # Define the script name and its arguments
    script_name = 'inventory_minimal_cmd.py'
    script_args = ['Plrm', '/dev/ttyUSB1']

    # Define the watch command with a 0.1-second interval
    watch_command = f'watch -n 0.1 python {script_name} {" ".join(script_args)}'

    return LaunchDescription([
        ExecuteProcess(
            cmd=['/bin/bash', '-c', watch_command],  # Run the watch command using Bash
            cwd=working_dir,                        # Set the correct working directory
            output='screen'                         # Show output on the terminal screen
        )
    ])

