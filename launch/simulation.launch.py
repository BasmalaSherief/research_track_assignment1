import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    home = os.path.expanduser('~')
    
    script_path = os.path.join(home, 'ros2_ws', 'src', 'assignment1_rt', 'src', 'turtle_spawn.py')
    
    if not os.path.exists(script_path):
        print(f"ERROR: Cannot find script at {script_path}")
        print("Please verify your folder structure!")

    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),

        Node(
            executable=script_path, 
            name='simple_spawner'
        ),

        Node(
            package='assignment1_rt',
            executable='DistanceControl',
            name='DistanceControl',
            output='screen' 
        ),
    ])