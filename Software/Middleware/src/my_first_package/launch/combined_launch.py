from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # TASK 1: Run the Talker
        Node(
            package='my_first_package',
            executable='hello',          # Matches 'entry_points' in setup.py
            name='talker_node'           # We can rename the node here if we want!
        ),
        
        # TASK 2: Run the Listener
        Node(
            package='my_first_package',
            executable='listener',       # Matches 'entry_points' in setup.py
            name='listener_node'
        )
    ])