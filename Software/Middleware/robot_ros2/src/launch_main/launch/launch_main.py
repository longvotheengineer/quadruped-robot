import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Launch the Controller node
    node_leg_controller = Node(
        package='leg_controller',
        executable='node_leg_controller',
        name='node_leg_controller',
        output='screen')
    
    # Launch the Simulation nodes: 
    # Gazebo, RViZ, Robot State Publisher, Entity Spawner, Encoder, Actuator
    sim_launch_path = os.path.join(
        get_package_share_directory('simulation'),
        'launch',
        'launch_simulation.py'  
    )
    launch_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch_path)
    )

    return LaunchDescription([
        node_leg_controller,
        launch_simulation
    ])