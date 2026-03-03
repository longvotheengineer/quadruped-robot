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
        executable='single_servo_test',
        name='leg_controller_node',
        output='screen')
    
    # Launch the Robot State Publisher and RViZ
    sim_launch_path = os.path.join(
        get_package_share_directory('simulation'),
        'launch',
        'launch.py'  
    )
    launch_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch_path)
    )

    # Launch the Gazebo Physics Engine
    gazebo_launch_path = os.path.join(
        get_package_share_directory('gazebo_ros'), 
        'launch', 
        'gazebo.launch.py'
    )
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path)
    )

    # Launch the robot into Gazebo
    launch_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'quadruped_robot', '-topic', 'robot_description'],
        output='screen'
    )

    return LaunchDescription([
        node_leg_controller,
        launch_simulation,
        launch_gazebo,
        launch_entity
    ])