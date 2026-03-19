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

    # Launch the robot into Gazebo
    # NOTE: Gazebo is already launched by simulation/launch/launch.py (included above)
    launch_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'quadruped_robot', 
                    '-topic', 'robot_description',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.4'],
        output='screen'
    )

    # Launch the Encoder (Reads the joint angles of the virtual motors in Gazebo)
    launch_encoder = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Launch the Angle (Write the angles to the virtual motors in Gazebo)
    launch_angle = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["leg_controller", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription([
        node_leg_controller,
        launch_simulation,
        launch_entity,
        launch_encoder,
        launch_angle
    ])