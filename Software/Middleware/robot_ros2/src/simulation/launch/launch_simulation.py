import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'simulation'
    package_share = get_package_share_directory(package_name)

    urdf_file = os.path.join(get_package_share_directory(package_name), 
        'urdf_new/quadrupedRobot2/urdf', 'quadrupedRobot2.urdf')
    rviz_config_file = os.path.join(get_package_share_directory(package_name), 
        'rviz', 'config.rviz')
    world_file = os.path.join(package_share, 
        'worlds', 'custom_physics.world')

    with open(urdf_file, 'r') as infp:
        robot_description_config = infp.read()
    
    # Replace $(find simulation) with the absolute path to the share directory    
    robot_description_config = robot_description_config.replace('$(find simulation)', package_share)
    # Replace package://simulation/ with file:// absolute paths for Gazebo mesh loading
    robot_description_config = robot_description_config.replace('package://simulation/', 'file://' + package_share + '/')
    
    # Use to control the joint position manually in the GUI mode
    node_joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # Publishes the robot's state to tf2 by processing the URDF and joint state messages
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config}]
    )

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_file}.items()
    )
    # Launch the robot entity into Gazebo
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
    # Launch the Actuator (Write the angles to the virtual motors in Gazebo)
    launch_actuator = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["leg_controller", "--controller-manager", "/controller_manager"],
    )

    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    # Tilts the ramp simultaneously on roll (side-to-side) AND pitch
    # (front-to-back) with independent sine waves.  Tune as needed:
    #   roll_amplitude_deg  / pitch_amplitude_deg  : peak angle (set 0 to disable)
    #   roll_period_sec     / pitch_period_sec      : oscillation period
    #   pitch_phase_deg     : phase offset pitch vs roll (90 = quarter cycle lag)
    node_ramp_mover = Node(
        package='simulation',
        executable='node_ramp_mover',
        name='node_ramp_mover',
        output='screen',
        parameters=[{
            'roll_amplitude_deg':  10.0,
            'roll_period_sec':     50.0,
            'pitch_amplitude_deg': 10.0,
            'pitch_period_sec':    50.0,
            'pitch_phase_deg':     90.0,   # pitch lags roll by T/4
            'update_rate_hz':      50.0,
        }]
    )

    return LaunchDescription([
        # node_joint_state_publisher,
        node_robot_state_publisher,
        gazebo,
        launch_entity,
        launch_encoder,
        launch_actuator,
        node_rviz,
        node_ramp_mover,
    ])