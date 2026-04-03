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

    # Launch the Posture Stabilizer node:
    # Reads IMU data, runs PID, publishes correction angles
    node_balance_controller = Node(
        package='balance_controller',
        executable='nodeBalanceController',
        name='nodeBalanceController',
        output='screen')

    # Launch the Diagnostic Data Recorder node:
    # Idles until ZERO command, then records all /diag/* signals.
    # On Ctrl+C, saves CSV files to simulation/realtime_data/
    # >>> TEMPORARILY DISABLED — uncomment to re-enable recording <<<
    # node_diag_recorder = Node(
    #     package='balance_controller',
    #     executable='nodeDiagRecorder',
    #     name='nodeDiagRecorder',
    #     output='screen')

    return LaunchDescription([
        node_leg_controller,
        node_balance_controller,
        # node_diag_recorder,  # TEMPORARILY DISABLED
        launch_simulation
    ])