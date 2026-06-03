"""
launch_sim.py — Launch file for Gazebo simulation.

Starts the leg controller, balance controller, and the full simulation stack.
Usage:  ros2 launch launch_main launch_sim.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # ── Leg Controller ────────────────────────────────────────────────
    node_leg_controller = Node(
        package='leg_controller',
        executable='node_leg_controller',
        name='node_leg_controller',
        output='screen',
    )

    # ── Balance Controller (IMU → PID → correction angles) ───────────
    node_balance_controller = Node(
        package='balance_controller',
        executable='nodeBalanceController',
        name='nodeBalanceController',
        output='screen',
    )

    # ── Simulation stack (Gazebo, RViz, robot_state_publisher, etc.) ─
    sim_launch_path = os.path.join(
        get_package_share_directory('simulation'),
        'launch',
        'launch_simulation.py',
    )
    include_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch_path)
    )

    # ── Web GUI (http://localhost:8080) ───────────────────────────────
    node_web_gui = Node(
        package='leg_controller',
        executable='node_web_gui',
        name='node_web_gui',
        output='screen',
    )

    return LaunchDescription([
        node_leg_controller,
        node_balance_controller,
        include_simulation,
        node_web_gui,
    ])

