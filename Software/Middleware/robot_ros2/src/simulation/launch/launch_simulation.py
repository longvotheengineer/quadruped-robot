"""
launch_simulation.py — Gazebo simulation stack.

Launches: robot_state_publisher, Gazebo, entity spawner,
          joint_state_broadcaster, leg effort controller,
          RViz, and the ramp disturbance mover.

Included by:  launch_main / launch_sim.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('simulation')

    # ── Asset paths ───────────────────────────────────────────────────
    urdf_path  = os.path.join(pkg_share, 'urdf',   'quadruped_robot.urdf')
    rviz_path  = os.path.join(pkg_share, 'rviz',   'config.rviz')
    world_path = os.path.join(pkg_share, 'worlds', 'custom_physics.world')

    # Load URDF and resolve package:// paths for Gazebo mesh loading
    with open(urdf_path, 'r') as f:
        robot_description = f.read()
    robot_description = robot_description.replace(
        '$(find simulation)', pkg_share)
    robot_description = robot_description.replace(
        'package://simulation/', f'file://{pkg_share}/')

    # ── Robot State Publisher ─────────────────────────────────────────
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # ── Joint State Publisher GUI (disabled — uncomment to enable) ────
    # node_joint_state_publisher = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui',
    #     output='screen',
    # )

    # ── Gazebo ────────────────────────────────────────────────────────
    include_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world_path}.items(),
    )

    # ── Spawn robot entity into Gazebo ────────────────────────────────
    node_spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'quadruped_robot',
                   '-topic', 'robot_description',
                   '-x', '0.0', '-y', '0.0', '-z', '0.4'],
        output='screen',
    )

    # ── ros2_control: joint state broadcaster (encoder) ───────────────
    node_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
    )

    # ── ros2_control: leg effort controller (actuator) ────────────────
    node_leg_effort_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['leg_controller',
                   '--controller-manager', '/controller_manager'],
    )

    # ── RViz ──────────────────────────────────────────────────────────
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_path],
    )

    # ── Ramp disturbance mover (roll + pitch sine waves) ──────────────
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
            'pitch_phase_deg':     90.0,
            'update_rate_hz':      50.0,
        }],
    )

    return LaunchDescription([
        node_robot_state_publisher,
        include_gazebo,
        node_spawn_entity,
        node_joint_state_broadcaster,
        node_leg_effort_controller,
        node_rviz,
        node_ramp_mover,
    ])