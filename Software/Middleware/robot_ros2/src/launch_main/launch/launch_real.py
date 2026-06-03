"""
launch_real.py — Launch file for real quadruped hardware.

Starts two serial drivers (one per bus) and the leg controller node.
Usage:  ros2 launch launch_main launch_real.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


# ─── Calibration & Topology Constants ─────────────────────────────────────────
_TICK_CENTER   = 2048
_TICK_MIN      = 0
_TICK_MAX      = 4095
_DEFAULT_SPEED = 0
_DEFAULT_ACC   = 0

# Servo‐bus A  →  Left-Front (LF) + Left-Back (LB)
_BUS_A_PORT       = '/dev/ttyACM0'
_BUS_A_SERVO_IDS  = [7, 8, 9, 4, 5, 6]
_BUS_A_JOINTS     = [
    'joint_lf_1', 'joint_lf_2', 'joint_lf_3',
    'joint_lb_1', 'joint_lb_2', 'joint_lb_3',
]
_BUS_A_OFFSETS    = [_TICK_CENTER, _TICK_CENTER, _TICK_CENTER,
                     _TICK_CENTER, 3144,         _TICK_CENTER]
_BUS_A_DIRECTIONS = [1, 1, 1, 1, -1, 1]

# Servo‐bus B  →  Right-Front (RF) + Right-Back (RB)
_BUS_B_PORT       = '/dev/ttyACM1'
_BUS_B_SERVO_IDS  = [1, 2, 3, 10, 11, 12]
_BUS_B_JOINTS     = [
    'joint_rf_1', 'joint_rf_2', 'joint_rf_3',
    'joint_rb_1', 'joint_rb_2', 'joint_rb_3',
]
_BUS_B_OFFSETS    = [_TICK_CENTER] * 6
_BUS_B_DIRECTIONS = [1, 1, 1, 1, 1, 1]


def _make_serial_driver_node(*, bus_name: str, port: str, servo_ids: list,
                             joint_names: list, offsets: list,
                             directions: list) -> Node:
    """Create a serial_driver node for one servo bus."""
    return Node(
        package='serial_driver',
        executable='serial_driver_node',
        name=f'node_serial_driver_{bus_name}',
        output='screen',
        parameters=[{
            'port':                  port,
            'num_servos':            len(servo_ids),
            'servo_ids':             servo_ids,
            'joint_names':           joint_names,
            'tick_offsets':           offsets,
            'directions':            directions,
            'tick_min':              [_TICK_MIN] * len(servo_ids),
            'tick_max':              [_TICK_MAX] * len(servo_ids),
            'command_topic':         f'/servo_commands_{bus_name}',
            'single_command_topic':  f'/servo_single_command_{bus_name}',
            'feedback_enable_topic': f'/feedback_enable_{bus_name}',
            'feedback_topic':        f'/joint_states_real_{bus_name}',
            'default_speed':         _DEFAULT_SPEED,
            'default_acc':           _DEFAULT_ACC,
            'enable_feedback':       True,
        }],
    )


def generate_launch_description() -> LaunchDescription:
    # ── Serial drivers (one per servo bus) ────────────────────────────
    node_serial_driver_a = _make_serial_driver_node(
        bus_name='a', port=_BUS_A_PORT, servo_ids=_BUS_A_SERVO_IDS,
        joint_names=_BUS_A_JOINTS, offsets=_BUS_A_OFFSETS,
        directions=_BUS_A_DIRECTIONS,
    )
    node_serial_driver_b = _make_serial_driver_node(
        bus_name='b', port=_BUS_B_PORT, servo_ids=_BUS_B_SERVO_IDS,
        joint_names=_BUS_B_JOINTS, offsets=_BUS_B_OFFSETS,
        directions=_BUS_B_DIRECTIONS,
    )

    # ── Leg Controller ────────────────────────────────────────────────
    node_leg_controller = Node(
        package='leg_controller',
        executable='node_leg_controller',
        name='node_leg_controller',
        output='screen',
        parameters=[{
            'use_real_hardware': True,
        }],
    )

    # ── Web GUI (http://localhost:8080) ────────────────────────────────
    node_web_gui = Node(
        package='leg_controller',
        executable='node_web_gui',
        name='node_web_gui',
        output='screen',
    )

    return LaunchDescription([
        node_serial_driver_a,
        node_serial_driver_b,
        node_leg_controller,
        node_web_gui,
    ])

