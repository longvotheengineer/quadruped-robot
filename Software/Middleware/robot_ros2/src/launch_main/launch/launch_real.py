"""Launch file for real hardware: two serial drivers + leg controller."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # ── Driver A: LF + RB legs (/dev/ttyACM0) ────────────────────────
    driver_a = Node(
        package='serial_driver',
        executable='serial_driver_node',
        name='serial_driver_a',
        output='screen',
        parameters=[{
            'port':            '/dev/ttyACM7',
            'num_servos':      6,
            'servo_ids':       [7, 8, 9, 10, 11, 12],
            'joint_names':     ['joint_lf_1', 'joint_lf_2', 'joint_lf_3',
                                'joint_rb_1', 'joint_rb_2', 'joint_rb_3'],

            # Calibration (LF + RB subset of the original 12-element arrays)
            'tick_offsets':     [2048, 2048, 2048, 2048, 2048, 2048],
            'directions':      [1, 1, 1, 1, 1, 1],
            'tick_min':        [0, 0, 0, 0, 0, 0],
            'tick_max':        [4095, 4095, 4095, 4095, 4095, 4095],

            # Topic names (unique per driver)
            'command_topic':         '/servo_commands_a',
            'single_command_topic':  '/servo_single_command_a',
            'feedback_enable_topic': '/feedback_enable_a',
            'feedback_topic':        '/joint_states_real_a',

            'default_speed':   0,
            'default_acc':     0,
            'enable_feedback': True,
        }],
    )

    # ── Driver B: LB + RF legs (/dev/ttyACM1) ────────────────────────
    driver_b = Node(
        package='serial_driver',
        executable='serial_driver_node',
        name='serial_driver_b',
        output='screen',
        parameters=[{
            'port':            '/dev/ttyACM6',
            'num_servos':      6,
            'servo_ids':       [4, 5, 6, 1, 2, 3],
            'joint_names':     ['joint_lb_1', 'joint_lb_2', 'joint_lb_3',
                                'joint_rf_1', 'joint_rf_2', 'joint_rf_3'],

            # Calibration (LB + RF subset)
            # LB joint2 (index 1 in this 6-element array): direction=-1, offset=3144
            'tick_offsets':     [2048, 3144, 2048, 2048, 2048, 2048],
            'directions':      [1, -1, 1, 1, 1, 1],
            'tick_min':        [0, 0, 0, 0, 0, 0],
            'tick_max':        [4095, 4095, 4095, 4095, 4095, 4095],

            # Topic names (unique per driver)
            'command_topic':         '/servo_commands_b',
            'single_command_topic':  '/servo_single_command_b',
            'feedback_enable_topic': '/feedback_enable_b',
            'feedback_topic':        '/joint_states_real_b',

            'default_speed':   0,
            'default_acc':     0,
            'enable_feedback': True,
        }],
    )

    # ── Leg Controller ────────────────────────────────────────────────
    leg_controller = Node(
        package='leg_controller',
        executable='node_leg_controller',
        name='node_leg_controller',
        output='screen',
        parameters=[{
            'use_real_hardware': True,
        }],
    )

    return LaunchDescription([
        driver_a,
        driver_b,
        leg_controller,
    ])
