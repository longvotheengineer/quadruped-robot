"""
Diagnostic Data Recorder
=========================
Subscribes to all /diag/* topics published by the balance controller pipeline.
All diagnostic topics use individual Float64 signals:
    /diag/home/pid/{roll,pitch}/01_meas .. 09_dt
    /diag/home/apply/01_roll_in .. 09_off_rb
    /diag/gait/pid/{roll,pitch}/01_meas .. 09_ff_ready
    /diag/torque/clamped/{joint_lf_1,joint_lf_2,...}

Stays idle until a 'ZERO' command is received on /gait_control.
On Ctrl+C (SIGINT), writes all buffered data to CSV files in:
    <workspace>/src/simulation/realtime_data/<timestamp>/

Usage:
    ros2 run balance_controller nodeDiagRecorder
    # ... wait for simulation, then send ZERO command ...
    # Ctrl+C to stop → CSVs are saved automatically
"""

import os
import csv
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64


# ── Signal name lists (must match publishers) ────────────────────────
_HOME_PID_SIGNALS = [
    '01_meas', '02_sp', '03_err', '04_p', '05_i',
    '06_d', '07_pid_sat', '08_pid_lpf', '09_dt',
]

_HOME_APPLY_SIGNALS = [
    '01_roll_in', '02_pitch_in', '03_dz_flag',
    '04_roll_scl', '05_pitch_scl',
    '06_off_lf', '07_off_lb', '08_off_rf', '09_off_rb',
]

_GAIT_PID_SIGNALS = [
    '01_meas', '02_avg', '03_err', '04_p', '05_i',
    '06_d', '07_pid_sat', '08_pid_lpf', '09_ff_ready',
]

_TORQUE_JOINTS = [
    'joint_lf_1', 'joint_lf_2', 'joint_lf_3',
    'joint_lb_1', 'joint_lb_2', 'joint_lb_3',
    'joint_rf_1', 'joint_rf_2', 'joint_rf_3',
    'joint_rb_1', 'joint_rb_2', 'joint_rb_3',
]

# ── CSV groups: (buffer_key, signal_list, topic_prefix) ──────────────
_DIAG_GROUPS = [
    ('home_pid_roll',   _HOME_PID_SIGNALS,    '/diag/home/pid/roll'),
    ('home_pid_pitch',  _HOME_PID_SIGNALS,    '/diag/home/pid/pitch'),
    ('home_apply',      _HOME_APPLY_SIGNALS,  '/diag/home/apply'),
    ('gait_pid_roll',   _GAIT_PID_SIGNALS,    '/diag/gait/pid/roll'),
    ('gait_pid_pitch',  _GAIT_PID_SIGNALS,    '/diag/gait/pid/pitch'),
    ('torque_clamped',  _TORQUE_JOINTS,        '/diag/torque/clamped'),
]


class DiagRecorder(Node):
    def __init__(self):
        super().__init__('nodeDiagRecorder')

        # ── State ─────────────────────────────────────────────────
        self.recording = False
        self.t0 = None

        # Build headers, buffers, and accumulators from group config
        self.headers = {}
        self.buffers = {}
        self._accum = {}
        self._signal_lists = {}

        for buf_key, signals, prefix in _DIAG_GROUPS:
            self.headers[buf_key] = ['t'] + list(signals)
            self.buffers[buf_key] = []
            self._accum[buf_key] = {}
            self._signal_lists[buf_key] = signals

            # Subscribe to each signal's individual topic
            for sig in signals:
                topic = f'{prefix}/{sig}'
                self.create_subscription(
                    Float64, topic,
                    lambda msg, k=buf_key, s=sig: self._sig_cb(k, s, msg), 10)

        # ── Output directory (source tree) ────────────────────────
        colcon_prefix = os.environ.get('COLCON_PREFIX_PATH', '')
        if colcon_prefix:
            ws_root = os.path.dirname(colcon_prefix.split(':')[0])
        else:
            ws_root = os.getcwd()
        self.output_root = os.path.join(
            ws_root, 'src', 'simulation', 'realtime_data')

        # ── Subscribe to gait command (trigger) ───────────────────
        self.create_subscription(
            String, '/gait_control', self._gait_cb, 10)

        self.get_logger().info(
            'DiagRecorder ready — waiting for ZERO command to start recording')

    # ── Gait command callback ─────────────────────────────────────
    def _gait_cb(self, msg: String):
        cmd = msg.data.strip().split()[0] if msg.data.strip() else ''
        if cmd == 'ZERO' and not self.recording:
            self.recording = True
            self.t0 = self.get_clock().now()
            self.get_logger().info(
                '▶ Recording started (triggered by ZERO command)')

    # ── Individual signal callback ────────────────────────────────
    def _sig_cb(self, buf_key: str, signal: str, msg: Float64):
        if not self.recording:
            return
        accum = self._accum[buf_key]
        accum[signal] = msg.data

        # Once all signals for this group have arrived, flush a row
        signals = self._signal_lists[buf_key]
        if len(accum) == len(signals):
            t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
            row = [t] + [accum[s] for s in signals]
            self.buffers[buf_key].append(row)
            self._accum[buf_key] = {}

    # ── Save to CSV on shutdown ──────────────────────────────────
    def save_all(self):
        if not self.recording:
            self.get_logger().warn('No data recorded (ZERO was never sent)')
            return

        stamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        out_dir = os.path.join(self.output_root, stamp)
        os.makedirs(out_dir, exist_ok=True)

        total_rows = 0
        for key, rows in self.buffers.items():
            if not rows:
                continue
            path = os.path.join(out_dir, f'{key}.csv')
            with open(path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(self.headers[key])
                writer.writerows(rows)
            total_rows += len(rows)

        self.get_logger().info(
            f'■ Saved {total_rows} total rows → {out_dir}')


def main(args=None):
    rclpy.init(args=args)
    node = DiagRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_all()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

