"""
Diagnostic Data Recorder
=========================
Subscribes to all /diag/* topics published by the balance controller pipeline.
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
from std_msgs.msg import String, Float64MultiArray


# ── Column headers for each CSV ──────────────────────────────────────
JOINT_NAMES = ['lf1', 'lf2', 'lf3',
               'lb1', 'lb2', 'lb3',
               'rf1', 'rf2', 'rf3',
               'rb1', 'rb2', 'rb3']

HEADERS = {
    'balance_roll':     ['t', 'meas', 'setpoint', 'error', 'P', 'I', 'D',
                         'raw_pid', 'filtered', 'integral', 'dt'],
    'balance_pitch':    ['t', 'meas', 'setpoint', 'error', 'P', 'I', 'D',
                         'raw_pid', 'filtered', 'integral', 'dt'],
    'imu_home_offsets': ['t', 'roll_in', 'pitch_in', 'deadzone',
                         'r_scaled', 'p_scaled',
                         'off_lf', 'off_lb', 'off_rf', 'off_rb'],
    'torque_clamped':   ['t'] + JOINT_NAMES,
}

# Map topic name → buffer key
TOPIC_MAP = {
    '/diag/balance/roll':     'balance_roll',
    '/diag/balance/pitch':    'balance_pitch',
    '/diag/imu_home/offsets': 'imu_home_offsets',
    '/diag/torque/clamped':   'torque_clamped',
}


class DiagRecorder(Node):
    def __init__(self):
        super().__init__('nodeDiagRecorder')

        # ── State ─────────────────────────────────────────────────
        self.recording = False
        self.t0 = None                     # wall-clock reference
        self.buffers = {k: [] for k in HEADERS}

        # ── Output directory (source tree) ────────────────────────
        # COLCON_PREFIX_PATH is set by `source install/setup.bash`
        # and points to the install/ dir.  Its parent is the workspace root.
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

        # ── Subscribe to all diagnostic topics ────────────────────
        for topic, key in TOPIC_MAP.items():
            self.create_subscription(
                Float64MultiArray, topic,
                lambda msg, k=key: self._diag_cb(k, msg), 10)

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

    # ── Generic diagnostic callback ──────────────────────────────
    def _diag_cb(self, key: str, msg: Float64MultiArray):
        if not self.recording:
            return
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        self.buffers[key].append([t] + list(msg.data))

    # ── Save to CSV on shutdown ──────────────────────────────────
    def save_all(self):
        if not self.recording:
            self.get_logger().warn('No data recorded (ZERO was never sent)')
            return

        # Create timestamped output folder
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
                writer.writerow(HEADERS[key])
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
