"""
Advanced Posture Stabilizer Node
=================================
Subscribes to /imu/data (sensor_msgs/Imu) from Gazebo IMU plugin.
Converts quaternion → roll, pitch.
Runs advanced PID controllers with:
  - Measurement pre-filter (EMA on IMU input)
  - Smooth deadzone (no chattering)
  - Gain scheduling (adapts strength to error magnitude)
  - Integral leakage (prevents long-term bias)
  - Derivative low-pass filter (prevents noise spikes)
  - Back-calculation anti-windup
Publishes corrections on /posture/correction (geometry_msgs/Vector3).

Diagnostic topics (for real-time signal probing like MATLAB scopes):
  /diag/balance/roll/<signal>   — individual Float64 per signal
  /diag/balance/pitch/<signal>  — individual Float64 per signal

  Signals:
    measurement    — filtered IMU angle (rad)
    setpoint       — always 0.0
    error          — setpoint − measurement (after smooth deadzone)
    p_term         — Kp × error (after gain scheduling)
    i_term         — Ki × integral (after windup clamp)
    d_term         — Kd × derivative (filtered)
    raw_pid        — P + I + D (after output saturation)
    filtered_out   — after low-pass filter (α)
    integral_state — accumulated integral value
    dt             — time step (sec)
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64, Bool


# Signal names for diagnostic topics (order matches publish calls)
_DIAG_SIGNALS = [
    'measurement',
    'setpoint',
    'error',
    'p_term',
    'i_term',
    'd_term',
    'raw_pid',
    'filtered_out',
    'integral_state',
    'dt',
]


class PostureStabilizer(Node):
    def __init__(self):
        super().__init__('nodeBalanceController')

        # ── PID base gains ────────────────────────────────────────
        # Reduced from original to prevent limit cycles; joint Kd=0.5
        # now provides the damping that was previously absent.
        # Total loop gain: 0.8 × 2.0(home_gain) × 13(joint_Kp) = 20.8
        self.Kp = 2.0
        self.Ki = 0.30
        self.Kd = 0.15

        # ── Output saturation ────────────────────────────────────
        self.sat = 1.5

        # ── Integral anti-windup clamp ────────────────────────────
        self.windup = 1.5

        # ── Integral leakage factor (per tick) ────────────────────
        # 0.9995 at 100 Hz → half-life ≈ 13.9 s, allows ~20× error
        # integral accumulation while still draining stale bias
        self.integral_leak = 0.9995

        # ── Smooth deadzone threshold (rad) ───────────────────────
        # Instead of hard on/off, linearly ramps from 0 to full
        # correction over [0, deadzone]. Eliminates chattering.
        self.deadzone = 0.02

        # ── Gain scheduling ───────────────────────────────────────
        # At small errors, reduce gain to prevent micro-oscillation.
        # At large errors, full gain for strong correction.
        # gain_factor = clamp(|error| / gs_threshold, gs_min, 1.0)
        self.gs_threshold = 0.08   # error (rad) at which full gain kicks in
        self.gs_min = 0.5          # minimum gain factor (50% at tiny errors)

        # ── Measurement pre-filter (EMA on raw IMU) ──────────────
        # Smooths IMU noise before it enters the PID. Prevents the
        # controller from chasing sensor noise.
        # filtered = α_meas * prev + (1 - α_meas) * raw
        self.alpha_meas = 0.4
        self.meas_roll = 0.0
        self.meas_pitch = 0.0
        self.meas_initialized = False

        # ── Derivative low-pass filter ────────────────────────────
        self.alpha_d = 0.7
        self.roll_d_filtered = 0.0
        self.pitch_d_filtered = 0.0

        # ── Output low-pass filter ────────────────────────────────
        # Reduced from 0.4 to 0.2 — less phase lag, faster response
        self.alpha = 0.15

        # ── PID state (roll) ──────────────────────────────────────
        self.roll_integral = 0.0
        self.roll_prev_error = 0.0

        # ── PID state (pitch) ─────────────────────────────────────
        self.pitch_integral = 0.0
        self.pitch_prev_error = 0.0

        # ── Timing ────────────────────────────────────────────────
        self.prev_time = None

        # ── Filtered output (published to gait) ───────────────────
        self.filtered_roll = 0.0
        self.filtered_pitch = 0.0

        # ── Enable gate (prevents integral pre-wind during homing) ─
        self.enabled = False

        # ── ROS 2 pub/sub ─────────────────────────────────────────
        self.sub_imu = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        self.sub_enable = self.create_subscription(
            Bool, '/balance/enable', self._enable_cb, 10)

        self.pub_correction = self.create_publisher(
            Vector3, '/posture/correction', 10)

        # ── Diagnostic publishers (individual named topics) ───────
        self.diag_roll_pubs = {}
        self.diag_pitch_pubs = {}
        for sig in _DIAG_SIGNALS:
            self.diag_roll_pubs[sig] = self.create_publisher(
                Float64, f'/diag/balance/roll/{sig}', 10)
            self.diag_pitch_pubs[sig] = self.create_publisher(
                Float64, f'/diag/balance/pitch/{sig}', 10)

        self.get_logger().info(
            f'Advanced Posture Stabilizer started '
            f'(Kp={self.Kp}, Ki={self.Ki}, Kd={self.Kd}, '
            f'deadzone={self.deadzone}, gs_thresh={self.gs_threshold})')

    # ── Enable callback ───────────────────────────────────────────
    def _enable_cb(self, msg: Bool):
        """Enable/disable PID. On rising edge, reset all state."""
        if msg.data and not self.enabled:
            self.roll_integral = 0.0
            self.pitch_integral = 0.0
            self.roll_prev_error = 0.0
            self.pitch_prev_error = 0.0
            self.roll_d_filtered = 0.0
            self.pitch_d_filtered = 0.0
            self.filtered_roll = 0.0
            self.filtered_pitch = 0.0
            self.meas_roll = 0.0
            self.meas_pitch = 0.0
            self.meas_initialized = False
            self.prev_time = None
            self.get_logger().info('Balance PID ENABLED — all state reset')
        elif not msg.data and self.enabled:
            self.get_logger().info('Balance PID DISABLED')
        self.enabled = msg.data

    # ── Quaternion → Euler ────────────────────────────────────────
    def quaternion_to_rp(self, x, y, z, w):
        """Convert quaternion to roll and pitch (radians)."""
        roll = math.atan2(2.0 * (w * x + y * z),
                          1.0 - 2.0 * (x * x + y * y))
        sinp = 2.0 * (w * y - z * x)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))
        return roll, pitch

    # ── Smooth deadzone ───────────────────────────────────────────
    @staticmethod
    def smooth_deadzone(value, deadzone):
        """Apply smooth deadzone: zero inside, linearly ramp outside.
        Eliminates the discontinuous jump that causes chattering.
        Output = sign(v) * max(0, |v| - deadzone)
        """
        if abs(value) <= deadzone:
            return 0.0
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - deadzone)

    # ── Gain scheduling ───────────────────────────────────────────
    def gain_schedule(self, error):
        """Compute gain multiplier based on error magnitude.
        Small errors → reduced gain (prevents micro-oscillation).
        Large errors → full gain (strong correction).
        """
        ratio = abs(error) / self.gs_threshold if self.gs_threshold > 0 else 1.0
        return max(self.gs_min, min(1.0, ratio))

    # ── PID compute ───────────────────────────────────────────────
    def pid_compute(self, error, integral, prev_error, d_filtered, dt):
        """Advanced PID step with:
        - Gain scheduling (adapts Kp/Ki to error magnitude)
        - Integral leakage (exponential decay)
        - Derivative low-pass filter
        - Back-calculation anti-windup
        Returns (output, new_integral, new_prev_error, new_d_filtered, P, I_term, D_filtered).
        """
        # Gain scheduling multiplier
        gs = self.gain_schedule(error)

        # Proportional (gain-scheduled)
        P = self.Kp * gs * error

        # Integral with leakage and gain scheduling
        integral *= self.integral_leak  # exponential decay
        integral += error * dt
        integral = max(-self.windup, min(self.windup, integral))
        I = self.Ki * gs * integral

        # Derivative with low-pass filter
        d_raw = self.Kd * (error - prev_error) / dt if dt > 0 else 0.0
        d_filtered = self.alpha_d * d_filtered + (1 - self.alpha_d) * d_raw

        # Reset derivative filter when error is negligible (prevents IIR drift)
        if abs(error) < 0.005:
            d_filtered = 0.0

        # Total output (before saturation)
        output_raw = P + I + d_filtered

        # Saturate output
        output = max(-self.sat, min(self.sat, output_raw))

        # Back-calculation anti-windup
        if abs(output_raw) > self.sat and self.Ki > 0:
            excess = output_raw - output
            integral -= excess / self.Ki * 0.5
            integral = max(-self.windup, min(self.windup, integral))

        return output, integral, error, d_filtered, P, self.Ki * gs * integral, d_filtered

    # ── Publish diagnostics helper ────────────────────────────────
    def _publish_diag(self, pubs, values):
        """Publish a dict of {signal_name: value} to individual topics."""
        msg = Float64()
        for sig, val in zip(_DIAG_SIGNALS, values):
            msg.data = val
            pubs[sig].publish(msg)

    # ── IMU callback (runs at 100 Hz) ─────────────────────────────
    def imu_callback(self, msg: Imu):
        # Skip processing until enabled by gait generator
        if not self.enabled:
            return

        # Get current time to calculate dt
        now = self.get_clock().now()
        if self.prev_time is None:
            self.prev_time = now
            return
        dt = (now - self.prev_time).nanoseconds * 1e-9
        self.prev_time = now
        if dt <= 0 or dt > 0.1:  # skip bad dt
            return

        # Convert quaternion → roll, pitch (raw)
        q = msg.orientation
        raw_roll, raw_pitch = self.quaternion_to_rp(q.x, q.y, q.z, q.w)

        # ── Measurement pre-filter (EMA) ─────────────────────────
        # Smooths IMU noise before the PID processes it
        if not self.meas_initialized:
            self.meas_roll = raw_roll
            self.meas_pitch = raw_pitch
            self.meas_initialized = True
        else:
            self.meas_roll = (self.alpha_meas * self.meas_roll +
                              (1 - self.alpha_meas) * raw_roll)
            self.meas_pitch = (self.alpha_meas * self.meas_pitch +
                               (1 - self.alpha_meas) * raw_pitch)

        # ── Smooth deadzone on measurement ────────────────────────
        # Instead of hard on/off, smoothly ramp from 0 to full
        roll_for_pid = self.smooth_deadzone(self.meas_roll, self.deadzone)
        pitch_for_pid = self.smooth_deadzone(self.meas_pitch, self.deadzone)

        # PID: setpoint = 0
        roll_error = 0.0 - roll_for_pid
        pitch_error = 0.0 - pitch_for_pid

        raw_roll_out, self.roll_integral, self.roll_prev_error, \
            self.roll_d_filtered, roll_P, roll_I, roll_D = \
            self.pid_compute(roll_error, self.roll_integral,
                             self.roll_prev_error, self.roll_d_filtered, dt)

        raw_pitch_out, self.pitch_integral, self.pitch_prev_error, \
            self.pitch_d_filtered, pitch_P, pitch_I, pitch_D = \
            self.pid_compute(pitch_error, self.pitch_integral,
                             self.pitch_prev_error, self.pitch_d_filtered, dt)

        # ── Output low-pass filter ────────────────────────────────
        # Smooths PID output transitions (α=0.2, minimal phase lag)
        self.filtered_roll = (self.alpha * self.filtered_roll +
                              (1 - self.alpha) * raw_roll_out)
        self.filtered_pitch = (self.alpha * self.filtered_pitch +
                               (1 - self.alpha) * raw_pitch_out)

        # Publish filtered correction
        correction = Vector3()
        correction.x = self.filtered_roll    # roll correction (rad)
        correction.y = self.filtered_pitch   # pitch correction (rad)
        correction.z = 0.0                   # yaw (unused)
        self.pub_correction.publish(correction)

        # ── Publish diagnostics (individual named topics) ─────────
        self._publish_diag(self.diag_roll_pubs, [
            self.meas_roll,        # measurement
            0.0,                   # setpoint
            roll_error,            # error
            roll_P,                # p_term
            roll_I,                # i_term
            roll_D,                # d_term
            raw_roll_out,          # raw_pid
            self.filtered_roll,    # filtered_out
            self.roll_integral,    # integral_state
            dt,                    # dt
        ])

        self._publish_diag(self.diag_pitch_pubs, [
            self.meas_pitch,       # measurement
            0.0,                   # setpoint
            pitch_error,           # error
            pitch_P,               # p_term
            pitch_I,               # i_term
            pitch_D,               # d_term
            raw_pitch_out,         # raw_pid
            self.filtered_pitch,   # filtered_out
            self.pitch_integral,   # integral_state
            dt,                    # dt
        ])


def main(args=None):
    rclpy.init(args=args)
    node = PostureStabilizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

