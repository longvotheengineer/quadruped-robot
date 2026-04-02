"""
Posture Stabilizer Node
========================
Subscribes to /imu/data (sensor_msgs/Imu) from Gazebo IMU plugin.
Converts quaternion → roll, pitch.
Runs PID controllers to compute correction angles.
Publishes corrections on /posture/correction (geometry_msgs/Vector3).

Diagnostic topics (for real-time signal probing like MATLAB scopes):
  /diag/balance/roll   — 10-element Float64MultiArray (roll PID internals)
  /diag/balance/pitch  — 10-element Float64MultiArray (pitch PID internals)

  Index layout:
    [0] measurement   — raw IMU angle (rad)
    [1] setpoint      — always 0.0
    [2] error         — setpoint − measurement
    [3] P_term        — Kp × error
    [4] I_term        — Ki × integral (after windup clamp)
    [5] D_term        — Kd × derivative
    [6] raw_pid       — P + I + D (after output saturation)
    [7] filtered_out  — after low-pass filter (α)
    [8] integral_state— accumulated integral value
    [9] dt            — time step (sec)
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64MultiArray, Bool


class PostureStabilizer(Node):
    def __init__(self):
        super().__init__('nodeBalanceController')

        # ── PID parameters (tuned 2026-04-01 — incremental strengthen) ─
        self.Kp = 1.8           # 1.5→1.8  stronger proportional response
        self.Ki = 0.12          # 0.10→0.12 faster integral build-up
        self.Kd = 0.14          # 0.12→0.14 more damping to match higher P/I
        self.sat = 1.2          # unchanged — plenty of headroom
        self.windup = 1.0       # 0.8→1.0  more integral authority for pitch

        # ── Low-pass filter coefficient ───────────────────────────
        # α = 0 → no filtering (raw PID), α = 1 → frozen output
        self.alpha = 0.4

        # ── Derivative low-pass filter ────────────────────────────
        # Filters the D-term to prevent spikes on rapid ramp changes
        # d_filtered = α_d * d_prev + (1 - α_d) * d_raw
        self.alpha_d = 0.5      # derivative filter coefficient
        self.roll_d_filtered = 0.0
        self.pitch_d_filtered = 0.0

        # ── Output rate limiter (rad/s) ───────────────────────────
        # Prevents sudden jumps in correction when tilt changes rapidly
        self.max_rate = 2.0     # max output change per second

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

        # ── Previous raw output (for rate limiting) ───────────────
        self.prev_raw_roll = 0.0
        self.prev_raw_pitch = 0.0

        # ── Enable gate (prevents integral pre-wind during homing) ─
        self.enabled = False

        # ── ROS 2 pub/sub ─────────────────────────────────────────
        self.sub_imu = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        self.sub_enable = self.create_subscription(
            Bool, '/balance/enable', self._enable_cb, 10)

        self.pub_correction = self.create_publisher(
            Vector3, '/posture/correction', 10)

        # ── Diagnostic publishers (MATLAB-style probes) ───────────
        self.pub_diag_roll = self.create_publisher(
            Float64MultiArray, '/diag/balance/roll', 10)
        self.pub_diag_pitch = self.create_publisher(
            Float64MultiArray, '/diag/balance/pitch', 10)

        self.get_logger().info(
            f'Posture Stabilizer started (Kp={self.Kp}, Ki={self.Ki}, '
            f'Kd={self.Kd}, alpha={self.alpha})')

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
            self.prev_raw_roll = 0.0
            self.prev_raw_pitch = 0.0
            self.prev_time = None
            self.get_logger().info('Balance PID ENABLED — integrals reset')
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

    # ── PID compute ───────────────────────────────────────────────
    def pid_compute(self, error, integral, prev_error, d_filtered, dt):
        """Run one PID step with derivative filtering and back-calc anti-windup.
        Returns (output, new_integral, new_prev_error, new_d_filtered, P, I_term, D_filtered).
        """
        # Proportional
        P = self.Kp * error

        # Integral accumulation (tentative)
        integral += error * dt
        integral = max(-self.windup, min(self.windup, integral))
        I = self.Ki * integral

        # Derivative with low-pass filter (prevents spikes on rapid changes)
        d_raw = self.Kd * (error - prev_error) / dt if dt > 0 else 0.0
        d_filtered = self.alpha_d * d_filtered + (1 - self.alpha_d) * d_raw

        # Total output (before saturation)
        output_raw = P + I + d_filtered

        # Saturate output
        output = max(-self.sat, min(self.sat, output_raw))

        # ── Back-calculation anti-windup ──────────────────────────
        # If output was clipped, undo the integral accumulation that
        # caused the saturation. This prevents integral from growing
        # when the controller is already at its limit.
        if abs(output_raw) > self.sat:
            # How much did saturation eat?
            excess = output_raw - output
            # Remove the excess from integral (scaled by Ki)
            if self.Ki > 0:
                integral -= excess / self.Ki * 0.5  # 0.5 = tracking gain
                integral = max(-self.windup, min(self.windup, integral))

        return output, integral, error, d_filtered, P, self.Ki * integral, d_filtered

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

        # Convert quaternion → roll, pitch
        q = msg.orientation
        roll, pitch = self.quaternion_to_rp(q.x, q.y, q.z, q.w)

        # PID: setpoint = 0
        roll_error = 0.0 - roll
        pitch_error = 0.0 - pitch

        raw_roll, self.roll_integral, self.roll_prev_error, \
            self.roll_d_filtered, roll_P, roll_I, roll_D = \
            self.pid_compute(roll_error, self.roll_integral,
                             self.roll_prev_error, self.roll_d_filtered, dt)

        raw_pitch, self.pitch_integral, self.pitch_prev_error, \
            self.pitch_d_filtered, pitch_P, pitch_I, pitch_D = \
            self.pid_compute(pitch_error, self.pitch_integral,
                             self.pitch_prev_error, self.pitch_d_filtered, dt)

        # ── Rate limiting: cap how fast the PID output can change ─
        max_delta = self.max_rate * dt
        raw_roll  = max(self.prev_raw_roll  - max_delta,
                        min(self.prev_raw_roll  + max_delta, raw_roll))
        raw_pitch = max(self.prev_raw_pitch - max_delta,
                        min(self.prev_raw_pitch + max_delta, raw_pitch))
        self.prev_raw_roll  = raw_roll
        self.prev_raw_pitch = raw_pitch

        # Low-pass filter: smooths output to prevent jitter
        #    filtered = α * filtered_prev + (1-α) * raw_PID
        self.filtered_roll  = self.alpha * self.filtered_roll  + (1 - self.alpha) * raw_roll
        self.filtered_pitch = self.alpha * self.filtered_pitch + (1 - self.alpha) * raw_pitch

        # Publish filtered correction
        correction = Vector3()
        correction.x = self.filtered_roll    # roll correction (rad)
        correction.y = self.filtered_pitch   # pitch correction (rad)
        correction.z = 0.0                   # yaw (unused)
        self.pub_correction.publish(correction)

        # ── Publish diagnostics ───────────────────────────────────
        # Roll: [meas, setpoint, error, P, I, D, raw, filtered, integral, dt]
        diag_roll = Float64MultiArray()
        diag_roll.data = [
            roll,                       # [0] measurement
            0.0,                        # [1] setpoint
            roll_error,                 # [2] error
            roll_P,                     # [3] P_term
            roll_I,                     # [4] I_term
            roll_D,                     # [5] D_term
            raw_roll,                   # [6] raw_pid (saturated)
            self.filtered_roll,         # [7] filtered_output
            self.roll_integral,         # [8] integral_state
            dt,                         # [9] dt
        ]
        self.pub_diag_roll.publish(diag_roll)

        # Pitch: same layout
        diag_pitch = Float64MultiArray()
        diag_pitch.data = [
            pitch,                      # [0] measurement
            0.0,                        # [1] setpoint
            pitch_error,                # [2] error
            pitch_P,                    # [3] P_term
            pitch_I,                    # [4] I_term
            pitch_D,                    # [5] D_term
            raw_pitch,                  # [6] raw_pid (saturated)
            self.filtered_pitch,        # [7] filtered_output
            self.pitch_integral,        # [8] integral_state
            dt,                         # [9] dt
        ]
        self.pub_diag_pitch.publish(diag_pitch)


def main(args=None):
    rclpy.init(args=args)
    node = PostureStabilizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
