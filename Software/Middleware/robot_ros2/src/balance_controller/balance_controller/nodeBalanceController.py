"""
Posture Stabilizer Node
========================
Subscribes to /imu/data (sensor_msgs/Imu) from Gazebo IMU plugin.
Converts quaternion → roll, pitch.
Runs PID controllers to compute correction angles.
Publishes corrections on /posture/correction (geometry_msgs/Vector3).

The gait pipeline will subscribe to /posture/correction and apply
the paper's rotation formula to adjust foot positions before IK.
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3


class PostureStabilizer(Node):
    def __init__(self):
        super().__init__('nodeBalanceController')

        # ── PID parameters (conservative to avoid oscillation) ────
        self.Kp = 1.5
        self.Ki = 1.5
        self.Kd = 0.3
        self.sat = 0.5        # ±29° output saturation (rad)
        self.windup = 0.8     # integral can accumulate up to ±0.8 rad·s

        # ── Low-pass filter coefficient ───────────────────────────
        # α = 0 → no filtering (raw PID), α = 1 → frozen output
        self.alpha = 0.85

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

        # ── ROS 2 pub/sub ─────────────────────────────────────────
        self.sub_imu = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)

        self.pub_correction = self.create_publisher(
            Vector3, '/posture/correction', 10)

        self.get_logger().info(
            f'Posture Stabilizer started (Kp={self.Kp}, Ki={self.Ki}, '
            f'Kd={self.Kd}, alpha={self.alpha})')

    # ── Quaternion → Euler ────────────────────────────────────────
    def quaternion_to_rp(self, x, y, z, w):
        """Convert quaternion to roll and pitch (radians)."""
        roll = math.atan2(2.0 * (w * x + y * z),
                          1.0 - 2.0 * (x * x + y * y))
        sinp = 2.0 * (w * y - z * x)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))
        return roll, pitch

    # ── PID compute ───────────────────────────────────────────────
    def pid_compute(self, error, integral, prev_error, dt):
        """Run one PID step. Returns (output, new_integral, new_prev_error)."""
        # Proportional
        P = self.Kp * error

        # Integral with anti-windup
        integral += error * dt
        integral = max(-self.windup, min(self.windup, integral)) # Saturation to prevent integral windup
        I = self.Ki * integral

        # Derivative
        D = self.Kd * (error - prev_error) / dt if dt > 0 else 0.0

        # Total output with saturation
        output = P + I + D
        output = max(-self.sat, min(self.sat, output))

        return output, integral, error

    # ── IMU callback (runs at 100 Hz) ─────────────────────────────
    def imu_callback(self, msg: Imu):
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

        raw_roll, self.roll_integral, self.roll_prev_error = \
            self.pid_compute(roll_error, self.roll_integral,
                             self.roll_prev_error, dt)

        raw_pitch, self.pitch_integral, self.pitch_prev_error = \
            self.pid_compute(pitch_error, self.pitch_integral,
                             self.pitch_prev_error, dt)

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


def main(args=None):
    rclpy.init(args=args)
    node = PostureStabilizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
