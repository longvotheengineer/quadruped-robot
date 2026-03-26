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

        # ── PID parameters ────────────────────────────────────────
        self.Kp = 0.5
        self.Ki = 0.0
        self.Kd = 0.1
        self.sat = 0.052      # ±3° output saturation (rad)
        self.windup = 0.03    # anti-windup integral limit (rad)

        # ── PID state (roll) ──────────────────────────────────────
        self.roll_integral = 0.0
        self.roll_prev_error = 0.0

        # ── PID state (pitch) ─────────────────────────────────────
        self.pitch_integral = 0.0
        self.pitch_prev_error = 0.0

        # ── Timing ────────────────────────────────────────────────
        self.prev_time = None

        # ── Current corrections (published to gait) ───────────────
        self.corr_roll = 0.0
        self.corr_pitch = 0.0

        # ── ROS 2 pub/sub ─────────────────────────────────────────
        self.sub_imu = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)

        self.pub_correction = self.create_publisher(
            Vector3, '/posture/correction', 10)

        self.get_logger().info(
            f'Posture Stabilizer started (Kp={self.Kp}, Ki={self.Ki}, Kd={self.Kd})')

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
        integral = max(-self.windup, min(self.windup, integral))
        I = self.Ki * integral

        # Derivative
        D = self.Kd * (error - prev_error) / dt if dt > 0 else 0.0

        # Total output with saturation
        output = P + I + D
        output = max(-self.sat, min(self.sat, output))

        return output, integral, error

    # ── IMU callback (runs at 100 Hz) ─────────────────────────────
    def imu_callback(self, msg: Imu):
        # 1. Get current time
        now = self.get_clock().now()
        if self.prev_time is None:
            self.prev_time = now
            return
        dt = (now - self.prev_time).nanoseconds * 1e-9
        self.prev_time = now
        if dt <= 0 or dt > 0.1:  # skip bad dt
            return

        # 2. Convert quaternion → roll, pitch
        q = msg.orientation
        roll, pitch = self.quaternion_to_rp(q.x, q.y, q.z, q.w)

        # 3. PID: setpoint = 0 (we want level body)
        roll_error = 0.0 - roll
        pitch_error = 0.0 - pitch

        self.corr_roll, self.roll_integral, self.roll_prev_error = \
            self.pid_compute(roll_error, self.roll_integral,
                             self.roll_prev_error, dt)

        self.corr_pitch, self.pitch_integral, self.pitch_prev_error = \
            self.pid_compute(pitch_error, self.pitch_integral,
                             self.pitch_prev_error, dt)

        # 4. Publish correction
        correction = Vector3()
        correction.x = self.corr_roll    # roll correction (rad)
        correction.y = self.corr_pitch   # pitch correction (rad)
        correction.z = 0.0               # yaw (unused)
        self.pub_correction.publish(correction)


def main(args=None):
    rclpy.init(args=args)
    node = PostureStabilizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
