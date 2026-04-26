#!/usr/bin/env python3
"""
node_ramp_mover.py
==================
Rotates the `tilted_ramp` Gazebo model with independent sine-wave disturbances
on both roll (X axis) and pitch (Y axis) simultaneously, so the robot's balance
controller must handle coupled 2-axis perturbations.

Parameters (ROS2, settable at launch):
  roll_amplitude_deg   : Peak roll angle in degrees          (default 10.0)
  roll_period_sec      : Roll full sine period in seconds     (default 30.0)
  pitch_amplitude_deg  : Peak pitch angle in degrees          (default 10.0)
  pitch_period_sec     : Pitch full sine period in seconds    (default 30.0)
  pitch_phase_deg      : Phase offset of pitch vs roll (deg)  (default 90.0)
  update_rate_hz       : Pose update frequency                (default 50.0)

  Setting an amplitude to 0 effectively disables that axis.

Diagnostic topics (std_msgs/Float64, radians — same unit as IMU meas):
  /diag/home/roll_disturbance      → compare vs /diag/home/roll_error
  /diag/home/pitch_disturbance     → compare vs /diag/home/pitch_error
  /diag/trotting/roll_disturbance   → compare vs /diag/trotting/roll_error
  /diag/trotting/pitch_disturbance  → compare vs /diag/trotting/pitch_error

Requires: libgazebo_ros_state.so loaded in the world file (provides
          /gazebo/set_entity_state service).
"""

import math
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Float64


class RampMoverNode(Node):

    def __init__(self):
        super().__init__('node_ramp_mover')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('roll_amplitude_deg',  10.0)
        self.declare_parameter('roll_period_sec',     30.0)
        self.declare_parameter('pitch_amplitude_deg', 10.0)
        self.declare_parameter('pitch_period_sec',    30.0)
        # Phase offset between pitch and roll (90° → pitch lags roll by T/4)
        self.declare_parameter('pitch_phase_deg',     90.0)
        self.declare_parameter('update_rate_hz',      50.0)

        self._A_roll  = math.radians(self.get_parameter('roll_amplitude_deg').value)
        self._T_roll  = self.get_parameter('roll_period_sec').value
        self._A_pitch = math.radians(self.get_parameter('pitch_amplitude_deg').value)
        self._T_pitch = self.get_parameter('pitch_period_sec').value
        phi_deg       = self.get_parameter('pitch_phase_deg').value
        self._phi     = math.radians(phi_deg)       # pitch phase offset (rad)
        rate_hz       = self.get_parameter('update_rate_hz').value

        self._w_roll  = 2.0 * math.pi / self._T_roll
        self._w_pitch = 2.0 * math.pi / self._T_pitch

        # ── Gazebo service client ─────────────────────────────────────────────
        self._client = self.create_client(SetEntityState,
                                          '/gazebo/set_entity_state')
        self.get_logger().info('Waiting for /gazebo/set_entity_state …')
        if not self._client.wait_for_service(timeout_sec=30.0):
            self.get_logger().error(
                '/gazebo/set_entity_state unavailable after 30 s. '
                'Ensure libgazebo_ros_state.so is loaded in the world file!')
            raise RuntimeError('SetEntityState service unavailable')

        self.get_logger().info(
            f'Ramp mover ready —\n'
            f'  roll:  A={math.degrees(self._A_roll):.1f}°  T={self._T_roll:.1f}s\n'
            f'  pitch: A={math.degrees(self._A_pitch):.1f}°  T={self._T_pitch:.1f}s'
            f'  phase_offset={phi_deg:.1f}°')

        # ── Diagnostic publishers (radians, same unit as IMU meas) ────────────
        # PlotJuggler: 4 disturbance topics (home + trotting × roll + pitch)
        self._pub_home_roll   = self.create_publisher(Float64, '/diag/home/roll_disturbance',     10)
        self._pub_home_pitch  = self.create_publisher(Float64, '/diag/home/pitch_disturbance',    10)
        self._pub_trot_roll   = self.create_publisher(Float64, '/diag/trotting/roll_disturbance',  10)
        self._pub_trot_pitch  = self.create_publisher(Float64, '/diag/trotting/pitch_disturbance', 10)

        # ── Timer ─────────────────────────────────────────────────────────────
        self._t0    = self.get_clock().now().nanoseconds * 1e-9
        self._timer = self.create_timer(1.0 / rate_hz, self._tick)

    # ─────────────────────────────────────────────────────────────────────────
    def _tick(self):
        now     = self.get_clock().now().nanoseconds * 1e-9
        elapsed = now - self._t0

        roll  = self._A_roll  * math.sin(self._w_roll  * elapsed)
        pitch = self._A_pitch * math.sin(self._w_pitch * elapsed + self._phi)

        # Compose roll (X-axis) + pitch (Y-axis) quaternion:
        #   q_roll  = [sin(r/2), 0,        0, cos(r/2)]
        #   q_pitch = [0,        sin(p/2), 0, cos(p/2)]
        #   q = q_roll ⊗ q_pitch
        sr, cr = math.sin(roll  / 2.0), math.cos(roll  / 2.0)
        sp, cp = math.sin(pitch / 2.0), math.cos(pitch / 2.0)

        qw =  cr * cp
        qx =  sr * cp
        qy =  cr * sp
        qz = -sr * sp

        pose = Pose()
        pose.position.x    = 0.0
        pose.position.y    = 0.0
        pose.position.z    = 0.0
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        state = EntityState()
        state.name            = 'tilted_ramp'
        state.pose            = pose
        state.twist           = Twist()   # zero velocity
        state.reference_frame = 'world'

        req = SetEntityState.Request()
        req.state = state
        self._client.call_async(req)

        # ── Publish diagnostic angles (rad) — same value to home & trotting ──
        msg_roll  = Float64(); msg_roll.data  = roll
        msg_pitch = Float64(); msg_pitch.data = pitch
        self._pub_home_roll.publish(msg_roll)
        self._pub_home_pitch.publish(msg_pitch)
        self._pub_trot_roll.publish(msg_roll)
        self._pub_trot_pitch.publish(msg_pitch)


def main(args=None):
    rclpy.init(args=args)
    node = RampMoverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
