"""Omnidirectional Kinematics Node.

Subscribes to /cmd_vel (geometry_msgs/Twist) and computes per-leg stride
amplitudes (X and Y) for omnidirectional locomotion.

Core equation:
  v_foot_i = -(V_body + Omega_body x r_i)
  Stride_X_i = v_foot_i_x * T_stance
  Stride_Y_i = v_foot_i_y * T_stance

Cross product for yaw rotation (omega_z):
  [0, 0, wz] x [rx, ry, 0] = [-wz*ry, +wz*rx, 0]

Publishes /stride_commands (Float64MultiArray) with 8 values:
  [LF_x, LF_y, LB_x, LB_y, RF_x, RF_y, RB_x, RB_y]
"""

import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class OmnidirectionalKinematicsNode(Node):
    def __init__(self):
        super().__init__('omnidirectional_kinematics_node')

        # ROS 2 parameters
        self.declare_parameter('t_stance', 0.6)       # seconds (200 frames * 3ms)
        self.declare_parameter('max_stride', 80.0)     # mm clamp

        # Neutral foot positions (CoM -> foot contact point, mm)
        # Matches GaitConfig.PARAMS_GAIT_FORWARD x_center/y_val
        self.declare_parameter('lf_x',  125.0)
        self.declare_parameter('lf_y',  135.0)
        self.declare_parameter('lb_x', -125.0)
        self.declare_parameter('lb_y',  135.0)
        self.declare_parameter('rf_x',  125.0)
        self.declare_parameter('rf_y', -135.0)
        self.declare_parameter('rb_x', -125.0)
        self.declare_parameter('rb_y', -135.0)

        # Load parameters
        self._t_stance = self.get_parameter('t_stance').value
        self._max_stride = self.get_parameter('max_stride').value

        # Build r_i vectors (CoM -> foot neutral position)
        self._r = {}
        for name, px, py in [('LF', 'lf_x', 'lf_y'), ('LB', 'lb_x', 'lb_y'),
                              ('RF', 'rf_x', 'rf_y'), ('RB', 'rb_x', 'rb_y')]:
            self._r[name] = np.array([
                self.get_parameter(px).value,
                self.get_parameter(py).value, 0.0])

        self._leg_order = ['LF', 'LB', 'RF', 'RB']

        # ROS 2 interfaces
        self._sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_cb, 10)
        self._pub = self.create_publisher(
            Float64MultiArray, '/stride_commands', 10)

        self.get_logger().info(
            f'Omnidirectional node started (T_stance={self._t_stance}s)')

    def _cmd_vel_cb(self, msg: Twist):
        """Compute per-leg strides from body velocity command.

        v_foot_i_x = -(v_x - omega_z * r_y_i)
        v_foot_i_y = -(v_y + omega_z * r_x_i)
        """
        v_x = msg.linear.x * 1000.0      # m/s -> mm/s
        v_y = msg.linear.y * 1000.0
        wz = msg.angular.z                # rad/s

        V = np.array([v_x, v_y, 0.0])
        W = np.array([0.0, 0.0, wz])

        strides = []
        for leg in self._leg_order:
            v_foot = -(V + np.cross(W, self._r[leg]))
            sx = np.clip(v_foot[0] * self._t_stance,
                         -self._max_stride, self._max_stride)
            sy = np.clip(v_foot[1] * self._t_stance,
                         -self._max_stride, self._max_stride)
            strides.extend([float(sx), float(sy)])

        out = Float64MultiArray()
        out.data = strides
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = OmnidirectionalKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
