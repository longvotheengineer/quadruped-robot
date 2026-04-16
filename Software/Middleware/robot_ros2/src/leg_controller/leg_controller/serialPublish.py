import math
from std_msgs.msg import Float64MultiArray
from rclpy.node import Node
from leg_controller.controllerSim import ControllerSim
from sensor_msgs.msg import JointState

class SerialPublish():
    # Standing IK servo_2 value for left legs (computed from inverse(125, 135, -170, 'left-front'))
    # Used to reflect motion direction — see _ik_to_servo_left.
    _LEFT_SERVO2_STANDING = -138.2
    def __init__(self, node):
        self.node = node
        self.get_logger = node.get_logger()

        # Mode: 'sim' (Gazebo) or 'real' (hardware servos)
        self.use_real = node.get_parameter('use_real_hardware').value

        # Publisher for effort commands to Gazebo
        self.pub_sim_gazebo = node.create_publisher(
            Float64MultiArray, '/leg_controller/commands', 10)

        # Publisher for real servo commands (degrees → serial_driver_node)
        self.pub_servo_commands = node.create_publisher(
            Float64MultiArray, '/servo_commands', 10)

        # PID controller for torque computation (Gazebo only)
        self.controller_sim = ControllerSim(node)

        self.real_positions = {}
        if self.use_real:
            node.create_subscription(
                JointState, '/joint_states_real', self._real_joint_cb, 10)

        if self.use_real:
            self.get_logger.info('SerialPublish: REAL HARDWARE mode enabled')
        else:
            self.get_logger.info('SerialPublish: SIMULATION mode (Gazebo)')

    def _real_joint_cb(self, msg):
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.real_positions[name] = msg.position[i]

    # ── IK-to-Servo conversion (empirically verified) ─────────────────

    @classmethod
    def _ik_to_servo_left(cls, theta1_deg, theta2_deg, theta3_deg):
        """Convert raw IK degrees to servo degrees for a LEFT leg.

        Mapping (verified on left-front leg):
          Joint 1 (hip):      direct — same zero, same direction
          Joint 2 (shoulder): reflected — left motor is physically mirrored,
                              so the motion delta must be negated while keeping
                              the standing position unchanged.
          Joint 3 (knee):     flip + offset — bar linkage reverses direction

        Args:
            theta1_deg: IK hip angle in degrees
            theta2_deg: IK shoulder angle in degrees (may be Gazebo-normalized)
            theta3_deg: IK knee angle in degrees

        Returns:
            tuple of (servo1_deg, servo2_deg, servo3_deg)
        """
        # Undo Gazebo normalization on θ2 (kinematics.py adds +360 if < 0)
        if theta2_deg > 180.0:
            theta2_deg -= 360.0

        servo_1 = theta1_deg                # hip: direct
        # Reflect θ2 around standing angle to reverse motion direction:
        # At standing: servo_2 = standing (unchanged)
        # For any delta: servo_2 = standing - delta (reversed)
        servo_2 = 2.0 * cls._LEFT_SERVO2_STANDING - theta2_deg
        servo_3 = -theta3_deg - 90.0        # knee: flip + offset (bar linkage)

        return servo_1, servo_2, servo_3

    @staticmethod
    def _ik_to_servo_right(theta1_deg, theta2_deg, theta3_deg):
        """Convert raw IK degrees to servo degrees for a RIGHT leg.

        The right-side IK produces MIRRORED angles vs left-side:
          LF standing: θ2 ≈ -90°, θ3 ≈ -90° (negative)
          RF standing: θ2 ≈ +90°, θ3 ≈ +90° (positive)

        Same physical servo behavior (verified empirically on RF leg):
          servo 0° = calibration zero = standing position
          servo + = backward (joint 2) / bends more (joint 3)
        """
        # Undo Gazebo normalization on θ2 (kinematics.py subtracts 360 if > 0)
        if theta2_deg < -180.0:
            theta2_deg += 360.0

        servo_1 = theta1_deg                # hip: direct
        servo_2 = theta2_deg         # shoulder: -90 (mirrored from LF's +90)
        servo_3 = theta3_deg - 90.0         # knee: -90, no flip (mirrored from LF)

        return servo_1, servo_2, servo_3

    # ── Publishing methods ────────────────────────────────────────────

    def publish_simulation(self, theta):
        """Publish to Gazebo via PD torque controller."""
        theta_lf_1 = math.radians(theta[0, 0])
        theta_lf_2 = math.radians(theta[0, 1])
        theta_lf_3 = math.radians(theta[0, 2])
        theta_lb_1 = math.radians(theta[1, 0])
        theta_lb_2 = math.radians(theta[1, 1])
        theta_lb_3 = math.radians(theta[1, 2])
        theta_rf_1 = math.radians(theta[2, 0])
        theta_rf_2 = math.radians(theta[2, 1])
        theta_rf_3 = math.radians(theta[2, 2])
        theta_rb_1 = math.radians(theta[3, 0])
        theta_rb_2 = math.radians(theta[3, 1])
        theta_rb_3 = math.radians(theta[3, 2])

        targets = [theta_lf_1, theta_lf_2, theta_lf_3 + math.radians(180),
                   theta_lb_1, theta_lb_2, theta_lb_3 + math.radians(180),
                   theta_rf_1, theta_rf_2, theta_rf_3 + math.radians(180),
                   theta_rb_1, theta_rb_2, theta_rb_3 + math.radians(180)]

        # Wrap targets to nearest ±π of actual position (prevents 2π jumps)
        if self.controller_sim.actual_positions:
            joint_names = self.controller_sim.joint_names
            for i, name in enumerate(joint_names):
                actual = self.controller_sim.actual_positions.get(name, 0.0)
                diff = targets[i] - actual
                # Wrap diff to [-π, π]
                diff = (diff + math.pi) % (2 * math.pi) - math.pi
                targets[i] = actual + diff

        # Compute torques via PID and publish
        torques = self.controller_sim.compute_torques(targets)
        msg_gazebo = Float64MultiArray()
        msg_gazebo.data = torques
        self.pub_sim_gazebo.publish(msg_gazebo)

    def publish_real(self, theta):
        """Publish to real servos via /servo_commands.

        Converts IK angles (degrees) to servo angles using the
        empirically verified mapping, then publishes as a
        Float64MultiArray of 12 servo angles in degrees.

        Args:
            theta: 4×3 numpy array of IK joint angles in degrees.
                   Rows: [LF, LB, RF, RB]
                   Cols: [θ1, θ2, θ3]
        """
        # Left legs
        lf1, lf2, lf3 = self._ik_to_servo_left(
            theta[0, 0], theta[0, 1], theta[0, 2])
        lb1, lb2, lb3 = self._ik_to_servo_left(
            theta[1, 0], theta[1, 1], theta[1, 2])

        # Right legs
        rf1, rf2, rf3 = self._ik_to_servo_right(
            theta[2, 0], theta[2, 1], theta[2, 2])
        rb1, rb2, rb3 = self._ik_to_servo_right(
            theta[3, 0], theta[3, 1], theta[3, 2])

        servo_angles = [lf1, lf2, lf3,
                        lb1, lb2, lb3,
                        rf1, rf2, rf3,
                        rb1, rb2, rb3]

        msg = Float64MultiArray()
        msg.data = servo_angles
        self.pub_servo_commands.publish(msg)

    def publish_message(self, theta):
        """Route to sim or real based on the use_real_hardware parameter."""
        if self.use_real:
            self.publish_real(theta)
        else:
            self.publish_simulation(theta)