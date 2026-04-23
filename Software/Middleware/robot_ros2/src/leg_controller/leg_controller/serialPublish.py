import math
from std_msgs.msg import Float64MultiArray
from rclpy.node import Node
from leg_controller.controllerSim import ControllerSim
from sensor_msgs.msg import JointState

class SerialPublish():
    def __init__(self, node):
        self.node = node
        self.get_logger = node.get_logger()

        # Mode: 'sim' (Gazebo) or 'real' (hardware servos)
        self.use_real = node.get_parameter('use_real_hardware').value

        # Publisher for effort commands to Gazebo
        self.pub_sim_gazebo = node.create_publisher(
            Float64MultiArray, '/leg_controller/commands', 10)

        # Publishers for real servo commands (degrees → serial_driver_node)
        # Two drivers: A handles LF+RB, B handles LB+RF
        self.pub_servo_commands_a = node.create_publisher(
            Float64MultiArray, '/servo_commands_a', 10)
        self.pub_servo_commands_b = node.create_publisher(
            Float64MultiArray, '/servo_commands_b', 10)

        # PID controller for torque computation (Gazebo only)
        self.controller_sim = ControllerSim(node)

        self.real_positions = {}
        if self.use_real:
            # Subscribe to feedback from both drivers
            node.create_subscription(
                JointState, '/joint_states_real_a', self._real_joint_cb, 10)
            node.create_subscription(
                JointState, '/joint_states_real_b', self._real_joint_cb, 10)

        if self.use_real:
            self.get_logger.info('SerialPublish: REAL HARDWARE mode enabled')
            self.get_logger.info('  Driver A topic: /servo_commands_a (LF+RB)')
            self.get_logger.info('  Driver B topic: /servo_commands_b (LB+RF)')
        else:
            self.get_logger.info('SerialPublish: SIMULATION mode (Gazebo)')

    def _real_joint_cb(self, msg):
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.real_positions[name] = msg.position[i]

    # ── IK-to-Servo conversion (empirically verified) ─────────────────

    @staticmethod
    def _ik_to_servo_lf(theta1_deg, theta2_deg, theta3_deg):
        """Convert raw IK degrees to servo degrees for the LEFT-FRONT leg.

        Args:
            theta1_deg: IK hip angle in degrees
            theta2_deg: IK shoulder angle in degrees
            theta3_deg: IK knee angle in degrees

        Returns:
            tuple of (servo1_deg, servo2_deg, servo3_deg)
        """
        # Undo normalization (kinematics.py adds +360 if < 0 for LF)
        # if theta2_deg > 180.0:
        #     theta2_deg -= 360.0

        servo_1 = theta1_deg                # hip: direct
        # theta2_deg = 180
        servo_2 = -(theta2_deg - 90)               # shoulder: direct
        # theta3_deg = -90
        servo_3 = theta3_deg + 90.0         # knee: bar linkage offset

        return servo_1, servo_2, servo_3

    @staticmethod
    def _ik_to_servo_lb(theta1_deg, theta2_deg, theta3_deg):
        """Convert raw IK degrees to servo degrees for the LEFT-BEHIND leg.

        LB joint2 servo physically rotates opposite to LF (same as right legs).
        The negation here keeps ticks within the EEPROM range [2046, 3763];
        the serial driver compensates with direction=-1 and tick_offset=3144.

        LB IK θ2 sign depends on foot placement relative to the hip:
          x_center < −L/2 → θ2 negative (no wrap)
          x_center > −L/2 → θ2 wraps past +180° (needs normalization)

        Args:
            theta1_deg: IK hip angle in degrees
            theta2_deg: IK shoulder angle in degrees
            theta3_deg: IK knee angle in degrees

        Returns:
            tuple of (servo1_deg, servo2_deg, servo3_deg)
        """
        # Normalize θ2 into (−180, +180] range.
        if theta2_deg > 180.0:
            theta2_deg -= 360.0

        servo_1 = theta1_deg                # hip: direct
        servo_2 = theta2_deg + 180          # shoulder: negated (driver inverts)
        servo_3 = theta3_deg + 90.0         # knee: bar linkage offset

        return servo_1, servo_2, servo_3

    @staticmethod
    def _ik_to_servo_rf(theta1_deg, theta2_deg, theta3_deg):
        """Convert raw IK degrees to servo degrees for the RIGHT-FRONT leg.

        RF IK returns θ2 ≈ −218.7° at standing (wraps past −180°).
        Normalization brings it to ~141.3° so the formula maps into
        the EEPROM range [2048, 3754].

        Args:
            theta1_deg: IK hip angle in degrees
            theta2_deg: IK shoulder angle in degrees
            theta3_deg: IK knee angle in degrees

        Returns:
            tuple of (servo1_deg, servo2_deg, servo3_deg)
        """
        # Normalize θ2 into (−180, +180] range.
        if theta2_deg < -180.0:
            theta2_deg += 360.0

        servo_1 = theta1_deg                # hip: direct
        servo_2 = -theta2_deg + 270         # shoulder
        servo_3 = theta3_deg - 90.0         # knee: bar linkage offset

        return servo_1, servo_2, servo_3

    @staticmethod
    def _ik_to_servo_rb(theta1_deg, theta2_deg, theta3_deg):
        """Convert raw IK degrees to servo degrees for the RIGHT-BEHIND leg.

        RB IK θ2 sign depends on foot placement relative to the hip:
          x_center < −L/2 → θ2 positive (no wrap)
          x_center > −L/2 → θ2 wraps past −180° (needs normalization)

        Args:
            theta1_deg: IK hip angle in degrees
            theta2_deg: IK shoulder angle in degrees
            theta3_deg: IK knee angle in degrees

        Returns:
            tuple of (servo1_deg, servo2_deg, servo3_deg)
        """
        # Normalize θ2 into (−180, +180] range.
        if theta2_deg < -180.0:
            theta2_deg += 360.0

        servo_1 = theta1_deg                # hip: direct
        servo_2 = -theta2_deg + 270         # shoulder
        servo_3 = theta3_deg - 90.0         # knee: bar linkage offset

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
        """Publish to real servos via dual /servo_commands_a and _b topics.

        Converts IK angles (degrees) to servo angles using the
        empirically verified mapping, then publishes as two
        Float64MultiArray messages — one per driver.

        Driver A (port A): LF + RB servos (6 values)
        Driver B (port B): LB + RF servos (6 values)

        Args:
            theta: 4×3 numpy array of IK joint angles in degrees.
                   Rows: [LF, LB, RF, RB]
                   Cols: [θ1, θ2, θ3]
        """
        # Left legs (per-leg mapping — LF and LB have different joint2 directions)
        lf1, lf2, lf3 = self._ik_to_servo_lf(
            theta[0, 0], theta[0, 1], theta[0, 2])
        lb1, lb2, lb3 = self._ik_to_servo_lb(
            theta[1, 0], theta[1, 1], theta[1, 2])

        # Right legs
        rf1, rf2, rf3 = self._ik_to_servo_rf(
            theta[2, 0], theta[2, 1], theta[2, 2])
        rb1, rb2, rb3 = self._ik_to_servo_rb(
            theta[3, 0], theta[3, 1], theta[3, 2])

        # Driver A: LF + RB (servo IDs 7,8,9 + 10,11,12)
        msg_a = Float64MultiArray()
        msg_a.data = [float(lf1), float(lf2), float(lf3),
                      float(rb1), float(rb2), float(rb3)]
        self.pub_servo_commands_a.publish(msg_a)

        # Driver B: LB + RF (servo IDs 4,5,6 + 1,2,3)
        msg_b = Float64MultiArray()
        msg_b.data = [float(lb1), float(lb2), float(lb3),
                      float(rf1), float(rf2), float(rf3)]
        self.pub_servo_commands_b.publish(msg_b)

    def publish_real_12(self, servo_angles_12):
        """Publish a flat 12-element servo-degree list to both drivers.

        This is used by gaitGenerator's _tick_home_real() which builds
        a full 12-element array [LF1..3, LB1..3, RF1..3, RB1..3].

        Splits into:
          Driver A: indices [0:3] (LF) + [9:12] (RB)
          Driver B: indices [3:6] (LB) + [6:9]  (RF)
        """
        msg_a = Float64MultiArray()
        msg_a.data = [float(v) for v in servo_angles_12[0:3]] + [float(v) for v in servo_angles_12[9:12]]

        msg_b = Float64MultiArray()
        msg_b.data = [float(v) for v in servo_angles_12[3:6]] + [float(v) for v in servo_angles_12[6:9]]

        self.pub_servo_commands_a.publish(msg_a)
        self.pub_servo_commands_b.publish(msg_b)

    def disable_feedback(self):
        """Disable feedback on both drivers to free serial bus for commands."""
        from std_msgs.msg import Bool
        pub_a = self.node.create_publisher(Bool, '/feedback_enable_a', 10)
        pub_b = self.node.create_publisher(Bool, '/feedback_enable_b', 10)
        pub_a.publish(Bool(data=False))
        pub_b.publish(Bool(data=False))
        self.get_logger.info('Feedback disabled on both drivers')

    def publish_message(self, theta):
        """Route to sim or real based on the use_real_hardware parameter."""
        if self.use_real:
            self.publish_real(theta)
        else:
            self.publish_simulation(theta)