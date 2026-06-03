"""Hardware abstraction: unified interface for simulation and real servo output.

The GaitController calls Actuator methods without knowing whether it's
driving Gazebo joints or physical servos. All sim/real branching for
publishing is encapsulated here.
"""

import math
import numpy as np
from std_msgs.msg import Float64MultiArray, Bool
from sensor_msgs.msg import JointState
from leg_controller.controllerSim import ControllerSim
from leg_controller.gaitConfig import JOINT_NAMES, Z_OFFSETS


class Actuator:
    """Unified hardware interface for simulation (Gazebo) and real servos."""

    def __init__(self, node):
        self._node = node
        self._logger = node.get_logger()
        self._useReal = node.get_parameter('use_real_hardware').value

        # Gazebo effort command publisher
        self._pubSim = node.create_publisher(
            Float64MultiArray, '/leg_controller/commands', 10)

        # Real servo command publishers (two drivers)
        self._pubServoA = node.create_publisher(
            Float64MultiArray, '/servo_commands_a', 10)
        self._pubServoB = node.create_publisher(
            Float64MultiArray, '/servo_commands_b', 10)

        # PD torque controller (Gazebo only)
        self._simController = ControllerSim(node)

        # Real servo feedback positions
        self._realPositions = {}
        if self._useReal:
            node.create_subscription(
                JointState, '/joint_states_real_a', self._realJointCb, 10)
            node.create_subscription(
                JointState, '/joint_states_real_b', self._realJointCb, 10)

        mode = 'REAL HARDWARE' if self._useReal else 'SIMULATION (Gazebo)'
        self._logger.info(f'Actuator: {mode} mode')

    @property
    def useReal(self):
        return self._useReal

    # ── 1. Public API ────────────────────────────────────────────────────

    def send(self, theta):
        """Publish 4×3 IK joint angles to sim or real hardware."""
        if self._useReal:
            self._sendReal(theta)
        else:
            self._sendSim(theta)

    def holdInitPose(self, initPose):
        """Hold at initial pose. Sim: PD control. Real: no-op (holds last)."""
        if self._useReal:
            return
        names = self._simController.joint_names
        targets = [initPose[n] for n in names]
        torques = self._simController.compute_torques(targets)
        msg = Float64MultiArray()
        msg.data = torques
        self._pubSim.publish(msg)

    def buildHomingTrajectory(self, kinematics, timing):
        """Build homing trajectory in native coordinate space.

        Sim:  interpolates from current Gazebo positions to IK targets (radians).
        Real: interpolates from current servo positions to IK targets (servo degrees).

        Returns:
            (trajectory_array, targets) or (None, None) if positions unavailable.
        """
        if self._useReal:
            return self._buildHomingReal(kinematics, timing)
        else:
            return self._buildHomingSim(kinematics, timing)

    def sendHomingFrame(self, frameData):
        """Publish one homing interpolation frame."""
        if self._useReal:
            self._sendRawServo(frameData)
        else:
            torques = self._simController.compute_torques(frameData)
            msg = Float64MultiArray()
            msg.data = torques
            self._pubSim.publish(msg)

    def holdHomingTarget(self, targets, sim_target_list=None):
        """Hold at homing position (called every tick after homing completes).

        Sim:  recompute PD torques toward target (needs joint name dict).
        Real: resend target servo degrees.
        """
        if self._useReal:
            self._sendRawServo(targets)
        else:
            if sim_target_list is not None:
                target_list = sim_target_list
            else:
                names = self._simController.joint_names
                target_list = [targets[n] for n in names]
            torques = self._simController.compute_torques(target_list)
            msg = Float64MultiArray()
            msg.data = torques
            self._pubSim.publish(msg)

    def sendShutdownFrame(self, frameData):
        """Publish one shutdown interpolation frame (real only, no-op for sim)."""
        if self._useReal:
            self._sendRawServo(frameData)

    def hasPositionData(self):
        """True if at least one set of joint positions has been received."""
        if self._useReal:
            return bool(self._realPositions)
        return bool(self._simController.actual_positions)

    def disableFeedback(self):
        """Disable feedback on both real drivers to free the serial bus."""
        pub_a = self._node.create_publisher(Bool, '/feedback_enable_a', 10)
        pub_b = self._node.create_publisher(Bool, '/feedback_enable_b', 10)
        pub_a.publish(Bool(data=False))
        pub_b.publish(Bool(data=False))
        self._logger.info('Feedback disabled on both drivers')

    # ── 2. Private Simulation Publishing ─────────────────────────────────

    def _sendSim(self, theta):
        """Publish to Gazebo via PD torque controller."""
        targets = []
        for leg in range(4):
            targets.extend([
                math.radians(theta[leg, 0]),
                math.radians(theta[leg, 1]),
                math.radians(theta[leg, 2]) + math.radians(180),
            ])

        # Wrap targets to nearest ±π of actual position (prevents 2π jumps)
        if self._simController.actual_positions:
            for i, name in enumerate(self._simController.joint_names):
                actual = self._simController.actual_positions.get(name, 0.0)
                diff = targets[i] - actual
                diff = (diff + math.pi) % (2 * math.pi) - math.pi
                targets[i] = actual + diff

        torques = self._simController.compute_torques(targets)
        msg = Float64MultiArray()
        msg.data = torques
        self._pubSim.publish(msg)

    # ── 3. Private Real Hardware Publishing ──────────────────────────────

    def _sendReal(self, theta):
        """Publish to real servos via dual drivers."""
        lf = self._ikToServoLF(theta[0, 0], theta[0, 1], theta[0, 2])
        lb = self._ikToServoLB(theta[1, 0], theta[1, 1], theta[1, 2])
        rf = self._ikToServoRF(theta[2, 0], theta[2, 1], theta[2, 2])
        rb = self._ikToServoRB(theta[3, 0], theta[3, 1], theta[3, 2])

        msg_a = Float64MultiArray()
        msg_a.data = [float(v) for v in lf + lb]
        self._pubServoA.publish(msg_a)

        msg_b = Float64MultiArray()
        msg_b.data = [float(v) for v in rf + rb]
        self._pubServoB.publish(msg_b)

    def _sendRawServo(self, servoAngles12):
        """Publish flat 12-element servo degree list to both drivers.

        Splits into Driver A [0:6] (LF+LB) and Driver B [6:12] (RF+RB).
        """
        msg_a = Float64MultiArray()
        msg_a.data = [float(v) for v in servoAngles12[0:6]]
        msg_b = Float64MultiArray()
        msg_b.data = [float(v) for v in servoAngles12[6:12]]
        self._pubServoA.publish(msg_a)
        self._pubServoB.publish(msg_b)

    @staticmethod
    def _ikToServoLF(t1, t2, t3):
        return (t1 - 5,
                -(t2 - 90) - 4,
                t3 + 90.0 - 4)

    @staticmethod
    def _ikToServoLB(t1, t2, t3):
        return (t1,
                t2 + 360 - 4,
                t3 + 90.0)

    @staticmethod
    def _ikToServoRF(t1, t2, t3):
        if t2 < -180.0:
            t2 += 360.0
        return (t1 + 7,
                -t2 + 270,
                t3 - 90.0 - 5)

    @staticmethod
    def _ikToServoRB(t1, t2, t3):
        if t2 < -180.0:
            t2 += 360.0
        return (t1 + 8,
                -t2 + 270 - 13 + 4 - 2,
                t3 - 90.0 - 3)

    # ── 4. Internal Homing Implementations ───────────────────────────────

    def _buildHomingSim(self, kinematics, timing):
        """Build sim homing trajectory (radian space)."""
        th1, th2, th3 = kinematics.inverse(125, 135, -170, "left-front")
        if th2 > 180:
            th2 -= 360

        targets = {
            'joint_lf_1': 0.0,
            'joint_lf_2':  np.radians(th2) + 2 * np.pi,
            'joint_lf_3':  np.radians(th3) + np.pi,
            'joint_lb_1': 0.0,
            'joint_lb_2':  np.radians(th2) + 2 * np.pi,
            'joint_lb_3':  np.radians(th3) + np.pi,
            'joint_rf_1': 0.0,
            'joint_rf_2': -np.radians(th2) - 2 * np.pi,
            'joint_rf_3': -np.radians(th3) - np.pi,
            'joint_rb_1': 0.0,
            'joint_rb_2': -np.radians(th2) - 2 * np.pi,
            'joint_rb_3': -np.radians(th3) - np.pi,
        }

        positions = self._simController.actual_positions
        if not positions:
            return None, None

        names = self._simController.joint_names
        frames = []
        for step in range(timing.homing):
            alpha = step / timing.homing
            frame = [positions[n] + alpha * (targets[n] - positions[n])
                     for n in names]
            frames.append(frame)

        return np.array(frames), targets

    def _buildHomingReal(self, kinematics, timing):
        """Build real homing trajectory (servo degree space)."""
        lf = kinematics.inverse(125, 135, -170 + Z_OFFSETS['left-front'], "left-front")
        lb = kinematics.inverse(-125, 135, -170 + Z_OFFSETS['left-behind'], "left-behind")
        rf = kinematics.inverse(125, -135, -170 + Z_OFFSETS['right-front'], "right-front")
        rb = kinematics.inverse(-125, -135, -170 + Z_OFFSETS['right-behind'], "right-behind")

        target = (list(self._ikToServoLF(*lf)) + list(self._ikToServoLB(*lb))
                  + list(self._ikToServoRF(*rf)) + list(self._ikToServoRB(*rb)))

        if not self._realPositions:
            self._logger.warn('No servo positions yet — waiting for feedback...')
            return None, None

        current = [self._realPositions.get(n, 0.0) for n in JOINT_NAMES]
        self.disableFeedback()

        trajectory = np.zeros((timing.homing, 12))
        for step in range(timing.homing):
            alpha = step / timing.homing
            for j in range(12):
                trajectory[step, j] = current[j] + alpha * (target[j] - current[j])

        return trajectory, target

    # ── 5. Feedback Callbacks ────────────────────────────────────────────

    def _realJointCb(self, msg):
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self._realPositions[name] = msg.position[i]
