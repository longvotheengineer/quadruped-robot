import math
import numpy as np
from dataclasses import dataclass
from leg_controller.kinematics import Kinematics
from leg_controller.serialPublish import SerialPublish
from leg_controller.quinticPlanning import quintic_planning
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Vector3

@dataclass(frozen=True)
class Waypoint:
    zero   : int
    stance : int
    swing  : int

@dataclass(frozen=True)
class RobotLength:
    L : float
    W : float
    l1: float   
    l2: float
    l3: float

class Gait:
    def __init__(self, node, gait_msg):
        self.node = node
        self.get_logger = node.get_logger()

        robot_length = RobotLength(L = 250, W = 193, l1 = 45, l2 = 107, l3 = 116)
        self.kinematics = Kinematics(self.node, robot_length)
        self.serial_publish = SerialPublish(self.node)
        self.gait_msg = gait_msg

        self.waypoint = Waypoint(3000, 300, 30)

        # Trajectory state
        self.gait_angle_data = None
        self.gait_foot_data  = None
        self.current_frame = 0
        self.complete_step = 0

        # Posture correction from balance_controller
        self.corr_roll  = 0.0
        self.corr_pitch = 0.0
        self.LEG_NAMES = ['left-front', 'left-behind', 'right-front', 'right-behind']
        node.create_subscription(
            Vector3, '/posture/correction', self._posture_callback, 10)

    # ── Leg configuration ────────────────────────────────────────

    INIT_POSE = {
        'joint_lf_1':  0.3,  'joint_lf_2':  3*np.pi/2,  'joint_lf_3':  0.5,
        'joint_lb_1':  0.3,  'joint_lb_2':  3*np.pi/2,  'joint_lb_3':  0.5,
        'joint_rf_1':  0.3,  'joint_rf_2': -3*np.pi/2,  'joint_rf_3': -0.5,
        'joint_rb_1':  0.3,  'joint_rb_2': -3*np.pi/2,  'joint_rb_3': -0.5,
    }

    PARAMS_GAIT_FORWARD = {
        "left-front":    {"x_center":  125, "y_val":  135, "reverse": False},
        "left-behind":   {"x_center": -125, "y_val":  135, "reverse": False},
        "right-front":   {"x_center":  125, "y_val": -135, "reverse": False},
        "right-behind":  {"x_center": -125, "y_val": -135, "reverse": False},
    }

    PARAMS_GAIT_BACKWARD = {
        "left-front":    {"x_center":  125, "y_val":  135, "reverse": True},
        "left-behind":   {"x_center": -125, "y_val":  135, "reverse": True},
        "right-front":   {"x_center":  125, "y_val": -135, "reverse": True},
        "right-behind":  {"x_center": -125, "y_val": -135, "reverse": True},
    }

    PARAMS_GAIT_TURN_RIGHT = {
        "left-front":    {"x_center":  125, "y_val":  135, "reverse": False},
        "left-behind":   {"x_center": -125, "y_val":  135, "reverse": False},
        "right-front":   {"x_center":  125, "y_val": -135, "reverse": True},
        "right-behind":  {"x_center": -125, "y_val": -135, "reverse": True},
    }

    PARAMS_GAIT_TURN_LEFT = {
        "left-front":    {"x_center":  125, "y_val":  135, "reverse": True},
        "left-behind":   {"x_center": -125, "y_val":  135, "reverse": True},
        "right-front":   {"x_center":  125, "y_val": -135, "reverse": False},
        "right-behind":  {"x_center": -125, "y_val": -135, "reverse": False},
    }

    PARAMS_PHASESHIFT_TROT = {
        "left-front":   0.00,
        "right-behind": 0.00,
        "left-behind":  0.50,
        "right-front":  0.50,
    }

    PARAMS_PHASESHIFT_WALK = {
        "left-front":   0.00,
        "right-behind": 0.25,
        "right-front":  0.50,
        "left-behind":  0.75,
    }

    PARAMS_PHASESHIFT_PUSHUP = {
        "left-front":   0.00,
        "left-behind":  0.00,
        "right-front":  0.00,
        "right-behind": 0.00,
    }

    PARAMS_PHASESHIFT_SWAY   = {
        "left-front":   0.00,
        "left-behind":  0.00,
        "right-front":  0.50,
        "right-behind": 0.50,
    }

    PARAMS_PHASESHIFT_CIRCLE = {
        "left-front":   0.00,
        "right-front":  0.25,
        "right-behind": 0.50,
        "left-behind":  0.75,
    }

    CONTROL_VELOCITY = True

    # ── Init pose (natural bent-leg position) ────────────────────

    def init_pose_tick(self):
        """Hold at INIT_POSE via PID."""
        joint_names = self.serial_publish.controller_sim.joint_names
        targets = [self.INIT_POSE[name] for name in joint_names]
        torques = self.serial_publish.controller_sim.compute_torques(targets)
        msg = Float64MultiArray()
        msg.data = torques
        self.serial_publish.pub_sim_gazebo.publish(msg)

    # ── Trajectory generation ────────────────────────────────────

    def trajectory_foot(self, x_center, y_val, reverse=False):
        """Build D-shape foot path in Cartesian space (x, y, z).
        If reverse=True, swap swing direction (for turning)"""
        stride_length = 45
        x_forward  = x_center + stride_length / 2
        x_backward = x_center - stride_length / 2
        z_stance = -170
        z_swing  = -130
        lift_height = z_swing - z_stance

        if reverse:
            pos_A = [x_forward,  y_val, z_stance]
            pos_D = [x_backward, y_val, z_stance]
        else:
            pos_A = [x_backward, y_val, z_stance]
            pos_D = [x_forward,  y_val, z_stance]

        if self.CONTROL_VELOCITY:
            return quintic_planning(pos_A, pos_D,
                                  T_swing     = self.waypoint.swing,
                                  T_stance    = self.waypoint.stance,
                                  lift_height = lift_height)
        else:
            # Swing: sine-wave lift from backward to forward
            swing = np.zeros((self.waypoint.swing, 3))
            swing[:, 0] = np.linspace(pos_A[0], pos_D[0], self.waypoint.swing)
            swing[:, 1] = y_val
            swing[:, 2] = z_stance + lift_height * np.sin(np.linspace(0, np.pi, self.waypoint.swing))

            # Stance: slide on ground from forward to backward
            stance = np.zeros((self.waypoint.stance, 3))
            stance[:, 0] = np.linspace(pos_D[0], pos_A[0], self.waypoint.stance)
            stance[:, 1] = y_val
            stance[:, 2] = z_stance

            return np.vstack([swing, stance])

    def trajectory_oscillation(self, x_center, y_val):
        """The robot doesn't move, feet stay planted, body oscillates"""
        z_low  = -170    # body down (legs bent)
        z_high = -130    # body up (legs extended)

        waypoint = np.zeros((self.waypoint.stance, 3))
        waypoint[:, 0] = x_center
        waypoint[:, 1] = y_val
        waypoint[:, 2] = z_low + (z_high - z_low) * (0.5 - 0.5 * np.cos(np.linspace(0, 2 * np.pi, self.waypoint.stance)))

        return waypoint

    def _posture_callback(self, msg: Vector3):
        """Receive correction angles from balance_controller."""
        self.corr_roll  = msg.x
        self.corr_pitch = msg.y

    def apply_rotation(self, foot_pos, corr_roll, corr_pitch):
        """Apply paper formula: p_t = R_inv · (p_c - p_0) + p_0.
        Simplifies to p_t = R_inv · p_c with p_0 at origin."""
        r = -corr_roll
        p = -corr_pitch
        cr, sr = math.cos(r), math.sin(r)
        cp, sp = math.cos(p), math.sin(p)
        R = np.array([[cp,    sr*sp,  cr*sp],
                      [0,     cr,    -sr   ],
                      [-sp,   sr*cp,  cr*cp]])
        return R @ foot_pos

    def generate_gait(self, leg_type):
        """Generate one leg's full gait cycle: foot path → IK → phase shift.
        Returns (joint_angles, foot_positions) tuple."""
        # Select params and phase pattern based on command
        match self.gait_msg.cmd:
            case "TROT_FORWARD":
                params = self.PARAMS_GAIT_FORWARD.get(leg_type)
                phase  = self.PARAMS_PHASESHIFT_TROT
            case "TROT_BACKWARD":
                params = self.PARAMS_GAIT_BACKWARD.get(leg_type)
                phase  = self.PARAMS_PHASESHIFT_TROT
            case "WALK_FORWARD":
                params = self.PARAMS_GAIT_FORWARD.get(leg_type)
                phase  = self.PARAMS_PHASESHIFT_WALK
            case "WALK_BACKWARD":
                params = self.PARAMS_GAIT_BACKWARD.get(leg_type)
                phase  = self.PARAMS_PHASESHIFT_WALK
            case "TURN_RIGHT":
                params = self.PARAMS_GAIT_TURN_RIGHT.get(leg_type)
                phase  = self.PARAMS_PHASESHIFT_TROT
            case "TURN_LEFT":
                params = self.PARAMS_GAIT_TURN_LEFT.get(leg_type)
                phase  = self.PARAMS_PHASESHIFT_TROT
            case "BODY_PUSHUP":
                params = self.PARAMS_GAIT_FORWARD.get(leg_type)
                phase  = self.PARAMS_PHASESHIFT_PUSHUP
            case "BODY_SWAY":
                params = self.PARAMS_GAIT_FORWARD.get(leg_type)
                phase  = self.PARAMS_PHASESHIFT_SWAY
            case "BODY_CIRCLE":
                params = self.PARAMS_GAIT_FORWARD.get(leg_type)
                phase  = self.PARAMS_PHASESHIFT_CIRCLE
            case _:
                return None, None

        if not params:
            return None, None

        # Build foot trajectory based on command
        if self.gait_msg.cmd in ("BODY_PUSHUP", "BODY_SWAY", "BODY_CIRCLE"):
            waypoint = self.trajectory_oscillation(params["x_center"], params["y_val"])
        else:
            reverse = params.get("reverse", False)
            waypoint = self.trajectory_foot(params["x_center"], params["y_val"], reverse)

        # Convert to joint angles via IK
        theta_i = np.zeros_like(waypoint)
        for i in range(waypoint.shape[0]):
            theta_i[i] = self.kinematics.inverse(*waypoint[i], leg_type)

        # Apply phase shift to BOTH angles and foot positions
        shift = round(waypoint.shape[0] * phase.get(leg_type, 0))
        return np.roll(theta_i, shift, axis=0), np.roll(waypoint, shift, axis=0)

    def generate_home(self):
        """Precalculate homing trajectory: all 4 legs move simultaneously."""
        th1, th2, th3 = self.kinematics.inverse(125, 135, -170, "left-front")

        if th2 > 180:
            th2 -= 360

        # Sign pattern: LF(+,+)  LB(-,+)  RF(-,-)  RB(-,-)
        homing_targets = {
            'joint_lf_1': 0.0,  'joint_lf_2':  np.radians(th2) + 2 * np.pi,  'joint_lf_3':  np.radians(th3) + np.pi,
            'joint_lb_1': 0.0,  'joint_lb_2':  np.radians(th2) + 2 * np.pi,  'joint_lb_3':  np.radians(th3) + np.pi,
            'joint_rf_1': 0.0,  'joint_rf_2': -np.radians(th2) - 2 * np.pi,  'joint_rf_3': -np.radians(th3) - np.pi,
            'joint_rb_1': 0.0,  'joint_rb_2': -np.radians(th2) - 2 * np.pi,  'joint_rb_3': -np.radians(th3) - np.pi,
        }

        # Read current encoder positions
        positions = self.serial_publish.controller_sim.actual_positions
        if not positions:
            return None

        joint_names = self.serial_publish.controller_sim.joint_names

        # Interpolate all joints simultaneously
        theta_i = []
        for step in range(self.waypoint.zero):
            alpha = step / self.waypoint.zero
            frame = [positions[n] + alpha * (homing_targets[n] - positions[n]) for n in joint_names]
            theta_i.append(frame)

        self.homing_targets = homing_targets
        return np.array(theta_i)

    def change(self):        
        match self.gait_msg.cmd:
            case "ZERO":
                self.gait_foot_data = None
                return self.generate_home()
            case _:
                theta_i = np.empty(4, dtype=object)
                foot_i  = np.empty(4, dtype=object)
                for idx, leg in enumerate(self.LEG_NAMES):
                    angles, feet = self.generate_gait(leg)
                    theta_i[idx] = angles
                    foot_i[idx]  = feet
                self.gait_foot_data = foot_i
                return theta_i   

    def control_init(self):
        self.gait_angle_data = self.change()
        self.current_frame = 0
        self.complete_step = 0

    def control_tick(self):
        if self.gait_angle_data is None:
            return False

        if self.gait_msg.cmd == "ZERO":
            return self._tick_homing()
        else:
            return self._tick_gait()

    def _tick_homing(self):
        """Advance homing by one frame: publish joint targets via PID."""
        total_frames = self.gait_angle_data.shape[0]
        if self.current_frame < total_frames:
            targets = self.gait_angle_data[self.current_frame].tolist()
            torques = self.serial_publish.controller_sim.compute_torques(targets)
            msg = Float64MultiArray()
            msg.data = torques
            self.serial_publish.pub_sim_gazebo.publish(msg)
            self.current_frame += 1
            return True
        else:
            # Hold at homing targets
            joint_names = self.serial_publish.controller_sim.joint_names
            targets = [self.homing_targets[name] for name in joint_names]
            torques = self.serial_publish.controller_sim.compute_torques(targets)
            msg = Float64MultiArray()
            msg.data = torques
            self.serial_publish.pub_sim_gazebo.publish(msg)
            return False

    def _tick_gait(self):
        """Advance gait by one frame. step=0 means run continuously."""
        theta_i = self.gait_angle_data

        # If finite steps completed, hold the last frame position
        if self.gait_msg.step > 0 and self.complete_step >= self.gait_msg.step:
            pos = self._get_corrected_frame(theta_i, self.current_frame)
            self.serial_publish.publish_message(pos)
            return False

        pos = self._get_corrected_frame(theta_i, self.current_frame)
        self.serial_publish.publish_message(pos)
        
        # Advance frame
        self.current_frame += 1
        if self.current_frame >= theta_i[0].shape[0]:
            self.current_frame = 0
            self.complete_step += 1
            
        return True

    def _get_corrected_frame(self, theta_i, frame):
        """Get joint angles for one frame, with posture correction applied."""
        # No correction or no foot data → use precalculated angles
        if self.gait_foot_data is None or True:  # TEMP: force no correction
            return np.array([theta_i[i][frame] for i in range(4)])

        # Apply rotation correction and re-run IK
        pos = np.zeros((4, 3))
        for i, leg in enumerate(self.LEG_NAMES):
            foot = self.gait_foot_data[i][frame].copy()
            adjusted = self.apply_rotation(foot, self.corr_roll, self.corr_pitch)
            pos[i] = self.kinematics.inverse(*adjusted, leg)
        return pos