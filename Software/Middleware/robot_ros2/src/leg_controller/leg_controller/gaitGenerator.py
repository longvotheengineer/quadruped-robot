import math
import time
import numpy as np
from dataclasses import dataclass
from leg_controller.kinematics import Kinematics
from leg_controller.serialPublish import SerialPublish
from leg_controller.quinticPlanning import quintic_planning
from std_msgs.msg import Float64MultiArray, Float64, Bool, String, Int32
from geometry_msgs.msg import Vector3

# Home correction application diagnostic signals — /diag/home/apply/
# Pipeline: received correction → deadzone → scale → per-leg offset → LPF
_HOME_APPLY_SIGNALS = [
    's01_roll_in',     # received roll correction from balance PID
    's02_pitch_in',    # received pitch correction from balance PID
    's03_dz_flag',     # deadzone active (1.0 = inside, 0.0 = outside)
    's04_roll_scl',    # after gain scaling
    's05_pitch_scl',   # after gain scaling
    's06_off_lf',      # smoothed offset left-front
    's07_off_lb',      # smoothed offset left-behind
    's08_off_rf',      # smoothed offset right-front
    's09_off_rb',      # smoothed offset right-behind
]


@dataclass(frozen=True)
class Waypoint:
    zero:   int
    stance: int
    swing:  int
    rest:   int


@dataclass(frozen=True)
class RobotLength:
    L:  float
    W:  float
    l1: float
    l2: float
    l3: float


def deadzone_linear(value, deadzone):
    if abs(value) <= deadzone:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - deadzone)


# ── Configuration Classes ─────────────────────────────────────────────


class ImuHomeConfig:
    # Timing
    DELAY_START = 3.0

    # Joint mapping
    THETA3_IDX = {
        'left-front':   2,
        'left-behind':  5,
        'right-front':  8,
        'right-behind': 11,
    }

    # Gain & limits
    GAIN_HOME = 7.0
    DEADZONE = 0.02
    SAT = 0.5

    # Filtering
    LPF_HOME = 0.95


class StanceConfig:
    """Stance detection thresholds for applying gait correction."""
    STANCE_Z = -170.0
    STANCE_Z_THRESH = 5.0


class GaitConfig:
    LEG_NAMES = [
        'left-front',
        'left-behind',
        'right-front',
        'right-behind',
    ]

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

    PARAMS_PHASESHIFT_SWAY = {
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


# ── State Classes ─────────────────────────────────────────────────────


class HomeCorrectionState:
    def __init__(self):
        # Input (from balance_controller)
        self.roll = 0.0
        self.pitch = 0.0

        # Timing
        self.t_start = None

        # Output (per-leg smoothed offsets)
        self.offsets_lpf = {
            'left-front': 0.0,
            'left-behind': 0.0,
            'right-front': 0.0,
            'right-behind': 0.0,
        }


class GaitCorrectionState:
    """Received gait PID correction from balance_controller."""
    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0


# ── Main Gait Class ──────────────────────────────────────────────────


class Gait:
    def __init__(self, node, gait_msg):
        self._node = node
        self._logger = node.get_logger()
        self._gait_msg = gait_msg

        robot_length = RobotLength(L=250, W=193, l1=45, l2=107, l3=116)
        self._kinematics = Kinematics(self._node, robot_length)
        self.serial_publish = SerialPublish(self._node)
        self._waypoint = Waypoint(3000, 200, 70, 600)

        # Trajectory state
        self._angle_data = None
        self._foot_data = None
        self._step_current = 0
        self._step_final = 0

        # IMU state
        self._cycle_len = self._waypoint.stance + self._waypoint.swing
        self._home_corr = HomeCorrectionState()
        self._gait_corr = GaitCorrectionState()

        # ROS 2 subscriptions
        node.create_subscription(
            Vector3, '/posture/home_correction', self._home_corr_callback, 10)
        node.create_subscription(
            Vector3, '/posture/gait_correction', self._gait_corr_callback, 10)

        # Diagnostic publishers (home offsets only — gait diag moved to
        # balance controller)
        self._diag_home_pubs = {}
        for sig in _HOME_APPLY_SIGNALS:
            self._diag_home_pubs[sig] = node.create_publisher(
                Float64, f'/diag/home/apply/{sig}', 10)

        # Balance controller gate & mode
        self._pub_balance_enable = node.create_publisher(
            Bool, '/balance/enable', 10)
        self._pub_balance_mode = node.create_publisher(
            String, '/balance/mode', 10)
        self._pub_gait_frame = node.create_publisher(
            Int32, '/balance/gait_frame', 10)
        self._balance_enabled = False

    # ── Callbacks ─────────────────────────────────────────────────

    def _home_corr_callback(self, msg: Vector3):
        """Receive filtered PID correction from balance_controller."""
        self._home_corr.roll = msg.x
        self._home_corr.pitch = msg.y

    def _gait_corr_callback(self, msg: Vector3):
        """Receive gait PID correction from balance_controller."""
        self._gait_corr.roll = msg.x
        self._gait_corr.pitch = msg.y

    # ── Tick dispatch ─────────────────────────────────────────────

    def control_init(self):
        """Initialize gait data and reset all IMU controller state."""
        self._angle_data = self._gait_change()
        self._step_current = 0
        self._step_final = 0

        # Reset gait correction state
        self._gait_corr = GaitCorrectionState()

    def control_tick(self):
        if self._angle_data is None:
            return False
        if self._gait_msg.cmd == "ZERO":
            return self._tick_home()
        else:
            return self._tick_gait()

    # ── Tick functions ────────────────────────────────────────────

    def _tick_home(self):
        """Advance homing by one frame."""
        total_frames = self._angle_data.shape[0]
        if self._step_current < total_frames:
            targets = self._angle_data[self._step_current].tolist()
            torques = self.serial_publish.controller_sim\
                .compute_torques(targets)
            msg = Float64MultiArray()
            msg.data = torques
            self.serial_publish.pub_sim_gazebo.publish(msg)
            self._step_current += 1
            return True
        else:
            if not self._balance_enabled:
                self._pub_balance_enable.publish(Bool(data=True))
                self._pub_balance_mode.publish(String(data='HOME'))
                self._balance_enabled = True
                self._logger.info(
                    'Homing complete — enabling balance PID (HOME mode)')
            joint_names = self.serial_publish.controller_sim.joint_names
            targets = [self._homing_targets[name]
                       for name in joint_names]
            self._apply_home_correction(targets)
            torques = self.serial_publish.controller_sim.compute_torques(
                targets)
            msg = Float64MultiArray()
            msg.data = torques
            self.serial_publish.pub_sim_gazebo.publish(msg)
            return False

    def _tick_gait(self):
        """Advance gait by one frame with IMU balance compensation."""
        angle_data = self._angle_data
        frame = self._step_current

        # Publish frame index so balance controller can track baseline
        self._pub_gait_frame.publish(Int32(data=frame))

        # Apply stance correction using received gait PID output
        pos = self._apply_stance_correction(frame, angle_data)

        if (self._gait_msg.step > 0
                and self._step_final >= self._gait_msg.step):
            self.serial_publish.publish_message(pos)
            return False

        self.serial_publish.publish_message(pos)

        self._step_current += 1
        if self._step_current >= angle_data[0].shape[0]:
            self._step_current = 0
            self._step_final += 1

        return True

    # ── Home-IMU helpers ──────────────────────────────────────────

    def _apply_home_correction(self, targets):
        """Apply theta3 posture correction to joint targets (in-place)."""
        now = time.time()
        h = self._home_corr
        if h.t_start is None:
            h.t_start = now
        if (now - h.t_start) < ImuHomeConfig.DELAY_START:
            return

        roll_avg_dz  = deadzone_linear(h.roll,  ImuHomeConfig.DEADZONE)
        pitch_avg_dz = deadzone_linear(h.pitch, ImuHomeConfig.DEADZONE)
        in_deadzone = (roll_avg_dz == 0.0 and pitch_avg_dz == 0.0)
        roll_scaled  = roll_avg_dz  * ImuHomeConfig.GAIN_HOME
        pitch_scaled = pitch_avg_dz * ImuHomeConfig.GAIN_HOME
        sat = ImuHomeConfig.SAT

        offsets = {
            'left-front':   max(-sat, min(sat, -(roll_scaled - pitch_scaled))),
            'left-behind':  max(-sat, min(sat, -(roll_scaled + pitch_scaled))),
            'right-front':  max(-sat, min(sat, -(roll_scaled + pitch_scaled))),
            'right-behind': max(-sat, min(sat, -(roll_scaled - pitch_scaled))),
        }

        diag_values = [
            h.roll, h.pitch,
            1.0 if in_deadzone else 0.0,
            roll_scaled, pitch_scaled,
            offsets['left-front'], offsets['left-behind'],
            offsets['right-front'], offsets['right-behind'],
        ]
        msg = Float64()
        for sig, val in zip(_HOME_APPLY_SIGNALS, diag_values):
            msg.data = val
            self._diag_home_pubs[sig].publish(msg)

        if in_deadzone:
            for leg in h.offsets_lpf:
                h.offsets_lpf[leg] *= ImuHomeConfig.LPF_HOME
            return
        alpha = ImuHomeConfig.LPF_HOME
        for leg in offsets:
            h.offsets_lpf[leg] = (alpha * h.offsets_lpf[leg]
                                  + (1 - alpha) * offsets[leg])
        for leg, offset in h.offsets_lpf.items():
            targets[ImuHomeConfig.THETA3_IDX[leg]] += offset

    # ── Gait correction helpers ───────────────────────────────────

    def _apply_stance_correction(self, frame, angle_data):
        """Apply phase-aware stance-only foot correction."""
        if self._foot_data is not None:
            pos = np.zeros((4, 3))
            for i, leg in enumerate(GaitConfig.LEG_NAMES):
                pos_foot_raw = self._foot_data[i][frame].copy()
                foot_z = pos_foot_raw[2]
                is_stance = (abs(foot_z - StanceConfig.STANCE_Z)
                             < StanceConfig.STANCE_Z_THRESH)

                if is_stance:
                    pos_foot_adj = self._apply_rotation_matrix(
                        pos_foot_raw,
                        self._gait_corr.roll,
                        self._gait_corr.pitch)
                    try:
                        pos[i] = self._kinematics.inverse(
                            *pos_foot_adj, leg)
                    except (ValueError, ZeroDivisionError):
                        pos[i] = angle_data[i][frame]
                else:
                    pos[i] = angle_data[i][frame]
        else:
            pos = np.array([angle_data[i][frame] for i in range(4)])
        return pos

    def _apply_rotation_matrix(self, pos_foot_raw, roll_angle, pitch_angle):
        """Apply body-tilt compensation R_y(pitch) x R_x(roll)."""
        cr, sr = math.cos(roll_angle),  math.sin(roll_angle)
        cp, sp = math.cos(pitch_angle), math.sin(pitch_angle)
        R = np.array([[cp,  sr*sp, cr*sp],
                      [0,   cr,    -sr],
                      [-sp, sr*cp, cr*cp]])
        pos_corrected    = R @ pos_foot_raw
        pos_corrected[0] = pos_foot_raw[0]
        pos_corrected[1] = pos_foot_raw[1]
        return pos_corrected

    # ── Init pose ─────────────────────────────────────────────────

    def init_pose(self):
        """Hold at init pose.

        Sim mode:  PD control to INIT_POSE targets (radians).
        Real mode: Hold servos at calibration zero (servo 0 degrees).
        """
        if self.serial_publish.use_real:
            # Real hardware: send servo zeros directly (no IK conversion).
            # Calibration zero = the physical rest position you set
            # during the calibration procedure.
            msg = Float64MultiArray()
            msg.data = [0.0] * 12  # 12 servo angles, all at 0 degrees
            self.serial_publish.pub_servo_commands.publish(msg)
        else:
            # Simulation: PD control to Gazebo INIT_POSE
            joint_names = self.serial_publish.controller_sim.joint_names
            targets = [GaitConfig.INIT_POSE[name] for name in joint_names]
            torques = self.serial_publish.controller_sim.compute_torques(targets)
            msg = Float64MultiArray()
            msg.data = torques
            self.serial_publish.pub_sim_gazebo.publish(msg)

    # ── Trajectory generation ─────────────────────────────────────

    def _trajectory_moving(self, x_center, y_val, reverse=False):
        """Build D-shape foot path in Cartesian space (x, y, z)."""
        stride_length = 45
        x_forward = x_center + stride_length / 2
        x_backward = x_center - stride_length / 2
        z_stance = -170
        z_swing = -130
        lift_height = z_swing - z_stance

        if reverse:
            pos_A = [x_forward, y_val, z_stance]
            pos_D = [x_backward, y_val, z_stance]
        else:
            pos_A = [x_backward, y_val, z_stance]
            pos_D = [x_forward, y_val, z_stance]

        if GaitConfig.CONTROL_VELOCITY:
            return quintic_planning(
                pos_A, pos_D,
                T_swing=self._waypoint.swing,
                T_stance=self._waypoint.stance,
                lift_height=lift_height)
        else:
            swing = np.zeros((self._waypoint.swing, 3))
            swing[:, 0] = np.linspace(
                pos_A[0], pos_D[0], self._waypoint.swing)
            swing[:, 1] = y_val
            swing[:, 2] = (z_stance + lift_height
                           * np.sin(np.linspace(
                               0, np.pi, self._waypoint.swing)))

            stance = np.zeros((self._waypoint.stance, 3))
            stance[:, 0] = np.linspace(
                pos_D[0], pos_A[0], self._waypoint.stance)
            stance[:, 1] = y_val
            stance[:, 2] = z_stance

            return np.vstack([swing, stance])

    def _trajectory_resting(self, x_center, y_val):
        """Resting trajectory: feet planted, body oscillates."""
        z_low = -170
        z_high = -130

        waypoint = np.zeros((self._waypoint.rest, 3))
        waypoint[:, 0] = x_center
        waypoint[:, 1] = y_val
        waypoint[:, 2] = (z_low + (z_high - z_low)
                          * (0.5 - 0.5 * np.cos(np.linspace(
                              0, 2 * np.pi, self._waypoint.rest))))
        return waypoint

    # ── Gait generation ───────────────────────────────────────────

    def _generate_home(self):
        """Precalculate homing trajectory."""
        th1, th2, th3 = self._kinematics.inverse(
            125, 135, -170, "left-front")

        if th2 > 180:
            th2 -= 360

        homing_targets = {
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

        positions = self.serial_publish.controller_sim.actual_positions
        if not positions:
            return None

        joint_names = self.serial_publish.controller_sim.joint_names

        theta_i = []
        for step in range(self._waypoint.zero):
            alpha = step / self._waypoint.zero
            frame = [positions[n] + alpha * (homing_targets[n]
                     - positions[n]) for n in joint_names]
            theta_i.append(frame)

        self._homing_targets = homing_targets
        return np.array(theta_i)

    def _generate_gait(self, leg_type):
        """Generate one leg's full gait cycle."""
        match self._gait_msg.cmd:
            case "TROT_FORWARD":
                params = GaitConfig.PARAMS_GAIT_FORWARD.get(leg_type)
                phase = GaitConfig.PARAMS_PHASESHIFT_TROT
            case "TROT_BACKWARD":
                params = GaitConfig.PARAMS_GAIT_BACKWARD.get(leg_type)
                phase = GaitConfig.PARAMS_PHASESHIFT_TROT
            case "WALK_FORWARD":
                params = GaitConfig.PARAMS_GAIT_FORWARD.get(leg_type)
                phase = GaitConfig.PARAMS_PHASESHIFT_WALK
            case "WALK_BACKWARD":
                params = GaitConfig.PARAMS_GAIT_BACKWARD.get(leg_type)
                phase = GaitConfig.PARAMS_PHASESHIFT_WALK
            case "TURN_RIGHT":
                params = GaitConfig.PARAMS_GAIT_TURN_RIGHT.get(leg_type)
                phase = GaitConfig.PARAMS_PHASESHIFT_TROT
            case "TURN_LEFT":
                params = GaitConfig.PARAMS_GAIT_TURN_LEFT.get(leg_type)
                phase = GaitConfig.PARAMS_PHASESHIFT_TROT
            case "BODY_PUSHUP":
                params = GaitConfig.PARAMS_GAIT_FORWARD.get(leg_type)
                phase = GaitConfig.PARAMS_PHASESHIFT_PUSHUP
            case "BODY_SWAY":
                params = GaitConfig.PARAMS_GAIT_FORWARD.get(leg_type)
                phase = GaitConfig.PARAMS_PHASESHIFT_SWAY
            case "BODY_CIRCLE":
                params = GaitConfig.PARAMS_GAIT_FORWARD.get(leg_type)
                phase = GaitConfig.PARAMS_PHASESHIFT_CIRCLE
            case _:
                return None, None

        if not params:
            return None, None

        if self._gait_msg.cmd in (
                "BODY_PUSHUP", "BODY_SWAY", "BODY_CIRCLE"):
            waypoint = self._trajectory_resting(
                params["x_center"], params["y_val"])
        else:
            reverse = params.get("reverse", False)
            waypoint = self._trajectory_moving(
                params["x_center"], params["y_val"], reverse)

        theta_i = np.zeros_like(waypoint)
        for i in range(waypoint.shape[0]):
            theta_i[i] = self._kinematics.inverse(
                *waypoint[i], leg_type)

        shift = round(waypoint.shape[0] * phase.get(leg_type, 0))
        return (np.roll(theta_i, shift, axis=0),
                np.roll(waypoint, shift, axis=0))

    # ── Command dispatch ──────────────────────────────────────────

    def _gait_change(self):
        match self._gait_msg.cmd:
            case "ZERO":
                self._foot_data = None
                # Tell balance controller to use HOME mode
                self._pub_balance_mode.publish(String(data='HOME'))
                return self._generate_home()
            case _:
                theta_i = np.empty(4, dtype=object)
                foot_i = np.empty(4, dtype=object)
                for idx, leg in enumerate(GaitConfig.LEG_NAMES):
                    angles, feet = self._generate_gait(leg)
                    theta_i[idx] = angles
                    foot_i[idx] = feet
                self._foot_data = foot_i
                # Tell balance controller to use GAIT mode
                self._pub_balance_mode.publish(String(data='GAIT'))
                return theta_i