import math
import time
import numpy as np
from dataclasses import dataclass
from leg_controller.kinematics import Kinematics
from leg_controller.serialPublish import SerialPublish
from leg_controller.quinticPlanning import quintic_planning
from std_msgs.msg import Float64MultiArray, Float64, Bool, String, Int32
from geometry_msgs.msg import Vector3

# Leg short names for PlotJuggler topics
_LEG_SHORT = {
    'left-front':   'lf',
    'left-behind':  'lb',
    'right-front':  'rf',
    'right-behind': 'rb',
}

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

    # ── ROBOTOFF safe-off angles (servo degrees, NO IK) ────────────
    # Three-phase shutdown:
    #   Phase 1 = bend knee   → body drops to ground
    #   Phase 2 = fold shoulder → leg tucks against body
    #   Phase 3 = splay hip   → legs collapse outward
    #
    # Sign conventions differ per leg (mirrored mounting):
    #   Joint3 collapse (knee fold):
    #     Left  legs: NEGATIVE = more knee bend
    #     Right legs: POSITIVE = more knee bend
    #   Joint2 fold (shoulder tuck):
    #     LF: toward EEPROM LOW  end (more negative)
    #     LB: toward EEPROM LOW  end (matching LF after direction fix)
    #     RF: toward EEPROM HIGH end (more positive)
    #     RB: toward EEPROM HIGH end (more positive)
    #   Joint1 splay (hip outward):
    #     Left  legs: POSITIVE = splay outward
    #     Right legs: NEGATIVE = splay outward
    #
    # Values have ~3° margin from EEPROM hard limits.
    SAFE_OFF_ANGLES = {
        'joint3': {
            'left-front':   -47.0,   # EEPROM limit ≈ -50.4°
            'left-behind':  -47.0,   # EEPROM limit ≈ -49.7°
            'right-front':   47.0,   # EEPROM limit ≈ +50.2°
            'right-behind':  47.0,   # EEPROM limit ≈ +50.1°
        },
        'joint2': {
            'left-front':  -160.0,   # EEPROM limit ≈ -149.9° (low end)
            'left-behind':  250.0,   # EEPROM limit ≈ +246.1° (low end)
            'right-front':  160.0,   # EEPROM limit ≈ +150.0° (high end)
            'right-behind': 160.0,   # EEPROM limit ≈ +150.8° (high end)
        },
        'joint1': {
            'left-front':   -50.0,   # EEPROM limit ≈ +30.5°
            'left-behind':   50.0,   # EEPROM limit ≈ +30.0°
            'right-front':   50.0,   # EEPROM limit ≈ -30.0°
            'right-behind': -50.0,   # EEPROM limit ≈ -30.0°
        },
    }

    # Frames per phase (three phases total).  At 7 ms/tick → ~2.1 s each.
    ROBOTOFF_FRAMES_PER_PHASE = 300

    # Per-leg z_offset: compensate mechanical height differences (mm).
    # Negative = foot reaches lower (use when a leg is physically higher).
    LF_Z_OFFSET = -2
    LB_Z_OFFSET = -3
    RF_Z_OFFSET = -4
    RB_Z_OFFSET = -5

    PARAMS_GAIT_FORWARD = {
        "left-front":    {"x_center":  125, "y_val":  135, "reverse": False, "z_offset": LF_Z_OFFSET},
        "left-behind":   {"x_center": -125, "y_val":  135, "reverse": False, "z_offset": LB_Z_OFFSET},
        "right-front":   {"x_center":  125, "y_val": -135, "reverse": False, "z_offset": RF_Z_OFFSET},
        "right-behind":  {"x_center": -125, "y_val": -135, "reverse": False, "z_offset": RB_Z_OFFSET},
    }

    # Body motions (PUSHUP / SWAY / CIRCLE): back feet tucked closer
    # under the body so the rear height matches the front.
    PARAMS_GAIT_BODY = {
        "left-front":    {"x_center":  125, "y_val":  135, "z_offset": LF_Z_OFFSET},
        "left-behind":   {"x_center": -125, "y_val":  135, "z_offset": LB_Z_OFFSET},
        "right-front":   {"x_center":  125, "y_val": -135, "z_offset": RF_Z_OFFSET},
        "right-behind":  {"x_center": -125, "y_val": -135, "z_offset": RB_Z_OFFSET},
    }

    PARAMS_GAIT_BACKWARD = {
        "left-front":    {"x_center":  125, "y_val":  135, "reverse": True, "z_offset": LF_Z_OFFSET},
        "left-behind":   {"x_center": -125, "y_val":  135, "reverse": True, "z_offset": LB_Z_OFFSET},
        "right-front":   {"x_center":  125, "y_val": -135, "reverse": True, "z_offset": RF_Z_OFFSET},
        "right-behind":  {"x_center": -125, "y_val": -135, "reverse": True, "z_offset": RB_Z_OFFSET},
    }

    PARAMS_GAIT_TURN_RIGHT = {
        "left-front":    {"x_center":  125, "y_val":  135, "reverse": False, "z_offset": LF_Z_OFFSET},
        "left-behind":   {"x_center": -60, "y_val":  135, "reverse": False, "z_offset": LB_Z_OFFSET},
        "right-front":   {"x_center":  125, "y_val": -135, "reverse": True, "z_offset": RF_Z_OFFSET},
        "right-behind":  {"x_center": -60, "y_val": -135, "reverse": True, "z_offset": RB_Z_OFFSET},
    }

    PARAMS_GAIT_TURN_LEFT = {
        "left-front":    {"x_center":  125, "y_val":  135, "reverse": True, "z_offset": LF_Z_OFFSET},
        "left-behind":   {"x_center": -110, "y_val":  135, "reverse": True, "z_offset": LB_Z_OFFSET},
        "right-front":   {"x_center":  125, "y_val": -135, "reverse": False, "z_offset": RF_Z_OFFSET},
        "right-behind":  {"x_center": -110, "y_val": -135, "reverse": False, "z_offset": RB_Z_OFFSET},
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

        robot_length = RobotLength(L=209, W=191, l1=26, l2=106, l3=125)
        self._kinematics = Kinematics(self._node, robot_length, use_real=node.use_real)
        self.serial_publish = SerialPublish(self._node)
        self._waypoint = Waypoint(200, 200, 35, 1000)

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
        self._robotoff_complete = False  # True after ROBOTOFF transition done

        # ── PlotJuggler: foot P-V-A publishers (12 topics) ────────
        self._pub_foot = {}
        for leg_long, leg_short in _LEG_SHORT.items():
            self._pub_foot[leg_long] = {
                'pos': node.create_publisher(
                    Float64, f'/diag/foot/{leg_short}/pos_z', 10),
                'vel': node.create_publisher(
                    Float64, f'/diag/foot/{leg_short}/vel_z', 10),
                'acc': node.create_publisher(
                    Float64, f'/diag/foot/{leg_short}/acc_z', 10),
            }
        self._foot_z_prev = {leg: 0.0 for leg in GaitConfig.LEG_NAMES}
        self._foot_vz_prev = {leg: 0.0 for leg in GaitConfig.LEG_NAMES}

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
        elif self._gait_msg.cmd == "ROBOTOFF":
            return self._tick_robotoff()
        else:
            return self._tick_gait()

    # ── Tick functions ────────────────────────────────────────────

    def _tick_home(self):
        """Advance homing by one frame."""
        if self.serial_publish.use_real:
            return self._tick_home_real()

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

    def _tick_home_real(self):
        """Advance real-hardware homing by one frame."""
        total_frames = self._angle_data.shape[0]

        if self._step_current < total_frames:
            # Publish one interpolated frame (servo degrees) to both drivers
            frame = self._angle_data[self._step_current].tolist()
            self.serial_publish.publish_real_12(frame)
            self._step_current += 1
            return True
        else:
            # Homing complete — hold at target
            self.serial_publish.publish_real_12(self._homing_targets_real)
            return False

    def _tick_gait(self):
        """Advance gait by one frame with IMU balance compensation."""
        angle_data = self._angle_data
        frame = self._step_current

        if self.serial_publish.use_real:
            # Real hardware: no IMU, skip all balance corrections
            pos = np.array([angle_data[i][frame] for i in range(4)])
        else:
            # Simulation: apply stance correction with IMU

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

        # Publish foot P-V-A for PlotJuggler
        self._publish_foot_pva(frame)

        return True

    def _publish_foot_pva(self, frame):
        """Publish foot Z position, velocity, acceleration per leg."""
        if self._foot_data is None:
            return
        msg = Float64()
        for i, leg in enumerate(GaitConfig.LEG_NAMES):
            z = float(self._foot_data[i][frame][2])  # Z position (mm)

            vz = z - self._foot_z_prev[leg]          # ΔZ per tick
            az = vz - self._foot_vz_prev[leg]        # ΔV per tick

            self._foot_z_prev[leg] = z
            self._foot_vz_prev[leg] = vz

            pubs = self._pub_foot[leg]
            msg.data = z;  pubs['pos'].publish(msg)
            msg.data = vz; pubs['vel'].publish(msg)
            msg.data = az; pubs['acc'].publish(msg)

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
        Real mode: Do nothing - random pose (hold at current pose).
        """
        if self.serial_publish.use_real:
            pass
        else:
            # Simulation: PD control to Gazebo INIT_POSE
            joint_names = self.serial_publish.controller_sim.joint_names
            targets = [GaitConfig.INIT_POSE[name] for name in joint_names]
            torques = self.serial_publish.controller_sim.compute_torques(targets)
            msg = Float64MultiArray()
            msg.data = torques
            self.serial_publish.pub_sim_gazebo.publish(msg)

    # ── Trajectory generation ─────────────────────────────────────

    def _trajectory_moving(self, x_center, y_val, reverse=False, z_offset=0):
        """Build D-shape foot path in Cartesian space (x, y, z)."""
        stride_length = 10
        x_forward = x_center + stride_length / 2
        x_backward = x_center - stride_length / 2
        z_stance = -170 + z_offset
        z_swing = -130
        lift_height = z_swing - z_stance
        lift_height = 100

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
        if self.serial_publish.use_real:
            z_low = -180
            z_high = -110
        else:
            z_low = -170
            z_high = -130

        waypoint = np.zeros((self._waypoint.rest, 3))
        waypoint[:, 0] = x_center
        waypoint[:, 1] = y_val
        waypoint[:, 2] = (z_low + (z_high - z_low)
                          * (0.5 - 0.5 * np.cos(np.linspace(
                              0, 2 * np.pi, self._waypoint.rest))))
        return waypoint

    # ── ROBOTOFF helpers ───────────────────────────────────────────

    def _tick_robotoff(self):
        """Advance ROBOTOFF (safe shutdown) by one frame."""
        if self.serial_publish.use_real:
            return self._tick_robotoff_real()
        # Simulation: not implemented (Ctrl+C is safe in sim)
        return False

    def _tick_robotoff_real(self):
        """Advance real-hardware ROBOTOFF by one frame."""
        total_frames = self._angle_data.shape[0]

        if self._step_current < total_frames:
            frame = self._angle_data[self._step_current].tolist()
            self.serial_publish.publish_real_12(frame)
            self._step_current += 1
            return True
        else:
            # Hold at safe-off position
            self.serial_publish.publish_real_12(self._robotoff_targets_real)
            if not self._robotoff_complete:
                self._robotoff_complete = True
                self._logger.info(
                    '╔══════════════════════════════════════════╗')
                self._logger.info(
                    '║   ROBOT OFF COMPLETE — SAFE TO Ctrl+C   ║')
                self._logger.info(
                    '╚══════════════════════════════════════════╝')
            return False

    def _generate_robotoff_real(self):
        """Generate three-phase safe-off trajectory in servo-degree space.

        Phase 1: linspace joint3 from current → collapse target
                 (knee folds → body drops to ground)
        Phase 2: linspace joint2 from current → fold target
                 (shoulder tucks → leg folds against body)
        Phase 3: linspace joint1 from current → splay target
                 (hip splays → legs collapse outward)

        No IK is used — targets are hardcoded SAFE_OFF_ANGLES.
        """
        # 1. Get current servo position (servo degrees)
        #    After ZERO homing, feedback is disabled so real_positions is
        #    stale.  Use the known homing targets as the true current state.
        if hasattr(self, '_homing_targets_real') and self._homing_targets_real:
            current = np.array(self._homing_targets_real, dtype=float)
            self._logger.info('ROBOTOFF: using homing targets as start position')
        else:
            positions = self.serial_publish.real_positions
            if not positions:
                self._logger.warn(
                    'No real servo positions yet — waiting for feedback...')
                return None
            joint_names = [
                'joint_lf_1', 'joint_lf_2', 'joint_lf_3',
                'joint_lb_1', 'joint_lb_2', 'joint_lb_3',
                'joint_rf_1', 'joint_rf_2', 'joint_rf_3',
                'joint_rb_1', 'joint_rb_2', 'joint_rb_3',
            ]
            current = np.array([positions.get(n, 0.0) for n in joint_names])
            self._logger.info('ROBOTOFF: using feedback positions as start')

        safe = GaitConfig.SAFE_OFF_ANGLES
        N = GaitConfig.ROBOTOFF_FRAMES_PER_PHASE
        legs = ['left-front', 'left-behind', 'right-front', 'right-behind']

        # Joint indices inside the 12-element array:
        #   LF: j1=0 j2=1 j3=2   LB: j1=3 j2=4 j3=5
        #   RF: j1=6 j2=7 j3=8   RB: j1=9 j2=10 j3=11
        j3_idx = [2, 5, 8, 11]
        j2_idx = [1, 4, 7, 10]
        j1_idx = [0, 3, 6, 9]

        j3_targets = [safe['joint3'][leg] for leg in legs]
        j2_targets = [safe['joint2'][leg] for leg in legs]
        j1_targets = [safe['joint1'][leg] for leg in legs]

        # ── Phase 1: collapse joint3 (joint1 & joint2 stay at current) ──
        phase1 = np.tile(current, (N, 1))          # start from current
        for k, idx in enumerate(j3_idx):
            phase1[:, idx] = np.linspace(
                current[idx], j3_targets[k], N)

        # Snapshot after phase 1: j3 at target, j1/j2 still at current
        mid1 = current.copy()
        for k, idx in enumerate(j3_idx):
            mid1[idx] = j3_targets[k]

        # ── Phase 2: fold joint2 (joint1 stays, joint3 stays at target) ──
        phase2 = np.tile(mid1, (N, 1))
        for k, idx in enumerate(j2_idx):
            phase2[:, idx] = np.linspace(
                mid1[idx], j2_targets[k], N)

        # Snapshot after phase 2: j3+j2 at target, j1 still at current
        mid2 = mid1.copy()
        for k, idx in enumerate(j2_idx):
            mid2[idx] = j2_targets[k]

        # ── Phase 3: splay joint1 (joint2+j3 stay at target) ──
        phase3 = np.tile(mid2, (N, 1))
        for k, idx in enumerate(j1_idx):
            phase3[:, idx] = np.linspace(
                mid2[idx], j1_targets[k], N)

        # Combine all three phases
        trajectory = np.vstack([phase1, phase2, phase3])

        # Final hold position (for after trajectory completes)
        final = mid2.copy()
        for k, idx in enumerate(j1_idx):
            final[idx] = j1_targets[k]
        self._robotoff_targets_real = final.tolist()

        self._logger.info(
            f'ROBOTOFF trajectory: 3 phases × {N} frames = '
            f'{3 * N} total ({3 * N * 7 / 1000:.1f} s)')
        self._logger.info(
            f'  Phase 1 — joint3 collapse: '
            f'LF→{j3_targets[0]}° LB→{j3_targets[1]}° '
            f'RF→{j3_targets[2]}° RB→{j3_targets[3]}°')
        self._logger.info(
            f'  Phase 2 — joint2 fold:     '
            f'LF→{j2_targets[0]}° LB→{j2_targets[1]}° '
            f'RF→{j2_targets[2]}° RB→{j2_targets[3]}°')
        self._logger.info(
            f'  Phase 3 — joint1 splay:    '
            f'LF→{j1_targets[0]}° LB→{j1_targets[1]}° '
            f'RF→{j1_targets[2]}° RB→{j1_targets[3]}°')
        return trajectory

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

    def _generate_home_real(self):
        """Generate homing trajectory for real hardware (in servo degrees)."""
        # 1. Compute IK standing targets for all 4 legs
        lf = self._kinematics.inverse(125, 135, -170 + GaitConfig.LF_Z_OFFSET, "left-front")
        lb = self._kinematics.inverse(-125, 135, -170 + GaitConfig.LB_Z_OFFSET, "left-behind")
        rf = self._kinematics.inverse(125, -135, -170 + GaitConfig.RF_Z_OFFSET, "right-front")
        rb = self._kinematics.inverse(-125, -135, -170 + GaitConfig.RB_Z_OFFSET, "right-behind")

        # 2. Convert IK degrees → servo degrees
        lf_servo = self.serial_publish._ik_to_servo_lf(*lf)
        lb_servo = self.serial_publish._ik_to_servo_lb(*lb)
        rf_servo = self.serial_publish._ik_to_servo_rf(*rf)
        rb_servo = self.serial_publish._ik_to_servo_rb(*rb)

        # Target: 12 servo angles in degrees
        target = list(lf_servo) + list(lb_servo) + list(rf_servo) + list(rb_servo)

        # 3. Read current servo positions (degrees from /joint_states_real)
        positions = self.serial_publish.real_positions
        if not positions:
            self._logger.warn('No real servo positions yet — waiting for feedback...')
            return None

        joint_names = [
            'joint_lf_1', 'joint_lf_2', 'joint_lf_3',
            'joint_lb_1', 'joint_lb_2', 'joint_lb_3',
            'joint_rf_1', 'joint_rf_2', 'joint_rf_3',
            'joint_rb_1', 'joint_rb_2', 'joint_rb_3',
        ]
        current = [positions.get(name, 0.0) for name in joint_names]

        # Disable feedback on both drivers to free the serial bus
        self.serial_publish.disable_feedback()

        # 4. Interpolate current → target over waypoint.zero frames
        num_frames = self._waypoint.zero
        trajectory = np.zeros((num_frames, 12))
        for step in range(num_frames):
            alpha = step / num_frames
            for j in range(12):
                trajectory[step, j] = current[j] + alpha * (target[j] - current[j])

        # Store targets for holding after homing completes
        self._homing_targets_real = target
        return trajectory

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
                if self.serial_publish.use_real:
                    params = GaitConfig.PARAMS_GAIT_BODY.get(leg_type)
                else:
                    params = GaitConfig.PARAMS_GAIT_FORWARD.get(leg_type)
                phase = GaitConfig.PARAMS_PHASESHIFT_PUSHUP
            case "BODY_SWAY":
                if self.serial_publish.use_real:
                    params = GaitConfig.PARAMS_GAIT_BODY.get(leg_type)
                else:
                    params = GaitConfig.PARAMS_GAIT_FORWARD.get(leg_type)
                phase = GaitConfig.PARAMS_PHASESHIFT_SWAY
            case "BODY_CIRCLE":
                if self.serial_publish.use_real:
                    params = GaitConfig.PARAMS_GAIT_BODY.get(leg_type)
                else:
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
            z_offset = params.get("z_offset", 0) if self.serial_publish.use_real else 0
            waypoint = self._trajectory_moving(
                params["x_center"], params["y_val"], reverse, z_offset)

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
                self._pub_balance_mode.publish(String(data='HOME'))
                if self.serial_publish.use_real:
                    return self._generate_home_real()
                else:
                    return self._generate_home()
            case "ROBOTOFF":
                self._foot_data = None
                self._robotoff_complete = False
                if self.serial_publish.use_real:
                    return self._generate_robotoff_real()
                else:
                    self._logger.info('ROBOTOFF: simulation mode — nothing to do.')
                    return None
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