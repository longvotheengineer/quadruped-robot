import math
import time
import numpy as np
from dataclasses import dataclass
from leg_controller.kinematics import Kinematics
from leg_controller.serialPublish import SerialPublish
from leg_controller.quinticPlanning import quintic_planning
from std_msgs.msg import Float64MultiArray, Bool
from geometry_msgs.msg import Vector3

@dataclass(frozen=True)
class Waypoint:
    zero   : int
    stance : int
    swing  : int
    rest   : int

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

        self.waypoint = Waypoint(3000, 150, 30, 600)

        # Trajectory state
        self.gait_angle_data = None
        self.gait_foot_data  = None
        self.step_current = 0
        self.step_final = 0

        # Posture correction from balance_controller
        self.imu_roll_corr  = 0.0
        self.imu_pitch_corr = 0.0
        self.imu_tim_current = None
        node.create_subscription(
            Vector3, '/posture/correction', self.imu_callback, 10)

        # ── Diagnostic publisher for IMU home controller ──────────
        self.pub_diag_imu_home = node.create_publisher(
            Float64MultiArray, '/diag/imu_home/offsets', 10)

        # ── Balance controller enable gate ────────────────────────
        self.pub_balance_enable = node.create_publisher(Bool, '/balance/enable', 10)
        self.balance_enabled = False

        # ── Smoothed θ₃ offsets (EMA) — ramps corrections gradually ─
        # Instead of applying new offsets instantly, blend toward them
        # each tick for smooth physical leg movement during tilting.
        self.smoothed_offsets = {
            'left-front': 0.0, 'left-behind': 0.0,
            'right-front': 0.0, 'right-behind': 0.0}
        self.OFFSET_SMOOTH_ALPHA = 0.92  # higher → smoother (slower ramp)

    # ── IMU Constants ───────────────────────────────────────────────

    # Joint indices for θ₃ in the 12-element target array
    IMU_HOME_THETA3INDICES = {
        'left-front':   2,
        'left-behind':  5,
        'right-front':  8,
        'right-behind': 11,}

    # Seconds to wait after startup before applying posture correction.
    # Prevents IMU transient during spawn from triggering false corrections.
    IMU_HOME_TIMSTART = 3.0

    # Multiplier for PID correction → θ₃ offset (rad).
    # At homing pose: |∂body_height/∂θ₃| ≈ 71 mm/rad (new URDF), ∂body_height/∂θ₂ ≈ 0.
    # θ₃ is chosen because it is the only effective axis for body height control.
    IMU_HOME_GAIN = 4.0

    # Maximum θ₃ offset (rad) per joint to prevent extreme poses.
    IMU_HOME_SATURATION = 0.8

    # Deadzone — uses smooth ramp instead of hard on/off
    IMU_HOME_DEADZONE = 0.02 # rad — matches balance controller smooth deadzone
    IMU_GAIT_DEADZONE = 0.10 # rad — slightly reduced for gait

    # Using the standing gain makes the robot acts explosively
    # IMU_GAIT_GAIN helps to reduce the gain
    IMU_GAIT_GAIN = 0.5

    # ── Leg configuration ──────────────────────────────────────────

    LEG_NAMES = [
        'left-front', 
        'left-behind', 
        'right-front', 
        'right-behind']

    INIT_POSE = {
        'joint_lf_1':  0.3,  'joint_lf_2':  3*np.pi/2,  'joint_lf_3':  0.5,
        'joint_lb_1':  0.3,  'joint_lb_2':  3*np.pi/2,  'joint_lb_3':  0.5,
        'joint_rf_1':  0.3,  'joint_rf_2': -3*np.pi/2,  'joint_rf_3': -0.5,
        'joint_rb_1':  0.3,  'joint_rb_2': -3*np.pi/2,  'joint_rb_3': -0.5,}

    PARAMS_GAIT_FORWARD      = {
        "left-front":    {"x_center":  125, "y_val":  135, "reverse": False},
        "left-behind":   {"x_center": -125, "y_val":  135, "reverse": False},
        "right-front":   {"x_center":  125, "y_val": -135, "reverse": False},
        "right-behind":  {"x_center": -125, "y_val": -135, "reverse": False},}

    PARAMS_GAIT_BACKWARD     = {
        "left-front":    {"x_center":  125, "y_val":  135, "reverse": True},
        "left-behind":   {"x_center": -125, "y_val":  135, "reverse": True},
        "right-front":   {"x_center":  125, "y_val": -135, "reverse": True},
        "right-behind":  {"x_center": -125, "y_val": -135, "reverse": True},}

    PARAMS_GAIT_TURN_RIGHT   = {
        "left-front":    {"x_center":  125, "y_val":  135, "reverse": False},
        "left-behind":   {"x_center": -125, "y_val":  135, "reverse": False},
        "right-front":   {"x_center":  125, "y_val": -135, "reverse": True},
        "right-behind":  {"x_center": -125, "y_val": -135, "reverse": True},}

    PARAMS_GAIT_TURN_LEFT    = {
        "left-front":    {"x_center":  125, "y_val":  135, "reverse": True},
        "left-behind":   {"x_center": -125, "y_val":  135, "reverse": True},
        "right-front":   {"x_center":  125, "y_val": -135, "reverse": False},
        "right-behind":  {"x_center": -125, "y_val": -135, "reverse": False},}

    PARAMS_PHASESHIFT_TROT   = {
        "left-front":   0.00,
        "right-behind": 0.00,
        "left-behind":  0.50,
        "right-front":  0.50,}

    PARAMS_PHASESHIFT_WALK   = {
        "left-front":   0.00,
        "right-behind": 0.25,
        "right-front":  0.50,
        "left-behind":  0.75,}

    PARAMS_PHASESHIFT_PUSHUP = {
        "left-front":   0.00,
        "left-behind":  0.00,
        "right-front":  0.00,
        "right-behind": 0.00,}

    PARAMS_PHASESHIFT_SWAY   = {
        "left-front":   0.00,
        "left-behind":  0.00,
        "right-front":  0.50,
        "right-behind": 0.50,}

    PARAMS_PHASESHIFT_CIRCLE = {
        "left-front":   0.00,
        "right-front":  0.25,
        "right-behind": 0.50,
        "left-behind":  0.75,}

    CONTROL_VELOCITY = True

    # ── IMU controller ────────────────────────────────────────────

    def imu_callback(self, msg: Vector3):
        """Receive filtered PID correction angles from balance_controller"""
        self.imu_roll_corr  = msg.x
        self.imu_pitch_corr = msg.y

    def imu_controllerOutput_home(self, targets):
        """Apply θ₃ posture correction to a 12-element joint target array (in-place).

        Guards:
          - Waits IMU_HOME_TIMSTART seconds after first call (IMU settles)
          - Ignores tilt below 0.005 rad dead zone (noise rejection)

        Sign convention (from Jacobian analysis at INIT_POSE):
          - Left  θ₃ increasing → body goes UP  (+97 mm/rad)
          - Right θ₃ increasing → body goes DOWN (-97 mm/rad)
          So both sides receive the SAME sign offset for roll,
          and OPPOSITE signs for front vs back (pitch).
        """
        # Startup delay: skip correction until IMU has settled
        now = time.time()
        if self.imu_tim_current is None:
            self.imu_tim_current = now
        if (now - self.imu_tim_current) < self.IMU_HOME_TIMSTART:
            return

        # ── Smooth deadzone: linearly ramp correction outside threshold ─
        # Eliminates chattering from hard on/off switching.
        def _smooth_dz(val, dz):
            if abs(val) <= dz:
                return 0.0
            sign = 1.0 if val > 0 else -1.0
            return sign * (abs(val) - dz)

        roll_corr  = _smooth_dz(self.imu_roll_corr,  self.IMU_HOME_DEADZONE)
        pitch_corr = _smooth_dz(self.imu_pitch_corr, self.IMU_HOME_DEADZONE)
        in_deadzone = (roll_corr == 0.0 and pitch_corr == 0.0)

        # Scale PID output → θ₃ offset
        r = roll_corr  * self.IMU_HOME_GAIN
        p = pitch_corr * self.IMU_HOME_GAIN
        clamp = self.IMU_HOME_SATURATION

        # Compute and clamp per-leg offsets
        offsets = {
            'left-front':   max(-clamp, min(clamp, -(r - p))),
            'left-behind':  max(-clamp, min(clamp, -(r + p))),
            'right-front':  max(-clamp, min(clamp, -(r + p))),
            'right-behind': max(-clamp, min(clamp, -(r - p))),}

        # ── Publish diagnostics (always, even in deadzone) ────────
        diag = Float64MultiArray()
        diag.data = [
            self.imu_roll_corr,         # [0] roll correction input
            self.imu_pitch_corr,        # [1] pitch correction input
            1.0 if in_deadzone else 0.0,# [2] deadzone active flag
            r,                          # [3] r_scaled (roll × gain)
            p,                          # [4] p_scaled (pitch × gain)
            offsets['left-front'],      # [5] offset_lf
            offsets['left-behind'],     # [6] offset_lb
            offsets['right-front'],     # [7] offset_rf
            offsets['right-behind'],    # [8] offset_rb
        ]
        self.pub_diag_imu_home.publish(diag)

        # Skip application if inside deadzone → ramp offsets toward zero
        if in_deadzone:
            for leg in self.smoothed_offsets:
                self.smoothed_offsets[leg] *= self.OFFSET_SMOOTH_ALPHA
            return

        # ── Smooth offset ramping (EMA) ───────────────────────────
        # Blend current offsets toward new targets each tick.
        # This makes leg movements gradual during ramp tilting.
        a = self.OFFSET_SMOOTH_ALPHA
        for leg in offsets:
            self.smoothed_offsets[leg] = (a * self.smoothed_offsets[leg] +
                                         (1 - a) * offsets[leg])

        # Apply smoothed offsets to θ₃ joints
        for leg, offset in self.smoothed_offsets.items():
            targets[self.IMU_HOME_THETA3INDICES[leg]] += offset

    def imu_controllerOutput_gait(self, pos_foot_raw, imu_roll_corr, imu_pitch_corr):
        """ This method is only used for one leg at a time
        Apply body-tilt compensation using rotation matrix R_y(pitch) x R_x(roll)
    
        From the paper: p_target = R_inverse · p_current.
        The correction angles are negated because we rotate the foot positions
        in the opposite direction of the body tilt.
        """
        cr, sr = math.cos(-imu_roll_corr), math.sin(-imu_roll_corr)
        cp, sp = math.cos(-imu_pitch_corr), math.sin(-imu_pitch_corr)
        R = np.array([[cp,    sr*sp,  cr*sp],
                      [0,     cr,    -sr   ],
                      [-sp,   sr*cp,  cr*cp]])
        return R @ pos_foot_raw

    # ── Temporary Init pose ───────────────────────────────────────

    def init_pose(self):
        """Hold at INIT_POSE (no balance correction)"""
        joint_names = self.serial_publish.controller_sim.joint_names
        targets = [self.INIT_POSE[name] for name in joint_names]
        torques = self.serial_publish.controller_sim.compute_torques(targets)
        msg = Float64MultiArray()
        msg.data = torques
        self.serial_publish.pub_sim_gazebo.publish(msg)

    # ── Trajectory generation ─────────────────────────────────────

    def trajectory_moving(self, x_center, y_val, reverse=False):
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

    def trajectory_resting(self, x_center, y_val):
        """The robot doesn't move, feet stay planted, body oscillates"""
        z_low  = -170    # body down (legs bent)
        z_high = -130    # body up (legs extended)

        waypoint = np.zeros((self.waypoint.rest, 3))
        waypoint[:, 0] = x_center
        waypoint[:, 1] = y_val
        waypoint[:, 2] = z_low + (z_high - z_low) * (0.5 - 0.5 * np.cos(np.linspace(0, 2 * np.pi, self.waypoint.rest)))

        return waypoint

    # ── Gait generation ───────────────────────────────────────────

    def generate_home(self):
        """Precalculate homing trajectory: all 4 legs move simultaneously"""
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

    def generate_gait(self, leg_type):
        """Generate one leg's full gait cycle: foot path → IK → phase shift.
        Returns (joint_angles, pos_foot_positions) tuple."""
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
            waypoint = self.trajectory_resting(params["x_center"], params["y_val"])
        else:
            reverse = params.get("reverse", False)
            waypoint = self.trajectory_moving(params["x_center"], params["y_val"], reverse)

        # Convert to joint angles via IK
        theta_i = np.zeros_like(waypoint)
        for i in range(waypoint.shape[0]):
            theta_i[i] = self.kinematics.inverse(*waypoint[i], leg_type)

        # Apply phase shift to BOTH angles and foot positions
        shift = round(waypoint.shape[0] * phase.get(leg_type, 0))
        return np.roll(theta_i, shift, axis=0), np.roll(waypoint, shift, axis=0)

    # ── Command dispatch ──────────────────────────────────────────

    def gait_change(self):        
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
        self.gait_angle_data = self.gait_change()
        self.step_current = 0
        self.step_final = 0

    def control_tick(self):
        if self.gait_angle_data is None:
            return False

        if self.gait_msg.cmd == "ZERO":
            return self._tick_home()
        else:
            return self._tick_gait()

    # ── Tick functions ────────────────────────────────────────────

    def _tick_home(self):
        """Advance homing by one frame: publish joint targets via PD torque control"""
        total_frames = self.gait_angle_data.shape[0]
        if self.step_current < total_frames:
            # Transitioning to zero pose — no correction during movement
            targets = self.gait_angle_data[self.step_current].tolist()
            torques = self.serial_publish.controller_sim.compute_torques(targets)
            msg = Float64MultiArray()
            msg.data = torques
            self.serial_publish.pub_sim_gazebo.publish(msg)
            self.step_current += 1
            return True
        else:
            # Holding at zero pose — apply posture correction
            if not self.balance_enabled:
                self.pub_balance_enable.publish(Bool(data=True))
                self.balance_enabled = True
                self.get_logger.info('Homing complete — enabling balance PID')
            joint_names = self.serial_publish.controller_sim.joint_names
            targets = [self.homing_targets[name] for name in joint_names]
            self.imu_controllerOutput_home(targets)
            torques = self.serial_publish.controller_sim.compute_torques(targets)
            msg = Float64MultiArray()
            msg.data = torques
            self.serial_publish.pub_sim_gazebo.publish(msg)
            return False

    def _tick_gait(self):
        """Advance gait by one frame. step=0 means run continuously"""
        theta_i = self.gait_angle_data
        frame = self.step_current

        # Deadzone makes the robot not react to small tilt angles
        activate_corr = (abs(self.imu_roll_corr) > self.IMU_GAIT_DEADZONE) or \
                        (abs(self.imu_pitch_corr) > self.IMU_GAIT_DEADZONE)

        # Pass the off-line planning into the PID controller
        # if self.gait_foot_data is not None and activate_corr:
            # pos = np.zeros((4, 3))
            # try:
            #     for i, leg in enumerate(self.LEG_NAMES):
            #         pos_foot_raw = self.gait_foot_data[i][frame].copy()
            #         pos_foot_adjusted = self.imu_controllerOutput_gait(pos_foot_raw,
            #                                                             self.imu_roll_corr * self.IMU_GAIT_GAIN,
            #                                                             self.imu_pitch_corr * self.IMU_GAIT_GAIN)
            #         pos[i] = self.kinematics.inverse(*pos_foot_adjusted, leg)
            # except:
            #     pos = np.array([theta_i[i][frame] for i in range(4)])
        # else:
        pos = np.array([theta_i[i][frame] for i in range(4)])

        # Keep publishing the last frame after gait completes to hold position.
        # The effort controller requires continuous torque commands;
        # if we stop publishing, effort = 0 and the robot collapses.
        if self.gait_msg.step > 0 and self.step_final >= self.gait_msg.step:
            self.serial_publish.publish_message(pos)
            return False

        self.serial_publish.publish_message(pos)

        # Advance frame
        self.step_current += 1
        if self.step_current >= theta_i[0].shape[0]:
            self.step_current = 0
            self.step_final += 1

        return True