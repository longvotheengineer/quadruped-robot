"""Gait configuration: constants, timing, per-leg parameters, and tuning.

This module contains all tunable parameters for the quadruped's gait system.
No logic — pure data definitions.
"""

import numpy as np
from dataclasses import dataclass


# ── 1. Data Structures ───────────────────────────────────────────────


@dataclass(frozen=True)
class GaitTiming:
    """Frame counts for each phase of a gait cycle."""
    homing: int    # frames for homing interpolation
    stance: int    # frames foot stays on ground
    swing:  int    # frames foot is in the air
    rest:   int    # frames for body posture oscillation


@dataclass(frozen=True)
class RobotDimensions:
    """Physical dimensions of the robot (mm)."""
    L:  float      # body length (hip-to-hip longitudinal)
    W:  float      # body width (hip-to-hip lateral)
    l1: float      # hip link length
    l2: float      # thigh link length
    l3: float      # shank link length


# ── 2. Physical Robot Properties ─────────────────────────────────────


ROBOT = RobotDimensions(L=209, W=191, l1=26, l2=106, l3=125)

LEG_NAMES = ['left-front', 'left-behind', 'right-front', 'right-behind']

LEG_ABBREVIATIONS = {
    'left-front':   'lf',
    'left-behind':  'lb',
    'right-front':  'rf',
    'right-behind': 'rb',
}

JOINT_NAMES = [
    'joint_lf_1', 'joint_lf_2', 'joint_lf_3',
    'joint_lb_1', 'joint_lb_2', 'joint_lb_3',
    'joint_rf_1', 'joint_rf_2', 'joint_rf_3',
    'joint_rb_1', 'joint_rb_2', 'joint_rb_3',
]


# ── 3. Global Trajectory & Timing ────────────────────────────────────


CONTROL_VELOCITY = True     # Use quintic polynomial (True) or linear (False)
STANCE_Z = -170.0           # nominal foot Z position (mm)

TIMING_TROT_FORWARD  = GaitTiming(homing=200, stance=800, swing=40,  rest=1000)
TIMING_TROT_BACKWARD = GaitTiming(homing=200, stance=600, swing=35,  rest=1000)
TIMING_WALK          = GaitTiming(homing=200, stance=200, swing=35,  rest=1000)
TIMING_WAVE          = GaitTiming(homing=200, stance=300, swing=125, rest=1000)
TIMING_TURN          = GaitTiming(homing=200, stance=300, swing=35,  rest=1000)
TIMING_STRAFE        = GaitTiming(homing=200, stance=400, swing=55,  rest=1000)
TIMING_BODY          = GaitTiming(homing=200, stance=200, swing=35,  rest=1200)


# ── 4. Hardware Calibration & Poses ──────────────────────────────────


# Per-leg Z offset for mechanical height compensation (real hardware)
Z_OFFSETS = {
    'left-front':  -5,
    'left-behind':  0,
    'right-front':  0,
    'right-behind': -5,
}

# Initial pose for Gazebo simulation (radians)
INIT_POSE = {
    'joint_lf_1':  0.3,  'joint_lf_2':  3 * np.pi / 2,  'joint_lf_3':  0.5,
    'joint_lb_1':  0.3,  'joint_lb_2':  3 * np.pi / 2,  'joint_lb_3':  0.5,
    'joint_rf_1':  0.3,  'joint_rf_2': -3 * np.pi / 2,  'joint_rf_3': -0.5,
    'joint_rb_1':  0.3,  'joint_rb_2': -3 * np.pi / 2,  'joint_rb_3': -0.5,
}

# Safe shutdown angles (servo degrees, real hardware only)
SAFE_OFF_ANGLES = {
    'joint3': {
        'left-front':   -47.0,
        'left-behind':  -47.0,
        'right-front':   47.0,
        'right-behind':  47.0,
    },
    'joint2': {
        'left-front':  -160.0,
        'left-behind':  250.0,
        'right-front':  160.0,
        'right-behind': 160.0,
    },
    'joint1': {
        'left-front':   -50.0,
        'left-behind':   50.0,
        'right-front':   50.0,
        'right-behind': -50.0,
    },
}

SHUTDOWN_FRAMES_PER_PHASE = 300


# ── 5. Phase Shift Definitions ───────────────────────────────────────


PHASESHIFT_TROT = {
    "left-front": 0.00,  "right-behind": 0.00,
    "left-behind": 0.50, "right-front":  0.50,
}

PHASESHIFT_WALK = {
    "left-front": 0.00,  "right-behind": 0.25,
    "right-front": 0.50, "left-behind":  0.75,
}

PHASESHIFT_HEAVE = {
    "left-front": 0.00,  "left-behind": 0.00,
    "right-front": 0.00, "right-behind": 0.00,
}

PHASESHIFT_ROLL = {
    "left-front": 0.00,  "left-behind": 0.00,
    "right-front": 0.50, "right-behind": 0.50,
}

PHASESHIFT_PITCH = {
    "left-front": 0.00,  "right-front": 0.00,
    "left-behind": 0.50, "right-behind": 0.50,
}

PHASESHIFT_CIRCLE = {
    "left-front": 0.00,  "right-front": 0.25,
    "right-behind": 0.50, "left-behind": 0.75,
}


# ── 6. Gait Command Parameters ───────────────────────────────────────


PARAMS_GAIT_FORWARD = {
    "left-front":   {"x_center":  125, "y_val":  135, "reverse": False, "z_offset": Z_OFFSETS['left-front'],  "stride": 20, "lift": 100},
    "left-behind":  {"x_center": -125, "y_val":  135, "reverse": False, "z_offset": Z_OFFSETS['left-behind'], "stride": 20, "lift": 100},
    "right-front":  {"x_center":  125, "y_val": -135, "reverse": False, "z_offset": Z_OFFSETS['right-front'], "stride": 20, "lift": 100},
    "right-behind": {"x_center": -125, "y_val": -135, "reverse": False, "z_offset": Z_OFFSETS['right-behind'],"stride": 20, "lift": 100},
}

PARAMS_GAIT_BACKWARD = {
    "left-front":   {"x_center":  125, "y_val":  135, "reverse": True, "z_offset": Z_OFFSETS['left-front'],  "stride": 20, "lift": 80},
    "left-behind":  {"x_center": -125, "y_val":  135, "reverse": True, "z_offset": Z_OFFSETS['left-behind'], "stride": 20, "lift": 80},
    "right-front":  {"x_center":  125, "y_val": -135, "reverse": True, "z_offset": Z_OFFSETS['right-front'], "stride": 20, "lift": 80},
    "right-behind": {"x_center": -125, "y_val": -135, "reverse": True, "z_offset": Z_OFFSETS['right-behind'],"stride": 20, "lift": 80},
}

PARAMS_GAIT_TURN_RIGHT = {
    "left-front":   {"x_center":  125, "y_val":  135, "reverse": False, "z_offset": Z_OFFSETS['left-front'],  "stride": 20, "lift": 80},
    "left-behind":  {"x_center": -125, "y_val":  135, "reverse": False, "z_offset": Z_OFFSETS['left-behind'], "stride": 20, "lift": 80},
    "right-front":  {"x_center":  125, "y_val": -135, "reverse": True,  "z_offset": Z_OFFSETS['right-front'], "stride": 20, "lift": 80},
    "right-behind": {"x_center": -125, "y_val": -135, "reverse": True,  "z_offset": Z_OFFSETS['right-behind'],"stride": 20, "lift": 80},
}

PARAMS_GAIT_TURN_LEFT = {
    "left-front":   {"x_center":  125, "y_val":  135, "reverse": True,  "z_offset": Z_OFFSETS['left-front'],  "stride": 20, "lift": 80},
    "left-behind":  {"x_center": -125, "y_val":  135, "reverse": True,  "z_offset": Z_OFFSETS['left-behind'], "stride": 20, "lift": 80},
    "right-front":  {"x_center":  125, "y_val": -135, "reverse": False, "z_offset": Z_OFFSETS['right-front'], "stride": 20, "lift": 80},
    "right-behind": {"x_center": -125, "y_val": -135, "reverse": False, "z_offset": Z_OFFSETS['right-behind'],"stride": 20, "lift": 80},
}

PARAMS_GAIT_STRAFE_RIGHT = {
    "left-front":   {"x_center":  125, "y_val":  135, "z_offset": Z_OFFSETS['left-front'],  "y_stride": 40, "lift": 80},
    "left-behind":  {"x_center": -125, "y_val":  135, "z_offset": Z_OFFSETS['left-behind'], "y_stride": 40, "lift": 80},
    "right-front":  {"x_center":  125, "y_val": -135, "z_offset": Z_OFFSETS['right-front'], "y_stride": 40, "lift": 80},
    "right-behind": {"x_center": -125, "y_val": -135, "z_offset": Z_OFFSETS['right-behind'],"y_stride": 40, "lift": 80},
}

PARAMS_GAIT_STRAFE_LEFT = {
    "left-front":   {"x_center":  125, "y_val":  135, "z_offset": Z_OFFSETS['left-front'],  "y_stride": 40, "lift": 80},
    "left-behind":  {"x_center": -125, "y_val":  135, "z_offset": Z_OFFSETS['left-behind'], "y_stride": 40, "lift": 80},
    "right-front":  {"x_center":  125, "y_val": -135, "z_offset": Z_OFFSETS['right-front'], "y_stride": 40, "lift": 80},
    "right-behind": {"x_center": -125, "y_val": -135, "z_offset": Z_OFFSETS['right-behind'],"y_stride": 40, "lift": 80},
}

PARAMS_GAIT_WAVE_FORWARD = {
    "left-front":   {"x_center":  125, "y_val":  135, "reverse": False, "z_offset": Z_OFFSETS['left-front'],  "stride": 40, "lift": 40},
    "left-behind":  {"x_center": -125, "y_val":  135, "reverse": False, "z_offset": Z_OFFSETS['left-behind'], "stride": 40, "lift": 40},
    "right-front":  {"x_center":  125, "y_val": -135, "reverse": False, "z_offset": Z_OFFSETS['right-front'], "stride": 40, "lift": 40},
    "right-behind": {"x_center": -125, "y_val": -135, "reverse": False, "z_offset": Z_OFFSETS['right-behind'],"stride": 40, "lift": 40},
}

PARAMS_GAIT_WAVE_BACKWARD = {
    "left-front":   {"x_center":  125, "y_val":  135, "reverse": True, "z_offset": Z_OFFSETS['left-front'],  "stride": 40, "lift": 40},
    "left-behind":  {"x_center": -125, "y_val":  135, "reverse": True, "z_offset": Z_OFFSETS['left-behind'], "stride": 40, "lift": 40},
    "right-front":  {"x_center":  125, "y_val": -135, "reverse": True, "z_offset": Z_OFFSETS['right-front'], "stride": 40, "lift": 40},
    "right-behind": {"x_center": -125, "y_val": -135, "reverse": True, "z_offset": Z_OFFSETS['right-behind'],"stride": 40, "lift": 40},
}

PARAMS_GAIT_BODY = {
    "left-front":   {"x_center":  125, "y_val":  135, "z_offset": Z_OFFSETS['left-front']},
    "left-behind":  {"x_center": -125, "y_val":  135, "z_offset": Z_OFFSETS['left-behind']},
    "right-front":  {"x_center":  125, "y_val": -135, "z_offset": Z_OFFSETS['right-front']},
    "right-behind": {"x_center": -125, "y_val": -135, "z_offset": Z_OFFSETS['right-behind']},
}


# ── 7. Special Subsystem Configurations ──────────────────────────────


# Body Posture
BODY_COMMANDS = {"BODY_HEAVE", "BODY_ROLL", "BODY_PITCH", "BODY_CIRCLE"}
BODY_BLEND_FRAMES = 200    # ~1.4 s at 7 ms/tick — cosine-eased transition
BODY_Z_RANGE = {           # Resting trajectory Z range (mm)
    'sim':  (-170, -130),
    'real': (-180, -100),
}

# Wave Gait (Z-only weight shifting)
WAVE_Z_EXTEND    = 25      # mm — lower Z for opposite-side support legs
WAVE_Z_COMPRESS  = 35      # mm — raise Z for same-side support leg
WAVE_LEAN_FRAMES = 250     # frames for the tilt transition

# Home Balance PID Tuning
class HomeBalanceConfig:
    """Tuning parameters for IMU-based home posture correction."""
    DELAY_START = 3.0               # seconds before correction starts
    THETA3_IDX = {                  # index of knee joint in 12-element array
        'left-front':   2,
        'left-behind':  5,
        'right-front':  8,
        'right-behind': 11,
    }
    GAIN       = 7.0                # scaling factor for roll/pitch error
    DEADZONE   = 0.02               # rad — errors below this are ignored
    SATURATION = 0.5                # rad — maximum correction magnitude
    LPF_ALPHA  = 0.95               # low-pass filter smoothing factor

# Stance Detection
class StanceDetection:
    """Thresholds for detecting stance phase during gait correction."""
    Z           = -170.0            # nominal stance Z (mm)
    Z_THRESHOLD = 5.0               # mm — tolerance band around stance Z


# ── 8. Diagnostics & Logging ─────────────────────────────────────────


HOME_DIAGNOSTIC_SIGNALS = [
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
