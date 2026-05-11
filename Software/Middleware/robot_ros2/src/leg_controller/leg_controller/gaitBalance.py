"""Balance correction: IMU-based home offset and gait stance correction.

Provides two correction modes:
  Home:  Adjusts knee angles to keep the body level while standing.
  Gait:  Adjusts stance-phase foot positions to counteract body tilt.
"""

import math
import time
import numpy as np
from std_msgs.msg import Float64
from leg_controller.gaitConfig import (
    LEG_NAMES, HomeBalanceConfig, StanceDetection, HOME_DIAGNOSTIC_SIGNALS,
)


# ── 1. State Structures ──────────────────────────────────────────────


class HomeBalanceState:
    """Tracks per-leg smoothed offsets for home posture correction."""

    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.t_start = None
        self.offsets = {leg: 0.0 for leg in LEG_NAMES}


class StanceCorrectionInput:
    """Received roll/pitch correction from balance controller during gait."""

    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0


# ── 2. Home Posture Correction ───────────────────────────────────────


def applyHomeOffset(targets, state, diagPubs):
    """Apply theta3 posture correction to joint targets (in-place).

    Adjusts knee angles based on filtered IMU roll/pitch to keep
    the body level while standing at the home position.

    Args:
        targets:  list of 12 joint angles (radians), modified in-place.
        state:    HomeBalanceState with current IMU readings.
        diagPubs: dict of ROS publishers for diagnostic signals.
    """
    cfg = HomeBalanceConfig
    now = time.time()

    if state.t_start is None:
        state.t_start = now
    if (now - state.t_start) < cfg.DELAY_START:
        return

    roll_dz = deadzone(state.roll, cfg.DEADZONE)
    pitch_dz = deadzone(state.pitch, cfg.DEADZONE)
    in_deadzone = (roll_dz == 0.0 and pitch_dz == 0.0)

    roll_scaled = roll_dz * cfg.GAIN
    pitch_scaled = pitch_dz * cfg.GAIN
    sat = cfg.SATURATION

    raw_offsets = {
        'left-front':   max(-sat, min(sat, -(roll_scaled - pitch_scaled))),
        'left-behind':  max(-sat, min(sat, -(roll_scaled + pitch_scaled))),
        'right-front':  max(-sat, min(sat, -(roll_scaled + pitch_scaled))),
        'right-behind': max(-sat, min(sat, -(roll_scaled - pitch_scaled))),
    }

    # Publish diagnostics
    diag_values = [
        state.roll, state.pitch,
        1.0 if in_deadzone else 0.0,
        roll_scaled, pitch_scaled,
        raw_offsets['left-front'], raw_offsets['left-behind'],
        raw_offsets['right-front'], raw_offsets['right-behind'],
    ]
    msg = Float64()
    for sig, val in zip(HOME_DIAGNOSTIC_SIGNALS, diag_values):
        msg.data = val
        diagPubs[sig].publish(msg)

    # Decay offsets when inside deadzone
    if in_deadzone:
        for leg in state.offsets:
            state.offsets[leg] *= cfg.LPF_ALPHA
        return

    # Low-pass filter and apply to targets
    alpha = cfg.LPF_ALPHA
    for leg in raw_offsets:
        state.offsets[leg] = (alpha * state.offsets[leg]
                              + (1 - alpha) * raw_offsets[leg])
    for leg, offset in state.offsets.items():
        targets[cfg.THETA3_IDX[leg]] += offset


# ── 3. Gait Stance Correction ────────────────────────────────────────


def applyStanceCorrection(frame, angleData, footData, correction, kinematics):
    """Apply phase-aware foot correction during locomotion.

    Only adjusts feet in stance phase (on the ground). Uses body-tilt
    compensation via rotation matrix to counteract IMU drift.

    Args:
        frame:      current frame index in the gait cycle.
        angleData:  array of 4 objects, each shape (N, 3) — joint angles.
        footData:   array of 4 objects, each shape (N, 3) — foot positions.
        correction: StanceCorrectionInput with roll/pitch values.
        kinematics: Kinematics instance for inverse kinematics.

    Returns:
        4×3 numpy array of joint angles for this frame.
    """
    if footData is None:
        return np.array([angleData[i][frame] for i in range(4)])

    pos = np.zeros((4, 3))
    for i, leg in enumerate(LEG_NAMES):
        foot = footData[i][frame].copy()
        is_stance = abs(foot[2] - StanceDetection.Z) < StanceDetection.Z_THRESHOLD

        if is_stance:
            adjusted = compensateBodyTilt(
                foot, correction.roll, correction.pitch)
            try:
                pos[i] = kinematics.inverse(*adjusted, leg)
            except (ValueError, ZeroDivisionError):
                pos[i] = angleData[i][frame]
        else:
            pos[i] = angleData[i][frame]

    return pos


def compensateBodyTilt(footPos, rollAngle, pitchAngle):
    """Apply body-tilt compensation R_y(pitch) × R_x(roll).

    Rotates the foot position to counteract measured body tilt,
    keeping only the Z-axis adjustment (X and Y are preserved).

    Args:
        footPos:    [x, y, z] foot position in mm.
        rollAngle:  body roll angle in radians.
        pitchAngle: body pitch angle in radians.

    Returns:
        Adjusted [x, y, z] foot position.
    """
    cr, sr = math.cos(rollAngle),  math.sin(rollAngle)
    cp, sp = math.cos(pitchAngle), math.sin(pitchAngle)
    R = np.array([
        [cp,  sr * sp, cr * sp],
        [0,   cr,      -sr],
        [-sp, sr * cp,  cr * cp],
    ])
    corrected = R @ footPos
    corrected[0] = footPos[0]   # preserve X
    corrected[1] = footPos[1]   # preserve Y
    return corrected


# ── 4. Utilities ─────────────────────────────────────────────────────


def deadzone(value, threshold):
    """Return zero inside threshold; reduce magnitude outside."""
    if abs(value) <= threshold:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - threshold)
