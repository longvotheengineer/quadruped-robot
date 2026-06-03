"""Trajectory generation: Cartesian foot paths and IK conversion.

Pure math module — no ROS dependencies, no sim/real branching.
All functions take explicit parameters and return numpy arrays.
"""

import numpy as np
from leg_controller.quinticPlanning import quintic_planning
from leg_controller.gaitConfig import (
    LEG_NAMES, CONTROL_VELOCITY, STANCE_Z, BODY_COMMANDS, BODY_Z_RANGE,
    PARAMS_GAIT_FORWARD, PARAMS_GAIT_BACKWARD, PARAMS_GAIT_BODY,
    PARAMS_GAIT_TURN_RIGHT, PARAMS_GAIT_TURN_LEFT,
    PARAMS_GAIT_STRAFE_RIGHT, PARAMS_GAIT_STRAFE_LEFT,
    PARAMS_GAIT_WAVE_FORWARD, PARAMS_GAIT_WAVE_BACKWARD,
    PHASESHIFT_TROT, PHASESHIFT_WALK, PHASESHIFT_HEAVE,
    PHASESHIFT_ROLL, PHASESHIFT_PITCH, PHASESHIFT_CIRCLE,
    WAVE_Z_EXTEND, WAVE_Z_COMPRESS, WAVE_LEAN_FRAMES,
)


# ── 1. Top-Level Interface ───────────────────────────────────────────


def generateLegCycle(cmd, leg, timing, kinematics, useReal):
    """Generate one leg's full gait cycle (joint angles + foot positions).

    Args:
        cmd:        gait command string.
        leg:        leg name.
        timing:     GaitTiming instance.
        kinematics: Kinematics instance for IK.
        useReal:    True for real hardware (applies z_offset).

    Returns:
        (angles, feet) — both shape (N, 3), or (None, None) if unknown.
    """
    params, phase = _lookupGaitParams(cmd, leg)
    if not params:
        return None, None

    z_offset = params.get("z_offset", 0) if useReal else 0

    if cmd in BODY_COMMANDS:
        z_low, z_high = BODY_Z_RANGE['real' if useReal else 'sim']
        waypoint = buildBodyPath(timing, params["x_center"],
                                 params["y_val"], z_low, z_high)
    elif cmd in ("STRAFE_RIGHT", "STRAFE_LEFT"):
        direction = -1 if cmd == "STRAFE_RIGHT" else 1
        waypoint = buildStrafePath(
            timing, params["x_center"], params["y_val"],
            params.get("y_stride", 20), z_offset,
            lift=params.get("lift", 80), direction=direction)
    else:
        waypoint = buildStridePath(
            timing, params["x_center"], params["y_val"],
            params.get("reverse", False), z_offset,
            stride=params.get("stride", 10),
            lift=params.get("lift", 80))

    # Convert foot positions → joint angles via IK
    angles = np.zeros_like(waypoint)
    for i in range(waypoint.shape[0]):
        angles[i] = kinematics.inverse(*waypoint[i], leg)

    # Apply phase shift
    shift = round(waypoint.shape[0] * phase.get(leg, 0))
    return np.roll(angles, shift, axis=0), np.roll(waypoint, shift, axis=0)


def generateWaveCycle(cmd, timing, kinematics, useReal):
    """Generate one-leg-at-a-time gait with Z-only weight shifting.

    For each leg in sequence (LF → LB → RF → RB):
      Phase 1 — Transition: smooth tilt from previous to current lean.
      Phase 2 — Rest:       hold tilted position.
      Phase 3 — Lift:       unloaded leg swings up and forward/back.
      Phase 4 — Rest:       hold tilted position.

    Args:
        cmd:        'WAVE_FORWARD' or 'WAVE_BACKWARD'.
        timing:     GaitTiming instance.
        kinematics: Kinematics instance.
        useReal:    True for real hardware.

    Returns:
        (angles_4, feet_4) — arrays of 4 objects, each shape (total, 3).
    """
    params = (PARAMS_GAIT_WAVE_BACKWARD if cmd == "WAVE_BACKWARD"
              else PARAMS_GAIT_WAVE_FORWARD)

    swing_frames = timing.swing
    hold_frames = timing.stance
    lean_frames = WAVE_LEAN_FRAMES
    seg_len = lean_frames + 2 * hold_frames + swing_frames
    total_frames = seg_len * 4

    # Precompute nominal foot positions
    normal_foot = {}
    for leg in LEG_NAMES:
        p = params[leg]
        z_off = p.get('z_offset', 0) if useReal else 0
        normal_foot[leg] = np.array([p['x_center'], p['y_val'],
                                     STANCE_Z + z_off])

    def _zAdjust(active_leg, this_leg):
        """Z adjustment for this_leg when active_leg is being lifted."""
        if this_leg == active_leg:
            return 0.0
        active_y = params[active_leg]['y_val']
        this_y = normal_foot[this_leg][1]
        if np.sign(this_y) == np.sign(active_y):
            return -WAVE_Z_EXTEND       # same side: extend (push down)
        return WAVE_Z_COMPRESS           # opposite side: compress (pull up)

    angles_all = np.empty(4, dtype=object)
    feet_all = np.empty(4, dtype=object)

    for idx, leg in enumerate(LEG_NAMES):
        lift = params[leg].get('lift', 80)
        full_angles = np.zeros((total_frames, 3))
        full_feet = np.zeros((total_frames, 3))

        for active_idx in range(4):
            active_leg = LEG_NAMES[active_idx]
            prev_leg = LEG_NAMES[(active_idx - 1) % 4]
            s0 = active_idx * seg_len

            prev_foot = normal_foot[leg].copy()
            prev_foot[2] += _zAdjust(prev_leg, leg)
            curr_foot = normal_foot[leg].copy()
            curr_foot[2] += _zAdjust(active_leg, leg)
            curr_ang = np.array(kinematics.inverse(*curr_foot, leg))

            # Phase 1 — Transition (previous tilt → current tilt)
            p1 = s0
            for f in range(lean_frames):
                alpha = f / lean_frames
                ft = prev_foot + alpha * (curr_foot - prev_foot)
                full_feet[p1 + f] = ft
                full_angles[p1 + f] = kinematics.inverse(*ft, leg)

            # Phase 2 — Rest at current tilt
            p2 = p1 + lean_frames
            full_feet[p2:p2 + hold_frames] = curr_foot
            full_angles[p2:p2 + hold_frames] = curr_ang

            # Phase 3 — Swing (lift) or hold
            p3 = p2 + hold_frames
            if active_idx == idx:
                for f in range(swing_frames):
                    ft = curr_foot.copy()
                    ft[2] += lift * np.sin(np.pi * f / swing_frames)
                    full_feet[p3 + f] = ft
                    full_angles[p3 + f] = kinematics.inverse(*ft, leg)
            else:
                full_feet[p3:p3 + swing_frames] = curr_foot
                full_angles[p3:p3 + swing_frames] = curr_ang

            # Phase 4 — Rest at current tilt
            p4 = p3 + swing_frames
            full_feet[p4:p4 + hold_frames] = curr_foot
            full_angles[p4:p4 + hold_frames] = curr_ang

        angles_all[idx] = full_angles
        feet_all[idx] = full_feet

    return angles_all, feet_all


# ── 2. Foot Path Builders ────────────────────────────────────────────


def buildStridePath(timing, x_center, y_val, reverse=False,
                    z_offset=0, stride=10, lift=80):
    """Build D-shape foot path for forward/backward stride.

    Swing phase: foot lifts from A to D (parabolic or quintic arc).
    Stance phase: foot drags from D back to A on the ground.

    Args:
        timing:   GaitTiming with stance/swing frame counts.
        x_center: nominal X position of the foot (mm).
        y_val:    nominal Y position of the foot (mm).
        reverse:  if True, swap swing direction (backward stride).
        z_offset: vertical compensation (mm).
        stride:   total forward/backward travel distance (mm).
        lift:     swing height (mm).

    Returns:
        np.ndarray of shape (swing + stance, 3) — [x, y, z] trajectory.
    """
    x_forward = x_center + stride / 2
    x_backward = x_center - stride / 2
    z_stance = STANCE_Z + z_offset

    if reverse:
        pos_A, pos_D = [x_forward, y_val, z_stance], [x_backward, y_val, z_stance]
    else:
        pos_A, pos_D = [x_backward, y_val, z_stance], [x_forward, y_val, z_stance]

    if CONTROL_VELOCITY:
        return quintic_planning(
            pos_A, pos_D,
            T_swing=timing.swing,
            T_stance=timing.stance,
            lift_height=lift)

    # Linear fallback
    swing = np.zeros((timing.swing, 3))
    swing[:, 0] = np.linspace(pos_A[0], pos_D[0], timing.swing)
    swing[:, 1] = y_val
    swing[:, 2] = z_stance + lift * np.sin(
        np.linspace(0, np.pi, timing.swing))

    stance = np.zeros((timing.stance, 3))
    stance[:, 0] = np.linspace(pos_D[0], pos_A[0], timing.stance)
    stance[:, 1] = y_val
    stance[:, 2] = z_stance

    return np.vstack([swing, stance])


def buildStrafePath(timing, x_center, y_val, y_stride, z_offset=0,
                    lift=80, direction=-1):
    """Build D-shape foot path for lateral (Y-axis) movement.

    Args:
        timing:    GaitTiming with stance/swing frame counts.
        x_center:  fixed X position (mm).
        y_val:     nominal Y position (mm).
        y_stride:  total lateral travel distance (mm, always positive).
        z_offset:  vertical compensation (mm).
        lift:      swing height (mm).
        direction: -1 for strafe-right, +1 for strafe-left.

    Returns:
        np.ndarray of shape (swing + stance, 3) — [x, y, z] trajectory.
    """
    half = y_stride / 2
    y_A = y_val - direction * half
    y_B = y_val + direction * half
    z_stance = STANCE_Z + z_offset

    pos_A = [x_center, y_A, z_stance]
    pos_B = [x_center, y_B, z_stance]

    if CONTROL_VELOCITY:
        return quintic_planning(
            pos_A, pos_B,
            T_swing=timing.swing,
            T_stance=timing.stance,
            lift_height=lift)

    # Linear fallback
    swing = np.zeros((timing.swing, 3))
    swing[:, 0] = x_center
    swing[:, 1] = np.linspace(y_A, y_B, timing.swing)
    swing[:, 2] = z_stance + lift * np.sin(
        np.linspace(0, np.pi, timing.swing))

    stance = np.zeros((timing.stance, 3))
    stance[:, 0] = x_center
    stance[:, 1] = np.linspace(y_B, y_A, timing.stance)
    stance[:, 2] = z_stance

    return np.vstack([swing, stance])


def buildBodyPath(timing, x_center, y_val, z_low, z_high):
    """Build body posture oscillation trajectory (feet planted).

    Args:
        timing:   GaitTiming with rest frame count.
        x_center: X position of the foot (mm).
        y_val:    Y position of the foot (mm).
        z_low:    lowest Z position (mm).
        z_high:   highest Z position (mm).

    Returns:
        np.ndarray of shape (rest, 3) — [x, y, z] trajectory.
    """
    waypoint = np.zeros((timing.rest, 3))
    waypoint[:, 0] = x_center
    waypoint[:, 1] = y_val
    waypoint[:, 2] = z_low + (z_high - z_low) * (
        0.5 - 0.5 * np.cos(np.linspace(0, 2 * np.pi, timing.rest)))
    return waypoint


# ── 3. Internal Lookups ──────────────────────────────────────────────


def _lookupGaitParams(cmd, leg):
    """Return (leg_params, phase_shift) for the given command and leg.

    Args:
        cmd: gait command string (e.g., 'TROT_FORWARD').
        leg: leg name (e.g., 'left-front').

    Returns:
        (params_dict, phaseshift_dict) or (None, None) if unknown.
    """
    match cmd:
        case "TROT_FORWARD":
            return PARAMS_GAIT_FORWARD.get(leg), PHASESHIFT_TROT
        case "TROT_BACKWARD":
            return PARAMS_GAIT_BACKWARD.get(leg), PHASESHIFT_TROT
        case "WALK_FORWARD":
            return PARAMS_GAIT_FORWARD.get(leg), PHASESHIFT_WALK
        case "WALK_BACKWARD":
            return PARAMS_GAIT_FORWARD.get(leg), PHASESHIFT_WALK
        case "TURN_RIGHT":
            return PARAMS_GAIT_TURN_RIGHT.get(leg), PHASESHIFT_TROT
        case "TURN_LEFT":
            return PARAMS_GAIT_TURN_LEFT.get(leg), PHASESHIFT_TROT
        case "STRAFE_RIGHT":
            return PARAMS_GAIT_STRAFE_RIGHT.get(leg), PHASESHIFT_TROT
        case "STRAFE_LEFT":
            return PARAMS_GAIT_STRAFE_LEFT.get(leg), PHASESHIFT_TROT
        case "BODY_HEAVE":
            return PARAMS_GAIT_BODY.get(leg), PHASESHIFT_HEAVE
        case "BODY_ROLL":
            return PARAMS_GAIT_BODY.get(leg), PHASESHIFT_ROLL
        case "BODY_PITCH":
            return PARAMS_GAIT_BODY.get(leg), PHASESHIFT_PITCH
        case "BODY_CIRCLE":
            return PARAMS_GAIT_BODY.get(leg), PHASESHIFT_CIRCLE
        case _:
            return None, None
