"""
Body Posture Adjustment
=======================
Based on the paper's method: converts PID output (roll/pitch corrections)
into foot-tip position adjustments for each leg.

The key idea from the paper:
1. IMU measures body tilt (roll φ, pitch θ)
2. PID computes correction angles (Δφ, Δθ)
3. Rotation matrix maps corrections to foot height adjustments (Δz) per leg
4. IK converts adjusted foot positions to joint angles

This module includes a standalone IK implementation (re-implemented from
the existing kinematics.py to keep the simulation self-contained).
"""

import math
import numpy as np
from robot_params import RobotParams, get_nominal_foot_positions


# ══════════════════════════════════════════════════════════════════
#  Standalone Inverse Kinematics (from existing kinematics.py)
# ══════════════════════════════════════════════════════════════════

def inverse_kinematics(x, y, z, leg_type, p: RobotParams):
    """
    Inverse kinematics for one leg.
    Input: foot position (x, y, z) in body frame, in meters.
    Output: (theta1, theta2, theta3) in radians.

    Re-implemented from leg_controller/kinematics.py but works in meters
    and returns radians directly (no degree conversion).
    """
    # Convert from body frame to leg-local DH frame
    # (same logic as existing kinematics.py but in meters)
    px, py, pz = x, y, z

    if leg_type == "left-front":
        xl =  pz
        yl =  p.W/2 - py
        zl = -p.L/2 + px
        theta1 = math.atan2(-xl, yl) + math.atan2(
            -math.sqrt(max(0, xl**2 + yl**2 - p.l1**2)), -p.l1)
        theta1 = -theta1
        sign_s3, sign_p2 = -1, 1

    elif leg_type == "left-behind":
        xl =  pz
        yl =  p.W/2 - py
        zl =  p.L/2 + px
        theta1 = math.atan2(-xl, yl) + math.atan2(
            -math.sqrt(max(0, xl**2 + yl**2 - p.l1**2)), -p.l1)
        theta1 = -theta1
        sign_s3, sign_p2 = -1, 1

    elif leg_type == "right-front":
        xl =  pz
        yl = -p.W/2 - py
        zl = -p.L/2 + px
        theta1 = math.atan2(-xl, yl) + math.atan2(
            -math.sqrt(max(0, xl**2 + yl**2 - p.l1**2)), p.l1)
        sign_s3, sign_p2 = 1, -1

    elif leg_type == "right-behind":
        xl =  pz
        yl = -p.W/2 - py
        zl =  p.L/2 + px
        theta1 = math.atan2(-xl, yl) + math.atan2(
            -math.sqrt(max(0, xl**2 + yl**2 - p.l1**2)), p.l1)
        sign_s3, sign_p2 = 1, -1
    else:
        raise ValueError(f"Unknown leg_type: {leg_type}")

    p1 = xl * math.cos(theta1) + yl * math.sin(theta1)
    p2 = sign_p2 * zl
    c3 = (p1**2 + p2**2 - p.l2**2 - p.l3**2) / (2 * p.l2 * p.l3)
    c3 = np.clip(c3, -1.0, 1.0)  # numerical safety
    s3 = sign_s3 * math.sqrt(max(0, 1 - c3**2))
    theta3 = math.atan2(s3, c3)
    theta2 = math.atan2(p2, p1) - math.atan2(
        p.l3 * s3, p.l2 + p.l3 * c3)

    return np.array([theta1, theta2, theta3])


# ══════════════════════════════════════════════════════════════════
#  Body posture adjustment (the paper's core method)
# ══════════════════════════════════════════════════════════════════

class BodyPosture:
    """Computes adjusted foot positions given body orientation corrections."""

    LEG_NAMES = ["left-front", "left-behind", "right-front", "right-behind"]

    def __init__(self, params: RobotParams):
        self.params = params
        self.nominal_feet = get_nominal_foot_positions(params)

        # History
        self.history = {
            'time': [],
            'delta_roll': [],
            'delta_pitch': [],
        }
        for leg in self.LEG_NAMES:
            self.history[f'{leg}_dz'] = []
            self.history[f'{leg}_theta1'] = []
            self.history[f'{leg}_theta2'] = []
            self.history[f'{leg}_theta3'] = []

    def reset(self):
        for key in self.history:
            self.history[key].clear()

    def compute_foot_adjustments(self, delta_roll: float, delta_pitch: float,
                                  t: float) -> dict:
        """
        Given PID-computed roll/pitch corrections, compute adjusted
        foot positions and corresponding joint angles.

        The paper's method:
        - Apply a rotation matrix R(δφ, δθ) to rotate the body
        - Each foot position in the world frame is:
            p_foot_world = R(-δφ, -δθ) · p_foot_body
        - The adjustment Δz for each foot depends on its (x, y) position:
            Δz ≈ x·sin(δθ) - y·sin(δφ)  (small angle approximation)

        Args:
            delta_roll:  PID output for roll correction (rad)
            delta_pitch: PID output for pitch correction (rad)
            t:           current time (s)

        Returns:
            dict with 'joint_angles' (12 values) and 'foot_positions' (4×3)
        """
        self.history['time'].append(t)
        self.history['delta_roll'].append(delta_roll)
        self.history['delta_pitch'].append(delta_pitch)

        # Rotation matrix for body tilt compensation
        # To compensate body tilt, we rotate the foot positions
        # by the NEGATIVE of the correction angles
        cos_r = math.cos(-delta_roll)
        sin_r = math.sin(-delta_roll)
        cos_p = math.cos(-delta_pitch)
        sin_p = math.sin(-delta_pitch)

        # Rotation matrix R = Ry(pitch) · Rx(roll)
        R = np.array([
            [cos_p,          sin_r*sin_p,   cos_r*sin_p],
            [0,              cos_r,        -sin_r       ],
            [-sin_p,         sin_r*cos_p,   cos_r*cos_p ],
        ])

        adjusted_feet = {}
        joint_angles = {}
        all_angles = []

        for leg_name in self.LEG_NAMES:
            # Nominal foot position in body frame
            p_nominal = self.nominal_feet[leg_name].copy()

            # Apply rotation to adjust foot position
            p_adjusted = R @ p_nominal

            adjusted_feet[leg_name] = p_adjusted

            # Compute IK
            angles = inverse_kinematics(
                p_adjusted[0], p_adjusted[1], p_adjusted[2],
                leg_name, self.params)
            joint_angles[leg_name] = angles
            all_angles.extend(angles.tolist())

            # Record
            dz = p_adjusted[2] - p_nominal[2]
            self.history[f'{leg_name}_dz'].append(dz)
            self.history[f'{leg_name}_theta1'].append(angles[0])
            self.history[f'{leg_name}_theta2'].append(angles[1])
            self.history[f'{leg_name}_theta3'].append(angles[2])

        return {
            'joint_angles': all_angles,       # flat list of 12 angles
            'foot_positions': adjusted_feet,   # dict of 4×[x,y,z]
            'joint_angles_dict': joint_angles, # dict of 4×[θ1,θ2,θ3]
        }

    def get_history_arrays(self) -> dict:
        return {k: np.array(v) for k, v in self.history.items()}
