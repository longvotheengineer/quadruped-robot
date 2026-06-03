"""
Robot Physical Parameters
=========================
Extracted from the existing URDF (quadrupedRobot.urdf) and source code.
All dimensions in meters, masses in kg, angles in radians.
"""

import numpy as np
from dataclasses import dataclass, field


@dataclass(frozen=True)
class RobotParams:
    """All physical parameters of the quadruped robot."""

    # ── Body dimensions (from URDF body_link) ─────────────────────
    body_length: float = 0.350       # m  (x-axis)
    body_width:  float = 0.222       # m  (y-axis)
    body_height: float = 0.081       # m  (z-axis)
    body_mass:   float = 1.0         # kg

    # Body inertia (from URDF)
    body_ixx: float = 0.004658
    body_iyy: float = 0.010767
    body_izz: float = 0.014333

    # ── Leg dimensions (from gaitGenerator.py RobotLength) ────────
    L:  float = 0.250   # m – body half-length x2 (distance between front/back hip)
    W:  float = 0.193   # m – body half-width x2 (distance between left/right hip)
    l1: float = 0.045   # m – hip link (coxa)
    l2: float = 0.107   # m – upper leg (femur)
    l3: float = 0.116   # m – lower leg (tibia)

    # Leg link masses (from URDF)
    link1_mass: float = 0.02    # hip link
    link2_mass: float = 0.09    # upper leg
    link3_mass: float = 0.02    # lower leg

    # ── Stance geometry (from gaitGenerator.py) ───────────────────
    stance_z:    float = -0.170   # m – foot z during stance (body frame)
    swing_z:     float = -0.130   # m – foot z at peak swing
    x_center_front: float =  0.125  # m
    x_center_back:  float = -0.125  # m
    y_val_left:     float =  0.135  # m
    y_val_right:    float = -0.135  # m

    # ── Joint PD controller (from controllerSim.py) ──────────────
    joint_Kp:      float = 20.0    # Nm/rad
    joint_Kd:      float = 0.5     # Nm·s/rad
    effort_limit:  float = 29.0    # Nm (clamp)

    # ── Joint dynamics (from URDF) ────────────────────────────────
    joint_damping:  float = 0.5
    joint_friction: float = 0.1

    # ── Servo bandwidth approximation ─────────────────────────────
    # From URDF: 0.2s/60deg for joints 1,2 → ~5.2 rad/s bandwidth
    # 0.1s/60deg for joint 3 → ~10.5 rad/s bandwidth
    servo_bandwidth_12: float = 5.2    # rad/s (joints 1, 2)
    servo_bandwidth_3:  float = 10.5   # rad/s (joint 3)

    # ── Simulation defaults ───────────────────────────────────────
    dt:             float = 0.001   # s – simulation time step (1 kHz, matches Gazebo)
    gravity:        float = 9.81    # m/s²
    nominal_height: float = 0.170   # m – nominal body height above ground


# ── Leg hip positions in body frame ──────────────────────────────

def get_hip_positions(p: RobotParams) -> dict:
    """Returns the 4 hip attachment points in the body frame [x, y, z]."""
    return {
        "left-front":   np.array([ p.L/2,  p.W/2, 0.0]),
        "left-behind":  np.array([-p.L/2,  p.W/2, 0.0]),
        "right-front":  np.array([ p.L/2, -p.W/2, 0.0]),
        "right-behind": np.array([-p.L/2, -p.W/2, 0.0]),
    }


def get_nominal_foot_positions(p: RobotParams) -> dict:
    """Returns the 4 nominal foot-tip positions in the body frame."""
    return {
        "left-front":   np.array([ p.x_center_front, p.y_val_left,  p.stance_z]),
        "left-behind":  np.array([ p.x_center_back,  p.y_val_left,  p.stance_z]),
        "right-front":  np.array([ p.x_center_front, p.y_val_right, p.stance_z]),
        "right-behind": np.array([ p.x_center_back,  p.y_val_right, p.stance_z]),
    }


# ── Total robot mass ─────────────────────────────────────────────

def get_total_mass(p: RobotParams) -> float:
    """Total robot mass: body + 4 legs × 3 links each."""
    leg_mass = p.link1_mass + p.link2_mass + p.link3_mass  # one leg
    return p.body_mass + 4 * leg_mass
