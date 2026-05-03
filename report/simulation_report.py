#!/usr/bin/env python3
"""
============================================================================
  QUADRUPED ROBOT — SIMULATION REPORT FIGURES
  Standalone script for thesis / scientific paper (Gazebo ROS2 context)
  
  Generates 8 publication-quality figures:
    1. Foot-end Trajectory 2D (X-Z)  — sine vs quintic comparison
    2. Foot-end Trajectory 3D        — all 4 legs in body frame
    3. Velocity & Acceleration Profiles
    4. Joint Angle Generation         — 4 legs overlay (radians, Gazebo convention)
    5. PD Controller Torque Estimation (with gravity load)
    6. Trot Gait Timing Diagram
    7. Support Polygon (Foot Polygon)
    8. Workspace Analysis
    
  Dependencies: numpy, matplotlib  (no ROS2 needed)
  All parameters match the actual URDF / gaitGenerator / controllerSim code.
============================================================================
"""

import os
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

# ═══════════════════════════════════════════════════════════════════════════
#  GLOBAL PLOT STYLE — LaTeX-like, publication quality
# ═══════════════════════════════════════════════════════════════════════════
plt.rcParams.update({
    'font.family':       'serif',
    'font.size':          11,
    'axes.titlesize':     13,
    'axes.labelsize':     12,
    'legend.fontsize':    10,
    'xtick.labelsize':    10,
    'ytick.labelsize':    10,
    'figure.dpi':        150,
    'savefig.dpi':       300,
    'savefig.bbox':      'tight',
    'axes.grid':          True,
    'grid.alpha':         0.3,
    'lines.linewidth':    1.5,
})

FIGURE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'figures')
os.makedirs(FIGURE_DIR, exist_ok=True)

# ═══════════════════════════════════════════════════════════════════════════
#  ROBOT PARAMETERS  (from URDF & gaitGenerator.py)
# ═══════════════════════════════════════════════════════════════════════════
# --- Link lengths (mm) ---
L  = 120.0   # body length — distance between front & rear hip
W  = 90.0    # body width  — distance between left & right hip
l1 = 20.0    # coxa  length (mm)  — link 1
l2 = 80.0    # femur length (mm)  — link 2 (URDF: 0.1063m ≈ 106.3mm, but IK uses 80)
l3 = 80.0    # tibia length (mm)  — link 3 (URDF: 0.10403m ≈ 104mm, but IK uses 80)

BODY_LENGTH_FULL = 300.0  # full body box (mm)  — URDF: 0.3m
BODY_WIDTH_FULL  = 180.0  # full body box (mm)  — URDF: 0.18049m

# --- Gait parameters (from gaitGenerator.py) ---
STRIDE_LENGTH = 45.0      # mm
Z_STANCE      = -150.0    # mm (ground level)
Z_SWING       = -110.0    # mm (max lift)
LIFT_HEIGHT   = Z_SWING - Z_STANCE   # 40 mm

T_SWING  = 30    # waypoints in swing phase
T_STANCE = 300   # waypoints in stance phase
T_TOTAL  = T_SWING + T_STANCE  # 330 waypoints per cycle

# --- PD controller (from controllerSim.py) ---
KP = 20.0    # Nm/rad
KD = 0.5     # Nm·s/rad
EFFORT_LIMIT_HIP   = 29.29   # Nm (joint 1 & 2 — URDF)
EFFORT_LIMIT_KNEE  = 19.19   # Nm (joint 3 — URDF)

# --- Physical properties (from URDF) ---
BODY_MASS   = 1.0    # kg
LINK1_MASS  = 0.02   # kg (coxa)
LINK2_MASS  = 0.09   # kg (femur)
LINK3_MASS  = 0.02   # kg (tibia)
GRAVITY     = 9.81   # m/s²

# --- Leg configurations (from gaitGenerator.py PARAMS_GAIT_TROT) ---
LEG_CONFIGS = {
    'LF': {'x_center':  60, 'y_val':  60, 'label': 'Left-Front',   'ik_name': 'left-front'},
    'LB': {'x_center': -60, 'y_val':  60, 'label': 'Left-Behind',  'ik_name': 'left-behind'},
    'RF': {'x_center':  60, 'y_val': -60, 'label': 'Right-Front',  'ik_name': 'right-front'},
    'RB': {'x_center': -60, 'y_val': -60, 'label': 'Right-Behind', 'ik_name': 'right-behind'},
}

# Phase shifts (trot gait — from gaitGenerator.py PARAMS_PHASESHIFT_TROT)
PHASE_SHIFT = {
    'LF': 0.00,
    'RB': 0.00,
    'LB': 0.50,
    'RF': 0.50,
}

COLORS = {'LF': '#e74c3c', 'LB': '#3498db', 'RF': '#2ecc71', 'RB': '#f39c12'}

# ═══════════════════════════════════════════════════════════════════════════
#  STANDALONE KINEMATICS  (copied from kinematics.py — no ROS2 import)
# ═══════════════════════════════════════════════════════════════════════════

def forward_kinematics(theta1_deg, theta2_deg, theta3_deg):
    """FK: joint angles (deg) → foot position (mm) in leg-local frame (px, py, pz)."""
    t1 = math.radians(theta1_deg)
    t2 = math.radians(theta2_deg)
    t3 = math.radians(theta3_deg)

    px =  l1 * math.sin(t1) \
        - l3 * (math.cos(t1)*math.sin(t2)*math.sin(t3) - math.cos(t1)*math.cos(t2)*math.cos(t3)) \
        + l2 * math.cos(t1) * math.cos(t2)
    py =  l2 * math.cos(t2) * math.sin(t1) \
        - l1 * math.cos(t1) \
        - l3 * (math.sin(t1)*math.sin(t2)*math.sin(t3) - math.cos(t2)*math.cos(t3)*math.sin(t1))
    pz =  l3 * math.sin(t2 + t3) + l2 * math.sin(t2)

    return px, py, pz


def inverse_kinematics(px, py, pz, leg_type):
    """IK: foot position (mm) in task space → joint angles (deg).
    Exactly matches kinematics.py logic."""
    if leg_type == 'left-front':
        x_l =  pz;  y_l =  W/2 - py;  z_l = -L/2 + px
        t1 = math.atan2(-x_l, y_l) + math.atan2(-math.sqrt(max(0, x_l**2 + y_l**2 - l1**2)), -l1)
        sign_s3, sign_p2 = -1, 1
    elif leg_type == 'left-behind':
        x_l =  pz;  y_l =  W/2 - py;  z_l =  L/2 + px
        t1 = math.atan2(-x_l, y_l) + math.atan2(-math.sqrt(max(0, x_l**2 + y_l**2 - l1**2)), -l1)
        sign_s3, sign_p2 = 1, 1
    elif leg_type == 'right-front':
        x_l =  pz;  y_l = -W/2 - py;  z_l = -L/2 + px
        t1 = math.atan2(-x_l, y_l) + math.atan2(-math.sqrt(max(0, x_l**2 + y_l**2 - l1**2)),  l1)
        sign_s3, sign_p2 = 1, -1
    elif leg_type == 'right-behind':
        x_l =  pz;  y_l = -W/2 - py;  z_l =  L/2 + px
        t1 = math.atan2(-x_l, y_l) + math.atan2(-math.sqrt(max(0, x_l**2 + y_l**2 - l1**2)),  l1)
        sign_s3, sign_p2 = -1, -1
    else:
        return None

    p1 = x_l * math.cos(t1) + y_l * math.sin(t1)
    p2 = sign_p2 * z_l
    c3 = (p1**2 + p2**2 - l2**2 - l3**2) / (2 * l2 * l3)
    c3 = max(-1.0, min(1.0, c3))
    s3 = sign_s3 * math.sqrt(max(0, 1 - c3**2))
    t3 = math.atan2(s3, c3)
    t2 = math.atan2(p2, p1) - math.atan2(l3*s3, l2 + l3*c3)

    theta1 = round(math.degrees(t1), 2)
    theta2 = round(math.degrees(t2), 2)
    # Normalize theta2 — matches kinematics.py
    if leg_type in ('left-front', 'right-behind'):
        if theta2 < 0:
            theta2 += 360
    elif leg_type in ('left-behind', 'right-front'):
        if theta2 > 0:
            theta2 -= 360
    theta3 = round(math.degrees(t3), 2)
    return theta1, theta2, theta3


def ik_to_gazebo_rad(theta1, theta2, theta3, leg_type):
    """Convert IK output (deg) to Gazebo joint command (rad).
    Matches the conversion in gaitGenerator.py generate_home()."""
    if theta2 > 180:
        theta2 -= 360

    if leg_type in ('left-front',):
        j1 = 0.0  # θ₁ is small, near zero for straight walking
        j2 = np.radians(theta2) + 2 * np.pi
        j3 = np.radians(theta3) + np.pi
    elif leg_type in ('left-behind',):
        j1 = 0.0
        j2 = -np.radians(theta2) - 2 * np.pi
        j3 = -np.radians(theta3) - np.pi
    elif leg_type in ('right-front',):
        j1 = 0.0
        j2 = -np.radians(theta2) - 2 * np.pi
        j3 = -np.radians(theta3) - np.pi
    elif leg_type in ('right-behind',):
        j1 = 0.0
        j2 = np.radians(theta2) + 2 * np.pi
        j3 = np.radians(theta3) + np.pi
    else:
        return None
    return j1, j2, j3


# ═══════════════════════════════════════════════════════════════════════════
#  STANDALONE TRAJECTORY PLANNING  (from quinticPlanning.py & gaitGenerator.py)
# ═══════════════════════════════════════════════════════════════════════════

def quintic_planning(pos_start, pos_end, T_sw=T_SWING, T_st=T_STANCE,
                     lift_h=LIFT_HEIGHT):
    """Quintic polynomial trajectory (swing + stance). From quinticPlanning.py."""
    T_total = T_sw + T_st
    wp = np.zeros((T_total, 3))

    xs, ys, zs = pos_start
    xe, ye, ze = pos_end
    zg = min(zs, ze)

    for i in range(T_sw):
        tau = i / max(T_sw - 1, 1)
        s   = 10*tau**3 - 15*tau**4 + 6*tau**5
        wp[i] = [xs + s*(xe-xs), ys + s*(ye-ys), zg + lift_h*4*s*(1-s)]

    for i in range(T_st):
        tau = i / max(T_st - 1, 1)
        s   = 10*tau**3 - 15*tau**4 + 6*tau**5
        wp[T_sw + i] = [xe + s*(xs-xe), ye + s*(ys-ye), zg]

    return wp


def sine_trajectory(x_center, y_val):
    """Sine-wave foot trajectory (CONTROL_VELOCITY=False). From gaitGenerator.py."""
    x_fwd = x_center + STRIDE_LENGTH / 2
    x_bwd = x_center - STRIDE_LENGTH / 2

    swing = np.zeros((T_SWING, 3))
    swing[:, 0] = np.linspace(x_bwd, x_fwd, T_SWING)
    swing[:, 1] = y_val
    swing[:, 2] = Z_STANCE + LIFT_HEIGHT * np.sin(np.linspace(0, np.pi, T_SWING))

    stance = np.zeros((T_STANCE, 3))
    stance[:, 0] = np.linspace(x_fwd, x_bwd, T_STANCE)
    stance[:, 1] = y_val
    stance[:, 2] = Z_STANCE

    return np.vstack([swing, stance])


def quintic_trajectory(x_center, y_val):
    """Quintic polynomial trajectory (CONTROL_VELOCITY=True). From gaitGenerator.py."""
    x_fwd = x_center + STRIDE_LENGTH / 2
    x_bwd = x_center - STRIDE_LENGTH / 2
    pos_A = [x_bwd, y_val, Z_STANCE]
    pos_D = [x_fwd, y_val, Z_STANCE]
    return quintic_planning(pos_A, pos_D, T_SWING, T_STANCE, LIFT_HEIGHT)


def compute_gait_angles(key, cfg):
    """Compute IK joint angles for one leg over a full gait cycle. Returns (N,3) in degrees."""
    wp = quintic_trajectory(cfg['x_center'], cfg['y_val'])
    shift = round(wp.shape[0] * PHASE_SHIFT[key])
    wp = np.roll(wp, shift, axis=0)

    angles = np.zeros((wp.shape[0], 3))
    for i in range(wp.shape[0]):
        ik = inverse_kinematics(wp[i, 0], wp[i, 1], wp[i, 2], cfg['ik_name'])
        if ik is not None:
            angles[i] = ik
    return wp, angles


# ═══════════════════════════════════════════════════════════════════════════
#  FIGURE 1 — Foot-end Trajectory 2D (X-Z plane)
# ═══════════════════════════════════════════════════════════════════════════

def fig1_foot_trajectory_2d():
    """Compare sine-wave vs quintic polynomial foot trajectory in sagittal plane."""
    xc, yv = 60, 60
    wp_sine  = sine_trajectory(xc, yv)
    wp_quint = quintic_trajectory(xc, yv)

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5), sharey=True)

    for idx, (wp, title) in enumerate([
        (wp_sine,  '(a)  Sine-wave Trajectory'),
        (wp_quint, '(b)  Quintic Polynomial Trajectory'),
    ]):
        ax = axes[idx]
        ax.plot(wp[:T_SWING, 0], wp[:T_SWING, 2], 'b-', lw=2, label='Swing phase')
        ax.plot(wp[T_SWING:, 0], wp[T_SWING:, 2], 'r--', lw=2, label='Stance phase')
        ax.set_xlabel('X position (mm)')
        ax.set_title(title)
        ax.legend(loc='upper left')
        ax.set_aspect('equal', adjustable='datalim')

        ax.annotate('A', xy=(wp[0, 0], wp[0, 2]), fontsize=12,
                    fontweight='bold', xytext=(-14, 8), textcoords='offset points')
        ax.annotate('D', xy=(wp[T_SWING-1, 0], wp[T_SWING-1, 2]),
                    fontsize=12, fontweight='bold', xytext=(5, 8), textcoords='offset points')
        # Start marker
        ax.plot(wp[0, 0], wp[0, 2], 'go', ms=8, zorder=5)

    axes[0].set_ylabel('Z position (mm)')

    # fig.suptitle('Figure 1 -- Foot-end Trajectory in Sagittal Plane (X-Z)',
    #              fontsize=14, fontweight='bold', y=1.02)
    fig.tight_layout()
    fig.savefig(os.path.join(FIGURE_DIR, 'fig1_foot_trajectory_2d.png'))
    plt.close(fig)
    print('  [OK] Figure 1 saved')


# ═══════════════════════════════════════════════════════════════════════════
#  FIGURE 2 — Foot-end Trajectory 3D (all 4 legs)
# ═══════════════════════════════════════════════════════════════════════════

def fig2_foot_trajectory_3d():
    """3D trajectories of all 4 feet in body frame."""
    fig = plt.figure(figsize=(9, 7))
    ax  = fig.add_subplot(111, projection='3d')

    for key, cfg in LEG_CONFIGS.items():
        wp = quintic_trajectory(cfg['x_center'], cfg['y_val'])
        phase = PHASE_SHIFT[key]
        shift = round(wp.shape[0] * phase)
        wp = np.roll(wp, shift, axis=0)

        ax.plot(wp[:, 0], wp[:, 1], wp[:, 2],
                color=COLORS[key], lw=1.8, label=cfg['label'])
        ax.scatter(wp[0, 0], wp[0, 1], wp[0, 2],
                   color=COLORS[key], s=40, zorder=5, marker='o')

    # Draw body rectangle
    bx, by = BODY_LENGTH_FULL / 2, BODY_WIDTH_FULL / 2
    body_corners = np.array([
        [ bx,  by, 0], [ bx, -by, 0],
        [-bx, -by, 0], [-bx,  by, 0], [ bx,  by, 0]
    ])
    ax.plot(body_corners[:, 0], body_corners[:, 1], body_corners[:, 2],
            'k-', lw=2, alpha=0.6, label='Body frame')

    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_zlabel('Z (mm)')
    # ax.set_title('Figure 2 -- 3D Foot Trajectories of All Four Legs',
    #              fontsize=13, fontweight='bold', pad=15)
    ax.legend(loc='upper left', fontsize=9)
    ax.view_init(elev=25, azim=-55)
    fig.tight_layout()
    fig.savefig(os.path.join(FIGURE_DIR, 'fig2_foot_trajectory_3d.png'))
    plt.close(fig)
    print('  [OK] Figure 2 saved')


# ═══════════════════════════════════════════════════════════════════════════
#  FIGURE 3 — Velocity & Acceleration Profiles
# ═══════════════════════════════════════════════════════════════════════════

def fig3_velocity_acceleration():
    """Velocity and acceleration of foot-end vs normalised time."""
    wp_sine  = sine_trajectory(60, 60)
    wp_quint = quintic_trajectory(60, 60)
    N = wp_sine.shape[0]
    dt = 1.0  # normalised time step

    def compute_vel_acc(wp):
        vx = np.gradient(wp[:, 0], dt)
        vz = np.gradient(wp[:, 2], dt)
        ax_ = np.gradient(vx, dt)
        az_ = np.gradient(vz, dt)
        return vx, vz, ax_, az_

    vx_s, vz_s, ax_s, az_s = compute_vel_acc(wp_sine)
    vx_q, vz_q, ax_q, az_q = compute_vel_acc(wp_quint)
    t = np.arange(N)

    fig, axes = plt.subplots(2, 2, figsize=(13, 8), sharex=True)
    subtitles = ['(a) X-axis Velocity', '(b) Z-axis Velocity',
                 '(c) X-axis Acceleration', '(d) Z-axis Acceleration']
    y_labels = ['dx/dt  (mm/step)', 'dz/dt  (mm/step)',
                'd$^2$x/dt$^2$  (mm/step$^2$)', 'd$^2$z/dt$^2$  (mm/step$^2$)']
    data_pairs = [(vx_s, vx_q), (vz_s, vz_q), (ax_s, ax_q), (az_s, az_q)]

    for idx, (ax, title, ylabel, (d_sine, d_quint)) in enumerate(
        zip(axes.flat, subtitles, y_labels, data_pairs)):
        ax.plot(t, d_sine,  'b-', alpha=0.7, label='Sine')
        ax.plot(t, d_quint, 'r-', alpha=0.9, label='Quintic')
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.legend()
        ax.axvline(T_SWING, color='gray', ls=':', lw=1, alpha=0.5)
        if idx >= 2:
            ax.set_xlabel('Waypoint index')

    # Annotate swing/stance regions
    axes[0, 0].text(T_SWING/2, axes[0, 0].get_ylim()[1]*0.85, 'Swing',
                    ha='center', fontsize=9, color='gray')
    axes[0, 0].text(T_SWING + T_STANCE/2, axes[0, 0].get_ylim()[1]*0.85,
                    'Stance', ha='center', fontsize=9, color='gray')

    # fig.suptitle('Figure 3 -- Foot-end Velocity and Acceleration Profiles\\n'
    #              '(Sine-wave  vs  Quintic Polynomial)',
    #              fontsize=14, fontweight='bold', y=1.02)
    fig.tight_layout()
    fig.savefig(os.path.join(FIGURE_DIR, 'fig3_velocity_acceleration.png'))
    plt.close(fig)
    print('  [OK] Figure 3 saved')


# ═══════════════════════════════════════════════════════════════════════════
#  FIGURE 4 — Joint Angle Generation (IK output in degrees)
# ═══════════════════════════════════════════════════════════════════════════

def fig4_joint_angles():
    """Joint angles from IK in degrees over one gait cycle for all four legs.
    Shows the raw inverse kinematics output — theta1, theta2, theta3."""

    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    joint_labels = [
        r'$\theta_1$ -- Hip (deg)',
        r'$\theta_2$ -- Femur (deg)',
        r'$\theta_3$ -- Tibia (deg)',
    ]

    for key, cfg in LEG_CONFIGS.items():
        wp, angles_deg = compute_gait_angles(key, cfg)
        N = wp.shape[0]
        t = np.arange(N)
        for j in range(3):
            axes[j].plot(t, angles_deg[:, j], color=COLORS[key], lw=1.4,
                         label=cfg['label'], alpha=0.85)

    for j in range(3):
        axes[j].set_ylabel(joint_labels[j])
        axes[j].legend(loc='upper right', ncol=2, fontsize=9)
        axes[j].axvline(T_SWING, color='gray', ls=':', lw=1, alpha=0.4)
        # Mark swing/stance transitions for phase-shifted legs
        axes[j].axvline(T_TOTAL * 0.5, color='gray', ls=':', lw=1, alpha=0.3)
        axes[j].axvline(T_TOTAL * 0.5 + T_SWING, color='gray', ls=':', lw=1, alpha=0.3)

    axes[2].set_xlabel('Waypoint index')
    # fig.suptitle('Figure 4 -- Joint Angle Profiles over One Gait Cycle\\n'
    #              '(Quintic Polynomial Trajectory, Trot Gait -- IK Output)',
    #              fontsize=14, fontweight='bold', y=1.01)
    fig.tight_layout()
    fig.savefig(os.path.join(FIGURE_DIR, 'fig4_joint_angles.png'))
    plt.close(fig)
    print('  [OK] Figure 4 saved')


# ═══════════════════════════════════════════════════════════════════════════
#  FIGURE 5 — PD Controller Torque Estimation
# ═══════════════════════════════════════════════════════════════════════════

def fig5_pd_torque():
    """Estimated PD torque for Left-Front leg.
    Uses direct PD torque = Kp*(q_desired - q_actual) - Kd*dq,
    modeling a realistic lag between actual and target joint trajectories."""
    key = 'LF'
    cfg = LEG_CONFIGS[key]
    _, angles_deg = compute_gait_angles(key, cfg)
    N = angles_deg.shape[0]

    # Convert IK to radians for PD computation
    target_rad = np.radians(angles_deg)

    # Model actual joint position as a low-pass-filtered version of target
    # (simulates servo tracking delay — typical lag ~5-15 waypoints)
    LAG_STEPS = 8
    kernel = np.ones(LAG_STEPS) / LAG_STEPS
    actual_rad = np.zeros_like(target_rad)
    for j in range(3):
        actual_rad[:, j] = np.convolve(target_rad[:, j], kernel, mode='same')

    # Compute velocity as derivative of actual position
    actual_vel = np.gradient(actual_rad, axis=0)

    # PD torque:  tau = Kp * (target - actual) - Kd * vel_actual
    torques = KP * (target_rad - actual_rad) - KD * actual_vel

    # Add gravity torque for joints 2 & 3
    leg_weight = (LINK2_MASS + LINK3_MASS) * GRAVITY  # N
    for i in range(N):
        torques[i, 1] += leg_weight * (l2/2*0.001) * math.cos(actual_rad[i, 1])
        torques[i, 2] += LINK3_MASS * GRAVITY * (l3/2*0.001) * math.cos(actual_rad[i, 2])

    # Clamp to servo limits
    limits_arr = np.array([EFFORT_LIMIT_HIP, EFFORT_LIMIT_HIP, EFFORT_LIMIT_KNEE])
    torques = np.clip(torques, -limits_arr, limits_arr)

    t = np.arange(N)
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    joint_titles = ['Joint 1 -- Hip (Coxa)', 'Joint 2 -- Femur', 'Joint 3 -- Tibia']
    limits = [EFFORT_LIMIT_HIP, EFFORT_LIMIT_HIP, EFFORT_LIMIT_KNEE]

    for j in range(3):
        axes[j].plot(t, torques[:, j], 'b-', lw=1.5, label='PD + Gravity Torque')
        axes[j].axhline( limits[j], color='r', ls='--', lw=1, alpha=0.7,
                         label=f'Limit: +/-{limits[j]:.1f} Nm')
        axes[j].axhline(-limits[j], color='r', ls='--', lw=1, alpha=0.7)
        axes[j].fill_between(t, -limits[j], limits[j],
                              color='green', alpha=0.05, label='Safe zone')
        axes[j].set_ylabel('Torque (Nm)')
        axes[j].set_title(joint_titles[j])
        axes[j].legend(loc='upper right', fontsize=9)
        axes[j].axvline(T_SWING, color='gray', ls=':', lw=1, alpha=0.4)

    axes[2].set_xlabel('Waypoint index')
    # fig.suptitle('Figure 5 -- Estimated PD Controller Torque (Left-Front Leg)\\n'
    #              f'Kp = {KP},  Kd = {KD},  with gravity compensation',
    #              fontsize=14, fontweight='bold', y=1.01)
    fig.tight_layout()
    fig.savefig(os.path.join(FIGURE_DIR, 'fig5_pd_torque.png'))
    plt.close(fig)
    print('  [OK] Figure 5 saved')


# ═══════════════════════════════════════════════════════════════════════════
#  FIGURE 6 — Trot Gait Timing Diagram
# ═══════════════════════════════════════════════════════════════════════════

def fig6_gait_timing():
    """Phase/timing diagram for trot gait: swing vs stance per leg."""
    legs   = ['LF', 'RB', 'LB', 'RF']
    labels = ['Left-Front', 'Right-Behind', 'Left-Behind', 'Right-Front']

    fig, ax = plt.subplots(figsize=(12, 4))
    bar_height = 0.6
    n_cycles = 2

    color_swing  = '#3498db'
    color_stance = '#ecf0f1'

    for idx, leg in enumerate(legs):
        phase = PHASE_SHIFT[leg]
        shift = round(T_TOTAL * phase)
        y_pos = len(legs) - 1 - idx

        for cycle in range(n_cycles):
            offset = cycle * T_TOTAL
            swing_start  = offset + shift
            stance_start = swing_start + T_SWING
            ax.barh(y_pos, T_SWING, left=swing_start, height=bar_height,
                    facecolor=color_swing, edgecolor='#2c3e50', linewidth=0.8)
            ax.barh(y_pos, T_STANCE, left=stance_start, height=bar_height,
                    facecolor=color_stance, edgecolor='#2c3e50', linewidth=0.8)

    ax.set_yticks(range(len(legs)))
    ax.set_yticklabels(labels[::-1])
    ax.set_xlabel('Waypoint index')
    ax.set_xlim(0, n_cycles * T_TOTAL)
    # ax.set_title('Figure 6 -- Trot Gait Timing Diagram',
    #              fontsize=14, fontweight='bold')

    swing_patch  = mpatches.Patch(facecolor=color_swing,
                                   edgecolor='#2c3e50', label='Swing phase')
    stance_patch = mpatches.Patch(facecolor=color_stance,
                                   edgecolor='#2c3e50', label='Stance phase')
    ax.legend(handles=[swing_patch, stance_patch], loc='upper right')

    for c in range(n_cycles + 1):
        ax.axvline(c * T_TOTAL, color='gray', ls='--', lw=0.8, alpha=0.5)

    fig.tight_layout()
    fig.savefig(os.path.join(FIGURE_DIR, 'fig6_gait_timing.png'))
    plt.close(fig)
    print('  [OK] Figure 6 saved')


# ═══════════════════════════════════════════════════════════════════════════
#  FIGURE 7 — Support Polygon (Foot Polygon)
# ═══════════════════════════════════════════════════════════════════════════

def fig7_support_polygon():
    """Show support polygons at different phases of trot gait.
    Correctly identifies swing legs based on foot Z-coordinate and phase shift."""

    # Compute foot positions for all 4 legs over one cycle
    foot_pos = {}
    for key, cfg in LEG_CONFIGS.items():
        wp = quintic_trajectory(cfg['x_center'], cfg['y_val'])
        shift = round(wp.shape[0] * PHASE_SHIFT[key])
        foot_pos[key] = np.roll(wp, shift, axis=0)

    # Select 4 representative time instants
    # t=5: LF,RB mid-swing;  t=T_SWING: LF,RB just landed;
    # t=T_SWING+50: all stance;  t=T_SWING+T_STANCE//2: LB,RF mid-swing
    instants = [
        T_SWING // 2,
        T_SWING,
        T_SWING + 50,
        T_SWING + T_STANCE // 2,
    ]
    titles = [
        f't = {instants[0]}\n(LF,RB mid-swing)',
        f't = {instants[1]}\n(LF,RB touchdown)',
        f't = {instants[2]}\n(All stance)',
        f't = {instants[3]}\n(LB,RF mid-swing)',
    ]

    fig, axes = plt.subplots(1, 4, figsize=(16, 4.5))

    for ax_idx, (ti, title) in enumerate(zip(instants, titles)):
        ax = axes[ax_idx]
        # Draw body
        bx, by = BODY_LENGTH_FULL/2, BODY_WIDTH_FULL/2
        body_rect = plt.Rectangle((-bx, -by), BODY_LENGTH_FULL, BODY_WIDTH_FULL,
                                   fill=True, facecolor='#bdc3c7', edgecolor='k',
                                   alpha=0.3, lw=1.5)
        ax.add_patch(body_rect)

        # Check each foot
        stance_feet = []
        for key in ['LF', 'LB', 'RF', 'RB']:
            fp = foot_pos[key][ti % T_TOTAL]
            is_swing = abs(fp[2] - Z_STANCE) > 1.0  # lifted > 1mm = swing
            marker = 'o' if not is_swing else 'X'
            ax.plot(fp[0], fp[1], marker, color=COLORS[key], ms=12, mew=2.5, zorder=5)
            ax.annotate(key, (fp[0], fp[1]), fontsize=8, fontweight='bold',
                        ha='center', va='bottom', xytext=(0, 8),
                        textcoords='offset points')
            if not is_swing:
                stance_feet.append([fp[0], fp[1]])

        # Draw support polygon
        if len(stance_feet) >= 3:
            poly = np.array(stance_feet)
            cx, cy = poly.mean(axis=0)
            angles = np.arctan2(poly[:, 1]-cy, poly[:, 0]-cx)
            order = np.argsort(angles)
            poly = poly[order]
            polygon = plt.Polygon(poly, closed=True, fill=True,
                                   facecolor='#27ae60', alpha=0.2,
                                   edgecolor='#27ae60', lw=2)
            ax.add_patch(polygon)
        elif len(stance_feet) == 2:
            pts = np.array(stance_feet)
            ax.plot(pts[:, 0], pts[:, 1], '-', color='#27ae60', lw=2, alpha=0.6)

        # CoM marker
        ax.plot(0, 0, 'k+', ms=14, mew=2.5, zorder=6)
        ax.annotate('CoM', (0, 0), fontsize=8, ha='center', va='top',
                    xytext=(0, -10), textcoords='offset points')

        ax.set_xlim(-200, 200)
        ax.set_ylim(-180, 180)
        ax.set_aspect('equal')
        ax.set_title(title, fontsize=10)
        ax.set_xlabel('X (mm)')
        if ax_idx == 0:
            ax.set_ylabel('Y (mm)')

    # fig.suptitle('Figure 7 -- Support Polygon at Different Trot Gait Phases\\n'
    #              '(o = stance,   X = swing,   shaded = support polygon)',
    #              fontsize=13, fontweight='bold', y=1.06)
    fig.tight_layout()
    fig.savefig(os.path.join(FIGURE_DIR, 'fig7_support_polygon.png'))
    plt.close(fig)
    print('  [OK] Figure 7 saved')


# ═══════════════════════════════════════════════════════════════════════════
#  FIGURE 8 — Workspace Analysis
# ═══════════════════════════════════════════════════════════════════════════

def fig8_workspace():
    """Reachable workspace of a single leg using vectorized FK sweep (NumPy)."""
    # Sweep joint angles within mechanical limits (from URDF)
    t1_range = np.radians(np.linspace(-90, 90, 30))
    t2_range = np.radians(np.linspace(-180, 180, 60))
    t3_range = np.radians(np.linspace(-120, 120, 50))

    # Vectorized FK using meshgrid — ~100x faster than Python loops
    T1, T2, T3 = np.meshgrid(t1_range, t2_range, t3_range, indexing='ij')
    T1f, T2f, T3f = T1.ravel(), T2.ravel(), T3.ravel()

    ct1, st1 = np.cos(T1f), np.sin(T1f)
    ct2, st2 = np.cos(T2f), np.sin(T2f)
    ct3, st3 = np.cos(T3f), np.sin(T3f)

    px = l1*st1 - l3*(ct1*st2*st3 - ct1*ct2*ct3) + l2*ct1*ct2
    py = l2*ct2*st1 - l1*ct1 - l3*(st1*st2*st3 - ct2*ct3*st1)
    pz = l3*np.sin(T2f + T3f) + l2*st2

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))

    # pz-px (sagittal projection of leg-local frame)
    ax = axes[0]
    ax.scatter(pz, px, s=0.1, alpha=0.12, c='#3498db', rasterized=True)
    ax.set_xlabel('pz (mm) -- along leg')
    ax.set_ylabel('px (mm) -- lateral')
    ax.set_title('(a) Sagittal Plane (pz-px)')
    ax.set_aspect('equal', adjustable='datalim')

    # Overlay gait trajectory
    wp = quintic_trajectory(60, 60)
    ax.plot(wp[:, 2], wp[:, 0], 'r-', lw=2, label='Gait trajectory')
    ax.legend(fontsize=9)

    # px-py
    ax = axes[1]
    ax.scatter(px, py, s=0.1, alpha=0.12, c='#e74c3c', rasterized=True)
    ax.set_xlabel('px (mm)')
    ax.set_ylabel('py (mm)')
    ax.set_title('(b) Horizontal Plane (px-py)')
    ax.set_aspect('equal', adjustable='datalim')

    # py-pz
    ax = axes[2]
    ax.scatter(py, pz, s=0.1, alpha=0.12, c='#2ecc71', rasterized=True)
    ax.set_xlabel('py (mm)')
    ax.set_ylabel('pz (mm)')
    ax.set_title('(c) Frontal Plane (py-pz)')
    ax.set_aspect('equal', adjustable='datalim')

    # fig.suptitle('Figure 8 -- Workspace Analysis of a Single Leg (Left-Front)\\n'
    #              f'Link lengths:  l1 = {l1} mm,  l2 = {l2} mm,  l3 = {l3} mm',
    #              fontsize=14, fontweight='bold', y=1.04)
    fig.tight_layout()
    fig.savefig(os.path.join(FIGURE_DIR, 'fig8_workspace.png'))
    plt.close(fig)
    print('  [OK] Figure 8 saved')


# ═══════════════════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════════════════

def main():
    print('=' * 60)
    print('  QUADRUPED ROBOT -- SIMULATION REPORT FIGURES')
    print(f'  Output: {FIGURE_DIR}')
    print('=' * 60)

    fig1_foot_trajectory_2d()
    fig2_foot_trajectory_3d()
    fig3_velocity_acceleration()
    fig4_joint_angles()
    fig5_pd_torque()
    fig6_gait_timing()
    fig7_support_polygon()
    fig8_workspace()

    print('=' * 60)
    print(f'  All 8 figures saved to:  {FIGURE_DIR}/')
    print('=' * 60)


if __name__ == '__main__':
    main()
