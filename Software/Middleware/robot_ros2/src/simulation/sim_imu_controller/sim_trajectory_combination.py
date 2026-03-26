"""
Trajectory Combination Simulation (v2 — CORRECTED)
====================================================
Fixes two issues from v1:

FIX 1 — D-shape swing:
  v1 used  z = lift * 4*s*(1-s)  where s is quintic in τ
  This clusters points at s=0,1 making swing look like line segments.
  v2 uses  z = lift * sin(π*τ)   with linearly-spaced x
  This matches gaitGenerator.py (CONTROL_VELOCITY=False) exactly.

FIX 2 — Paper's rotation formula:
  The paper uses:  p_t = R_inv · (p_c − p_0) + p_0
  NOT simply:      p_t = R · p_c
  The difference:
    - p_c  = current foot position (from offline trajectory)
    - p_0  = reference point (body center in body frame = [0, 0, 0])
    - R_inv = inverse rotation = R(-Δφ, -Δθ)
    - The subtraction/addition of p_0 performs rotation ABOUT p_0
  When p_0 = [0,0,0] (body center), both formulas are equivalent.
  But when foot positions include an offset from body center, p_0
  must be the body center (or CoG), and p_c is the foot position
  relative to that center. In our coordinate system, p_0 = [0,0,0]
  already (foot positions ARE relative to body center), so the
  formulas are mathematically identical.

  HOWEVER, the paper uses R_INVERSE (= R(-Δφ,-Δθ)), not R(Δφ,Δθ).
  This is correct because: if the body has tilted by +φ (IMU reading),
  the PID outputs a correction to bring it back. To compensate, we
  rotate foot positions by -φ (the inverse direction).
"""

import os, sys, math
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.gridspec import GridSpec
from matplotlib.patches import FancyBboxPatch

from robot_params import RobotParams, get_nominal_foot_positions
from pid_controller import PIDController, PIDGains
from imu_simulator import IMUSimulator
from body_posture import BodyPosture, inverse_kinematics
from plant_model import PlantModel
from disturbance import Disturbance

# ── Style ─────────────────────────────────────────────────────────
plt.rcParams.update({
    'figure.facecolor': '#1a1a2e',
    'axes.facecolor':   '#16213e',
    'axes.edgecolor':   '#e94560',
    'axes.labelcolor':  '#eaeaea',
    'text.color':       '#eaeaea',
    'xtick.color':      '#aaaaaa',
    'ytick.color':      '#aaaaaa',
    'grid.color':       '#2a2a4a',
    'grid.alpha':       0.5,
    'legend.facecolor': '#1a1a2e',
    'legend.edgecolor': '#e94560',
    'font.size':        10,
})

COLORS = {
    'offline':  '#ffa502', 'adjusted': '#54a0ff', 'pid_corr': '#2ed573',
    'final':    '#ff6b6b', 'dist':     '#e94560', 'body':     '#a29bfe',
    'setpt':    '#ffffff',
}

OUTPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")


# ══════════════════════════════════════════════════════════════════
#  CORRECTED: D-shape gait trajectory generator
# ══════════════════════════════════════════════════════════════════

def generate_trot_trajectory(params, n_stance=300, n_swing=150):
    """
    Generates the OFFLINE trot gait trajectory for all 4 legs.
    Matches gaitGenerator.py trajectory_foot() exactly.

    D-shape = swing uses sine-wave lift with linear X spacing,
    stance slides on ground.
    """
    stride = 0.045       # 45mm stride
    z_stance = -0.170    # stance Z
    z_swing_h = -0.130   # swing peak Z
    lift = z_swing_h - z_stance  # 0.040m = 40mm

    legs = {
        'left-front':   {'x_c':  0.125, 'y': 0.135},
        'left-behind':  {'x_c': -0.125, 'y': 0.135},
        'right-front':  {'x_c':  0.125, 'y':-0.135},
        'right-behind': {'x_c': -0.125, 'y':-0.135},
    }

    phases = {
        'left-front': 0.00, 'right-behind': 0.00,
        'left-behind': 0.50, 'right-front': 0.50,
    }

    T = n_swing + n_stance
    trajectories = {}

    for leg_name, cfg in legs.items():
        x_c = cfg['x_c']
        y   = cfg['y']
        x_fwd  = x_c + stride / 2
        x_back = x_c - stride / 2

        # ── SWING PHASE ──────────────────────────────────────────
        # CORRECTED: use sin(π·τ) with linearly-spaced x
        # This matches gaitGenerator.py trajectory_foot()
        # (CONTROL_VELOCITY=False branch)
        swing = np.zeros((n_swing, 3))
        swing[:, 0] = np.linspace(x_back, x_fwd, n_swing)  # linear x
        swing[:, 1] = y
        swing[:, 2] = z_stance + lift * np.sin(
            np.linspace(0, np.pi, n_swing))  # sine arch

        # ── STANCE PHASE ─────────────────────────────────────────
        stance = np.zeros((n_stance, 3))
        stance[:, 0] = np.linspace(x_fwd, x_back, n_stance)
        stance[:, 1] = y
        stance[:, 2] = z_stance

        full_cycle = np.vstack([swing, stance])

        # Apply trot phase shift
        shift = round(T * phases[leg_name])
        full_cycle = np.roll(full_cycle, shift, axis=0)

        trajectories[leg_name] = full_cycle

    return trajectories, T


# ══════════════════════════════════════════════════════════════════
#  CORRECTED: Rotation formula matching the paper exactly
# ══════════════════════════════════════════════════════════════════

def apply_paper_rotation(p_c, delta_roll, delta_pitch, p_0=None):
    """
    Apply the paper's foot position adjustment formula:

        p_B_t = R_inv · (p_B_c − p_B_0) + p_B_0

    Where:
        p_B_c   = current foot position (from offline trajectory)
        p_B_0   = reference point (body center / CoG)
        R_inv   = R(-Δφ, -Δθ) = inverse of R(Δφ, Δθ)
        p_B_t   = target/adjusted foot position

    If p_0 = [0,0,0] (body center), this simplifies to:
        p_B_t = R_inv · p_B_c
    which is what v1 used. So v1 was mathematically correct for our
    coordinate system, but didn't show the full formula.

    Args:
        p_c:          foot position vector [x, y, z]
        delta_roll:   PID correction for roll (rad)
        delta_pitch:  PID correction for pitch (rad)
        p_0:          reference point [x, y, z] (default: body center = [0,0,0])

    Returns:
        p_t: adjusted foot position vector [x, y, z]
    """
    if p_0 is None:
        p_0 = np.array([0.0, 0.0, 0.0])  # body center

    # R_inverse = R(-delta_roll, -delta_pitch)
    # This is the INVERSE rotation: if body tilted by +φ,
    # we rotate feet by -φ to compensate
    r = -delta_roll
    p = -delta_pitch
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)

    # R = Ry(pitch) · Rx(roll)
    R_inv = np.array([
        [cp,      sr*sp,   cr*sp],
        [0,       cr,     -sr   ],
        [-sp,     sr*cp,   cr*cp],
    ])

    # Paper formula: p_t = R_inv · (p_c - p_0) + p_0
    p_t = R_inv @ (p_c - p_0) + p_0

    return p_t


# ══════════════════════════════════════════════════════════════════
#  Combined simulation (corrected)
# ══════════════════════════════════════════════════════════════════

def run_combined_simulation(enable_pid=True, T_seconds=3.0, dt=0.003):
    """Run the full system with corrected D-shape and paper formula.

    Uses a SIMPLE first-order plant model:
        body_angle = LPF( disturbance - correction )
    This cleanly shows how PID reduces body tilt without oscillation.
    """
    params = RobotParams(dt=dt)
    n_stance, n_swing = 300, 150
    T_cycle = n_stance + n_swing

    traj_offline, _ = generate_trot_trajectory(params, n_stance, n_swing)

    gains = PIDGains(Kp=3.0, Ki=0.5, Kd=0.2)
    pid_roll  = PIDController(gains, dt, 0.087, 0.03, "Roll")   # sat=±5°, windup=0.03
    pid_pitch = PIDController(gains, dt, 0.087, 0.03, "Pitch")
    imu = IMUSimulator(noise_std_roll=0.001, noise_std_pitch=0.001, dt=dt)
    dist = Disturbance(dist_type="sinusoidal", roll_amplitude=np.radians(3),
                       pitch_amplitude=np.radians(2), roll_frequency=0.5,
                       pitch_frequency=0.3)

    leg_names = ['left-front', 'left-behind', 'right-front', 'right-behind']
    n_steps = int(T_seconds / dt)
    p_0 = np.array([0.0, 0.0, 0.0])

    # ── Simple first-order plant state ────────────────────────────
    body_roll  = 0.0
    body_pitch = 0.0
    tau_plant = 0.05  # 50ms time constant (servo response speed)
    alpha_plant = dt / (tau_plant + dt)  # LPF coefficient

    history = {
        'time': [], 'gait_frame': [],
        'body_roll': [], 'body_pitch': [],
        'pid_roll': [], 'pid_pitch': [],
        'dist_roll': [], 'dist_pitch': [],
    }
    for leg in leg_names:
        for suffix in ['_offline_x', '_offline_y', '_offline_z',
                       '_adjusted_x', '_adjusted_y', '_adjusted_z',
                       '_offline_th1', '_offline_th2', '_offline_th3',
                       '_final_th1', '_final_th2', '_final_th3']:
            history[f'{leg}{suffix}'] = []

    gait_frame = 0

    for step in range(n_steps):
        t = step * dt
        dr, dp = dist.get(t)

        # IMU reads body tilt
        mr, mp = imu.measure(body_roll, body_pitch, t)

        # PID compute
        if enable_pid:
            corr_roll  = pid_roll.compute(0.0, mr, t)
            corr_pitch = pid_pitch.compute(0.0, mp, t)
        else:
            corr_roll, corr_pitch = 0.0, 0.0

        for leg in leg_names:
            p_offline = traj_offline[leg][gait_frame].copy()
            offline_angles = inverse_kinematics(
                p_offline[0], p_offline[1], p_offline[2], leg, params)
            p_adjusted = apply_paper_rotation(
                p_offline, corr_roll, corr_pitch, p_0)
            final_angles = inverse_kinematics(
                p_adjusted[0], p_adjusted[1], p_adjusted[2], leg, params)

            for i, axis in enumerate(['x', 'y', 'z']):
                history[f'{leg}_offline_{axis}'].append(p_offline[i])
                history[f'{leg}_adjusted_{axis}'].append(p_adjusted[i])
            for i in range(3):
                history[f'{leg}_offline_th{i+1}'].append(offline_angles[i])
                history[f'{leg}_final_th{i+1}'].append(final_angles[i])

        # ── Simple plant dynamics ─────────────────────────────────
        # Target body angle = disturbance - PID correction
        # The body follows this through a first-order lag (servo response)
        target_roll  = dr - corr_roll
        target_pitch = dp - corr_pitch
        body_roll  += alpha_plant * (target_roll  - body_roll)
        body_pitch += alpha_plant * (target_pitch - body_pitch)

        history['time'].append(t)
        history['gait_frame'].append(gait_frame)
        history['body_roll'].append(body_roll)
        history['body_pitch'].append(body_pitch)
        history['pid_roll'].append(corr_roll)
        history['pid_pitch'].append(corr_pitch)
        history['dist_roll'].append(dr)
        history['dist_pitch'].append(dp)

        gait_frame = (gait_frame + 1) % T_cycle

    return {k: np.array(v) for k, v in history.items()}


# ══════════════════════════════════════════════════════════════════
#  Static Plot: D-Shape Explained
# ══════════════════════════════════════════════════════════════════

def plot_dshape_explained(output_dir):
    """Show corrected D-shape trajectory shape."""
    params = RobotParams()
    traj, T = generate_trot_trajectory(params, n_swing=150)

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle("Corrected D-Shape Foot Trajectory (Swing + Stance)",
                 fontsize=14, fontweight='bold', color='#ffffff', y=0.98)

    leg = 'left-front'
    x = traj[leg][:, 0] * 1000  # to mm
    z = traj[leg][:, 2] * 1000

    # XZ path
    ax = axes[0]
    ax.plot(x, z, '-', color=COLORS['offline'], lw=2, alpha=0.8)
    ax.plot(x[:150], z[:150], 'o-', color='#ff6348', lw=2, ms=3,
            label=f'Swing ({150} points)')
    ax.plot(x[150:], z[150:], '.-', color='#1e90ff', lw=1, ms=2, alpha=0.5,
            label=f'Stance ({300} points)')
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Z (mm)')
    ax.set_title('Left-Front Leg — XZ Path (D-Shape)', fontsize=11)
    ax.legend(fontsize=9)
    ax.grid(True)
    ax.set_aspect('auto')

    # Z vs frame number
    ax = axes[1]
    frames = np.arange(T)
    ax.plot(frames[:150], z[:150], 'o-', color='#ff6348', lw=2, ms=3,
            label='Swing (sine arch)')
    ax.plot(frames[150:], z[150:], '.-', color='#1e90ff', lw=1, ms=2, alpha=0.5,
            label='Stance (flat)')
    ax.set_xlabel('Gait Frame')
    ax.set_ylabel('Z (mm)')
    ax.set_title('Z Height vs Gait Frame — Smooth Parabola', fontsize=11)
    ax.legend(fontsize=9)
    ax.grid(True)

    fig.tight_layout(rect=[0, 0, 1, 0.95])
    path = os.path.join(output_dir, 'dshape_corrected.png')
    os.makedirs(output_dir, exist_ok=True)
    fig.savefig(path, dpi=150, bbox_inches='tight', facecolor=fig.get_facecolor())
    plt.close(fig)
    print(f"  ✓ dshape_corrected.png")


# ══════════════════════════════════════════════════════════════════
#  Static Plot: Paper Formula Explained
# ══════════════════════════════════════════════════════════════════

def plot_formula_explained(output_dir):
    """Visualize the paper's rotation formula vs naive multiplication."""
    fig, ax = plt.subplots(1, 1, figsize=(16, 10))
    ax.set_xlim(0, 16)
    ax.set_ylim(0, 10)
    ax.axis('off')
    fig.suptitle("Paper's Rotation Formula Explained",
                 fontsize=16, fontweight='bold', color='#ffffff', y=0.98)

    def box(x, y, w, h, text, color='#16213e', edge='#e94560', fontsize=10):
        b = FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.15",
                          facecolor=color, edgecolor=edge, linewidth=2, alpha=0.9)
        ax.add_patch(b)
        ax.text(x+w/2, y+h/2, text, ha='center', va='center',
                fontsize=fontsize, color='#ffffff', family='monospace')

    # Title
    ax.text(8, 9.3, "Paper's Formula:   p_B_t = R⁻¹ · (p_B_c − p_B_0) + p_B_0",
            ha='center', fontsize=14, fontweight='bold', color='#ffd700',
            family='monospace')

    # Step by step
    y = 7.8
    ax.text(1, y, "Step-by-step breakdown:", fontsize=12, fontweight='bold',
            color='#ffffff')

    steps = [
        ("p_B_c", "= current foot position (from offline gait trajectory)",
         "e.g. left-front foot at [0.125, 0.135, -0.170] meters", '#ffa502'),
        ("p_B_0", "= reference point = body center (Center of Gravity)",
         "In body frame: [0, 0, 0]  (origin of the coordinate system)", '#2ed573'),
        ("p_B_c − p_B_0", "= foot position RELATIVE TO body center",
         "= [0.125, 0.135, -0.170] − [0, 0, 0] = [0.125, 0.135, -0.170]", '#54a0ff'),
        ("R⁻¹", "= R(−Δφ, −Δθ) = INVERSE rotation matrix",
         "If body tilted +3°, we rotate feet by −3° to compensate", '#e94560'),
        ("R⁻¹·(p_B_c−p_B_0)", "= rotated foot position (relative to body center)",
         "This rotates the foot around the body center, NOT around origin", '#a29bfe'),
        ("+ p_B_0", "= translate back to world coordinates",
         "Since p_B_0 = [0,0,0], this step does nothing (in our case)", '#fd79a8'),
    ]

    for i, (var, desc, detail, color) in enumerate(steps):
        yp = y - 1.1 - i * 1.1
        ax.text(1.0, yp, f"{i+1}.", fontsize=11, fontweight='bold', color=color)
        ax.text(1.5, yp, var, fontsize=11, fontweight='bold', color=color,
                family='monospace')
        ax.text(4.5, yp, desc, fontsize=9, color='#eaeaea')
        ax.text(4.5, yp - 0.35, detail, fontsize=8, color='#999999',
                style='italic')

    # Key insight box
    props = dict(boxstyle='round,pad=0.5', facecolor='#27ae60',
                 edgecolor='#ffd700', alpha=0.9)
    insight = ("KEY INSIGHT: In our coordinate system, p_B_0 = [0,0,0]\n"
               "because foot positions are already expressed relative\n"
               "to the body center. So the formula simplifies to:\n\n"
               "    p_B_t = R⁻¹ · p_B_c\n\n"
               "This is exactly what our code does! R⁻¹ = R(−Δφ, −Δθ)\n"
               "is built using cos(-Δφ), sin(-Δφ), cos(-Δθ), sin(-Δθ).\n"
               "Both formulas give identical results for our robot.")
    ax.text(1.0, 0.2, insight, fontsize=9, color='#ffffff', bbox=props)

    # Wrong vs right box
    props2 = dict(boxstyle='round,pad=0.5', facecolor='#c0392b',
                  edgecolor='#ffd700', alpha=0.9)
    wrong = ("IMPORTANT DIFFERENCE:\n"
             "• R(Δφ,Δθ) · p_c   ← WRONG (rotates the wrong direction)\n"
             "• R⁻¹ · p_c         ← RIGHT (compensates body tilt)\n"
             "• R(-Δφ,-Δθ) · p_c  ← SAME AS R⁻¹ (our implementation)")
    ax.text(9.0, 0.2, wrong, fontsize=9, color='#ffffff', bbox=props2)

    path = os.path.join(output_dir, 'formula_explained.png')
    os.makedirs(output_dir, exist_ok=True)
    fig.savefig(path, dpi=150, bbox_inches='tight', facecolor=fig.get_facecolor())
    plt.close(fig)
    print(f"  ✓ formula_explained.png")


# ══════════════════════════════════════════════════════════════════
#  Static Plot: Combination Data
# ══════════════════════════════════════════════════════════════════

def plot_combination_explained(h, output_dir):
    """Show offline vs adjusted foot positions and joint angles."""
    t = h['time']
    leg = 'left-front'

    fig, axes = plt.subplots(4, 2, figsize=(16, 14), sharex=True)
    fig.suptitle("Offline Trajectory + PID Correction = Final Joint Angles\n"
                 "(Left-Front Leg — Corrected D-Shape + Paper Formula)",
                 fontsize=14, fontweight='bold', color='#ffffff', y=0.99)

    ax = axes[0, 0]
    ax.plot(t, h[f'{leg}_offline_x']*1000, color=COLORS['offline'], lw=1.5, label='Offline')
    ax.plot(t, h[f'{leg}_adjusted_x']*1000, color=COLORS['adjusted'], lw=1.5, ls='--', label='Adjusted')
    ax.set_ylabel('Foot X (mm)'); ax.set_title('Foot X Position'); ax.legend(fontsize=8); ax.grid(True)

    ax = axes[0, 1]
    ax.plot(t, h[f'{leg}_offline_z']*1000, color=COLORS['offline'], lw=1.5, label='Offline')
    ax.plot(t, h[f'{leg}_adjusted_z']*1000, color=COLORS['adjusted'], lw=1.5, ls='--', label='Adjusted')
    ax.set_ylabel('Foot Z (mm)'); ax.set_title('Foot Z Position — Key Change'); ax.legend(fontsize=8); ax.grid(True)

    ax = axes[1, 0]
    dx = (h[f'{leg}_adjusted_x'] - h[f'{leg}_offline_x']) * 1000
    ax.plot(t, dx, color=COLORS['pid_corr'], lw=1.5); ax.fill_between(t, dx, 0, alpha=0.2, color=COLORS['pid_corr'])
    ax.set_ylabel('PID Δx (mm)'); ax.set_title('PID Correction in X'); ax.grid(True)

    ax = axes[1, 1]
    dz = (h[f'{leg}_adjusted_z'] - h[f'{leg}_offline_z']) * 1000
    ax.plot(t, dz, color=COLORS['pid_corr'], lw=1.5); ax.fill_between(t, dz, 0, alpha=0.2, color=COLORS['pid_corr'])
    ax.set_ylabel('PID Δz (mm)'); ax.set_title('PID Correction in Z (height)'); ax.grid(True)

    ax = axes[2, 0]
    th2_off = np.degrees(np.unwrap(h[f'{leg}_offline_th2']))
    th2_fin = np.degrees(np.unwrap(h[f'{leg}_final_th2']))
    ax.plot(t, th2_off, color=COLORS['offline'], lw=1.5, label='Offline θ₂')
    ax.plot(t, th2_fin, color=COLORS['adjusted'], lw=1.5, ls='--', label='Final θ₂')
    ax.set_ylabel('θ₂ (deg)'); ax.set_title('Joint θ₂ (Upper Leg)'); ax.legend(fontsize=8); ax.grid(True)

    ax = axes[2, 1]
    th3_off = np.degrees(np.unwrap(h[f'{leg}_offline_th3']))
    th3_fin = np.degrees(np.unwrap(h[f'{leg}_final_th3']))
    ax.plot(t, th3_off, color=COLORS['offline'], lw=1.5, label='Offline θ₃')
    ax.plot(t, th3_fin, color=COLORS['adjusted'], lw=1.5, ls='--', label='Final θ₃')
    ax.set_ylabel('θ₃ (deg)'); ax.set_title('Joint θ₃ (Lower Leg)'); ax.legend(fontsize=8); ax.grid(True)

    ax = axes[3, 0]
    ax.plot(t, np.degrees(h['pid_roll']), color=COLORS['pid_corr'], lw=1.5, label='PID roll')
    ax.plot(t, np.degrees(h['pid_pitch']), color=COLORS['body'], lw=1.5, label='PID pitch')
    ax.set_ylabel('Correction (deg)'); ax.set_title('PID Output'); ax.legend(fontsize=8); ax.grid(True)
    ax.set_xlabel('Time (s)')

    ax = axes[3, 1]
    ax.plot(t, np.degrees(h['dist_roll']), '--', color=COLORS['dist'], lw=1, alpha=0.5, label='Disturbance')
    ax.plot(t, np.degrees(h['body_roll']), color=COLORS['final'], lw=1.5, label='Body roll')
    ax.axhline(0, color=COLORS['setpt'], ls=':', alpha=0.3)
    ax.set_ylabel('Angle (deg)'); ax.set_title('Result: Body stays level'); ax.legend(fontsize=8); ax.grid(True)
    ax.set_xlabel('Time (s)')

    fig.tight_layout(rect=[0, 0, 1, 0.96])
    path = os.path.join(output_dir, 'trajectory_combination_v2.png')
    os.makedirs(output_dir, exist_ok=True)
    fig.savefig(path, dpi=150, bbox_inches='tight', facecolor=fig.get_facecolor())
    plt.close(fig)
    print(f"  ✓ trajectory_combination_v2.png")


# ══════════════════════════════════════════════════════════════════
#  Animation: Corrected Foot Trajectory
# ══════════════════════════════════════════════════════════════════

def anim_foot_trajectory(h, output_dir):
    """Animate foot paths with corrected D-shape."""
    print("  Generating corrected foot trajectory animation...")

    ds = 10
    t = h['time'][::ds]
    n_frames = len(t)

    fig = plt.figure(figsize=(18, 10))
    gs = GridSpec(2, 2, figure=fig, hspace=0.35, wspace=0.3)
    fig.suptitle("Corrected D-Shape: Offline (orange) vs PID-Adjusted (blue)\n"
                 "Swing = smooth sine arch, NOT line segments",
                 fontsize=13, fontweight='bold', color='#ffffff', y=0.98)

    legs = ['left-front', 'left-behind', 'right-front', 'right-behind']
    leg_titles = ['Left-Front', 'Left-Behind', 'Right-Front', 'Right-Behind']
    axes_list = [fig.add_subplot(gs[i//2, i%2]) for i in range(4)]

    for ax, title in zip(axes_list, leg_titles):
        ax.set_xlabel('X (mm)'); ax.set_ylabel('Z (mm)')
        ax.set_title(title, fontsize=11, fontweight='bold')
        ax.grid(True)
        ax.set_xlim(-170, 170)
        ax.set_ylim(-185, -115)

    offline_trails, adjusted_trails, offline_dots, adjusted_dots = [], [], [], []
    for ax in axes_list:
        ot, = ax.plot([], [], '-', color=COLORS['offline'], lw=1, alpha=0.5)
        at, = ax.plot([], [], '-', color=COLORS['adjusted'], lw=1, alpha=0.5)
        od, = ax.plot([], [], 'o', color=COLORS['offline'], ms=8, label='Offline')
        ad, = ax.plot([], [], 's', color=COLORS['adjusted'], ms=8, label='Adjusted (PID)')
        ax.legend(fontsize=8, loc='upper right')
        offline_trails.append(ot); adjusted_trails.append(at)
        offline_dots.append(od); adjusted_dots.append(ad)

    time_txt = fig.text(0.5, 0.01, '', ha='center', fontsize=12, color='#ffd700')

    data = {}
    for leg in legs:
        data[f'{leg}_ox'] = h[f'{leg}_offline_x'][::ds] * 1000
        data[f'{leg}_oz'] = h[f'{leg}_offline_z'][::ds] * 1000
        data[f'{leg}_ax'] = h[f'{leg}_adjusted_x'][::ds] * 1000
        data[f'{leg}_az'] = h[f'{leg}_adjusted_z'][::ds] * 1000

    trail_len = 120

    def animate(i):
        start = max(0, i - trail_len)
        s = slice(start, i+1)
        for j, leg in enumerate(legs):
            offline_trails[j].set_data(data[f'{leg}_ox'][s], data[f'{leg}_oz'][s])
            adjusted_trails[j].set_data(data[f'{leg}_ax'][s], data[f'{leg}_az'][s])
            offline_dots[j].set_data([data[f'{leg}_ox'][i]], [data[f'{leg}_oz'][i]])
            adjusted_dots[j].set_data([data[f'{leg}_ax'][i]], [data[f'{leg}_az'][i]])

        dz = data['left-front_az'][i] - data['left-front_oz'][i]
        time_txt.set_text(f't = {t[i]:.2f}s   |   LF: Offline Z={data["left-front_oz"][i]:.1f}mm   '
                          f'Adjusted Z={data["left-front_az"][i]:.1f}mm   ΔZ={dz:+.2f}mm')
        return ()

    anim = animation.FuncAnimation(fig, animate, frames=n_frames, interval=50, blit=True)
    fig.tight_layout(rect=[0, 0.03, 1, 0.95])

    path = os.path.join(output_dir, 'foot_trajectory_v2.gif')
    os.makedirs(output_dir, exist_ok=True)
    anim.save(path, writer='pillow', fps=20,
              savefig_kwargs={'facecolor': fig.get_facecolor()})
    plt.close(fig)
    size = os.path.getsize(path) / 1024
    print(f"  ✓ foot_trajectory_v2.gif ({size:.0f} KB, {n_frames} frames)")


# ══════════════════════════════════════════════════════════════════
#  Animation: Rotation Formula Visualized
# ══════════════════════════════════════════════════════════════════

def anim_rotation_formula(h, output_dir):
    """Animate the paper's rotation formula step by step on one foot."""
    print("  Generating rotation formula animation...")

    ds = 10
    t = h['time'][::ds]
    n_frames = len(t)
    leg = 'left-front'

    fig, axes = plt.subplots(1, 2, figsize=(16, 7))
    fig.suptitle("Paper Formula in Action: p_t = R⁻¹ · (p_c − p₀) + p₀\n"
                 "Left-Front Leg — Top View (XY) and Side View (XZ)",
                 fontsize=13, fontweight='bold', color='#ffffff', y=0.98)

    ax_xy = axes[0]
    ax_xz = axes[1]

    ax_xy.set_xlabel('X (mm)'); ax_xy.set_ylabel('Y (mm)')
    ax_xy.set_title('Top View (XY)', fontsize=11, fontweight='bold')
    ax_xy.grid(True); ax_xy.set_aspect('equal')
    ax_xy.set_xlim(80, 160); ax_xy.set_ylim(100, 170)

    ax_xz.set_xlabel('X (mm)'); ax_xz.set_ylabel('Z (mm)')
    ax_xz.set_title('Side View (XZ)', fontsize=11, fontweight='bold')
    ax_xz.grid(True)
    ax_xz.set_xlim(80, 160); ax_xz.set_ylim(-185, -115)

    # Body center marker
    ax_xy.plot(0, 0, 'x', color=COLORS['setpt'], ms=10, mew=2, label='Body center (p₀)')
    ax_xz.plot(0, 0, 'x', color=COLORS['setpt'], ms=10, mew=2)

    ox = h[f'{leg}_offline_x'][::ds] * 1000
    oy = h[f'{leg}_offline_y'][::ds] * 1000
    oz = h[f'{leg}_offline_z'][::ds] * 1000
    ax_ = h[f'{leg}_adjusted_x'][::ds] * 1000
    ay = h[f'{leg}_adjusted_y'][::ds] * 1000
    az = h[f'{leg}_adjusted_z'][::ds] * 1000

    trail = 100

    ot_xy, = ax_xy.plot([], [], '-', color=COLORS['offline'], lw=1, alpha=0.4)
    at_xy, = ax_xy.plot([], [], '-', color=COLORS['adjusted'], lw=1, alpha=0.4)
    od_xy, = ax_xy.plot([], [], 'o', color=COLORS['offline'], ms=8, label='p_c (offline)')
    ad_xy, = ax_xy.plot([], [], 's', color=COLORS['adjusted'], ms=8, label='p_t (adjusted)')
    conn_xy, = ax_xy.plot([], [], '--', color=COLORS['pid_corr'], lw=1, alpha=0.5)
    ax_xy.legend(fontsize=8, loc='upper left')

    ot_xz, = ax_xz.plot([], [], '-', color=COLORS['offline'], lw=1, alpha=0.4)
    at_xz, = ax_xz.plot([], [], '-', color=COLORS['adjusted'], lw=1, alpha=0.4)
    od_xz, = ax_xz.plot([], [], 'o', color=COLORS['offline'], ms=8, label='p_c (offline)')
    ad_xz, = ax_xz.plot([], [], 's', color=COLORS['adjusted'], ms=8, label='p_t (adjusted)')
    conn_xz, = ax_xz.plot([], [], '--', color=COLORS['pid_corr'], lw=1, alpha=0.5)
    ax_xz.legend(fontsize=8, loc='upper left')

    time_txt = fig.text(0.5, 0.01, '', ha='center', fontsize=11, color='#ffd700')

    def animate(i):
        s0 = max(0, i - trail)
        s = slice(s0, i+1)

        ot_xy.set_data(ox[s], oy[s])
        at_xy.set_data(ax_[s], ay[s])
        od_xy.set_data([ox[i]], [oy[i]])
        ad_xy.set_data([ax_[i]], [ay[i]])
        conn_xy.set_data([ox[i], ax_[i]], [oy[i], ay[i]])

        ot_xz.set_data(ox[s], oz[s])
        at_xz.set_data(ax_[s], az[s])
        od_xz.set_data([ox[i]], [oz[i]])
        ad_xz.set_data([ax_[i]], [az[i]])
        conn_xz.set_data([ox[i], ax_[i]], [oz[i], az[i]])

        dx = ax_[i] - ox[i]
        dy = ay[i] - oy[i]
        dz = az[i] - oz[i]
        time_txt.set_text(f't={t[i]:.2f}s   |   p_c=[{ox[i]:.1f}, {oy[i]:.1f}, {oz[i]:.1f}]   '
                          f'p_t=[{ax_[i]:.1f}, {ay[i]:.1f}, {az[i]:.1f}]   '
                          f'Δ=[{dx:+.2f}, {dy:+.2f}, {dz:+.2f}]mm')
        return ()

    anim = animation.FuncAnimation(fig, animate, frames=n_frames, interval=50, blit=True)
    fig.tight_layout(rect=[0, 0.04, 1, 0.95])

    path = os.path.join(output_dir, 'rotation_formula_v2.gif')
    anim.save(path, writer='pillow', fps=20,
              savefig_kwargs={'facecolor': fig.get_facecolor()})
    plt.close(fig)
    size = os.path.getsize(path) / 1024
    print(f"  ✓ rotation_formula_v2.gif ({size:.0f} KB, {n_frames} frames)")


# ══════════════════════════════════════════════════════════════════
#  Animation: Full Pipeline (corrected)
# ══════════════════════════════════════════════════════════════════

def anim_full_pipeline(h, output_dir):
    """Full pipeline animation with corrected D-shape."""
    print("  Generating corrected pipeline animation...")

    ds = 10
    t = h['time'][::ds]
    n_frames = len(t)
    leg = 'left-front'

    fig = plt.figure(figsize=(18, 11))
    gs = GridSpec(3, 3, figure=fig, hspace=0.4, wspace=0.3)
    fig.suptitle("Complete Pipeline (Corrected): Offline + PID → Final Angles\n"
                 "Left-Front Leg  |  Formula: p_t = R⁻¹·(p_c − p₀) + p₀",
                 fontsize=13, fontweight='bold', color='#ffffff', y=0.99)

    ax_off  = fig.add_subplot(gs[0, 0])
    ax_pid  = fig.add_subplot(gs[0, 1])
    ax_adj  = fig.add_subplot(gs[0, 2])
    ax_th_off = fig.add_subplot(gs[1, 0])
    ax_th_d   = fig.add_subplot(gs[1, 1])
    ax_th_fin = fig.add_subplot(gs[1, 2])
    ax_body = fig.add_subplot(gs[2, :])

    ax_off.set_title('① Offline D-Shape (XZ)', fontsize=10, fontweight='bold')
    ax_off.set_xlabel('X (mm)'); ax_off.set_ylabel('Z (mm)')
    ax_off.set_xlim(95, 155); ax_off.set_ylim(-185, -115); ax_off.grid(True)

    ax_pid.set_title('② PID Correction', fontsize=10, fontweight='bold')
    ax_pid.set_xlim(0, t[-1]); ax_pid.set_ylabel('Correction (deg)'); ax_pid.grid(True)

    ax_adj.set_title('③ Adjusted Path (XZ)', fontsize=10, fontweight='bold')
    ax_adj.set_xlabel('X (mm)'); ax_adj.set_ylabel('Z (mm)')
    ax_adj.set_xlim(95, 155); ax_adj.set_ylim(-185, -115); ax_adj.grid(True)

    for axs, title in [(ax_th_off, '④ Offline θ₂'), (ax_th_d, '⑤ Δθ₂ = PID change'),
                        (ax_th_fin, '⑥ Final θ₂ (to servo)')]:
        axs.set_title(title, fontsize=10, fontweight='bold')
        axs.set_xlim(0, t[-1]); axs.set_ylabel('θ₂ (deg)'); axs.grid(True)

    ax_body.set_title('Result: Body Roll Stabilized', fontsize=11, fontweight='bold')
    ax_body.set_xlim(0, t[-1]); ax_body.set_ylabel('Angle (deg)'); ax_body.set_xlabel('Time (s)')
    ax_body.grid(True)

    ox = h[f'{leg}_offline_x'][::ds]*1000; oz = h[f'{leg}_offline_z'][::ds]*1000
    adj_x = h[f'{leg}_adjusted_x'][::ds]*1000; adj_z = h[f'{leg}_adjusted_z'][::ds]*1000
    pid_r = np.degrees(h['pid_roll'][::ds]); pid_p = np.degrees(h['pid_pitch'][::ds])
    th2_off = np.degrees(np.unwrap(h[f'{leg}_offline_th2'])[::ds])
    th2_fin = np.degrees(np.unwrap(h[f'{leg}_final_th2'])[::ds])
    th2_diff = th2_fin - th2_off
    body_r = np.degrees(h['body_roll'][::ds]); dist_r = np.degrees(h['dist_roll'][::ds])

    for data, ax_ in [(pid_r, ax_pid), (th2_off, ax_th_off),
                      (th2_diff, ax_th_d), (th2_fin, ax_th_fin)]:
        mx = np.max(np.abs(data)) * 1.2
        if mx > 0: ax_.set_ylim(-mx, mx)
    mx = max(np.max(np.abs(body_r)), np.max(np.abs(dist_r))) * 1.2
    ax_body.set_ylim(-mx, mx)

    trail_len = 100
    ot, = ax_off.plot([], [], '-', color=COLORS['offline'], lw=1.5, alpha=0.6)
    od, = ax_off.plot([], [], 'o', color=COLORS['offline'], ms=8)
    at, = ax_adj.plot([], [], '-', color=COLORS['adjusted'], lw=1.5, alpha=0.6)
    ad, = ax_adj.plot([], [], 's', color=COLORS['adjusted'], ms=8)
    ag, = ax_adj.plot([], [], '-', color=COLORS['offline'], lw=1, alpha=0.15)

    lpr, = ax_pid.plot([], [], color=COLORS['pid_corr'], lw=1.5, label='Roll')
    lpp, = ax_pid.plot([], [], color=COLORS['body'], lw=1.5, label='Pitch')
    ax_pid.legend(fontsize=7)

    lto, = ax_th_off.plot([], [], color=COLORS['offline'], lw=1.5)
    ltd, = ax_th_d.plot([], [], color=COLORS['pid_corr'], lw=1.5)
    ltf, = ax_th_fin.plot([], [], color=COLORS['adjusted'], lw=1.5)

    lb, = ax_body.plot([], [], color=COLORS['final'], lw=1.5, label='Body roll')
    ld, = ax_body.plot([], [], '--', color=COLORS['dist'], lw=1, alpha=0.5, label='Disturbance')
    ax_body.axhline(0, color=COLORS['setpt'], ls=':', alpha=0.3)
    ax_body.legend(fontsize=8)

    tt = fig.text(0.5, 0.01, '', ha='center', fontsize=11, color='#ffd700')

    fig.text(0.36, 0.72, '→', ha='center', fontsize=18, color='#ffd700')
    fig.text(0.66, 0.72, '→', ha='center', fontsize=18, color='#ffd700')
    fig.text(0.36, 0.42, '→', ha='center', fontsize=18, color='#ffd700')
    fig.text(0.66, 0.42, '→', ha='center', fontsize=18, color='#ffd700')

    def animate(i):
        s0 = max(0, i-trail_len)
        st = slice(s0, i+1); sf = slice(0, i+1)

        ot.set_data(ox[st], oz[st]); od.set_data([ox[i]], [oz[i]])
        at.set_data(adj_x[st], adj_z[st]); ad.set_data([adj_x[i]], [adj_z[i]])
        ag.set_data(ox[st], oz[st])

        lpr.set_data(t[sf], pid_r[sf]); lpp.set_data(t[sf], pid_p[sf])
        lto.set_data(t[sf], th2_off[sf]); ltd.set_data(t[sf], th2_diff[sf])
        ltf.set_data(t[sf], th2_fin[sf])
        lb.set_data(t[sf], body_r[sf]); ld.set_data(t[sf], dist_r[sf])

        tt.set_text(f't={t[i]:.2f}s   |   Offline θ₂={th2_off[i]:.2f}°   '
                    f'+ PID Δθ₂={th2_diff[i]:+.3f}°   = Final θ₂={th2_fin[i]:.2f}°')
        return ()

    anim = animation.FuncAnimation(fig, animate, frames=n_frames, interval=50, blit=True)
    fig.tight_layout(rect=[0, 0.03, 1, 0.96])

    path = os.path.join(output_dir, 'full_pipeline_v2.gif')
    anim.save(path, writer='pillow', fps=20,
              savefig_kwargs={'facecolor': fig.get_facecolor()})
    plt.close(fig)
    size = os.path.getsize(path) / 1024
    print(f"  ✓ full_pipeline_v2.gif ({size:.0f} KB, {n_frames} frames)")


# ══════════════════════════════════════════════════════════════════
#  Combination Pipeline Diagram (corrected)
# ══════════════════════════════════════════════════════════════════

def plot_pipeline_diagram(output_dir):
    """Draw the corrected pipeline with paper's formula."""
    fig, ax = plt.subplots(1, 1, figsize=(18, 9))
    ax.set_xlim(0, 18); ax.set_ylim(0, 9); ax.axis('off')
    fig.suptitle("How 12 Final Joint Angles Are Computed (Corrected)\n"
                 "Paper Formula: p_t = R⁻¹ · (p_c − p₀) + p₀",
                 fontsize=15, fontweight='bold', color='#ffffff', y=0.97)

    def box(x, y, w, h, label, sub="", color='#0f3460'):
        b = FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.1",
                          facecolor=color, edgecolor='#e94560', linewidth=2, alpha=0.9)
        ax.add_patch(b)
        ax.text(x+w/2, y+h/2+0.12, label, ha='center', va='center',
                fontsize=10, fontweight='bold', color='#ffffff')
        if sub:
            ax.text(x+w/2, y+h/2-0.25, sub, ha='center', va='center',
                    fontsize=7, color='#cccccc', style='italic')

    def arrow(x1, y1, x2, y2, label="", color='#54a0ff'):
        ax.annotate("", xy=(x2, y2), xytext=(x1, y1),
                    arrowprops=dict(arrowstyle='->', color=color, lw=2.5))
        if label:
            mx, my = (x1+x2)/2, (y1+y2)/2
            ax.text(mx, my+0.2, label, ha='center', va='bottom',
                    fontsize=8, color=color, fontweight='bold')

    y_top = 7.0
    box(0.3, y_top, 2.5, 1.0, "OFFLINE\nGait Planner", "D-shape\n(sin arch swing)", '#c0392b')
    arrow(2.8, y_top+0.5, 4.3, y_top+0.5, "p_c = foot_pos[i]", '#ffa502')
    box(4.3, y_top, 2.0, 1.0, "p_B_c\n(foot position)", "(x, y, z) per leg", '#e67e22')

    y_bot = 4.5
    box(0.3, y_bot, 1.8, 1.0, "IMU\nSensor", "roll, pitch", '#8e44ad')
    arrow(2.1, y_bot+0.5, 3.3, y_bot+0.5, "φ, θ", '#a29bfe')
    box(3.3, y_bot, 2.0, 1.0, "PID", "Kp·e + Ki·∫e + Kd·de/dt", '#e94560')
    arrow(5.3, y_bot+0.5, 6.8, y_bot+0.5, "Δφ, Δθ", '#2ed573')
    box(6.8, y_bot, 2.2, 1.0, "Build R⁻¹", "R⁻¹ = R(-Δφ,-Δθ)", '#27ae60')

    # Combination
    box(9.5, 5.5, 2.8, 1.2, "PAPER FORMULA", "p_t = R⁻¹·(p_c−p₀)+p₀", '#ffd700')
    for child in ax.get_children():
        if isinstance(child, plt.Text) and child.get_text() == "PAPER FORMULA":
            child.set_color('#1a1a2e')
        if isinstance(child, plt.Text) and "p_t = R" in child.get_text():
            child.set_color('#1a1a2e')

    arrow(6.3, y_top+0.5, 9.5, 6.3, "", '#ffa502')
    arrow(9.0, y_bot+0.5, 9.5, 5.9, "", '#2ed573')

    arrow(12.3, 6.1, 13.5, 6.1, "p_t (adjusted)", '#54a0ff')
    box(13.5, 5.5, 2.0, 1.2, "Inverse\nKinematics", "3-DOF per leg", '#2980b9')
    arrow(15.5, 6.1, 16.2, 6.1, "", '#ff6b6b')
    box(16.2, 5.5, 1.5, 1.2, "12\nAngles", "→ Servos", '#c0392b')

    # Reference point note
    props = dict(boxstyle='round,pad=0.4', facecolor='#2d3436', edgecolor='#ffd700', alpha=0.9)
    ax.text(9.5, 4.2, "p₀ = [0, 0, 0] (body center)\n"
            "In our frame, foot positions are\n"
            "already relative to body center,\n"
            "so p_c − p₀ = p_c", fontsize=8, color='#ffd700', bbox=props)

    ax.text(0.5, 2.5, "R⁻¹ = R(-Δφ, -Δθ)\n\n"
            "If body tilted +3° (IMU),\n"
            "PID says correct by Δφ,\n"
            "R⁻¹ rotates feet by -Δφ\n"
            "to compensate.", fontsize=8, color='#2ed573', bbox=props)

    ax.text(0.5, 0.3, "WHY R⁻¹ NOT R?  If body tilted right,\n"
            "right legs must push DOWN (extend),\n"
            "left legs pull UP (retract).\n"
            "R⁻¹ = inverse rotation = opposite direction.",
            fontsize=8, color='#ff6b6b', bbox=dict(boxstyle='round,pad=0.4',
            facecolor='#c0392b', edgecolor='#ffd700', alpha=0.9))

    path = os.path.join(output_dir, 'combination_pipeline_v2.png')
    os.makedirs(output_dir, exist_ok=True)
    fig.savefig(path, dpi=150, bbox_inches='tight', facecolor=fig.get_facecolor())
    plt.close(fig)
    print(f"  ✓ combination_pipeline_v2.png")


# ══════════════════════════════════════════════════════════════════
#  Main
# ══════════════════════════════════════════════════════════════════

def main():
    print("╔══════════════════════════════════════════════════════════════╗")
    print("║  Trajectory Combination Simulation v2 (CORRECTED)           ║")
    print("║  D-Shape: sin arch  |  Formula: p_t = R⁻¹·(p_c-p₀)+p₀     ║")
    print("╚══════════════════════════════════════════════════════════════╝\n")

    print("Running combined simulation...")
    h = run_combined_simulation(enable_pid=True, T_seconds=6.0, dt=0.003)
    print(f"  Done: {len(h['time'])} steps\n")

    print("Generating plots and animations...")
    plot_dshape_explained(OUTPUT_DIR)
    plot_formula_explained(OUTPUT_DIR)
    plot_pipeline_diagram(OUTPUT_DIR)
    plot_combination_explained(h, OUTPUT_DIR)
    anim_foot_trajectory(h, OUTPUT_DIR)
    anim_rotation_formula(h, OUTPUT_DIR)
    anim_full_pipeline(h, OUTPUT_DIR)

    print(f"\n✅ All outputs saved to: {OUTPUT_DIR}/")


if __name__ == "__main__":
    main()
