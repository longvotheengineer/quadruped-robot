"""
Visualization Module
====================
Generates all plots, block diagrams, and animations for the IMU PID
posture stabilization simulation.

Output files (saved in results/ directory):
1. block_diagram.png       – Control loop block diagram (Simulink-style)
2. time_response.png       – Roll/pitch time response for PID scenario
3. comparison.png          – No control vs P vs PD vs PID comparison
4. pid_signals.png         – Detailed PID internal signals (P, I, D terms)
5. imu_signals.png         – IMU true vs noisy vs filtered signals
6. step_response.png       – Step disturbance response with metrics
7. combined_response.png   – Combined (sin + step) disturbance response
8. joint_angles.png        – Joint angle trajectories
9. foot_adjustments.png    – Foot height adjustments (Δz) per leg
10. body_animation.gif     – Animated body tilt over time
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
import matplotlib.animation as animation

from robot_params import RobotParams


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
    'axes.titlesize':   13,
    'axes.titleweight': 'bold',
    'figure.titlesize': 16,
    'figure.titleweight': 'bold',
})

COLORS = {
    'no_control': '#ff6b6b',
    'p_only':     '#ffa502',
    'pd':         '#1dd1a1',
    'pid':        '#54a0ff',
    'roll':       '#e94560',
    'pitch':      '#0f3460',
    'disturbance':'#ffa502',
    'setpoint':   '#ffffff',
    'p_term':     '#ff6348',
    'i_term':     '#2ed573',
    'd_term':     '#1e90ff',
    'output':     '#ffd700',
    'accent1':    '#a29bfe',
    'accent2':    '#fd79a8',
    'accent3':    '#00cec9',
    'accent4':    '#fdcb6e',
}


def _save_fig(fig, output_dir, filename):
    os.makedirs(output_dir, exist_ok=True)
    path = os.path.join(output_dir, filename)
    fig.savefig(path, dpi=150, bbox_inches='tight', facecolor=fig.get_facecolor())
    plt.close(fig)
    print(f"  ✓ Saved: {filename}")


# ══════════════════════════════════════════════════════════════════
#  1. Control Block Diagram (Simulink-style)
# ══════════════════════════════════════════════════════════════════

def plot_block_diagram(results, output_dir):
    """Draw the control loop block diagram."""
    fig, ax = plt.subplots(1, 1, figsize=(16, 7))
    ax.set_xlim(0, 16)
    ax.set_ylim(0, 7)
    ax.axis('off')
    fig.suptitle("IMU PID Posture Stabilization — Control Block Diagram",
                 fontsize=16, fontweight='bold', color='#ffffff', y=0.96)

    # Block style
    def draw_block(x, y, w, h, label, sublabel="", color='#0f3460'):
        box = FancyBboxPatch((x, y), w, h,
                             boxstyle="round,pad=0.1",
                             facecolor=color, edgecolor='#e94560',
                             linewidth=2, alpha=0.9)
        ax.add_patch(box)
        ax.text(x + w/2, y + h/2 + 0.12, label,
                ha='center', va='center', fontsize=10, fontweight='bold',
                color='#ffffff')
        if sublabel:
            ax.text(x + w/2, y + h/2 - 0.2, sublabel,
                    ha='center', va='center', fontsize=7, color='#aaaaaa',
                    style='italic')

    def draw_arrow(x1, y1, x2, y2, label="", color='#54a0ff'):
        ax.annotate("", xy=(x2, y2), xytext=(x1, y1),
                    arrowprops=dict(arrowstyle='->', color=color, lw=2))
        if label:
            mx, my = (x1+x2)/2, (y1+y2)/2
            ax.text(mx, my + 0.2, label, ha='center', va='bottom',
                    fontsize=8, color=color)

    def draw_summing(x, y, r=0.2):
        circle = plt.Circle((x, y), r, fill=True, facecolor='#1a1a2e',
                            edgecolor='#e94560', linewidth=2)
        ax.add_patch(circle)
        ax.text(x, y, 'Σ', ha='center', va='center', fontsize=12,
                fontweight='bold', color='#ffffff')

    # ── Blocks ────────────────────────────────────────────────────
    # Row 1 (top): Main control loop
    y_main = 4.0
    bh = 1.0

    # Setpoint
    draw_block(0.3, y_main, 1.8, bh, "Setpoint", "roll=0°, pitch=0°", '#2d3436')

    # Summing junction
    draw_summing(2.8, y_main + bh/2)

    # PID Controller
    draw_block(3.5, y_main, 2.2, bh, "PID Controller", "Kp·e + Ki·∫e + Kd·de/dt", '#e94560')

    # Body Posture Adj
    draw_block(6.5, y_main, 2.0, bh, "Body Posture", "Rotation Matrix\n→ Foot Δz", '#0f3460')

    # IK
    draw_block(9.3, y_main, 1.8, bh, "Inverse\nKinematics", "3-DOF per leg", '#6c5ce7')

    # Joint PD
    draw_block(11.9, y_main, 1.8, bh, "Joint PD\nController", "Kp=20, Kd=0.5", '#00b894')

    # Plant (Robot)
    draw_block(11.9, y_main - 2.2, 1.8, bh, "Robot Plant", "Rigid body\n+ Servo lag", '#d63031')

    # IMU
    draw_block(6.5, y_main - 2.2, 2.0, bh, "IMU Sensor", "Roll, Pitch\n+ Noise", '#e17055')

    # Disturbance
    draw_block(9.3, y_main - 4.0, 1.8, bh, "Platform\nDisturbance", "Sinusoidal sway", '#fdcb6e')

    # ── Arrows ────────────────────────────────────────────────────
    draw_arrow(2.1, y_main + bh/2, 2.6, y_main + bh/2, "", '#54a0ff')
    draw_arrow(3.0, y_main + bh/2, 3.5, y_main + bh/2, "error", '#ff6b6b')
    draw_arrow(5.7, y_main + bh/2, 6.5, y_main + bh/2, "Δφ, Δθ", '#ffd700')
    draw_arrow(8.5, y_main + bh/2, 9.3, y_main + bh/2, "Foot pos", '#1dd1a1')
    draw_arrow(11.1, y_main + bh/2, 11.9, y_main + bh/2, "θ₁,θ₂,θ₃", COLORS['accent1'])
    draw_arrow(12.8, y_main, 12.8, y_main - 1.0, "Torques", '#00cec9')

    # Feedback path: Plant → IMU → Summing
    draw_arrow(11.9, y_main - 1.7, 8.5, y_main - 1.7, "Body state", '#fd79a8')
    draw_arrow(6.5, y_main - 1.7, 2.8, y_main - 1.7, "", '#e17055')
    draw_arrow(2.8, y_main - 1.7, 2.8, y_main + bh/2 - 0.2, "φ, θ measured", '#e17055')

    # Disturbance arrow
    draw_arrow(10.2, y_main - 3.0, 12.8, y_main - 2.2, "Sway", '#fdcb6e')

    # Plus/minus labels on summing junction
    ax.text(2.55, y_main + bh/2 + 0.15, '+', color='#2ed573', fontsize=10, fontweight='bold')
    ax.text(2.65, y_main + bh/2 - 0.35, '−', color='#ff6348', fontsize=10, fontweight='bold')

    # Legend box
    legend_text = (
        "Paper: Li et al., IEEE CASE 2022\n"
        "\"Posture Stabilization Control for a\n"
        " Quadruped Robot Walking on Swaying Platforms\"\n\n"
        "Sensors: IMU only (no force/vision)\n"
        "Motors: 12 × position-feedback servos"
    )
    props = dict(boxstyle='round,pad=0.5', facecolor='#2d3436',
                 edgecolor='#e94560', alpha=0.9)
    ax.text(1.0, 1.0, legend_text, fontsize=8, color='#aaaaaa',
            bbox=props, verticalalignment='bottom')

    _save_fig(fig, output_dir, 'block_diagram.png')


# ══════════════════════════════════════════════════════════════════
#  2. Time Response (PID scenario)
# ══════════════════════════════════════════════════════════════════

def plot_time_response(results, output_dir):
    """Plot roll/pitch time response for the PID scenario."""
    r = results['pid']
    plant = r['plant']
    dist = r['disturbance']
    t = plant['time']

    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    fig.suptitle("Full PID Controller — Time Domain Response",
                 color='#ffffff', fontsize=16)

    # Roll
    ax = axes[0]
    ax.plot(t, np.degrees(dist['roll']), '--', color=COLORS['disturbance'],
            alpha=0.6, label='Platform disturbance', linewidth=1)
    ax.plot(t, np.degrees(plant['roll']), color=COLORS['roll'],
            label='Body roll (actual)', linewidth=1.5)
    ax.axhline(0, color=COLORS['setpoint'], linestyle=':', alpha=0.3, label='Setpoint (0°)')
    ax.set_ylabel('Roll (degrees)')
    ax.legend(loc='upper right', fontsize=9)
    ax.set_title('Roll Channel')
    ax.grid(True)

    # Pitch
    ax = axes[1]
    ax.plot(t, np.degrees(dist['pitch']), '--', color=COLORS['disturbance'],
            alpha=0.6, label='Platform disturbance', linewidth=1)
    ax.plot(t, np.degrees(plant['pitch']), color=COLORS['pitch'],
            label='Body pitch (actual)', linewidth=1.5)
    ax.axhline(0, color=COLORS['setpoint'], linestyle=':', alpha=0.3, label='Setpoint (0°)')
    ax.set_ylabel('Pitch (degrees)')
    ax.set_xlabel('Time (s)')
    ax.legend(loc='upper right', fontsize=9)
    ax.set_title('Pitch Channel')
    ax.grid(True)

    fig.tight_layout(rect=[0, 0, 1, 0.94])
    _save_fig(fig, output_dir, 'time_response.png')


# ══════════════════════════════════════════════════════════════════
#  3. Comparison Plot
# ══════════════════════════════════════════════════════════════════

def plot_comparison(results, output_dir):
    """Compare no-control, P, PD, PID for roll and pitch."""
    fig, axes = plt.subplots(2, 1, figsize=(14, 9), sharex=True)
    fig.suptitle("Controller Comparison — Posture Stabilization Performance",
                 color='#ffffff', fontsize=16)

    scenarios = [
        ('no_control', 'No Control', COLORS['no_control'], 1.0),
        ('p_only',     'P-Only',     COLORS['p_only'],     1.5),
        ('pd',         'PD',         COLORS['pd'],         1.5),
        ('pid',        'PID',        COLORS['pid'],        2.0),
    ]

    for channel_idx, (channel, label) in enumerate([('roll', 'Roll'), ('pitch', 'Pitch')]):
        ax = axes[channel_idx]

        # Disturbance (same for all)
        t = results['pid']['disturbance']['time']
        ax.plot(t, np.degrees(results['pid']['disturbance'][channel]),
                '--', color=COLORS['disturbance'], alpha=0.4,
                label='Platform disturbance', linewidth=1)

        for key, name, color, lw in scenarios:
            r = results[key]
            t = r['plant']['time']
            data = np.degrees(r['plant'][channel])
            ax.plot(t, data, color=color, label=name, linewidth=lw, alpha=0.9)

        ax.axhline(0, color=COLORS['setpoint'], linestyle=':', alpha=0.3)
        ax.set_ylabel(f'{label} (degrees)')
        ax.set_title(f'{label} Channel')
        ax.legend(loc='upper right', fontsize=9, ncol=3)
        ax.grid(True)

    axes[1].set_xlabel('Time (s)')
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    _save_fig(fig, output_dir, 'comparison.png')


# ══════════════════════════════════════════════════════════════════
#  4. PID Internal Signals
# ══════════════════════════════════════════════════════════════════

def plot_pid_signals(results, output_dir):
    """Show P, I, D terms and total output for PID scenario."""
    r = results['pid']

    fig, axes = plt.subplots(4, 2, figsize=(16, 12), sharex=True)
    fig.suptitle("PID Controller Internal Signals",
                 color='#ffffff', fontsize=16)

    for col, (channel, ch_data) in enumerate([
            ('Roll', r['pid_roll']), ('Pitch', r['pid_pitch'])]):
        t = ch_data['time']

        # Error
        axes[0, col].plot(t, np.degrees(ch_data['error']), color=COLORS['roll'] if col==0 else COLORS['pitch'])
        axes[0, col].set_ylabel('Error (°)')
        axes[0, col].set_title(f'{channel} — Error Signal')
        axes[0, col].grid(True)

        # P term
        axes[1, col].plot(t, np.degrees(ch_data['p_term']), color=COLORS['p_term'], label='P')
        axes[1, col].set_ylabel('P term (°)')
        axes[1, col].set_title(f'{channel} — Proportional Term')
        axes[1, col].grid(True)

        # I term
        axes[2, col].plot(t, np.degrees(ch_data['i_term']), color=COLORS['i_term'], label='I')
        axes[2, col].set_ylabel('I term (°)')
        axes[2, col].set_title(f'{channel} — Integral Term')
        axes[2, col].grid(True)

        # D term
        axes[3, col].plot(t, np.degrees(ch_data['d_term']), color=COLORS['d_term'], label='D')
        axes[3, col].set_ylabel('D term (°)')
        axes[3, col].set_title(f'{channel} — Derivative Term')
        axes[3, col].set_xlabel('Time (s)')
        axes[3, col].grid(True)

    fig.tight_layout(rect=[0, 0, 1, 0.94])
    _save_fig(fig, output_dir, 'pid_signals.png')


# ══════════════════════════════════════════════════════════════════
#  5. IMU Signals
# ══════════════════════════════════════════════════════════════════

def plot_imu_signals(results, output_dir):
    """Show true vs noisy vs filtered IMU readings."""
    r = results['pid']
    imu = r['imu']
    t = imu['time']

    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    fig.suptitle("IMU Sensor Signals — True vs Noisy vs Filtered",
                 color='#ffffff', fontsize=16)

    for ax_idx, (ch, label) in enumerate([('roll', 'Roll'), ('pitch', 'Pitch')]):
        ax = axes[ax_idx]
        ax.plot(t, np.degrees(imu[f'true_{ch}']), color=COLORS['accent3'],
                label='True', linewidth=1.5, alpha=0.9)
        ax.plot(t, np.degrees(imu[f'noisy_{ch}']), color=COLORS['accent2'],
                label='Noisy', linewidth=0.5, alpha=0.4)
        ax.plot(t, np.degrees(imu[f'filtered_{ch}']), color=COLORS['accent1'],
                label='Filtered', linewidth=1.0, alpha=0.8)
        ax.set_ylabel(f'{label} (degrees)')
        ax.set_title(f'{label} — IMU Measurement')
        ax.legend(loc='upper right', fontsize=9)
        ax.grid(True)

    axes[1].set_xlabel('Time (s)')
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    _save_fig(fig, output_dir, 'imu_signals.png')


# ══════════════════════════════════════════════════════════════════
#  6. Step Response
# ══════════════════════════════════════════════════════════════════

def plot_step_response(results, output_dir):
    """Step disturbance response with transient metrics."""
    r = results['pid_step']
    plant = r['plant']
    dist = r['disturbance']
    t = plant['time']

    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    fig.suptitle("PID Step Response — Transient Analysis",
                 color='#ffffff', fontsize=16)

    for ax_idx, (ch, label, color) in enumerate([
            ('roll', 'Roll', COLORS['roll']),
            ('pitch', 'Pitch', COLORS['pitch'])]):
        ax = axes[ax_idx]
        ax.plot(t, np.degrees(dist[ch]), '--', color=COLORS['disturbance'],
                alpha=0.6, label='Step disturbance', linewidth=1)
        ax.plot(t, np.degrees(plant[ch]), color=color,
                label=f'Body {ch} (PID)', linewidth=1.5)
        ax.axhline(0, color=COLORS['setpoint'], linestyle=':', alpha=0.3)

        # Compute transient metrics
        data_deg = np.degrees(plant[ch])
        dist_deg = np.degrees(dist[ch])
        step_idx = np.argmax(np.array(dist[ch]) > 0.001) if np.any(np.array(dist[ch]) > 0.001) else 0

        if step_idx > 0:
            response_after_step = data_deg[step_idx:]
            peak = np.max(np.abs(response_after_step))
            steady_state = np.mean(np.abs(response_after_step[-1000:])) if len(response_after_step) > 1000 else np.nan

            # Find settling time (within 5% of steady state)
            t_arr = np.array(t)
            threshold = 0.05 * np.max(np.abs(dist_deg))
            settle_mask = np.abs(data_deg[step_idx:]) < threshold
            if np.any(settle_mask):
                settle_idx = np.argmax(settle_mask) + step_idx
                settle_time = t_arr[settle_idx] - t_arr[step_idx]
            else:
                settle_time = float('nan')

            metrics_text = (f"Peak: {peak:.3f}°\n"
                          f"Steady-state error: {steady_state:.3f}°\n"
                          f"Settling time (5%): {settle_time:.3f}s")
            props = dict(boxstyle='round,pad=0.3', facecolor='#2d3436',
                        edgecolor=color, alpha=0.9)
            ax.text(0.98, 0.95, metrics_text, transform=ax.transAxes,
                    fontsize=9, verticalalignment='top', horizontalalignment='right',
                    bbox=props, color='#eaeaea')

        ax.set_ylabel(f'{label} (degrees)')
        ax.set_title(f'{label} — Step Response')
        ax.legend(loc='upper left', fontsize=9)
        ax.grid(True)

    axes[1].set_xlabel('Time (s)')
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    _save_fig(fig, output_dir, 'step_response.png')


# ══════════════════════════════════════════════════════════════════
#  7. Combined Disturbance Response
# ══════════════════════════════════════════════════════════════════

def plot_combined_response(results, output_dir):
    """Combined sinusoidal + step disturbance response."""
    r = results['pid_combined']
    plant = r['plant']
    dist = r['disturbance']
    t = plant['time']

    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    fig.suptitle("PID Response — Combined (Sinusoidal + Step) Disturbance",
                 color='#ffffff', fontsize=16)

    for ax_idx, (ch, label, color) in enumerate([
            ('roll', 'Roll', COLORS['roll']),
            ('pitch', 'Pitch', COLORS['pitch'])]):
        ax = axes[ax_idx]
        ax.plot(t, np.degrees(dist[ch]), '--', color=COLORS['disturbance'],
                alpha=0.6, label='Combined disturbance', linewidth=1)
        ax.plot(t, np.degrees(plant[ch]), color=color,
                label=f'Body {ch} (PID)', linewidth=1.5)
        ax.axhline(0, color=COLORS['setpoint'], linestyle=':', alpha=0.3)
        ax.set_ylabel(f'{label} (degrees)')
        ax.set_title(f'{label} — Combined Disturbance Response')
        ax.legend(loc='upper right', fontsize=9)
        ax.grid(True)

    axes[1].set_xlabel('Time (s)')
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    _save_fig(fig, output_dir, 'combined_response.png')


# ══════════════════════════════════════════════════════════════════
#  8. Joint Angles
# ══════════════════════════════════════════════════════════════════

def plot_joint_angles(results, output_dir):
    """Plot joint angle trajectories from the body posture adjustment."""
    r = results['pid']
    posture = r['posture']
    t = posture['time']

    legs = ["left-front", "left-behind", "right-front", "right-behind"]
    leg_colors = [COLORS['roll'], COLORS['pitch'], COLORS['accent1'], COLORS['accent3']]

    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    fig.suptitle("Joint Angle Trajectories — Posture Correction",
                 color='#ffffff', fontsize=16)

    joint_labels = ['θ₁ (Hip)', 'θ₂ (Upper Leg)', 'θ₃ (Lower Leg)']

    for j_idx, j_label in enumerate(joint_labels):
        ax = axes[j_idx]
        for i, (leg, color) in enumerate(zip(legs, leg_colors)):
            key = f'{leg}_theta{j_idx+1}'
            ax.plot(t, np.degrees(posture[key]), color=color,
                    label=leg, linewidth=1, alpha=0.8)
        ax.set_ylabel(f'{j_label} (degrees)')
        ax.set_title(j_label)
        ax.legend(loc='upper right', fontsize=8, ncol=2)
        ax.grid(True)

    axes[2].set_xlabel('Time (s)')
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    _save_fig(fig, output_dir, 'joint_angles.png')


# ══════════════════════════════════════════════════════════════════
#  9. Foot Height Adjustments
# ══════════════════════════════════════════════════════════════════

def plot_foot_adjustments(results, output_dir):
    """Plot foot height adjustments (Δz) for each leg."""
    r = results['pid']
    posture = r['posture']
    t = posture['time']

    legs = ["left-front", "left-behind", "right-front", "right-behind"]
    leg_colors = [COLORS['roll'], COLORS['pitch'], COLORS['accent1'], COLORS['accent3']]

    fig, ax = plt.subplots(1, 1, figsize=(14, 6))
    fig.suptitle("Foot Height Adjustments (Δz) — Posture Compensation",
                 color='#ffffff', fontsize=16)

    for leg, color in zip(legs, leg_colors):
        dz = posture[f'{leg}_dz'] * 1000  # convert to mm
        ax.plot(t, dz, color=color, label=leg, linewidth=1.2, alpha=0.9)

    ax.set_ylabel('Δz (mm)')
    ax.set_xlabel('Time (s)')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True)
    ax.set_title('Each leg adjusts height to compensate body tilt')

    fig.tight_layout(rect=[0, 0, 1, 0.94])
    _save_fig(fig, output_dir, 'foot_adjustments.png')


# ══════════════════════════════════════════════════════════════════
#  10. Body Animation (GIF)
# ══════════════════════════════════════════════════════════════════

def plot_body_animation(results, output_dir):
    """Create animated visualization of body tilt stabilization."""
    r = results['pid']
    plant = r['plant']
    dist = r['disturbance']
    t_all = plant['time']

    # Downsample for animation (every 50th step = 20 FPS at 1kHz sim)
    step = 50
    t = t_all[::step]
    roll_deg = np.degrees(plant['roll'][::step])
    pitch_deg = np.degrees(plant['pitch'][::step])
    dist_roll_deg = np.degrees(dist['roll'][::step])
    dist_pitch_deg = np.degrees(dist['pitch'][::step])

    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    fig.suptitle("Body Posture Stabilization Animation",
                 color='#ffffff', fontsize=16)

    # Axis 0: Front view (roll)
    ax_front = axes[0]
    ax_front.set_xlim(-0.2, 0.2)
    ax_front.set_ylim(-0.05, 0.25)
    ax_front.set_aspect('equal')
    ax_front.set_title('Front View (Roll)', fontsize=12)
    ax_front.set_xlabel('Y (m)')
    ax_front.set_ylabel('Z (m)')
    ax_front.grid(True)

    # Axis 1: Side view (pitch)
    ax_side = axes[1]
    ax_side.set_xlim(-0.25, 0.25)
    ax_side.set_ylim(-0.05, 0.25)
    ax_side.set_aspect('equal')
    ax_side.set_title('Side View (Pitch)', fontsize=12)
    ax_side.set_xlabel('X (m)')
    ax_side.set_ylabel('Z (m)')
    ax_side.grid(True)

    # Axis 2: Time trace
    ax_trace = axes[2]
    ax_trace.set_xlim(0, t[-1])
    trace_max = max(np.max(np.abs(roll_deg)), np.max(np.abs(pitch_deg)),
                    np.max(np.abs(dist_roll_deg)), np.max(np.abs(dist_pitch_deg)))
    ax_trace.set_ylim(-trace_max * 1.3, trace_max * 1.3)
    ax_trace.set_title('Orientation vs Time', fontsize=12)
    ax_trace.set_xlabel('Time (s)')
    ax_trace.set_ylabel('Angle (°)')
    ax_trace.grid(True)

    # Body dimensions
    body_w = 0.222
    body_h_vis = 0.081
    body_l = 0.350
    nom_h = 0.170

    # Initialize artists
    body_front, = ax_front.plot([], [], 's-', color=COLORS['pid'], linewidth=4,
                                 markersize=8, solid_capstyle='round')
    legs_front_l, = ax_front.plot([], [], '-', color=COLORS['accent3'], linewidth=2)
    legs_front_r, = ax_front.plot([], [], '-', color=COLORS['accent3'], linewidth=2)
    platform_front, = ax_front.plot([], [], '-', color=COLORS['disturbance'],
                                     linewidth=3, alpha=0.5)

    body_side, = ax_side.plot([], [], 's-', color=COLORS['pid'], linewidth=4,
                               markersize=8, solid_capstyle='round')
    legs_side_f, = ax_side.plot([], [], '-', color=COLORS['accent3'], linewidth=2)
    legs_side_b, = ax_side.plot([], [], '-', color=COLORS['accent3'], linewidth=2)
    platform_side, = ax_side.plot([], [], '-', color=COLORS['disturbance'],
                                   linewidth=3, alpha=0.5)

    trace_roll, = ax_trace.plot([], [], color=COLORS['roll'], label='Body Roll', linewidth=1.5)
    trace_pitch, = ax_trace.plot([], [], color=COLORS['pitch'], label='Body Pitch', linewidth=1.5)
    trace_dist_r, = ax_trace.plot([], [], '--', color=COLORS['disturbance'],
                                   alpha=0.5, label='Dist Roll', linewidth=1)
    trace_dist_p, = ax_trace.plot([], [], ':', color=COLORS['disturbance'],
                                   alpha=0.5, label='Dist Pitch', linewidth=1)
    time_marker = ax_trace.axvline(0, color='#ffffff', alpha=0.5, linewidth=1)
    ax_trace.legend(loc='upper right', fontsize=8)

    time_text = fig.text(0.5, 0.02, '', ha='center', fontsize=12, color='#ffd700')

    def init():
        return (body_front, legs_front_l, legs_front_r, platform_front,
                body_side, legs_side_f, legs_side_b, platform_side,
                trace_roll, trace_pitch, trace_dist_r, trace_dist_p,
                time_marker, time_text)

    def animate(frame):
        roll_rad = np.radians(roll_deg[frame])
        pitch_rad = np.radians(pitch_deg[frame])

        # Front view: body tilted by roll
        cos_r = np.cos(roll_rad)
        sin_r = np.sin(roll_rad)
        # Body endpoints
        y_l = -body_w/2 * cos_r
        z_l = nom_h - body_w/2 * sin_r
        y_r = body_w/2 * cos_r
        z_r = nom_h + body_w/2 * sin_r
        body_front.set_data([y_l, y_r], [z_l, z_r])

        # Legs (front view)
        legs_front_l.set_data([y_l, y_l], [z_l, 0])
        legs_front_r.set_data([y_r, y_r], [z_r, 0])

        # Platform (ground line, tilted)
        p_roll = np.radians(dist_roll_deg[frame])
        py_l = -0.18 * np.cos(p_roll)
        pz_l = -0.18 * np.sin(p_roll)
        py_r = 0.18 * np.cos(p_roll)
        pz_r = 0.18 * np.sin(p_roll)
        platform_front.set_data([py_l, py_r], [pz_l, pz_r])

        # Side view: body tilted by pitch
        cos_p = np.cos(pitch_rad)
        sin_p = np.sin(pitch_rad)
        x_f = body_l/2 * cos_p
        z_f = nom_h + body_l/2 * sin_p
        x_b = -body_l/2 * cos_p
        z_b = nom_h - body_l/2 * sin_p
        body_side.set_data([x_b, x_f], [z_b, z_f])

        # Legs (side view)
        legs_side_f.set_data([x_f, x_f], [z_f, 0])
        legs_side_b.set_data([x_b, x_b], [z_b, 0])

        # Platform (side view)
        p_pitch = np.radians(dist_pitch_deg[frame])
        px_f = 0.22 * np.cos(p_pitch)
        pz_f = 0.22 * np.sin(p_pitch)
        px_b = -0.22 * np.cos(p_pitch)
        pz_b = -0.22 * np.sin(p_pitch)
        platform_side.set_data([px_b, px_f], [pz_b, pz_f])

        # Time traces
        trace_roll.set_data(t[:frame+1], roll_deg[:frame+1])
        trace_pitch.set_data(t[:frame+1], pitch_deg[:frame+1])
        trace_dist_r.set_data(t[:frame+1], dist_roll_deg[:frame+1])
        trace_dist_p.set_data(t[:frame+1], dist_pitch_deg[:frame+1])
        time_marker.set_xdata([t[frame]])

        time_text.set_text(f't = {t[frame]:.2f}s  |  Roll: {roll_deg[frame]:+.2f}°  |  Pitch: {pitch_deg[frame]:+.2f}°')

        return (body_front, legs_front_l, legs_front_r, platform_front,
                body_side, legs_side_f, legs_side_b, platform_side,
                trace_roll, trace_pitch, trace_dist_r, trace_dist_p,
                time_marker, time_text)

    n_frames = len(t)
    anim = animation.FuncAnimation(fig, animate, init_func=init,
                                    frames=n_frames, interval=50, blit=True)

    fig.tight_layout(rect=[0, 0.05, 1, 0.94])

    gif_path = os.path.join(output_dir, 'body_animation.gif')
    os.makedirs(output_dir, exist_ok=True)
    anim.save(gif_path, writer='pillow', fps=20,
              savefig_kwargs={'facecolor': fig.get_facecolor()})
    plt.close(fig)
    print(f"  ✓ Saved: body_animation.gif")


# ══════════════════════════════════════════════════════════════════
#  11. Performance Summary Table (as image)
# ══════════════════════════════════════════════════════════════════

def plot_performance_summary(results, output_dir):
    """Create a summary table image comparing all controllers."""
    fig, ax = plt.subplots(1, 1, figsize=(14, 5))
    ax.axis('off')
    fig.suptitle("Performance Summary — Controller Comparison",
                 color='#ffffff', fontsize=16)

    scenarios = ['no_control', 'p_only', 'pd', 'pid']
    names = ['No Control', 'P-Only', 'PD', 'Full PID']
    headers = ['Controller', 'Roll RMS (°)', 'Roll Peak (°)',
               'Pitch RMS (°)', 'Pitch Peak (°)', 'Status']

    cell_data = []
    for key, name in zip(scenarios, names):
        r = results[key]
        plant = r['plant']
        roll_deg = np.degrees(plant['roll'])
        pitch_deg = np.degrees(plant['pitch'])

        # Skip first 0.5s for metrics (transient)
        skip = int(0.5 / RobotParams().dt)
        roll_ss = roll_deg[skip:]
        pitch_ss = pitch_deg[skip:]

        roll_rms = np.sqrt(np.mean(roll_ss**2))
        roll_peak = np.max(np.abs(roll_ss))
        pitch_rms = np.sqrt(np.mean(pitch_ss**2))
        pitch_peak = np.max(np.abs(pitch_ss))

        status = '✅ < 1°' if roll_peak < 1 and pitch_peak < 1 else '⚠️ > 1°'

        cell_data.append([name, f'{roll_rms:.3f}', f'{roll_peak:.3f}',
                         f'{pitch_rms:.3f}', f'{pitch_peak:.3f}', status])

    table = ax.table(cellText=cell_data, colLabels=headers,
                     cellLoc='center', loc='center')

    # Style the table
    table.auto_set_font_size(False)
    table.set_fontsize(11)
    table.scale(1, 2.0)

    for (row, col), cell in table.get_celld().items():
        cell.set_edgecolor('#e94560')
        if row == 0:
            cell.set_facecolor('#e94560')
            cell.set_text_props(color='#ffffff', fontweight='bold')
        else:
            cell.set_facecolor('#16213e')
            cell.set_text_props(color='#eaeaea')

    fig.tight_layout(rect=[0, 0, 1, 0.9])
    _save_fig(fig, output_dir, 'performance_summary.png')


# ══════════════════════════════════════════════════════════════════
#  Generate All Plots
# ══════════════════════════════════════════════════════════════════

def generate_all_plots(results, output_dir):
    """Generate all visualization plots."""
    print(f"\n  Output directory: {output_dir}")
    os.makedirs(output_dir, exist_ok=True)

    plot_block_diagram(results, output_dir)
    plot_time_response(results, output_dir)
    plot_comparison(results, output_dir)
    plot_pid_signals(results, output_dir)
    plot_imu_signals(results, output_dir)
    plot_step_response(results, output_dir)
    plot_combined_response(results, output_dir)
    plot_joint_angles(results, output_dir)
    plot_foot_adjustments(results, output_dir)
    plot_performance_summary(results, output_dir)

    print("\n  Generating animation (this may take a moment)...")
    plot_body_animation(results, output_dir)

    print(f"\n  ✅ All {11} visualizations generated successfully!")
