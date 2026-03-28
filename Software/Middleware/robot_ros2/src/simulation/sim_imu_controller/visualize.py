"""
Visualization Module
====================
Generates professional, publication-quality plots for the posture
stabilization simulation.

Outputs (7 files):
    1. block_diagram.png      — Simulink-style control block diagram
    2. step_response.png      — 3-scenario body orientation comparison
    3. pid_signals.png        — P/I/D terms breakdown for HOME scenario
    4. filter_effect.png      — Raw PID vs low-pass filtered output
    5. correction_output.png  — Final applied correction with deadzone visible
    6. foot_adjustments.png   — Per-leg Δz corrections
    7. signal_flow.gif        — Animated signal pipeline
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation

# ── Plot style (dark, professional) ──────────────────────────────
plt.rcParams.update({
    'figure.facecolor': '#0f1923',
    'axes.facecolor':   '#162231',
    'axes.labelcolor':  '#c8d6e5',
    'axes.edgecolor':   '#2c3e50',
    'text.color':       '#c8d6e5',
    'xtick.color':      '#8395a7',
    'ytick.color':      '#8395a7',
    'grid.color':       '#2c3e50',
    'grid.alpha':       0.6,
    'legend.facecolor': '#1e2d3d',
    'legend.edgecolor': '#2c3e50',
    'font.family':      'sans-serif',
    'font.size':        10,
})

C = {  # curated color palette
    'cyan':    '#00d2ff',
    'red':     '#ff6b6b',
    'green':   '#6bcb77',
    'orange':  '#ff9f43',
    'yellow':  '#ffd93d',
    'white':   '#ecf0f1',
    'grey':    '#576574',
    'pink':    '#ff6b9d',
}


def generate_all(results, output_dir):
    """Generate all 7 output files."""
    os.makedirs(output_dir, exist_ok=True)

    plot_block_diagram(output_dir)
    plot_step_response(results, output_dir)
    plot_pid_signals(results['home'], output_dir)
    plot_filter_effect(results['home'], output_dir)
    plot_correction_output(results, output_dir)
    plot_foot_adjustments(results['home'], output_dir)
    generate_signal_flow_gif(results['home'], output_dir)


# ══════════════════════════════════════════════════════════════════
#  1. BLOCK DIAGRAM (Simulink-style)
# ══════════════════════════════════════════════════════════════════

def plot_block_diagram(output_dir):
    """Draw a Simulink-style control block diagram."""
    fig, ax = plt.subplots(figsize=(16, 5))
    ax.set_xlim(-1, 17)
    ax.set_ylim(-2.5, 3)
    ax.set_aspect('equal')
    ax.axis('off')
    fig.patch.set_facecolor('#0f1923')

    # ── Helper functions ──────────────────────────────────────────
    def block(x, y, w, h, label, sub='', color='#1e88e5'):
        rect = patches.FancyBboxPatch(
            (x, y - h/2), w, h, boxstyle='round,pad=0.1',
            facecolor=color, edgecolor='#ecf0f1', linewidth=1.5, alpha=0.9)
        ax.add_patch(rect)
        ax.text(x + w/2, y + 0.08, label, ha='center', va='center',
                fontsize=9, fontweight='bold', color='white')
        if sub:
            ax.text(x + w/2, y - 0.28, sub, ha='center', va='center',
                    fontsize=7, color='#b0bec5', style='italic')

    def summing(x, y, r=0.22):
        circle = plt.Circle((x, y), r, facecolor='#162231',
                             edgecolor='#ecf0f1', linewidth=1.5)
        ax.add_patch(circle)
        ax.text(x, y, '⊕', ha='center', va='center', fontsize=14,
                color='white', fontweight='bold')

    def arrow(x1, y1, x2, y2, label=''):
        ax.annotate('', xy=(x2, y2), xytext=(x1, y1),
                    arrowprops=dict(arrowstyle='->', color='#ecf0f1',
                                   lw=1.5, connectionstyle='arc3,rad=0'))
        if label:
            mx, my = (x1 + x2) / 2, (y1 + y2) / 2 + 0.25
            ax.text(mx, my, label, ha='center', va='center',
                    fontsize=7, color='#8395a7')

    # ── Title ─────────────────────────────────────────────────────
    ax.text(8, 2.6, 'Posture Stabilization — Control Block Diagram',
            ha='center', va='center', fontsize=13, fontweight='bold',
            color='#ecf0f1')

    # ── Signal labels ─────────────────────────────────────────────
    y = 0.5

    # Reference input r=0
    ax.text(-0.5, y, 'r = 0', ha='center', va='center', fontsize=10,
            color=C['green'], fontweight='bold')
    arrow(-0.1, y, 0.6, y)

    # Summing junction
    summing(1.0, y)
    ax.text(1.0, y + 0.45, 'e = r − y', ha='center', fontsize=7, color='#8395a7')

    # PID block
    arrow(1.22, y, 2.0, y, '')
    block(2.0, y, 1.5, 0.8, 'PID', 'Kp=1.5  Ki=1.5  Kd=0.3', '#1e88e5')

    # Saturation block
    arrow(3.5, y, 4.2, y, 'u(t)')
    block(4.2, y, 1.3, 0.8, 'Saturation', '±0.5 rad', '#e53935')

    # LPF block
    arrow(5.5, y, 6.2, y, '')
    block(6.2, y, 1.3, 0.8, 'LPF', 'α = 0.85', '#ff9800')

    # Dead zone block
    arrow(7.5, y, 8.3, y, '')
    block(8.3, y, 1.5, 0.8, 'Dead Zone', 'HOME: 0.005 rad\nGAIT: 0.08 rad', '#7b1fa2')

    # Gain block
    arrow(9.8, y, 10.5, y, '')
    block(10.5, y, 1.3, 0.8, 'Gain (K)', 'HOME: 1.8\nGAIT: 0.5', '#00897b')

    # Plant block
    arrow(11.8, y, 12.5, y, 'u_corr')
    block(12.5, y, 1.5, 0.8, 'Plant', 'Rigid Body\nI·α = Στ', '#455a64')

    # Output
    arrow(14.0, y, 14.8, y, '')
    ax.text(15.2, y, 'y (φ, θ)', ha='center', va='center', fontsize=10,
            color=C['cyan'], fontweight='bold')

    # ── Feedback path ─────────────────────────────────────────────
    # From output down and back to summing junction
    fb_y = -1.2
    ax.plot([14.5, 14.5], [y, fb_y], color='#ecf0f1', lw=1.5)
    ax.plot([14.5, 1.0], [fb_y, fb_y], color='#ecf0f1', lw=1.5)

    # IMU sensor block on feedback path
    block(7.0, fb_y, 1.5, 0.7, 'IMU Sensor', 'σ = 0.002 rad', '#6d4c41')

    # Negative sign at summing junction
    ax.annotate('', xy=(1.0, y - 0.22), xytext=(1.0, fb_y + 0.35),
                arrowprops=dict(arrowstyle='->', color='#ecf0f1', lw=1.5))
    ax.text(0.7, fb_y + 0.7, '−', fontsize=14, color=C['red'], fontweight='bold')

    # ── Disturbance input ─────────────────────────────────────────
    dist_x = 13.25
    ax.text(dist_x, 2.0, 'd(t)', ha='center', va='center', fontsize=10,
            color=C['yellow'], fontweight='bold')
    ax.text(dist_x, 1.6, 'Ramp\n5°/8°', ha='center', va='center',
            fontsize=7, color='#8395a7')
    ax.annotate('', xy=(dist_x, y + 0.4), xytext=(dist_x, 1.3),
                arrowprops=dict(arrowstyle='->', color=C['yellow'], lw=1.5))

    fig.tight_layout()
    path = os.path.join(output_dir, 'block_diagram.png')
    fig.savefig(path, dpi=200, bbox_inches='tight')
    plt.close(fig)
    print(f"  ✓ {path}")


# ══════════════════════════════════════════════════════════════════
#  2. STEP RESPONSE (3-scenario comparison)
# ══════════════════════════════════════════════════════════════════

def plot_step_response(results, output_dir):
    """Plot body orientation for all 3 scenarios on same axes."""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 7), sharex=True)
    fig.suptitle('Step Response — Ramp Disturbance at t = 3 s',
                 fontsize=14, fontweight='bold')

    styles = {
        'no_control': (C['grey'],   '--', 'No Control'),
        'home':       (C['cyan'],   '-',  'HOME (θ₃ Offsets, K=1.8)'),
        'gait':       (C['green'],  '-',  'GAIT (Rotation, K=0.5)'),
    }

    for key, (color, ls, label) in styles.items():
        p = results[key]['plant']
        d = results[key]['disturbance']
        ax1.plot(p['time'], np.degrees(p['roll']), color=color, ls=ls,
                 label=label, linewidth=1.2)
        ax2.plot(p['time'], np.degrees(p['pitch']), color=color, ls=ls,
                 label=label, linewidth=1.2)

    # Disturbance reference (from any scenario — same for all)
    d = results['no_control']['disturbance']
    t = results['no_control']['plant']['time']
    ax1.plot(t, np.degrees(d['roll']), ':', color=C['yellow'],
             alpha=0.5, label='Disturbance', linewidth=1)
    ax2.plot(t, np.degrees(d['pitch']), ':', color=C['yellow'],
             alpha=0.5, label='Disturbance', linewidth=1)

    for ax, ylabel in [(ax1, 'Roll (°)'), (ax2, 'Pitch (°)')]:
        ax.axhline(0, color=C['white'], ls=':', alpha=0.2)
        ax.axvline(3.0, color=C['yellow'], ls=':', alpha=0.3, label='_')
        ax.set_ylabel(ylabel)
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True)

    ax2.set_xlabel('Time (s)')
    fig.tight_layout()
    path = os.path.join(output_dir, 'step_response.png')
    fig.savefig(path, dpi=200)
    plt.close(fig)
    print(f"  ✓ {path}")


# ══════════════════════════════════════════════════════════════════
#  3. PID SIGNALS (P/I/D terms for HOME)
# ══════════════════════════════════════════════════════════════════

def plot_pid_signals(data, output_dir):
    """Plot P, I, D terms and total output for both channels."""
    fig, axes = plt.subplots(2, 2, figsize=(14, 8))
    fig.suptitle(f'PID Signal Decomposition — {data["name"]}',
                 fontsize=14, fontweight='bold')

    channels = [('Roll', data['pid_roll'], C['cyan']),
                ('Pitch', data['pid_pitch'], C['red'])]

    for row, (ch, pid, color) in enumerate(channels):
        t = pid['time']

        # P, I, D terms
        ax = axes[row, 0]
        ax.plot(t, pid['p_term'], color=C['cyan'],  lw=0.8, label='P')
        ax.plot(t, pid['i_term'], color=C['red'],   lw=0.8, label='I')
        ax.plot(t, pid['d_term'], color=C['green'],  lw=0.8, label='D')
        ax.set_ylabel(f'{ch} — PID Terms')
        ax.legend(fontsize=7, loc='upper right')
        ax.grid(True)
        ax.axvline(3.0, color=C['yellow'], ls=':', alpha=0.3)

        # Total output
        ax = axes[row, 1]
        ax.plot(t, pid['output'], color=color, lw=1)
        ax.axhline(0.5, ls='--', color=C['red'], alpha=0.4, lw=0.8)
        ax.axhline(-0.5, ls='--', color=C['red'], alpha=0.4, lw=0.8)
        ax.set_ylabel(f'{ch} — Output (rad)')
        ax.grid(True)
        ax.axvline(3.0, color=C['yellow'], ls=':', alpha=0.3)
        ax.text(t[-1] * 0.95, 0.45, 'Saturation ±0.5', fontsize=7,
                ha='right', color=C['red'], alpha=0.6)

    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 1].set_xlabel('Time (s)')
    fig.tight_layout()
    path = os.path.join(output_dir, 'pid_signals.png')
    fig.savefig(path, dpi=200)
    plt.close(fig)
    print(f"  ✓ {path}")


# ══════════════════════════════════════════════════════════════════
#  4. FILTER EFFECT (raw vs filtered)
# ══════════════════════════════════════════════════════════════════

def plot_filter_effect(data, output_dir):
    """Show raw PID output vs low-pass filtered output."""
    filt = data['filter']
    t = filt['time']

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 6), sharex=True)
    fig.suptitle(f'Low-Pass Filter Effect (α = 0.85) — {data["name"]}',
                 fontsize=14, fontweight='bold')

    for ax, raw, filtered, ch, color in [
        (ax1, filt['raw_r'], filt['filt_r'], 'Roll', C['cyan']),
        (ax2, filt['raw_p'], filt['filt_p'], 'Pitch', C['red']),
    ]:
        ax.plot(t, raw, color=color, alpha=0.3, lw=0.5, label='Raw PID')
        ax.plot(t, filtered, color=C['orange'], lw=1.2, label='Filtered (α=0.85)')
        ax.set_ylabel(f'{ch} (rad)')
        ax.legend(fontsize=8, loc='upper right')
        ax.grid(True)
        ax.axvline(3.0, color=C['yellow'], ls=':', alpha=0.3)

    ax2.set_xlabel('Time (s)')
    fig.tight_layout()
    path = os.path.join(output_dir, 'filter_effect.png')
    fig.savefig(path, dpi=200)
    plt.close(fig)
    print(f"  ✓ {path}")


# ══════════════════════════════════════════════════════════════════
#  5. CORRECTION OUTPUT (HOME vs GAIT with dead zones)
# ══════════════════════════════════════════════════════════════════

def plot_correction_output(results, output_dir):
    """Show final applied correction for HOME and GAIT modes."""
    fig, axes = plt.subplots(2, 2, figsize=(14, 8))
    fig.suptitle('Applied Correction Output — HOME vs GAIT',
                 fontsize=14, fontweight='bold')

    for col, (key, title, color) in enumerate([
        ('home', 'HOME (K=1.8, DZ=0.005)', C['cyan']),
        ('gait', 'GAIT (K=0.5, DZ=0.08)', C['green']),
    ]):
        filt = results[key]['filter']
        t = filt['time']

        axes[0, col].plot(t, filt['corr_r'], color=color, lw=0.8)
        axes[0, col].set_title(f'{title} — Roll', fontsize=10)
        axes[0, col].set_ylabel('Correction (rad)')
        axes[0, col].grid(True)
        axes[0, col].axvline(3.0, color=C['yellow'], ls=':', alpha=0.3)

        axes[1, col].plot(t, filt['corr_p'], color=color, lw=0.8)
        axes[1, col].set_title(f'{title} — Pitch', fontsize=10)
        axes[1, col].set_ylabel('Correction (rad)')
        axes[1, col].set_xlabel('Time (s)')
        axes[1, col].grid(True)
        axes[1, col].axvline(3.0, color=C['yellow'], ls=':', alpha=0.3)

    fig.tight_layout()
    path = os.path.join(output_dir, 'correction_output.png')
    fig.savefig(path, dpi=200)
    plt.close(fig)
    print(f"  ✓ {path}")


# ══════════════════════════════════════════════════════════════════
#  6. FOOT ADJUSTMENTS (Δz per leg)
# ══════════════════════════════════════════════════════════════════

def plot_foot_adjustments(data, output_dir):
    """Show Δz height adjustment per leg for HOME scenario."""
    posture = data['posture']
    t = posture['time']

    fig, axes = plt.subplots(2, 2, figsize=(14, 7))
    fig.suptitle(f'Foot Height Adjustments (Δz) — {data["name"]}',
                 fontsize=14, fontweight='bold')

    legs = ['left-front', 'left-behind', 'right-front', 'right-behind']
    colors = [C['cyan'], C['red'], C['green'], C['orange']]

    for idx, (leg, color) in enumerate(zip(legs, colors)):
        ax = axes[idx // 2, idx % 2]
        key = f'{leg}_dz'
        if key in posture:
            dz_mm = np.array(posture[key]) * 1000
            ax.plot(t, dz_mm, color=color, lw=0.8)
        ax.set_title(leg.replace('-', ' ').title(), fontsize=10)
        ax.set_ylabel('Δz (mm)')
        ax.grid(True)
        ax.axhline(0, color=C['white'], ls=':', alpha=0.2)
        ax.axvline(3.0, color=C['yellow'], ls=':', alpha=0.3)

    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 1].set_xlabel('Time (s)')
    fig.tight_layout()
    path = os.path.join(output_dir, 'foot_adjustments.png')
    fig.savefig(path, dpi=200)
    plt.close(fig)
    print(f"  ✓ {path}")


# ══════════════════════════════════════════════════════════════════
#  7. SIGNAL FLOW ANIMATION
# ══════════════════════════════════════════════════════════════════

def generate_signal_flow_gif(data, output_dir, fps=25, duration=10.0):
    """Animate the full signal pipeline over time."""
    plant = data['plant']
    dist  = data['disturbance']
    imu_d = data['imu']
    pid_r = data['pid_roll']
    pid_p = data['pid_pitch']
    filt  = data['filter']

    t_all = plant['time']
    n_frames = int(fps * duration)
    step = max(1, len(t_all) // n_frames)
    indices = list(range(0, len(t_all), step))[:n_frames]

    fig, axes = plt.subplots(5, 1, figsize=(14, 11), sharex=True)
    fig.suptitle(f'Signal Flow Pipeline — {data["name"]}',
                 fontsize=13, fontweight='bold')

    labels = ['Disturbance\nd(t)', 'IMU\nMeasurement', 'PID\nOutput u(t)',
              'LPF\nFiltered', 'Body\nOrientation y(t)']

    def update(frame_idx):
        idx = indices[frame_idx]
        sl = slice(0, idx + 1)

        for i, ax in enumerate(axes):
            ax.clear()
            ax.grid(True)
            ax.set_xlim(0, t_all[-1])
            ax.set_ylabel(labels[i], fontsize=8, rotation=0,
                          labelpad=60, va='center')

        # 1. Disturbance
        axes[0].plot(t_all[sl], np.degrees(dist['roll'][sl]), color=C['cyan'], lw=0.8)
        axes[0].plot(t_all[sl], np.degrees(dist['pitch'][sl]), color=C['red'], lw=0.8)
        axes[0].set_ylim(-12, 12)
        axes[0].set_title(f't = {t_all[idx]:.1f} s', fontsize=10)

        # 2. IMU
        axes[1].plot(t_all[sl], np.degrees(imu_d['filtered_roll'][sl]), color=C['cyan'], lw=0.8)
        axes[1].plot(t_all[sl], np.degrees(imu_d['filtered_pitch'][sl]), color=C['red'], lw=0.8)
        axes[1].set_ylim(-12, 12)

        # 3. PID output
        axes[2].plot(t_all[sl], pid_r['output'][sl], color=C['cyan'], lw=0.8)
        axes[2].plot(t_all[sl], pid_p['output'][sl], color=C['red'], lw=0.8)
        axes[2].set_ylim(-0.6, 0.6)

        # 4. Filtered
        axes[3].plot(t_all[sl], filt['filt_r'][sl], color=C['cyan'], lw=0.8)
        axes[3].plot(t_all[sl], filt['filt_p'][sl], color=C['red'], lw=0.8)
        axes[3].set_ylim(-0.6, 0.6)

        # 5. Body orientation
        axes[4].plot(t_all[sl], np.degrees(plant['roll'][sl]), color=C['cyan'], lw=1)
        axes[4].plot(t_all[sl], np.degrees(plant['pitch'][sl]), color=C['red'], lw=1)
        axes[4].axhline(0, color=C['white'], ls=':', alpha=0.2)
        axes[4].set_ylim(-12, 12)
        axes[4].set_xlabel('Time (s)')

        return []

    anim = animation.FuncAnimation(fig, update, frames=len(indices),
                                   interval=1000 // fps, blit=False)
    path = os.path.join(output_dir, 'signal_flow.gif')
    anim.save(path, writer='pillow', fps=fps, dpi=100)
    plt.close(fig)
    print(f"  ✓ {path}")
