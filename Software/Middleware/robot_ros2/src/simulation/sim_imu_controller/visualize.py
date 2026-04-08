"""
Visualization Module — Professional Edition
============================================
Publication-quality static plots AND animated GIFs for the posture
stabilization simulation.

Static outputs (7):
    1. block_diagram.png        — Simulink-style control block diagram
    2. step_response.png        — 3-scenario comparison with metrics
    3. pid_signals_home.png     — HOME PID decomposition (P/I/D/Σ/Sat/LPF)
    4. pid_signals_gait.png     — GAIT PID decomposition (adaptive)
    5. filter_pipeline.png      — Signal pipeline stages (Raw → Sat → LPF)
    6. correction_output.png    — Applied corrections (HOME vs GAIT)
    7. foot_adjustments.png     — Per-leg Δz corrections

Animated outputs (3):
    8. signal_flow.gif          — Full signal pipeline animation
    9. step_response.gif        — Animated scenario comparison
   10. pid_decomposition.gif    — P/I/D term buildup animation
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation

# ══════════════════════════════════════════════════════════════════
#  Global Style Configuration
# ══════════════════════════════════════════════════════════════════

plt.rcParams.update({
    'figure.facecolor':   '#0d1117',
    'axes.facecolor':     '#161b22',
    'axes.labelcolor':    '#c9d1d9',
    'axes.edgecolor':     '#30363d',
    'text.color':         '#c9d1d9',
    'xtick.color':        '#8b949e',
    'ytick.color':        '#8b949e',
    'grid.color':         '#21262d',
    'grid.alpha':         0.7,
    'grid.linestyle':     '--',
    'grid.linewidth':     0.5,
    'legend.facecolor':   '#161b22',
    'legend.edgecolor':   '#30363d',
    'legend.fontsize':    8,
    'font.family':        'sans-serif',
    'font.size':          10,
    'axes.titlesize':     11,
    'axes.labelsize':     10,
    'figure.titlesize':   14,
})

# Curated color palette — accessible, harmonious
C = {
    'cyan':     '#58a6ff',
    'blue':     '#388bfd',
    'green':    '#3fb950',
    'red':      '#f85149',
    'orange':   '#d29922',
    'yellow':   '#e3b341',
    'purple':   '#bc8cff',
    'pink':     '#f778ba',
    'white':    '#e6edf3',
    'grey':     '#8b949e',
    'dim':      '#484f58',
    'accent':   '#79c0ff',
}


# ══════════════════════════════════════════════════════════════════
#  Helper Functions — Metrics & Annotations
# ══════════════════════════════════════════════════════════════════

def _ss_value(sig, t, t_start=20.0):
    """Compute steady-state mean value (average over t > t_start)."""
    mask = t >= t_start
    if np.sum(mask) == 0:
        return sig[-1]
    return np.mean(sig[mask])


def _settling_time(sig, t, ss, band_pct=0.05, t_dist=3.0):
    """Find settling time = first t after disturbance where signal stays
    within ±band_pct of steady-state value permanently."""
    if abs(ss) < 1e-9:
        return None
    band = abs(ss) * band_pct
    mask = t >= t_dist
    idx_start = np.argmax(mask)
    within = np.abs(sig[idx_start:] - ss) <= band

    # Find last crossing out of band, settling time is just after it
    crossings = np.where(~within)[0]
    if len(crossings) == 0:
        return t[idx_start]
    last_exit = crossings[-1] + idx_start
    if last_exit + 1 < len(t):
        return t[last_exit + 1]
    return None


def _overshoot_pct(sig, t, ss, t_dist=3.0):
    """Compute max overshoot beyond steady-state, as percentage."""
    if abs(ss) < 1e-9:
        return 0.0
    mask = t >= t_dist
    sig_after = sig[mask]
    if ss < 0:
        peak = np.min(sig_after)
        if peak < ss:
            return abs((peak - ss) / ss) * 100
    else:
        peak = np.max(sig_after)
        if peak > ss:
            return abs((peak - ss) / ss) * 100
    return 0.0


def _rms(sig, t, t_start=5.0):
    """RMS value of signal after t_start."""
    mask = t >= t_start
    return np.sqrt(np.mean(sig[mask] ** 2))


def _add_info_box(ax, text, loc='upper right', fontsize=7):
    """Add a styled information box to an axes."""
    props = dict(boxstyle='round,pad=0.4', facecolor='#0d1117',
                 edgecolor='#30363d', alpha=0.92)
    anchors = {
        'upper right': (0.98, 0.96),
        'upper left':  (0.02, 0.96),
        'lower right': (0.98, 0.04),
        'lower left':  (0.02, 0.04),
    }
    x, y = anchors.get(loc, (0.98, 0.96))
    ha = 'right' if 'right' in loc else 'left'
    va = 'top' if 'upper' in loc else 'bottom'
    ax.text(x, y, text, transform=ax.transAxes, fontsize=fontsize,
            verticalalignment=va, horizontalalignment=ha, bbox=props,
            fontfamily='monospace', color=C['accent'])


def _add_ss_line(ax, t, ss_val, color, label='', fmt='.2f', unit='°'):
    """Add dashed horizontal line at steady-state value with label."""
    ax.axhline(ss_val, ls='--', color=color, alpha=0.5, lw=0.8)
    ax.text(t[-1] * 0.99, ss_val, f' {ss_val:{fmt}}{unit}',
            fontsize=7, color=color, va='bottom' if ss_val < 0 else 'top',
            ha='right', alpha=0.8)


def _shade_startup(ax, t_start=3.0, alpha=0.08):
    """Shade the startup / settling delay region."""
    ax.axvspan(0, t_start, alpha=alpha, color=C['yellow'],
               label='_startup')
    ax.axvline(t_start, color=C['yellow'], ls=':', alpha=0.4, lw=0.8)


# Downsampling for animations — keeps plotting fast
def _ds(arr, step):
    """Downsample array by step."""
    return arr[::step]


# ══════════════════════════════════════════════════════════════════
#  Entry Point
# ══════════════════════════════════════════════════════════════════

def generate_all(results, output_dir):
    """Generate all 7 static plots + 3 animated GIFs."""
    os.makedirs(output_dir, exist_ok=True)

    # ── Static Plots ──────────────────────────────────────────────
    plot_block_diagram(output_dir)
    plot_step_response(results, output_dir)
    plot_pid_signals_home(results['home'], output_dir)
    plot_pid_signals_gait(results['gait'], output_dir)
    plot_filter_pipeline(results['home'], output_dir)
    plot_correction_output(results, output_dir)
    plot_foot_adjustments(results['home'], output_dir)

    # ── Animations ────────────────────────────────────────────────
    animate_signal_flow(results['home'], output_dir)
    animate_step_response(results, output_dir)
    animate_pid_decomposition(results['home'], output_dir)


# ══════════════════════════════════════════════════════════════════
#  1. BLOCK DIAGRAM
# ══════════════════════════════════════════════════════════════════

def plot_block_diagram(output_dir):
    """Draw a Simulink-style control block diagram for HOME mode."""
    fig, ax = plt.subplots(figsize=(18, 6))
    ax.set_xlim(-1.5, 18.5)
    ax.set_ylim(-3, 3.5)
    ax.set_aspect('equal')
    ax.axis('off')

    # ── Helpers ───────────────────────────────────────────────────
    def block(x, y, w, h, label, sub='', color='#1e88e5'):
        rect = patches.FancyBboxPatch(
            (x, y - h / 2), w, h, boxstyle='round,pad=0.12',
            facecolor=color, edgecolor='#e6edf3', linewidth=1.3, alpha=0.92)
        ax.add_patch(rect)
        ax.text(x + w / 2, y + 0.1, label, ha='center', va='center',
                fontsize=9, fontweight='bold', color='white')
        if sub:
            ax.text(x + w / 2, y - 0.25, sub, ha='center', va='center',
                    fontsize=6.5, color='#b0bec5', style='italic')

    def arrow(x1, y1, x2, y2, label='', lbl_offset=0.22):
        ax.annotate('', xy=(x2, y2), xytext=(x1, y1),
                    arrowprops=dict(arrowstyle='->', color='#e6edf3',
                                    lw=1.3))
        if label:
            mx, my = (x1 + x2) / 2, (y1 + y2) / 2 + lbl_offset
            ax.text(mx, my, label, ha='center', va='center',
                    fontsize=6.5, color=C['grey'])

    # ── Title ─────────────────────────────────────────────────────
    ax.text(8.5, 3.0, 'Posture Stabilization — Control Block Diagram (HOME Mode)',
            ha='center', va='center', fontsize=13, fontweight='bold',
            color='#e6edf3')

    y = 0.5

    # r=0
    ax.text(-1.0, y, 'r = 0°', ha='center', fontsize=10,
            color=C['green'], fontweight='bold')
    arrow(-0.4, y, 0.3, y)

    # Summing junction
    circle = plt.Circle((0.55, y), 0.22, facecolor='#161b22',
                         edgecolor='#e6edf3', linewidth=1.3)
    ax.add_patch(circle)
    ax.text(0.55, y, 'Σ', ha='center', va='center', fontsize=10,
            color='white', fontweight='bold')
    ax.text(0.35, y + 0.15, '+', fontsize=8, color=C['green'])

    arrow(0.77, y, 1.3, y)

    # EMA
    block(1.3, y, 1.2, 0.8, 'EMA', 'α = 0.4', '#6d4c41')
    arrow(2.5, y, 3.1, y)

    # Dead Zone
    block(3.1, y, 1.2, 0.8, 'Dead Zone', '±0.02 rad', '#7b1fa2')
    arrow(4.3, y, 4.95, y, 'e(t)')

    # Gain Schedule + PID
    block(4.95, y, 2.0, 0.8, 'GS + PID', 'Kp=2.0  Ki=0.30  Kd=0.15', '#1565c0')
    # Sub-line for gain schedule
    ax.text(5.95, y - 0.55, 'GS: min(1, |e|/0.08), floor=0.5',
            ha='center', fontsize=5.5, color=C['dim'], style='italic')

    # Saturation
    arrow(6.95, y, 7.6, y, 'u(t)')
    block(7.6, y, 1.3, 0.8, 'Saturation', '±1.5 rad', '#c62828')

    # LPF
    arrow(8.9, y, 9.5, y)
    block(9.5, y, 1.3, 0.8, 'Output LPF', 'α = 0.15', '#e65100')

    # Application Gain
    arrow(10.8, y, 11.5, y)
    block(11.5, y, 1.3, 0.8, 'Apply', 'DZ→K×7→Sat±0.5', '#00695c')

    # Plant
    arrow(12.8, y, 13.6, y, 'Δθ₃')
    block(13.6, y, 1.6, 0.8, 'Plant', 'Rigid-Body  I·α=Στ', '#37474f')

    # Output
    arrow(15.2, y, 16.0, y)
    ax.text(16.5, y, 'y (φ, θ)', ha='center', fontsize=11,
            color=C['cyan'], fontweight='bold')
    ax.text(16.5, y - 0.35, 'Body Roll, Pitch', ha='center',
            fontsize=6.5, color=C['grey'])

    # ── Feedback ──────────────────────────────────────────────────
    fb_y = -1.5
    ax.plot([15.8, 15.8], [y - 0.4, fb_y], color='#e6edf3', lw=1.3)
    ax.plot([15.8, 0.55], [fb_y, fb_y], color='#e6edf3', lw=1.3)

    # IMU on feedback
    block(7.5, fb_y, 1.8, 0.7, 'IMU Sensor', 'σ_noise = 0.002 rad', '#5d4037')

    ax.annotate('', xy=(0.55, y - 0.22), xytext=(0.55, fb_y + 0.35),
                arrowprops=dict(arrowstyle='->', color='#e6edf3', lw=1.3))
    ax.text(0.25, fb_y + 0.65, '−', fontsize=14, color=C['red'],
            fontweight='bold')

    # ── Disturbance ───────────────────────────────────────────────
    dx = 14.4
    ax.text(dx, 2.3, 'd(t)', ha='center', fontsize=10,
            color=C['yellow'], fontweight='bold')
    ax.text(dx, 1.85, 'Step disturbance\nRoll=5°  Pitch=8°\nt_step=3.0 s',
            ha='center', fontsize=6.5, color=C['grey'])
    ax.annotate('', xy=(dx, y + 0.4), xytext=(dx, 1.6),
                arrowprops=dict(arrowstyle='->', color=C['yellow'], lw=1.3))

    # ── Legend box ────────────────────────────────────────────────
    legend_text = (
        "Parameters (nodeBalanceController.py)\n"
        "────────────────────────────────\n"
        "Integral decay:   0.9995/step\n"
        "Anti-windup:      ±1.5 (back-calc)\n"
        "Deriv LPF:        α=0.7\n"
        "Deriv ramp gate:  |e|<0.005→D=0"
    )
    props = dict(boxstyle='round,pad=0.4', facecolor='#0d1117',
                 edgecolor='#30363d', alpha=0.9)
    ax.text(17.8, -1.0, legend_text, fontsize=6, color=C['accent'],
            fontfamily='monospace', va='top', ha='right', bbox=props)

    fig.tight_layout()
    path = os.path.join(output_dir, 'block_diagram.png')
    fig.savefig(path, dpi=200, bbox_inches='tight')
    plt.close(fig)
    print(f"  ✓ {path}")


# ══════════════════════════════════════════════════════════════════
#  2. STEP RESPONSE (with metrics)
# ══════════════════════════════════════════════════════════════════

def plot_step_response(results, output_dir):
    """3-scenario comparison with steady-state, settling time, overshoot."""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(16, 9), sharex=True)
    fig.suptitle('Step Response — 5° Roll + 8° Pitch Disturbance at t = 3 s',
                 fontsize=14, fontweight='bold', y=0.97)

    styles = {
        'no_control': (C['dim'],    '--', 1.0, 'No Control (Open Loop)'),
        'home':       (C['cyan'],   '-',  1.5, 'HOME (GS-PID, K=7.0)'),
        'gait':       (C['green'],  '-',  1.5, 'GAIT (Adaptive PID)'),
    }

    # Plot data + compute metrics
    metrics = {}
    for key, (color, ls, lw, label) in styles.items():
        p = results[key]['plant']
        t = np.array(p['time'])
        roll_deg = np.degrees(np.array(p['roll']))
        pitch_deg = np.degrees(np.array(p['pitch']))

        ax1.plot(t, roll_deg, color=color, ls=ls, lw=lw, label=label)
        ax2.plot(t, pitch_deg, color=color, ls=ls, lw=lw, label=label)

        # Metrics
        ss_r = _ss_value(roll_deg, t)
        ss_p = _ss_value(pitch_deg, t)
        ts_r = _settling_time(roll_deg, t, ss_r)
        ts_p = _settling_time(pitch_deg, t, ss_p)
        os_r = _overshoot_pct(roll_deg, t, ss_r)
        os_p = _overshoot_pct(pitch_deg, t, ss_p)
        metrics[key] = {'ss_r': ss_r, 'ss_p': ss_p,
                        'ts_r': ts_r, 'ts_p': ts_p,
                        'os_r': os_r, 'os_p': os_p}

        # Steady-state line
        if key != 'no_control':
            _add_ss_line(ax1, t, ss_r, color)
            _add_ss_line(ax2, t, ss_p, color)

    # Disturbance reference
    d = results['no_control']['disturbance']
    t0 = np.array(results['no_control']['plant']['time'])
    ax1.plot(t0, np.degrees(d['roll']), ':', color=C['yellow'],
             alpha=0.4, lw=0.8, label='Disturbance')
    ax2.plot(t0, np.degrees(d['pitch']), ':', color=C['yellow'],
             alpha=0.4, lw=0.8, label='Disturbance')

    # Startup shading
    for ax in [ax1, ax2]:
        _shade_startup(ax)

    # Metrics info box
    nc = metrics['no_control']
    hm = metrics['home']
    gt = metrics['gait']

    rej_home_r = (1 - abs(hm['ss_r']) / abs(nc['ss_r'])) * 100 if abs(nc['ss_r']) > 0.01 else 0
    rej_home_p = (1 - abs(hm['ss_p']) / abs(nc['ss_p'])) * 100 if abs(nc['ss_p']) > 0.01 else 0
    rej_gait_r = (1 - abs(gt['ss_r']) / abs(nc['ss_r'])) * 100 if abs(nc['ss_r']) > 0.01 else 0
    rej_gait_p = (1 - abs(gt['ss_p']) / abs(nc['ss_p'])) * 100 if abs(nc['ss_p']) > 0.01 else 0

    box1 = (
        f"Roll Steady-State\n"
        f"─────────────────\n"
        f"Open:  {nc['ss_r']:+.2f}°\n"
        f"HOME:  {hm['ss_r']:+.2f}° (↓{rej_home_r:.0f}%)\n"
        f"GAIT:  {gt['ss_r']:+.2f}° (↓{rej_gait_r:.0f}%)\n"
    )
    if hm['ts_r'] is not None:
        box1 += f"HOME t_s: {hm['ts_r']:.1f}s\n"
    if hm['os_r'] > 0.5:
        box1 += f"HOME OS:  {hm['os_r']:.1f}%"
    _add_info_box(ax1, box1, 'lower right')

    box2 = (
        f"Pitch Steady-State\n"
        f"──────────────────\n"
        f"Open:  {nc['ss_p']:+.2f}°\n"
        f"HOME:  {hm['ss_p']:+.2f}° (↓{rej_home_p:.0f}%)\n"
        f"GAIT:  {gt['ss_p']:+.2f}° (↓{rej_gait_p:.0f}%)\n"
    )
    if hm['ts_p'] is not None:
        box2 += f"HOME t_s: {hm['ts_p']:.1f}s\n"
    if hm['os_p'] > 0.5:
        box2 += f"HOME OS:  {hm['os_p']:.1f}%"
    _add_info_box(ax2, box2, 'lower right')

    # Formatting
    for ax, ylabel in [(ax1, 'Roll (°)'), (ax2, 'Pitch (°)')]:
        ax.axhline(0, color=C['white'], ls=':', alpha=0.15, lw=0.5)
        ax.set_ylabel(ylabel)
        ax.legend(loc='upper right', fontsize=8, framealpha=0.9)
        ax.grid(True)

    ax2.set_xlabel('Time (s)')
    ax2.set_xlim(0, 30)
    fig.tight_layout()
    path = os.path.join(output_dir, 'step_response.png')
    fig.savefig(path, dpi=200)
    plt.close(fig)
    print(f"  ✓ {path}")


# ══════════════════════════════════════════════════════════════════
#  3. PID SIGNALS — HOME (6-panel)
# ══════════════════════════════════════════════════════════════════

def plot_pid_signals_home(data, output_dir):
    """6-panel: P, I, D, PID Sum, Saturated, After LPF for HOME."""
    filt = data['filter']
    t = filt['time']

    fig, axes = plt.subplots(3, 2, figsize=(16, 11), sharex=True)
    fig.suptitle('PID Signal Decomposition — HOME Mode (Kp=2.0  Ki=0.30  Kd=0.15)',
                 fontsize=13, fontweight='bold', y=0.98)

    # ── Row 0: P and I terms ──────────────────────────────────────
    ax = axes[0, 0]
    ax.plot(t, filt['p_out_r'], color=C['cyan'], lw=0.6, alpha=0.8, label='Roll P')
    ax.plot(t, filt['p_out_p'], color=C['red'], lw=0.6, alpha=0.8, label='Pitch P')
    ax.set_ylabel('P term (rad)')
    ax.set_title('Proportional (Kp × GS × error)')
    ax.legend(fontsize=7, loc='upper right')
    ax.grid(True)
    _shade_startup(ax)
    _add_info_box(ax, f"Kp = 2.0\nGS: 0.5–1.0", 'upper left', 6)

    ax = axes[0, 1]
    ax.plot(t, filt['i_out_r'], color=C['cyan'], lw=0.6, alpha=0.8, label='Roll I')
    ax.plot(t, filt['i_out_p'], color=C['red'], lw=0.6, alpha=0.8, label='Pitch I')
    ax.set_ylabel('I term (rad)')
    ax.set_title('Integral (Ki × GS × ∫e·dt)')
    ax.legend(fontsize=7, loc='upper right')
    ax.grid(True)
    _shade_startup(ax)
    _add_info_box(ax, f"Ki = 0.30\ndecay = 0.9995\nwindup = ±1.5", 'upper left', 6)

    # ── Row 1: D and Gain Schedule ────────────────────────────────
    ax = axes[1, 0]
    ax.plot(t, filt['d_out_r'], color=C['cyan'], lw=0.5, alpha=0.7, label='Roll D')
    ax.plot(t, filt['d_out_p'], color=C['red'], lw=0.5, alpha=0.7, label='Pitch D')
    ax.set_ylabel('D term (rad)')
    ax.set_title('Derivative (Kd × d(e)/dt, LPF α=0.7)')
    ax.legend(fontsize=7, loc='upper right')
    ax.grid(True)
    _shade_startup(ax)
    _add_info_box(ax, f"Kd = 0.15\nramp gate < 0.005", 'upper left', 6)

    ax = axes[1, 1]
    ax.plot(t, filt['gs_r'], color=C['cyan'], lw=0.8, label='Roll GS')
    ax.plot(t, filt['gs_p'], color=C['red'], lw=0.8, label='Pitch GS')
    ax.axhline(0.5, ls='--', color=C['orange'], alpha=0.5, lw=0.8)
    ax.axhline(1.0, ls='--', color=C['green'], alpha=0.5, lw=0.8)
    ax.text(t[-1] * 0.99, 0.52, 'floor = 0.5', fontsize=6, ha='right',
            color=C['orange'], alpha=0.7)
    ax.text(t[-1] * 0.99, 0.97, 'max = 1.0', fontsize=6, ha='right',
            color=C['green'], alpha=0.7)
    ax.set_ylabel('Gain Scale')
    ax.set_title('Gain Scheduling Factor')
    ax.set_ylim(0.3, 1.1)
    ax.legend(fontsize=7, loc='lower right')
    ax.grid(True)
    _shade_startup(ax)

    # ── Row 2: PID Sum (before sat) and After LPF ─────────────────
    ax = axes[2, 0]
    ax.plot(t, filt['pid_sum_r'], color=C['cyan'], lw=0.5, alpha=0.6,
            label='Roll PID')
    ax.plot(t, filt['pid_sum_p'], color=C['red'], lw=0.5, alpha=0.6,
            label='Pitch PID')
    ax.plot(t, filt['corr_sat_r'], color=C['cyan'], lw=0.8, label='Roll Sat')
    ax.plot(t, filt['corr_sat_p'], color=C['red'], lw=0.8, label='Pitch Sat')
    ax.axhline(1.5, ls='--', color=C['orange'], alpha=0.4, lw=0.8)
    ax.axhline(-1.5, ls='--', color=C['orange'], alpha=0.4, lw=0.8)
    ax.set_ylabel('Output (rad)')
    ax.set_title('PID Sum → Saturation (±1.5 rad)')
    ax.legend(fontsize=6, loc='upper right', ncol=2)
    ax.grid(True)
    _shade_startup(ax)
    ax.set_xlabel('Time (s)')

    ax = axes[2, 1]
    ax.plot(t, filt['corr_sat_r'], color=C['cyan'], alpha=0.25, lw=0.4,
            label='Saturated')
    ax.plot(t, filt['corr_lpf_r'], color=C['orange'], lw=1.0,
            label='After LPF (α=0.15)')
    ax.plot(t, filt['corr_sat_p'], color=C['red'], alpha=0.25, lw=0.4)
    ax.plot(t, filt['corr_lpf_p'], color=C['yellow'], lw=1.0,
            label='After LPF (Pitch)')
    ax.set_ylabel('Output (rad)')
    ax.set_title('Output LPF Smoothing')
    ax.legend(fontsize=7, loc='upper right')
    ax.grid(True)
    _shade_startup(ax)
    ax.set_xlabel('Time (s)')

    fig.tight_layout()
    path = os.path.join(output_dir, 'pid_signals_home.png')
    fig.savefig(path, dpi=200)
    plt.close(fig)
    print(f"  ✓ {path}")


# ══════════════════════════════════════════════════════════════════
#  4. PID SIGNALS — GAIT (6-panel)
# ══════════════════════════════════════════════════════════════════

def plot_pid_signals_gait(data, output_dir):
    """6-panel: Moving Avg, Adaptive P, I, D, Correction, Gain Schedule for GAIT."""
    gait = data['gait']
    filt = data['filter']
    t = filt['time']

    if len(gait.get('roll_avg', [])) == 0:
        print(f"  ⊘ Skipping pid_signals_gait.png (no gait data)")
        return

    fig, axes = plt.subplots(3, 2, figsize=(16, 11), sharex=True)
    fig.suptitle('PID Signal Decomposition — GAIT Mode (Kp=3.0  Ki=0.005  Kd=1.2)',
                 fontsize=13, fontweight='bold', y=0.98)

    # ── Row 0: Moving average and error ───────────────────────────
    ax = axes[0, 0]
    ax.plot(t, np.degrees(filt['meas_r']), color=C['cyan'], alpha=0.2,
            lw=0.3, label='EMA meas')
    ax.plot(t, np.degrees(gait['roll_avg']), color=C['cyan'], lw=0.8,
            label='Roll avg')
    ax.plot(t, np.degrees(gait['pitch_avg']), color=C['red'], lw=0.8,
            label='Pitch avg')
    ax.axhline(np.degrees(0.03), ls=':', color=C['orange'], alpha=0.4, lw=0.7)
    ax.axhline(np.degrees(-0.03), ls=':', color=C['orange'], alpha=0.4, lw=0.7)
    ax.set_ylabel('Angle (°)')
    ax.set_title('Moving Average (window=270 samples)')
    ax.legend(fontsize=7, loc='upper right')
    ax.grid(True)
    _shade_startup(ax)
    _add_info_box(ax, f"DZ = ±0.03 rad\n     ±{np.degrees(0.03):.2f}°", 'lower right', 6)

    ax = axes[0, 1]
    ax.plot(t, gait['roll_error'], color=C['cyan'], lw=0.6, label='Roll error')
    ax.plot(t, gait['pitch_error'], color=C['red'], lw=0.6, label='Pitch error')
    ax.set_ylabel('Error (rad)')
    ax.set_title('Error after deadzone (±0.03 rad)')
    ax.legend(fontsize=7, loc='upper right')
    ax.grid(True)
    _shade_startup(ax)

    # ── Row 1: P and I terms ──────────────────────────────────────
    ax = axes[1, 0]
    ax.plot(t, gait['roll_p'], color=C['cyan'], lw=0.6, label='Roll P')
    ax.plot(t, gait['pitch_p'], color=C['red'], lw=0.6, label='Pitch P')
    ax.set_ylabel('P term')
    ax.set_title('Adaptive Proportional (Kp_eff = 3.0 × gain_scale)')
    ax.legend(fontsize=7, loc='upper right')
    ax.grid(True)
    _shade_startup(ax)

    ax = axes[1, 1]
    ax.plot(t, gait['roll_i'], color=C['cyan'], lw=0.8, label='Roll I')
    ax.plot(t, gait['pitch_i'], color=C['red'], lw=0.8, label='Pitch I')
    ax.set_ylabel('I term')
    ax.set_title('Integral (Ki=0.005, zero-crossing reset)')
    ax.legend(fontsize=7, loc='upper right')
    ax.grid(True)
    _shade_startup(ax)
    _add_info_box(ax, f"Ki = 0.005\nwindup = ±0.3\nzero-cross reset", 'upper left', 6)

    # ── Row 2: D term and Gain schedule / Output ──────────────────
    ax = axes[2, 0]
    ax.plot(t, gait['roll_d'], color=C['cyan'], lw=0.5, label='Roll D')
    ax.plot(t, gait['pitch_d'], color=C['red'], lw=0.5, label='Pitch D')
    ax.set_ylabel('D term')
    ax.set_title('Derivative (Kd=1.2 × Δavg)')
    ax.legend(fontsize=7, loc='upper right')
    ax.grid(True)
    _shade_startup(ax)
    ax.set_xlabel('Time (s)')

    ax = axes[2, 1]
    ax.plot(t, gait['roll_corr'], color=C['cyan'], alpha=0.3, lw=0.4,
            label='Roll saturated')
    ax.plot(t, gait['roll_corr_lpf'], color=C['cyan'], lw=1.0,
            label='Roll LPF (α=0.97)')
    ax.plot(t, gait['pitch_corr'], color=C['red'], alpha=0.3, lw=0.4,
            label='Pitch saturated')
    ax.plot(t, gait['pitch_corr_lpf'], color=C['red'], lw=1.0,
            label='Pitch LPF (α=0.97)')
    ax.axhline(0.30, ls='--', color=C['orange'], alpha=0.4, lw=0.8)
    ax.axhline(-0.30, ls='--', color=C['orange'], alpha=0.4, lw=0.8)
    ax.set_ylabel('Correction (rad)')
    ax.set_title('Output: Sat(±0.3) → LPF(α=0.97)')
    ax.legend(fontsize=6, loc='upper right', ncol=2)
    ax.grid(True)
    _shade_startup(ax)
    ax.set_xlabel('Time (s)')

    fig.tight_layout()
    path = os.path.join(output_dir, 'pid_signals_gait.png')
    fig.savefig(path, dpi=200)
    plt.close(fig)
    print(f"  ✓ {path}")


# ══════════════════════════════════════════════════════════════════
#  5. FILTER PIPELINE (4-panel: raw + sat + LPF + applied)
# ══════════════════════════════════════════════════════════════════

def plot_filter_pipeline(data, output_dir):
    """Show the full signal pipeline: measurement → PID → sat → LPF → applied."""
    filt = data['filter']
    t = filt['time']

    fig, axes = plt.subplots(2, 2, figsize=(16, 9), sharex=True)
    fig.suptitle('Signal Pipeline — HOME Mode (Roll Channel Highlighted)',
                 fontsize=13, fontweight='bold', y=0.97)

    # Panel 1: Raw measurement vs EMA filtered
    ax = axes[0, 0]
    imu = data['imu']
    ax.plot(imu['time'], np.degrees(imu['noisy_roll']), color=C['dim'],
            lw=0.2, alpha=0.5, label='Raw IMU')
    ax.plot(imu['time'], np.degrees(imu['filtered_roll']), color=C['purple'],
            lw=0.5, alpha=0.7, label='IMU filtered')
    ax.plot(t, np.degrees(filt['meas_r']), color=C['cyan'], lw=0.8,
            label='After EMA (α=0.4)')
    ax.set_ylabel('Roll (°)')
    ax.set_title('① IMU Measurement → EMA Pre-filter')
    ax.legend(fontsize=7, loc='upper right')
    ax.grid(True)
    _shade_startup(ax)

    # Panel 2: Error after deadzone + gain schedule
    ax = axes[0, 1]
    ax.plot(t, filt['error_r'], color=C['cyan'], lw=0.5, alpha=0.7,
            label='Error (Roll)')
    ax.plot(t, filt['error_p'], color=C['red'], lw=0.5, alpha=0.7,
            label='Error (Pitch)')
    ax.axhline(0, color=C['white'], ls=':', alpha=0.15)
    ax.set_ylabel('Error (rad)')
    ax.set_title('② Error = 0 − deadzone(meas, 0.02)')
    ax.legend(fontsize=7, loc='upper right')
    ax.grid(True)
    _shade_startup(ax)

    # Panel 3: PID sum vs Saturated output
    ax = axes[1, 0]
    ax.plot(t, filt['pid_sum_r'], color=C['cyan'], alpha=0.3, lw=0.4,
            label='PID sum (raw)')
    ax.plot(t, filt['corr_sat_r'], color=C['cyan'], lw=0.8,
            label='Saturated (±1.5)')
    ax.plot(t, filt['pid_sum_p'], color=C['red'], alpha=0.3, lw=0.4)
    ax.plot(t, filt['corr_sat_p'], color=C['red'], lw=0.8)
    ax.axhline(1.5, ls='--', color=C['orange'], alpha=0.4, lw=0.8)
    ax.axhline(-1.5, ls='--', color=C['orange'], alpha=0.4, lw=0.8)
    ax.set_ylabel('Output (rad)')
    ax.set_title('③ PID Output → Saturation (±1.5 rad)')
    ax.legend(fontsize=7, loc='upper right')
    ax.grid(True)
    _shade_startup(ax)
    ax.set_xlabel('Time (s)')

    # Panel 4: Saturated vs LPF output
    ax = axes[1, 1]
    ax.plot(t, filt['corr_sat_r'], color=C['cyan'], alpha=0.2, lw=0.3,
            label='Saturated')
    ax.plot(t, filt['corr_lpf_r'], color=C['orange'], lw=1.0,
            label='After LPF (α=0.15)')
    ax.plot(t, filt['corr_r'], color=C['green'], lw=0.8, alpha=0.7,
            label='Applied correction')
    ax.set_ylabel('Output (rad)')
    ax.set_title('④ Output LPF → Applied Correction')
    ax.legend(fontsize=7, loc='upper right')
    ax.grid(True)
    _shade_startup(ax)
    ax.set_xlabel('Time (s)')

    # RMS annotation
    rms_sat = _rms(filt['corr_sat_r'], t)
    rms_lpf = _rms(filt['corr_lpf_r'], t)
    _add_info_box(ax, f"RMS saturated: {rms_sat:.4f}\nRMS after LPF: {rms_lpf:.4f}\nSmoothing:     {(1-rms_lpf/rms_sat)*100:.1f}%",
                  'lower right', 6)

    fig.tight_layout()
    path = os.path.join(output_dir, 'filter_pipeline.png')
    fig.savefig(path, dpi=200)
    plt.close(fig)
    print(f"  ✓ {path}")


# ══════════════════════════════════════════════════════════════════
#  6. CORRECTION OUTPUT (HOME vs GAIT)
# ══════════════════════════════════════════════════════════════════

def plot_correction_output(results, output_dir):
    """Compare final applied correction for HOME and GAIT with metrics."""
    fig, axes = plt.subplots(2, 2, figsize=(16, 9))
    fig.suptitle('Applied Correction Output — HOME vs GAIT',
                 fontsize=13, fontweight='bold', y=0.97)

    configs = [
        ('home', 'HOME (K=7.0, DZ=±0.02)', C['cyan']),
        ('gait', 'GAIT (Adaptive, DZ=±0.03)', C['green']),
    ]

    for col, (key, title, color) in enumerate(configs):
        filt = results[key]['filter']
        t = filt['time']

        for row, (sig_key, ch) in enumerate([('corr_r', 'Roll'), ('corr_p', 'Pitch')]):
            ax = axes[row, col]
            sig = filt[sig_key]
            ax.plot(t, sig, color=color, lw=0.6, alpha=0.9)

            # Steady-state & RMS
            ss = _ss_value(sig, t)
            rms = _rms(sig, t)
            peak = np.max(np.abs(sig[t >= 3.0]))

            ax.axhline(ss, ls='--', color=color, alpha=0.4, lw=0.8)
            _add_info_box(ax, f"SS:   {ss:+.4f} rad\nRMS:  {rms:.4f} rad\nPeak: {peak:.4f} rad",
                          'upper right' if ss < 0 else 'lower right', 6)

            ax.set_title(f'{title} — {ch}', fontsize=10)
            ax.set_ylabel('Correction (rad)')
            ax.grid(True)
            _shade_startup(ax)

        axes[1, col].set_xlabel('Time (s)')

    fig.tight_layout()
    path = os.path.join(output_dir, 'correction_output.png')
    fig.savefig(path, dpi=200)
    plt.close(fig)
    print(f"  ✓ {path}")


# ══════════════════════════════════════════════════════════════════
#  7. FOOT ADJUSTMENTS
# ══════════════════════════════════════════════════════════════════

def plot_foot_adjustments(data, output_dir):
    """Per-leg Δz height adjustment with RMS and peak annotations."""
    posture = data['posture']
    t = np.array(posture['time'])

    fig, axes = plt.subplots(2, 2, figsize=(16, 8))
    fig.suptitle('Foot Height Adjustments (Δz) — HOME Mode',
                 fontsize=13, fontweight='bold', y=0.97)

    legs = [
        ('left-front',   'Left Front',   C['cyan']),
        ('left-behind',  'Left Behind',  C['red']),
        ('right-front',  'Right Front',  C['green']),
        ('right-behind', 'Right Behind', C['orange']),
    ]

    for idx, (leg_key, leg_name, color) in enumerate(legs):
        ax = axes[idx // 2, idx % 2]
        dz_key = f'{leg_key}_dz'
        if dz_key in posture:
            dz_mm = np.array(posture[dz_key]) * 1000
            ax.plot(t, dz_mm, color=color, lw=0.5, alpha=0.9)

            # Metrics
            ss = _ss_value(dz_mm, t)
            rms = _rms(dz_mm, t)
            peak = np.max(np.abs(dz_mm[t >= 3.0])) if np.any(t >= 3.0) else 0

            _add_info_box(ax, f"SS:   {ss:+.1f} mm\nRMS:  {rms:.1f} mm\nPeak: {peak:.1f} mm",
                          'upper right', 6)

        ax.set_title(leg_name, fontsize=10, fontweight='bold')
        ax.set_ylabel('Δz (mm)')
        ax.grid(True)
        ax.axhline(0, color=C['white'], ls=':', alpha=0.15)
        _shade_startup(ax)

    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 1].set_xlabel('Time (s)')
    fig.tight_layout()
    path = os.path.join(output_dir, 'foot_adjustments.png')
    fig.savefig(path, dpi=200)
    plt.close(fig)
    print(f"  ✓ {path}")


# ══════════════════════════════════════════════════════════════════
#  8. ANIMATION — Signal Flow Pipeline
# ══════════════════════════════════════════════════════════════════

def animate_signal_flow(data, output_dir, fps=15, duration_s=12):
    """5-panel signal flow animation showing the full HOME pipeline."""
    filt = data['filter']
    plant = data['plant']
    dist_data = data['disturbance']
    t_full = filt['time']
    N = len(t_full)

    n_frames = fps * duration_s
    stride = max(1, N // n_frames)

    # Downsample
    t      = _ds(t_full, stride)
    dist_r = np.degrees(_ds(np.array(dist_data['roll']), stride))
    meas_r = np.degrees(_ds(filt['meas_r'], stride))
    sat_r  = _ds(filt['corr_sat_r'], stride)
    lpf_r  = _ds(filt['corr_lpf_r'], stride)
    body_r = np.degrees(_ds(np.array(plant['roll']), stride))

    fig, axes = plt.subplots(5, 1, figsize=(14, 10), sharex=True)
    fig.suptitle('Signal Flow Animation — HOME Mode (Roll Channel)',
                 fontsize=13, fontweight='bold', y=0.98)

    panels = [
        ('① Disturbance d(t)', 'Angle (°)', C['yellow'], dist_r),
        ('② IMU Measurement (EMA)', 'Angle (°)', C['purple'], meas_r),
        ('③ PID Output (Saturated)', 'Output (rad)', C['cyan'], sat_r),
        ('④ After LPF (α=0.15)', 'Output (rad)', C['orange'], lpf_r),
        ('⑤ Body Roll', 'Angle (°)', C['green'], body_r),
    ]

    lines = []
    time_markers = []
    for ax, (title, ylabel, color, _) in zip(axes, panels):
        ax.set_xlim(0, t_full[-1])
        ax.set_ylabel(ylabel, fontsize=8)
        ax.set_title(title, fontsize=9, fontweight='bold', loc='left')
        ax.grid(True)
        ax.axhline(0, color=C['white'], ls=':', alpha=0.1)
        line, = ax.plot([], [], color=color, lw=0.8)
        lines.append(line)
        vline = ax.axvline(0, color=C['white'], alpha=0.3, ls='--', lw=0.5)
        time_markers.append(vline)

    # Set y-limits from data
    for ax, (_, _, _, sig) in zip(axes, panels):
        margin = (np.max(sig) - np.min(sig)) * 0.1 + 0.01
        ax.set_ylim(np.min(sig) - margin, np.max(sig) + margin)

    axes[-1].set_xlabel('Time (s)')

    # Time text
    time_text = fig.text(0.92, 0.02, '', fontsize=10, color=C['accent'],
                         fontfamily='monospace', ha='right',
                         bbox=dict(boxstyle='round,pad=0.3',
                                   facecolor='#0d1117', edgecolor='#30363d'))

    fig.tight_layout(rect=[0, 0.04, 1, 0.95])

    def init():
        for line in lines:
            line.set_data([], [])
        return lines + time_markers

    def update(frame):
        idx = frame + 1
        for i, (line, (_, _, _, sig)) in enumerate(zip(lines, panels)):
            line.set_data(t[:idx], sig[:idx])
            time_markers[i].set_xdata([t[min(idx - 1, len(t) - 1)]])
        time_text.set_text(f't = {t[min(idx - 1, len(t) - 1)]:.1f} s')
        return lines + time_markers + [time_text]

    anim = FuncAnimation(fig, update, frames=len(t),
                         init_func=init, interval=1000 // fps, blit=True)

    path = os.path.join(output_dir, 'signal_flow.gif')
    anim.save(path, writer='pillow', fps=fps, dpi=100)
    plt.close(fig)
    kb = os.path.getsize(path) / 1024
    print(f"  ✓ {path}  ({kb:.0f} KB, {len(t)} frames)")


# ══════════════════════════════════════════════════════════════════
#  9. ANIMATION — Step Response
# ══════════════════════════════════════════════════════════════════

def animate_step_response(results, output_dir, fps=15, duration_s=12):
    """Animated 3-scenario step response comparison."""
    N = len(results['no_control']['plant']['time'])
    n_frames = fps * duration_s
    stride = max(1, N // n_frames)

    # Downsample all scenarios
    scenarios = {}
    for key in ['no_control', 'home', 'gait']:
        p = results[key]['plant']
        scenarios[key] = {
            't': _ds(np.array(p['time']), stride),
            'roll': np.degrees(_ds(np.array(p['roll']), stride)),
            'pitch': np.degrees(_ds(np.array(p['pitch']), stride)),
        }

    t = scenarios['no_control']['t']

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    fig.suptitle('Step Response Animation — 5° Roll + 8° Pitch Disturbance',
                 fontsize=13, fontweight='bold', y=0.97)

    styles = {
        'no_control': (C['dim'],   '--', 1.0, 'No Control'),
        'home':       (C['cyan'],  '-',  1.5, 'HOME (GS-PID)'),
        'gait':       (C['green'], '-',  1.5, 'GAIT (Adpt-PID)'),
    }

    lines_r, lines_p = [], []
    for key, (color, ls, lw, label) in styles.items():
        lr, = ax1.plot([], [], color=color, ls=ls, lw=lw, label=label)
        lp, = ax2.plot([], [], color=color, ls=ls, lw=lw, label=label)
        lines_r.append((key, lr))
        lines_p.append((key, lp))

    # Set limits from full data
    all_roll = np.concatenate([s['roll'] for s in scenarios.values()])
    all_pitch = np.concatenate([s['pitch'] for s in scenarios.values()])
    ax1.set_ylim(np.min(all_roll) - 0.5, np.max(all_roll) + 0.5)
    ax2.set_ylim(np.min(all_pitch) - 0.5, np.max(all_pitch) + 0.5)
    ax1.set_xlim(0, t[-1])

    for ax, ylabel in [(ax1, 'Roll (°)'), (ax2, 'Pitch (°)')]:
        ax.axhline(0, color=C['white'], ls=':', alpha=0.15)
        ax.axvline(3.0, color=C['yellow'], ls=':', alpha=0.3, lw=0.8)
        ax.set_ylabel(ylabel)
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True)

    ax2.set_xlabel('Time (s)')

    # Time cursor
    vline1 = ax1.axvline(0, color=C['accent'], alpha=0.5, ls='-', lw=0.5)
    vline2 = ax2.axvline(0, color=C['accent'], alpha=0.5, ls='-', lw=0.5)
    time_text = fig.text(0.92, 0.02, '', fontsize=10, color=C['accent'],
                         fontfamily='monospace', ha='right',
                         bbox=dict(boxstyle='round,pad=0.3',
                                   facecolor='#0d1117', edgecolor='#30363d'))

    fig.tight_layout(rect=[0, 0.04, 1, 0.94])

    def init():
        for _, l in lines_r:
            l.set_data([], [])
        for _, l in lines_p:
            l.set_data([], [])
        return [l for _, l in lines_r + lines_p] + [vline1, vline2]

    def update(frame):
        idx = frame + 1
        cur_t = t[min(idx - 1, len(t) - 1)]
        for key, l in lines_r:
            l.set_data(scenarios[key]['t'][:idx], scenarios[key]['roll'][:idx])
        for key, l in lines_p:
            l.set_data(scenarios[key]['t'][:idx], scenarios[key]['pitch'][:idx])
        vline1.set_xdata([cur_t])
        vline2.set_xdata([cur_t])
        time_text.set_text(f't = {cur_t:.1f} s')
        return [l for _, l in lines_r + lines_p] + [vline1, vline2, time_text]

    anim = FuncAnimation(fig, update, frames=len(t),
                         init_func=init, interval=1000 // fps, blit=True)

    path = os.path.join(output_dir, 'step_response.gif')
    anim.save(path, writer='pillow', fps=fps, dpi=100)
    plt.close(fig)
    kb = os.path.getsize(path) / 1024
    print(f"  ✓ {path}  ({kb:.0f} KB, {len(t)} frames)")


# ══════════════════════════════════════════════════════════════════
#  10. ANIMATION — PID Decomposition
# ══════════════════════════════════════════════════════════════════

def animate_pid_decomposition(data, output_dir, fps=15, duration_s=12):
    """Animated P/I/D term buildup for HOME mode roll channel."""
    filt = data['filter']
    t_full = filt['time']
    N = len(t_full)

    n_frames = fps * duration_s
    stride = max(1, N // n_frames)

    t  = _ds(t_full, stride)
    P  = _ds(filt['p_out_r'], stride)
    I  = _ds(filt['i_out_r'], stride)
    D  = _ds(filt['d_out_r'], stride)
    S  = _ds(filt['pid_sum_r'], stride)

    fig, axes = plt.subplots(4, 1, figsize=(14, 10), sharex=True)
    fig.suptitle('PID Term Buildup — HOME Roll Channel',
                 fontsize=13, fontweight='bold', y=0.98)

    configs = [
        ('P term (Kp=2.0 × GS × error)', C['cyan'], P),
        ('I term (Ki=0.30 × GS × ∫e dt)', C['red'], I),
        ('D term (Kd=0.15 × de/dt, LPF)', C['green'], D),
        ('PID Sum (P + I + D)', C['orange'], S),
    ]

    lines = []
    time_markers = []
    for ax, (title, color, sig) in zip(axes, configs):
        ax.set_xlim(0, t_full[-1])
        margin = (np.max(sig) - np.min(sig)) * 0.15 + 0.001
        ax.set_ylim(np.min(sig) - margin, np.max(sig) + margin)
        ax.set_title(title, fontsize=9, fontweight='bold', loc='left')
        ax.set_ylabel('Value (rad)')
        ax.grid(True)
        ax.axhline(0, color=C['white'], ls=':', alpha=0.1)
        _shade_startup(ax)

        line, = ax.plot([], [], color=color, lw=0.8)
        lines.append(line)
        vl = ax.axvline(0, color=C['white'], alpha=0.3, ls='--', lw=0.5)
        time_markers.append(vl)

    # Saturation lines on PID Sum panel
    axes[3].axhline(1.5, ls='--', color=C['red'], alpha=0.3, lw=0.8)
    axes[3].axhline(-1.5, ls='--', color=C['red'], alpha=0.3, lw=0.8)
    axes[3].text(t_full[-1] * 0.99, 1.42, 'Sat ±1.5', fontsize=6,
                 ha='right', color=C['red'], alpha=0.5)

    axes[-1].set_xlabel('Time (s)')

    time_text = fig.text(0.92, 0.02, '', fontsize=10, color=C['accent'],
                         fontfamily='monospace', ha='right',
                         bbox=dict(boxstyle='round,pad=0.3',
                                   facecolor='#0d1117', edgecolor='#30363d'))

    fig.tight_layout(rect=[0, 0.04, 1, 0.95])

    def init():
        for line in lines:
            line.set_data([], [])
        return lines + time_markers

    def update(frame):
        idx = frame + 1
        cur_t = t[min(idx - 1, len(t) - 1)]
        for i, (line, (_, _, sig)) in enumerate(zip(lines, configs)):
            line.set_data(t[:idx], sig[:idx])
            time_markers[i].set_xdata([cur_t])
        time_text.set_text(f't = {cur_t:.1f} s')
        return lines + time_markers + [time_text]

    anim = FuncAnimation(fig, update, frames=len(t),
                         init_func=init, interval=1000 // fps, blit=True)

    path = os.path.join(output_dir, 'pid_decomposition.gif')
    anim.save(path, writer='pillow', fps=fps, dpi=100)
    plt.close(fig)
    kb = os.path.getsize(path) / 1024
    print(f"  ✓ {path}  ({kb:.0f} KB, {len(t)} frames)")
