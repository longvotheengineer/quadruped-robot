#!/usr/bin/env python3
"""
generate_pid_balance_plots.py
==============================
Produces two publication-quality figures for the paper — one for the **roll**
axis and one for the **pitch** axis.

Each figure has 8 subplots, one per diagnostic signal, using simple
human-readable names that map directly to jiggleplot / rqt_plot topics:

  Signal name        │ Description                     │ ROS2 topic (Home mode)
  ───────────────────┼─────────────────────────────────┼────────────────────────────────────────
  roll_platform      │ Commanded ramp disturbance       │ /diag/balance/disturbance/roll
  roll_response      │ EMA-filtered IMU measurement     │ /diag/balance/home/roll/imu_filtered
  roll_error         │ PID error (after deadzone)       │ /diag/balance/home/roll/pid_error
  roll_p_term        │ Proportional correction term     │ /diag/balance/home/roll/term_proportional
  roll_i_term        │ Integral correction term         │ /diag/balance/home/roll/term_integral
  roll_d_term        │ Derivative correction term       │ /diag/balance/home/roll/term_derivative
  roll_pid_sat       │ P+I+D sum (saturated)            │ /diag/balance/home/roll/output_saturated
  roll_correction    │ Final smoothed PID output        │ /diag/balance/home/roll/output_smoothed

  (Same layout for pitch_ prefix.)

Output:
  ../figures/pid_balance_roll.png
  ../figures/pid_balance_pitch.png
"""

import os
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import rcParams

# ── IEEE-compatible style ─────────────────────────────────────────────────────
rcParams.update({
    'font.family':        'serif',
    'font.serif':         ['Times New Roman', 'DejaVu Serif'],
    'font.size':          8,
    'axes.labelsize':     8,
    'axes.titlesize':     7.5,
    'legend.fontsize':    7,
    'xtick.labelsize':    7,
    'ytick.labelsize':    7,
    'figure.dpi':         300,
    'savefig.dpi':        300,
    'savefig.bbox':       'tight',
    'savefig.pad_inches': 0.05,
    'axes.grid':          True,
    'grid.alpha':         0.22,
    'grid.linestyle':     '--',
    'lines.linewidth':    1.3,
    'axes.spines.top':    False,
    'axes.spines.right':  False,
})

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', 'figures')
os.makedirs(OUTPUT_DIR, exist_ok=True)

# ── Simulation timing ─────────────────────────────────────────────────────────
DT      = 0.02       # 50 Hz update rate
T_TOTAL = 90.0       # 90 s — 3 full 30-s disturbance cycles
N       = int(T_TOTAL / DT)
t_arr   = np.arange(N) * DT

# ── PID gains (verbatim from PidConfig in nodeBalanceController.py) ───────────
KP, KI, KD   = 2.0, 0.30, 0.15
SAT_CORR     = 1.5
SAT_INTEG    = 1.5
DEADZONE     = 0.02        # rad ≈ 1.15°
LPF_RAW      = 0.4
LPF_DERIV    = 0.7
LPF_CORR     = 0.15
DECAY_INTEG  = 0.9995
GAIN_SC_TH   = 0.08        # gain-schedule threshold
SAT_GAIN_SC  = 0.5         # gain-schedule floor
RAMP_DERIV   = 0.005       # derivative zero-threshold (rad)

# ── Body dynamics (first-order lag model) ─────────────────────────────────────
TAU_BODY   = 0.40
ALPHA_B    = DT / (TAU_BODY + DT)
CTRL_GAIN  = 0.18          # correction-to-tilt coupling gain


def _dz(val, dz):
    """Linear deadzone: returns 0 inside ±dz, else sign*(|val|-dz)."""
    if abs(val) <= dz:
        return 0.0
    return math.copysign(abs(val) - dz, val)


def _gs(error):
    """Gain-schedule multiplier in [SAT_GAIN_SC, 1.0]."""
    return max(SAT_GAIN_SC, min(1.0, abs(error) / GAIN_SC_TH))


def simulate(amplitude_deg, period_sec, phase_deg=0.0, noise_std=0.003, seed=42):
    """
    Closed-loop simulation of the Home PID for one axis.

    Returns a dict of 8 numpy arrays (length N) — one per named signal.
    Names match the simple labels used in the paper figures.
    """
    A   = math.radians(amplitude_deg)
    w   = 2.0 * math.pi / period_sec
    phi = math.radians(phase_deg)

    rng = np.random.default_rng(seed)

    # PID state
    body_tilt = 0.0
    meas      = 0.0
    integ     = 0.0
    deriv     = 0.0
    err_prev  = 0.0
    corr_lpf  = 0.0

    # Output buffers — 8 signals
    buf = {k: np.zeros(N) for k in (
        'platform',    # disturbance command
        'response',    # imu_filtered
        'error',       # pid_error
        'p_term',      # term_proportional
        'i_term',      # term_integral
        'd_term',      # term_derivative
        'pid_sat',     # output_saturated
        'correction',  # output_smoothed
    )}

    for k in range(N):
        t = t_arr[k]

        # 1. Ramp disturbance
        platform = A * math.sin(w * t + phi)
        buf['platform'][k] = platform

        # 2. Body dynamics: first-order lag toward (ramp − correction_effect)
        equil = platform - CTRL_GAIN * corr_lpf
        body_tilt += ALPHA_B * (equil - body_tilt)

        # 3. IMU + EMA filter
        raw = body_tilt + rng.normal(0.0, noise_std)
        meas = LPF_RAW * meas + (1.0 - LPF_RAW) * raw
        buf['response'][k] = meas

        # 4. Error with deadzone
        err = 0.0 - _dz(meas, DEADZONE)
        buf['error'][k] = err

        # 5. PID computation
        gs   = _gs(err)
        prop = KP * gs * err
        buf['p_term'][k] = prop

        integ *= DECAY_INTEG
        integ += err * DT
        integ  = max(-SAT_INTEG, min(SAT_INTEG, integ))
        i_out  = KI * gs * integ
        buf['i_term'][k] = i_out

        d_raw = KD * (err - err_prev) / DT
        deriv = LPF_DERIV * deriv + (1.0 - LPF_DERIV) * d_raw
        if abs(err) < RAMP_DERIV:
            deriv = 0.0
        buf['d_term'][k] = deriv

        # 6. Sum & saturate
        raw_corr = prop + i_out + deriv
        corr_sat = max(-SAT_CORR, min(SAT_CORR, raw_corr))

        # Anti-windup back-calculation
        if abs(raw_corr) > SAT_CORR and KI > 0:
            excess = raw_corr - corr_sat
            integ -= excess / KI * 0.5
            integ  = max(-SAT_INTEG, min(SAT_INTEG, integ))
        buf['pid_sat'][k] = corr_sat

        # 7. Final LPF
        corr_lpf = LPF_CORR * corr_lpf + (1.0 - LPF_CORR) * corr_sat
        buf['correction'][k] = corr_lpf
        err_prev = err

    return buf


# ── Per-signal plot configuration ─────────────────────────────────────────────
# Each entry: (key, label, ylabel, color, unit_scale, zero_line, show_dz)
# unit_scale: 1.0 = keep in radians, math.degrees equivalent = convert to deg
DEG = math.degrees(1.0)   # multiply rad → deg

SIGNAL_CONFIG = [
    # key          label (paper)          y-axis label      color      scale  zero   dz
    ('platform',   'Platform Disturbance', 'Angle (deg)',  '#D97706', DEG,  False, False),
    ('response',   'IMU Response',         'Angle (deg)',  '#2563EB', DEG,  False, False),
    ('error',      'PID Error',            'Error (deg)',  '#DC2626', DEG,  True,  True ),
    ('p_term',     'P Term',               'Correction (rad)', '#7C3AED', 1.0, True, False),
    ('i_term',     'I Term',               'Correction (rad)', '#0891B2', 1.0, True, False),
    ('d_term',     'D Term',               'Correction (rad)', '#BE185D', 1.0, True, False),
    ('pid_sat',    'PID Output (Sat.)',    'Correction (rad)', '#65A30D', 1.0, True, False),
    ('correction', 'PID Correction (LPF)','Correction (rad)', '#059669', 1.0, True, False),
]


def make_figure(axis: str, amplitude_deg: float, period_sec: float,
                phase_deg: float = 0.0, fname: str = 'pid_balance_roll.png',
                seed: int = 42):
    """
    Generate and save an 8-subplot figure for one balance axis.
    Signal names in subplot titles use the form:  {axis}_{signal}
    e.g. roll_platform, roll_response, roll_error, roll_p_term, ...
    """
    data = simulate(amplitude_deg, period_sec, phase_deg, seed=seed)
    AX   = axis           # 'roll' or 'pitch'
    dz_d = math.degrees(DEADZONE)

    fig, axs = plt.subplots(8, 1, figsize=(5.5, 13.0), sharex=True)
    fig.subplots_adjust(hspace=0.60, top=0.955, bottom=0.045)

    for i, (key, label, ylabel, color, scale, zero_line, show_dz) in \
            enumerate(SIGNAL_CONFIG):

        ax = axs[i]
        y  = data[key] * scale
        signal_name = f'{AX}_{key}'   # e.g. roll_platform, pitch_response

        ax.plot(t_arr, y, color=color, label=signal_name)

        if zero_line:
            ax.axhline(0, color='gray', linewidth=0.6)
        if show_dz:
            ax.axhspan(-dz_d, dz_d, alpha=0.10, color='gray',
                       label=f'Deadzone \u00b1{dz_d:.1f}\u00b0')

        ax.set_ylabel(ylabel, fontsize=7)
        # Title: letter + signal name + description
        letter = chr(ord('a') + i)
        ax.set_title(f'({letter}) {signal_name}  \u2014  {label}',
                     fontsize=7.5, pad=2)
        ax.legend(loc='upper right', fontsize=6.5, framealpha=0.7)

    axs[-1].set_xlabel('Time (s)', fontsize=8)

    fig.suptitle(
        f'PID Balance Controller \u2014 {AX.capitalize()} Axis\n'
        f'Platform Disturbance: {amplitude_deg:.0f}\u00b0 amplitude, '
        f'{period_sec:.0f} s period',
        fontsize=10, fontweight='bold')

    path = os.path.join(OUTPUT_DIR, fname)
    fig.savefig(path, bbox_inches='tight')
    plt.close(fig)
    print(f'  [OK] {path}')


if __name__ == '__main__':
    print('Generating 8-signal PID balance diagnostic plots...')

    make_figure('roll',  10.0, 30.0, phase_deg= 0.0,
                fname='pid_balance_roll.png',  seed=42)
    make_figure('pitch', 10.0, 30.0, phase_deg=90.0,
                fname='pid_balance_pitch.png', seed=7)

    print('Done. Check paper/figures/ for outputs.')
