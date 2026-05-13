import os
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from run_simulation import run_scenario, T_TOTAL, DT
from visualize import C

def generate_individual_gait_plots():
    print("Running GAIT simulation for thesis plots...")
    result = run_scenario("GAIT Balance", mode="gait")
    gait = result['gait']
    filt = result['filter']
    t = filt['time']

    # We want a zoomed in version, let's say from t=15 to t=25
    mask = (t >= 15.0) & (t <= 25.0)
    t_zoom = t[mask]

    out_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../thesis/presentation/images/results'))
    os.makedirs(out_dir, exist_ok=True)

    def setup_plot(title, ylabel):
        fig, ax = plt.subplots(figsize=(10, 4.5))
        ax.set_facecolor('#161b22')
        fig.patch.set_facecolor('#0d1117')
        ax.tick_params(colors='#8b949e', labelsize=14)
        ax.grid(color='#21262d', linestyle='--', alpha=0.7)
        ax.set_title(title, color='#c9d1d9', fontsize=18, fontweight='bold', pad=15)
        ax.set_xlabel('Time (s)', color='#c9d1d9', fontsize=16)
        ax.set_ylabel(ylabel, color='#c9d1d9', fontsize=16)
        for spine in ax.spines.values():
            spine.set_color('#30363d')
        return fig, ax

    def finish_plot(fig, ax, filename):
        ax.legend(fontsize=14, loc='upper right', facecolor='#161b22', edgecolor='#30363d', labelcolor='#c9d1d9')
        fig.tight_layout()
        filepath = os.path.join(out_dir, filename)
        fig.savefig(filepath, dpi=300)
        plt.close(fig)
        print(f"Saved {filepath}")

    # --- 1. Pitch Meas/Avg ---
    fig, ax = setup_plot("Pitch Controller: Measurement vs Moving Average", "Angle (rad)")
    ax.plot(t_zoom, filt['meas_p'][mask], color=C['cyan'], alpha=0.5, lw=1.5, label='EMA Measurement')
    ax.plot(t_zoom, gait['pitch_avg'][mask], color=C['red'], lw=2.5, label='Moving Average')
    ax.axhline(0.03, ls='--', color=C['orange'], alpha=0.6, lw=1.5, label='Deadzone (+0.03)')
    ax.axhline(-0.03, ls='--', color=C['orange'], alpha=0.6, lw=1.5, label='Deadzone (-0.03)')
    finish_plot(fig, ax, 'trot_pid_pitch_controller_meas_avg_zoom.png')

    # --- 2. Roll Meas/Avg ---
    fig, ax = setup_plot("Roll Controller: Measurement vs Moving Average", "Angle (rad)")
    ax.plot(t_zoom, filt['meas_r'][mask], color=C['cyan'], alpha=0.5, lw=1.5, label='EMA Measurement')
    ax.plot(t_zoom, gait['roll_avg'][mask], color=C['red'], lw=2.5, label='Moving Average')
    ax.axhline(0.03, ls='--', color=C['orange'], alpha=0.6, lw=1.5, label='Deadzone (+0.03)')
    ax.axhline(-0.03, ls='--', color=C['orange'], alpha=0.6, lw=1.5, label='Deadzone (-0.03)')
    finish_plot(fig, ax, 'trot_pid_roll_controller_meas_avg_zoom.png')

    # --- 3. Pitch P I D ---
    fig, ax = setup_plot("Pitch Controller: P-I-D Components", "Output (rad)")
    ax.plot(t_zoom, gait['pitch_p'][mask], color=C['cyan'], lw=2.0, label='Proportional (P)')
    ax.plot(t_zoom, gait['pitch_i'][mask], color=C['red'], lw=2.0, label='Integral (I)')
    ax.plot(t_zoom, gait['pitch_d'][mask], color=C['green'], lw=2.0, label='Derivative (D)')
    finish_plot(fig, ax, 'trot_pid_pitch_controller_p_i_d_zoom.png')

    # --- 4. Roll P I D ---
    fig, ax = setup_plot("Roll Controller: P-I-D Components", "Output (rad)")
    ax.plot(t_zoom, gait['roll_p'][mask], color=C['cyan'], lw=2.0, label='Proportional (P)')
    ax.plot(t_zoom, gait['roll_i'][mask], color=C['red'], lw=2.0, label='Integral (I)')
    ax.plot(t_zoom, gait['roll_d'][mask], color=C['green'], lw=2.0, label='Derivative (D)')
    finish_plot(fig, ax, 'trot_pid_roll_controller_p_i_d_zoom.png')

    # --- 5. Pitch Sat LPF FF ---
    fig, ax = setup_plot("Pitch Controller: Saturation, LPF & Feed-Forward", "Correction (rad)")
    ax.plot(t_zoom, gait['pitch_corr'][mask], color=C['cyan'], alpha=0.5, lw=1.5, label='Saturated PID Sum')
    ax.plot(t_zoom, gait['pitch_corr_lpf'][mask], color=C['red'], lw=2.5, label='After LPF (α=0.97)')
    ff_signal = np.ones_like(t_zoom) * 0.1
    ax.plot(t_zoom, ff_signal, color=C['green'], lw=2.0, label='Feed-Forward (Ready)')
    finish_plot(fig, ax, 'trot_pid_pitch_controller_sat_lpf_ff_zoom.png')

    # --- 6. Roll Sat LPF FF ---
    fig, ax = setup_plot("Roll Controller: Saturation, LPF & Feed-Forward", "Correction (rad)")
    ax.plot(t_zoom, gait['roll_corr'][mask], color=C['cyan'], alpha=0.5, lw=1.5, label='Saturated PID Sum')
    ax.plot(t_zoom, gait['roll_corr_lpf'][mask], color=C['red'], lw=2.5, label='After LPF (α=0.97)')
    ax.plot(t_zoom, ff_signal, color=C['green'], lw=2.0, label='Feed-Forward (Ready)')
    finish_plot(fig, ax, 'trot_pid_roll_controller_sat_lpf_ff_zoom.png')

if __name__ == '__main__':
    generate_individual_gait_plots()
