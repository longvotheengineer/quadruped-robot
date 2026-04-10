import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import rcParams
import math
import os

rcParams.update({
    'font.family': 'serif',
    'font.serif': ['Times New Roman', 'DejaVu Serif'],
    'font.size': 10,
    'axes.labelsize': 11,
    'axes.titlesize': 12,
    'legend.fontsize': 9,
    'xtick.labelsize': 9,
    'ytick.labelsize': 9,
    'figure.dpi': 300,
    'savefig.dpi': 300,
    'savefig.bbox': 'tight',
    'savefig.pad_inches': 0.05,
    'axes.grid': True,
    'grid.alpha': 0.3,
    'grid.linestyle': '--',
    'lines.linewidth': 1.5,
    'axes.spines.top': False,
    'axes.spines.right': False,
})

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', 'figures')

L, W = 250, 193
l1, l2, l3 = 45, 107, 116
T_swing = 70
T_stance = 200
stride_length = 45
x_center = 125
y_val = 135
z_stance = -170
z_swing = -130
lift_height = z_swing - z_stance

# ══════════════════════════════════════════════════════════════════════
# PLOT 1: Quintic PVA — legend OUTSIDE the plot
# ══════════════════════════════════════════════════════════════════════
def plot_quintic_pva():
    tau = np.linspace(0, 1, 500)
    s   = 10*tau**3 - 15*tau**4 + 6*tau**5
    ds  = 30*tau**2 - 60*tau**3 + 30*tau**4
    dds = 60*tau   - 180*tau**2 + 120*tau**3

    fig, ax = plt.subplots(figsize=(5.5, 3.8))

    ax.plot(tau, s,   color='#2563EB', linewidth=2.5, label=r'Position $s(\tau)$')
    ax.plot(tau, ds,  color='#059669', linewidth=2.5, label=r'Velocity $\dot{s}(\tau)$')
    ax.plot(tau, dds, color='#D97706', linewidth=2.5, label=r'Acceleration $\ddot{s}(\tau)$')
    ax.axhline(0, color='gray', linestyle='-', linewidth=0.8)

    ax.set_xlabel(r'Normalized time $\tau$')
    ax.set_ylabel('Amplitude')
    ax.set_title('Quintic Polynomial Trajectory Profile')

    # Legend BELOW the plot, well spaced, no overlap
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.18),
              ncol=3, fontsize=9, frameon=True, fancybox=True,
              columnspacing=1.5, handlelength=2.0)

    fig.subplots_adjust(bottom=0.25)
    path = os.path.join(OUTPUT_DIR, 'quintic_pva.png')
    fig.savefig(path, bbox_inches='tight')
    plt.close(fig)
    print(f'  [OK] {path}')


# ══════════════════════════════════════════════════════════════════════
# PLOT 2: Single 3D trajectory with XZ-style legends
# ══════════════════════════════════════════════════════════════════════
def plot_gait_trajectory_3d():
    x_forward  = x_center + stride_length / 2
    x_backward = x_center - stride_length / 2

    swing_x, swing_y, swing_z = [], [], []
    for i in range(T_swing):
        tau = i / (T_swing - 1)
        s = 10*tau**3 - 15*tau**4 + 6*tau**5
        swing_x.append(x_backward + s * (x_forward - x_backward))
        swing_y.append(y_val)
        swing_z.append(z_stance + lift_height * 4 * s * (1 - s))

    stance_x, stance_y, stance_z = [], [], []
    for i in range(T_stance):
        tau = i / (T_stance - 1)
        s = 10*tau**3 - 15*tau**4 + 6*tau**5
        stance_x.append(x_forward + s * (x_backward - x_forward))
        stance_y.append(y_val)
        stance_z.append(z_stance)

    fig = plt.figure(figsize=(5.5, 4.5))
    ax  = fig.add_subplot(111, projection='3d')

    # Swing & Stance lines
    ax.plot(swing_x, swing_y, swing_z,
            color='#2563EB', linewidth=2.5, label='Swing phase', zorder=3)
    ax.plot(stance_x, stance_y, stance_z,
            color='#DC2626', linewidth=2.5, linestyle='--', label='Stance phase', zorder=3)

    # Liftoff & Touchdown markers (same style as XZ plot)
    ax.scatter([x_backward], [y_val], [z_stance],
               marker='s', color='#059669', s=60, zorder=5,
               label=r'Liftoff $\mathbf{A}$')
    ax.scatter([x_forward],  [y_val], [z_stance],
               marker='D', color='#D97706', s=60, zorder=5,
               label=r'Touchdown $\mathbf{D}$')

    ax.set_xlabel('X (mm)', fontsize=8, labelpad=2)
    ax.set_ylabel('Y (mm)', fontsize=8, labelpad=2)
    ax.set_zlabel('Z (mm)', fontsize=8, labelpad=2)
    ax.set_title('3D Foot Trajectory (Left Front)')

    # Shrink tick labels so they don't crowd
    ax.tick_params(axis='x', labelsize=7, pad=0)
    ax.tick_params(axis='y', labelsize=7, pad=0)
    ax.tick_params(axis='z', labelsize=7, pad=0)

    ax.view_init(elev=22, azim=-55)

    # Legend outside the 3D axes, below the plot
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.05),
              ncol=2, fontsize=8, frameon=True, fancybox=True,
              columnspacing=1.2, handlelength=2.0)

    fig.subplots_adjust(bottom=0.18)
    path = os.path.join(OUTPUT_DIR, 'gait_trajectory_3d.png')
    fig.savefig(path, bbox_inches='tight')
    plt.close(fig)
    print(f'  [OK] {path}')


if __name__ == '__main__':
    print('Regenerating plots...')
    plot_quintic_pva()
    plot_gait_trajectory_3d()
    print('Done.')
