import matplotlib.pyplot as plt
import numpy as np
import os

# Create img directory if not exists
os.makedirs('img', exist_ok=True)

# Set common font parameters for academic look
plt.rcParams.update({
    'font.size': 12,
    'font.family': 'serif',
    'axes.labelsize': 12,
    'axes.titlesize': 14,
    'xtick.labelsize': 10,
    'ytick.labelsize': 10,
    'legend.fontsize': 12,
})

def compute_cubic(t):
    # s = 3*t^2 - 2*t^3
    if t < 0:
        return 0, 0, 0
    elif t > 1:
        return 1, 0, 0
    else:
        s = 3 * t**2 - 2 * t**3
        v = 6 * t - 6 * t**2
        a = 6 - 12 * t
        return s, v, a

def compute_quintic(t):
    if t < 0:
        return 0, 0, 0
    elif t > 1:
        return 1, 0, 0
    else:
        s = 10 * t**3 - 15 * t**4 + 6 * t**5
        v = 30 * t**2 - 60 * t**3 + 30 * t**4
        a = 60 * t - 180 * t**2 + 120 * t**3
        return s, v, a

# Generate time vector for 3 cycles: [-0.5, 0] Stance, [0, 1] Swing, [1, 1.5] Stance
tau_full = np.linspace(-0.5, 1.5, 1000)

s3, v3, a3 = np.zeros_like(tau_full), np.zeros_like(tau_full), np.zeros_like(tau_full)
s5, v5, a5 = np.zeros_like(tau_full), np.zeros_like(tau_full), np.zeros_like(tau_full)

for i, t in enumerate(tau_full):
    s3[i], v3[i], a3[i] = compute_cubic(t)
    s5[i], v5[i], a5[i] = compute_quintic(t)

def plot_pva_extended(tau, s, v, a, color, filename):
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 10))

    # 1. Position Plot
    ax1.plot(tau, s, color=color, linewidth=2, label='Position')
    ax1.set_ylabel('$s(\\tau)$')
    ax1.grid(True, linestyle=':', alpha=0.7)
    ax1.set_xlim(-0.5, 1.5)
    ax1.axvspan(-0.5, 0, color='gray', alpha=0.1)
    ax1.axvspan(1, 1.5, color='gray', alpha=0.1)
    ax1.text(-0.25, 0.5, 'Stance phase', ha='center', va='center', alpha=0.5)
    ax1.text(1.25, 0.5, 'Stance phase', ha='center', va='center', alpha=0.5)
    ax1.text(0.5, 0.5, 'Swing phase', ha='center', va='center', alpha=0.5)
    ax1.legend(loc='upper left')

    # 2. Velocity Plot
    ax2.plot(tau, v, color=color, linewidth=2, label='Velocity')
    ax2.set_ylabel('$\\dot{s}(\\tau)$')
    ax2.grid(True, linestyle=':', alpha=0.7)
    ax2.set_xlim(-0.5, 1.5)
    ax2.axvspan(-0.5, 0, color='gray', alpha=0.1)
    ax2.axvspan(1, 1.5, color='gray', alpha=0.1)
    ax2.legend(loc='upper left')

    # 3. Acceleration Plot
    ax3.plot(tau, a, color=color, linewidth=2, label='Acceleration')
    ax3.set_ylabel('$\\ddot{s}(\\tau)$')
    ax3.set_xlabel('Normalized time $\\tau$')
    ax3.grid(True, linestyle=':', alpha=0.7)
    ax3.set_xlim(-0.5, 1.5)
    ax3.axvspan(-0.5, 0, color='gray', alpha=0.1)
    ax3.axvspan(1, 1.5, color='gray', alpha=0.1)
    ax3.legend(loc='upper left')

    plt.tight_layout()
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"Generated {filename}")

plot_pva_extended(tau_full, s3, v3, a3, 'blue', 'img/fig_cubic_pva.pdf')
plot_pva_extended(tau_full, s5, v5, a5, 'red', 'img/fig_quintic_pva.pdf')

def plot_comparison(tau, s3, v3, a3, s5, v5, a5, filename):
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 10))

    # 1. Position Plot
    ax1.plot(tau, s3, color='blue', linewidth=2, linestyle='--', label='Cubic polynomial')
    ax1.plot(tau, s5, color='red', linewidth=2, label='Quintic polynomial')
    ax1.set_ylabel('Position $s(\\tau)$')
    ax1.grid(True, linestyle=':', alpha=0.7)
    ax1.set_xlim(-0.5, 1.5)
    ax1.axvspan(-0.5, 0, color='gray', alpha=0.1)
    ax1.axvspan(1, 1.5, color='gray', alpha=0.1)
    ax1.legend(loc='upper left')

    # 2. Velocity Plot
    ax2.plot(tau, v3, color='blue', linewidth=2, linestyle='--', label='Cubic polynomial')
    ax2.plot(tau, v5, color='red', linewidth=2, label='Quintic polynomial')
    ax2.set_ylabel('Velocity $\\dot{s}(\\tau)$')
    ax2.grid(True, linestyle=':', alpha=0.7)
    ax2.set_xlim(-0.5, 1.5)
    ax2.axvspan(-0.5, 0, color='gray', alpha=0.1)
    ax2.axvspan(1, 1.5, color='gray', alpha=0.1)
    ax2.legend(loc='upper left')

    # 3. Acceleration Plot
    ax3.plot(tau, a3, color='blue', linewidth=2, linestyle='--', label='Cubic polynomial')
    ax3.plot(tau, a5, color='red', linewidth=2, label='Quintic polynomial')
    ax3.set_ylabel('Acceleration $\\ddot{s}(\\tau)$')
    ax3.set_xlabel('Normalized time $\\tau$')
    ax3.grid(True, linestyle=':', alpha=0.7)
    ax3.set_xlim(-0.5, 1.5)
    ax3.axvspan(-0.5, 0, color='gray', alpha=0.1)
    ax3.axvspan(1, 1.5, color='gray', alpha=0.1)
    ax3.legend(loc='upper left')

    plt.tight_layout()
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"Generated {filename}")

plot_comparison(tau_full, s3, v3, a3, s5, v5, a5, 'img/fig_cubic_vs_quintic.pdf')
