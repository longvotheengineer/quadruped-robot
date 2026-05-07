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

# Generate time vector
tau_full = np.linspace(-0.5, 1.5, 1000)

s3, v3, a3 = np.zeros_like(tau_full), np.zeros_like(tau_full), np.zeros_like(tau_full)
s5, v5, a5 = np.zeros_like(tau_full), np.zeros_like(tau_full), np.zeros_like(tau_full)

for i, t in enumerate(tau_full):
    s3[i], v3[i], a3[i] = compute_cubic(t)
    s5[i], v5[i], a5[i] = compute_quintic(t)


fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 10))

# 1. Position Plot
ax1.plot(tau_full, s3, 'b--', linewidth=2, label='Đa thức bậc 3 (Cubic)')
ax1.plot(tau_full, s5, 'r-', linewidth=2, label='Đa thức bậc 5 (Quintic)')
ax1.set_ylabel('$s(\\tau)$')
ax1.grid(True, linestyle=':', alpha=0.7)
ax1.legend(loc='upper left')
ax1.set_xlim(-0.5, 1.5)
ax1.axvspan(-0.5, 0, color='gray', alpha=0.1)
ax1.axvspan(1, 1.5, color='gray', alpha=0.1)

# 2. Velocity Plot
ax2.plot(tau_full, v3, 'b--', linewidth=2, label='Đa thức bậc 3 (Cubic)')
ax2.plot(tau_full, v5, 'r-', linewidth=2, label='Đa thức bậc 5 (Quintic)')
ax2.set_ylabel('$\\dot{s}(\\tau)$')
ax2.grid(True, linestyle=':', alpha=0.7)
ax2.set_xlim(-0.5, 1.5)
ax2.axvspan(-0.5, 0, color='gray', alpha=0.1)
ax2.axvspan(1, 1.5, color='gray', alpha=0.1)
# Highlight start/end velocity
ax2.plot([0, 1], [0, 0], 'ko')

# 3. Acceleration Plot
ax3.plot(tau_full, a3, 'b--', linewidth=2, label='Đa thức bậc 3 (Cubic)')
ax3.plot(tau_full, a5, 'r-', linewidth=2, label='Đa thức bậc 5 (Quintic)')
ax3.set_ylabel('$\\ddot{s}(\\tau)$')
ax3.set_xlabel('Thời gian chuẩn hóa $\\tau$')
ax3.grid(True, linestyle=':', alpha=0.7)
ax3.set_xlim(-0.5, 1.5)
ax3.axvspan(-0.5, 0, color='gray', alpha=0.1)
ax3.axvspan(1, 1.5, color='gray', alpha=0.1)

# Highlight acceleration differences at boundaries
ax3.plot([0, 1], [6, -6], 'bo', label='Gia tốc bậc 3 khác không')
ax3.plot([0, 1], [0, 0], 'ro', label='Gia tốc bậc 5 bằng không')
ax3.legend(loc='upper right')

plt.tight_layout()
plt.savefig('img/fig_cubic_vs_quintic.pdf', dpi=300, bbox_inches='tight')
plt.close()

print("Comparison figure generated at img/fig_cubic_vs_quintic.pdf")
