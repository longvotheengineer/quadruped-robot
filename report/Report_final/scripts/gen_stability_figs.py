import matplotlib.pyplot as plt
import matplotlib.patches as patches
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

# ==========================================
# Figure 1: Static Stability (SSM)
# ==========================================
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 5))

L = 2.0  # Half-length
W = 1.0  # Half-width
legs = {'FL': (L, W), 'FR': (L, -W), 'HL': (-L, W), 'HR': (-L, -W)}

# Left subplot: 3-leg stance (Walk)
ax1.set_xlim(-L-0.5, L+0.5)
ax1.set_ylim(-W-0.5, W+0.5)
ax1.set_aspect('equal')
ax1.axis('off')
# ax1.set_title("Walk Gait (3 chân tiếp đất)\nĐa giác đỡ & Biên ổn định tĩnh (SSM)", pad=20)

# Draw robot body outline
rect = patches.Rectangle((-L, -W), 2*L, 2*W, linewidth=1, edgecolor='gray', facecolor='lightgray', alpha=0.3, linestyle='--')
ax1.add_patch(rect)

# Draw legs
stance_legs = ['FR', 'HL', 'HR']
swing_legs = ['FL']

for name, pos in legs.items():
    if name in stance_legs:
        ax1.plot(pos[0], pos[1], 'ko', markersize=8)
        ax1.text(pos[0]+0.1, pos[1]+0.1, name, fontsize=10)
    else:
        ax1.plot(pos[0], pos[1], 'wo', markeredgecolor='k', markersize=8)
        ax1.text(pos[0]+0.1, pos[1]+0.1, f"{name} (Swing)", fontsize=10, color='gray')

# Draw support polygon
poly_points = np.array([legs['FR'], legs['HL'], legs['HR']])
poly = patches.Polygon(poly_points, closed=True, fill=True, color='green', alpha=0.2)
ax1.add_patch(poly)

# CoM
com = np.array([0, 0])
ax1.plot(com[0], com[1], 'r+', markersize=12, markeredgewidth=2)
ax1.text(com[0]-0.2, com[1]+0.2, "CoM", color='red', fontweight='bold')

# Calculate SSM (distance from CoM to line segment FR-HL)
p1 = np.array(legs['FR'])
p2 = np.array(legs['HL'])
# Line from p2 to p1
line_vec = p1 - p2
line_vec_norm = line_vec / np.linalg.norm(line_vec)
com_vec = com - p2
proj_len = np.dot(com_vec, line_vec_norm)
proj_pt = p2 + proj_len * line_vec_norm

# Draw SSM line
ax1.plot([com[0], proj_pt[0]], [com[1], proj_pt[1]], 'b--', linewidth=2)
ax1.text((com[0]+proj_pt[0])/2 - 0.5, (com[1]+proj_pt[1])/2 + 0.1, "SSM", color='blue', fontweight='bold')

# Right subplot: Trot stance
ax2.set_xlim(-L-0.5, L+0.5)
ax2.set_ylim(-W-0.5, W+0.5)
ax2.set_aspect('equal')
ax2.axis('off')
# ax2.set_title("Trot Gait (2 chân chéo tiếp đất)\nĐa giác đỡ suy biến (SSM $\\approx$ 0)", pad=20)

# Draw robot body outline
rect2 = patches.Rectangle((-L, -W), 2*L, 2*W, linewidth=1, edgecolor='gray', facecolor='lightgray', alpha=0.3, linestyle='--')
ax2.add_patch(rect2)

stance_legs_trot = ['FL', 'HR']
for name, pos in legs.items():
    if name in stance_legs_trot:
        ax2.plot(pos[0], pos[1], 'ko', markersize=8)
        ax2.text(pos[0]+0.1, pos[1]+0.1, name, fontsize=10)
    else:
        ax2.plot(pos[0], pos[1], 'wo', markeredgecolor='k', markersize=8)
        ax2.text(pos[0]+0.1, pos[1]+0.1, f"{name} (Swing)", fontsize=10, color='gray')

# Draw support line (degenerated polygon)
ax2.plot([legs['FL'][0], legs['HR'][0]], [legs['FL'][1], legs['HR'][1]], 'g-', linewidth=3, alpha=0.5)

# CoM
ax2.plot(com[0], com[1], 'r+', markersize=12, markeredgewidth=2)
ax2.text(com[0]+0.2, com[1]+0.2, "CoM", color='red', fontweight='bold')

plt.tight_layout()
plt.savefig('img/fig_static_stability.pdf', dpi=300, bbox_inches='tight')
plt.close()

# ==========================================
# Figure 2: Dynamic Stability (ZMP)
# ==========================================
fig, ax = plt.subplots(figsize=(8, 6))

H = 2.0 # Height of CoM
L_leg = 1.5

ax.set_xlim(-L_leg-1, L_leg+2)
ax.set_ylim(-0.5, H+1)
ax.axis('off')

# Ground
ax.plot([-L_leg-1, L_leg+2], [0, 0], 'k-', linewidth=2)

# Robot body
body = patches.Rectangle((-1, H-0.2), 2, 0.4, linewidth=2, edgecolor='k', facecolor='lightgray', zorder=2)
ax.add_patch(body)

# Legs
ax.plot([-L_leg, -0.5], [0, H], 'k-', linewidth=4, zorder=1) # Rear leg
ax.plot([L_leg-0.5, 0.5], [0, H], 'k-', linewidth=4, zorder=1) # Front leg (swing)
ax.plot([L_leg, 0.5], [0, H], 'k-', linewidth=4, zorder=1) # Front leg (stance)

# CoM
com = np.array([0, H])
ax.plot(com[0], com[1], 'ro', markersize=10, zorder=3)
ax.text(com[0]+0.2, com[1]+0.2, "Trọng tâm (CoM)", color='red', fontweight='bold')

# Forces
gravity = np.array([0, -1.2])
inertial = np.array([-0.8, 0]) # Acceleration is to the right, so D'Alembert force is to the left
resultant = gravity + inertial

ax.annotate("", xy=com+gravity, xytext=com, arrowprops=dict(arrowstyle="->", color="blue", lw=2))
ax.text(com[0]+0.1, com[1]-0.6, "Trọng lực\n($F_g = mg$)", color='blue')

ax.annotate("", xy=com+inertial, xytext=com, arrowprops=dict(arrowstyle="->", color="orange", lw=2))
ax.text(com[0]-1.8, com[1]+0.1, "Lực quán tính\n($F_{in} = -ma$)", color='orange')

ax.annotate("", xy=com+resultant, xytext=com, arrowprops=dict(arrowstyle="->", color="purple", lw=3))
ax.text(com[0]-0.8, com[1]-0.8, "Lực tổng hợp ($F_{res}$)", color='purple', fontweight='bold')

# ZMP line
# line from com along resultant
zmp_x = com[0] - com[1] * (resultant[0]/resultant[1])
ax.plot([com[0], zmp_x], [com[1], 0], 'm--', linewidth=1.5)

# ZMP Point
ax.plot(zmp_x, 0, 'go', markersize=10, zorder=3)
ax.text(zmp_x-0.2, -0.3, "ZMP", color='green', fontweight='bold', fontsize=14)

# Support Polygon boundary (1D line segment from rear to front stance leg)
ax.plot([-L_leg, L_leg], [0, 0], 'g-', linewidth=5, alpha=0.5, solid_capstyle='round')
ax.text(0, -0.2, "Đa giác đỡ (Support Polygon)", color='green')

# Direction of motion
ax.annotate("", xy=(2.5, H), xytext=(1.5, H), arrowprops=dict(arrowstyle="->", color="black", lw=2))
ax.text(1.8, H+0.2, "Hướng di chuyển\n(Gia tốc $a$)", color='black', ha='center')

# plt.title("Minh họa Ổn định động & Điểm Moment Không (ZMP)\nNhìn từ mặt bên (Sagittal Plane)")
plt.tight_layout()
plt.savefig('img/fig_zmp_stability.pdf', dpi=300, bbox_inches='tight')
plt.close()

print("Figures generated successfully!")
