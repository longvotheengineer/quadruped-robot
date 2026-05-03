import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import math
from matplotlib.lines import Line2D

# ── Style ──────────────────────────────────────────────────────
plt.rcParams.update({
    'font.size': 11,
    'font.family': 'serif',
    'mathtext.fontset': 'cm',
    'axes.linewidth': 0,
})

fig, ax = plt.subplots(figsize=(11, 7))

# ── Parameters ─────────────────────────────────────────────────
pitch = math.radians(12)
L = 5.0
H = 0.8
ground_y = -4.2

def rot2d(theta):
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]])

R = rot2d(pitch)

# ── Ground plane ───────────────────────────────────────────────
ax.plot([-5.5, 6.5], [ground_y, ground_y], 'k-', linewidth=2.5, zorder=1)
for xh in np.arange(-5.5, 6.5, 0.4):
    ax.plot([xh, xh - 0.3], [ground_y, ground_y - 0.25],
            'k-', linewidth=0.5, alpha=0.35)

# ── Body (tilted rectangle) ────────────────────────────────────
body_local = np.array([
    [-L/2, -H/2], [L/2, -H/2], [L/2, H/2], [-L/2, H/2],
])
body_world = (R @ body_local.T).T
body_patch = patches.Polygon(body_world, closed=True, fill=True,
                             facecolor='#D6E4F0', edgecolor='#2C3E50',
                             linewidth=2.5, zorder=5)
ax.add_patch(body_patch)

# ── Body label ─────────────────────────────────────────────────
label_angle = math.degrees(pitch)
body_center_up = R @ np.array([0, H/2 + 0.35])
ax.text(body_center_up[0], body_center_up[1],
        'Thân robot (nghiêng góc $\\theta$)',
        ha='center', va='bottom', fontsize=12, fontstyle='italic',
        color='#2C3E50', rotation=label_angle)

# ── Body coordinate frame {B} ─────────────────────────────────
arrow_len = 1.3
xb_dir = R @ np.array([arrow_len, 0])
ax.annotate('', xy=xb_dir, xytext=(0, 0),
            arrowprops=dict(arrowstyle='->', color='#E74C3C', lw=2.5), zorder=10)
ax.text(xb_dir[0] + 0.2, xb_dir[1] + 0.1, '$X_B$',
        color='#E74C3C', fontsize=13, fontweight='bold', zorder=10)

zb_dir = R @ np.array([0, arrow_len])
ax.annotate('', xy=zb_dir, xytext=(0, 0),
            arrowprops=dict(arrowstyle='->', color='#2980B9', lw=2.5), zorder=10)
ax.text(zb_dir[0] - 0.35, zb_dir[1] + 0.15, '$Z_B$',
        color='#2980B9', fontsize=13, fontweight='bold', zorder=10)

# ── Hip positions ──────────────────────────────────────────────
hip_front_local = np.array([L/2 - 0.3, -H/2])
hip_rear_local  = np.array([-L/2 + 0.3, -H/2])
hip_front = R @ hip_front_local
hip_rear  = R @ hip_rear_local

# ── Raw foot positions ─────────────────────────────────────────
Z_nominal = -3.2
foot_front_raw_local = np.array([L/2 - 0.3, Z_nominal])
foot_rear_raw_local  = np.array([-L/2 + 0.3, Z_nominal])
foot_front_raw = R @ foot_front_raw_local
foot_rear_raw  = R @ foot_rear_raw_local

# ── Adjusted foot positions ───────────────────────────────────
z_adj_front = (ground_y - math.sin(pitch)*(L/2 - 0.3)) / math.cos(pitch)
z_adj_rear  = (ground_y - math.sin(pitch)*(-L/2 + 0.3)) / math.cos(pitch)
foot_front_adj = R @ np.array([L/2 - 0.3, z_adj_front])
foot_rear_adj  = R @ np.array([-L/2 + 0.3, z_adj_rear])

# ── Draw raw legs (dashed red) ─────────────────────────────────
raw_kw = dict(color='#E74C3C', linewidth=2.0, linestyle='--', alpha=0.55, zorder=3)
ax.plot([hip_front[0], foot_front_raw[0]],
        [hip_front[1], foot_front_raw[1]], **raw_kw)
ax.plot([hip_rear[0],  foot_rear_raw[0]],
        [hip_rear[1],  foot_rear_raw[1]],  **raw_kw)
ax.plot(*foot_front_raw, 'o', color='#E74C3C', ms=8, alpha=0.65, zorder=6)
ax.plot(*foot_rear_raw,  'o', color='#E74C3C', ms=8, alpha=0.65, zorder=6)

# ── Draw adjusted legs (solid green) ──────────────────────────
adj_kw = dict(color='#27AE60', linewidth=3.0, linestyle='-', zorder=4)
ax.plot([hip_front[0], foot_front_adj[0]],
        [hip_front[1], foot_front_adj[1]], **adj_kw)
ax.plot([hip_rear[0],  foot_rear_adj[0]],
        [hip_rear[1],  foot_rear_adj[1]],  **adj_kw)
ax.plot(*foot_front_adj, 's', color='#27AE60', ms=10, zorder=6)
ax.plot(*foot_rear_adj,  's', color='#27AE60', ms=10, zorder=6)

# ── Delta-Z arrows ─────────────────────────────────────────────
# Front foot (right side — more space)
ax.annotate('', xy=foot_front_adj, xytext=foot_front_raw,
            arrowprops=dict(arrowstyle='->', color='#8E44AD', lw=2.2,
                            connectionstyle='arc3,rad=0.25'), zorder=7)
ax.text(foot_front_raw[0] + 1.0,
        (foot_front_raw[1] + foot_front_adj[1]) / 2 + 0.1,
        '$\\Delta Z_B$', color='#8E44AD', fontsize=14, fontweight='bold',
        ha='left', va='center', zorder=8)

# Rear foot (left side)
ax.annotate('', xy=foot_rear_adj, xytext=foot_rear_raw,
            arrowprops=dict(arrowstyle='->', color='#8E44AD', lw=2.2,
                            connectionstyle='arc3,rad=-0.25'), zorder=7)
ax.text(foot_rear_raw[0] - 1.2,
        (foot_rear_raw[1] + foot_rear_adj[1]) / 2,
        '$\\Delta Z_B$', color='#8E44AD', fontsize=14, fontweight='bold',
        ha='right', va='center', zorder=8)

# ── Foot labels (carefully positioned) ─────────────────────────
# --- Front ---
ax.annotate('$\\mathbf{p}_{raw}$',
            xy=foot_front_raw,
            xytext=(foot_front_raw[0] + 1.2, foot_front_raw[1] + 0.5),
            fontsize=13, color='#C0392B',
            arrowprops=dict(arrowstyle='->', color='#C0392B', lw=1, alpha=0.5),
            zorder=8)

ax.annotate('$\\mathbf{p}_{adj}$',
            xy=foot_front_adj,
            xytext=(foot_front_adj[0] + 1.2, foot_front_adj[1] + 0.5),
            fontsize=13, color='#1E8449', fontweight='bold',
            arrowprops=dict(arrowstyle='->', color='#1E8449', lw=1, alpha=0.5),
            zorder=8)

# --- Rear ---
ax.annotate('$\\mathbf{p}_{raw}$',
            xy=foot_rear_raw,
            xytext=(foot_rear_raw[0] - 2.0, foot_rear_raw[1] + 1.0),
            fontsize=13, color='#C0392B',
            arrowprops=dict(arrowstyle='->', color='#C0392B', lw=1, alpha=0.5),
            zorder=8)

ax.annotate('$\\mathbf{p}_{adj}$',
            xy=foot_rear_adj,
            xytext=(foot_rear_adj[0] - 2.5, foot_rear_adj[1] - 0.3),
            fontsize=13, color='#1E8449', fontweight='bold',
            arrowprops=dict(arrowstyle='->', color='#1E8449', lw=1, alpha=0.5),
            zorder=8)

# ── "X,Y unchanged" annotation ─────────────────────────────────
ax.annotate(
    '$X_B$, $Y_B$ giữ nguyên\n(chỉ đổi $Z_B$)',
    xy=(foot_front_adj[0], ground_y),
    xytext=(foot_front_adj[0] + 1.0, ground_y - 0.9),
    fontsize=10, color='#7F8C8D', fontstyle='italic',
    ha='center', va='top',
    arrowprops=dict(arrowstyle='->', color='#BDC3C7', lw=1.2),
    zorder=8)

# ── Ground label ───────────────────────────────────────────────
ax.text(0.5, ground_y + 0.25, 'Mặt phẳng chuyển động (Ground)',
        fontsize=11, color='#2C3E50', fontstyle='italic', ha='center')

# ── World coordinate frame ─────────────────────────────────────
ow = np.array([-5.0, ground_y])
aw = 0.8
ax.annotate('', xy=ow + [aw, 0], xytext=ow,
            arrowprops=dict(arrowstyle='->', color='gray', lw=1.5))
ax.text(ow[0] + aw + 0.1, ow[1], '$X_W$', color='gray', fontsize=11)
ax.annotate('', xy=ow + [0, aw], xytext=ow,
            arrowprops=dict(arrowstyle='->', color='gray', lw=1.5))
ax.text(ow[0] - 0.05, ow[1] + aw + 0.15, '$Z_W$', color='gray', fontsize=11)
ax.text(ow[0] - 0.15, ow[1] - 0.2, '$O_W$', color='gray', fontsize=10, ha='right')

# ── Angle arc ──────────────────────────────────────────────────
arc_r = 2.0
angle_arc = np.linspace(0, pitch, 30)
ax.plot(arc_r * np.cos(angle_arc), arc_r * np.sin(angle_arc),
        '-', color='#E67E22', linewidth=1.8, zorder=6)
ax.text(arc_r * math.cos(pitch/2) + 0.25,
        arc_r * math.sin(pitch/2) + 0.25,
        '$\\theta$', color='#E67E22', fontsize=15, fontweight='bold', zorder=8)

# ── Legend ─────────────────────────────────────────────────────
legend_elements = [
    Line2D([0], [0], color='#E74C3C', lw=2, ls='--',
           label='Chân chưa bù ($\\mathbf{p}_{raw}$)'),
    Line2D([0], [0], color='#27AE60', lw=3, ls='-',
           label='Chân sau bù ($\\mathbf{p}_{adj}$, chỉ đổi $Z_B$)'),
    Line2D([0], [0], color='#8E44AD', lw=2, ls='-', marker='>', ms=5,
           label='Hiệu chỉnh $\\Delta Z_B$ (dọc trục thân)'),
]
ax.legend(handles=legend_elements, loc='upper right', fontsize=10,
          framealpha=0.92, edgecolor='#BDC3C7', fancybox=True)

# ── Final ──────────────────────────────────────────────────────
ax.set_aspect('equal')
ax.set_xlim(-5.8, 7.5)
ax.set_ylim(-5.8, 3.0)
ax.axis('off')

# plt.title('Minh họa cơ chế điều chỉnh tư thế giữ nguyên X, Y cục bộ',
#           fontsize=15, pad=15, fontweight='bold', color='#2C3E50')
plt.tight_layout()
plt.savefig('img/fig_attitude_adjustment.pdf', dpi=300, bbox_inches='tight')
plt.savefig('img/fig_attitude_adjustment.png', dpi=300, bbox_inches='tight')
print("Generated img/fig_attitude_adjustment.pdf")
print("Generated img/fig_attitude_adjustment.png")
