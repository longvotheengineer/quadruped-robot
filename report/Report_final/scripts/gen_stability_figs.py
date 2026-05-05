import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import FancyArrowPatch
import numpy as np
import os

# Create img directory if not exists
os.makedirs('img', exist_ok=True)

# Set common font parameters for academic look
plt.rcParams.update({
    'font.size': 11,
    'font.family': 'serif',
    'mathtext.fontset': 'cm',
    'axes.labelsize': 12,
    'axes.titlesize': 13,
    'xtick.labelsize': 10,
    'ytick.labelsize': 10,
    'legend.fontsize': 11,
})

# ==========================================
# Figure 1: Static Stability (SSM)
# Two subplots: Walk gait (3 legs) and Trot gait (2 legs)
# Top-down view with robot body and stick-figure legs
# ==========================================
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6.5))

# Robot dimensions (top-down view)
body_L = 2.2   # half-length along X (front-back)
body_W = 0.9   # half-width along Y (left-right)

# Hip positions (on the body)
hips = {
    'FL': np.array([ body_L,  body_W]),
    'FR': np.array([ body_L, -body_W]),
    'HL': np.array([-body_L,  body_W]),
    'HR': np.array([-body_L, -body_W]),
}

# Foot positions (extended outward from hips)
leg_reach = 1.6
feet = {
    'FL': np.array([ body_L + leg_reach * 0.5,  body_W + leg_reach * 0.7]),
    'FR': np.array([ body_L + leg_reach * 0.5, -body_W - leg_reach * 0.7]),
    'HL': np.array([-body_L - leg_reach * 0.5,  body_W + leg_reach * 0.7]),
    'HR': np.array([-body_L - leg_reach * 0.5, -body_W - leg_reach * 0.7]),
}

# Knee positions (midpoint, offset outward)
knees = {}
for name in hips:
    mid = (hips[name] + feet[name]) / 2
    # Push knee slightly outward/forward
    direction = feet[name] - hips[name]
    perp = np.array([-direction[1], direction[0]])
    perp = perp / np.linalg.norm(perp) * 0.3
    knees[name] = mid + perp


def draw_robot_leg(ax, hip, knee, foot, color='#2c3e50', lw=3.5, alpha=1.0, zorder=3):
    """Draw a 2-segment stick leg: hip -> knee -> foot"""
    # Upper leg (thigh)
    ax.plot([hip[0], knee[0]], [hip[1], knee[1]], color=color,
            linewidth=lw, solid_capstyle='round', alpha=alpha, zorder=zorder)
    # Lower leg (calf)
    ax.plot([knee[0], foot[0]], [knee[1], foot[1]], color=color,
            linewidth=lw - 0.8, solid_capstyle='round', alpha=alpha, zorder=zorder)
    # Hip joint
    ax.plot(hip[0], hip[1], 'o', color='#d63031', markersize=5,
            alpha=alpha, zorder=zorder + 1)
    # Knee joint
    ax.plot(knee[0], knee[1], 'o', color='#d63031', markersize=4,
            alpha=alpha, zorder=zorder + 1)


def draw_foot_marker(ax, pos, stance=True, zorder=5):
    """Draw foot contact point"""
    if stance:
        ax.plot(pos[0], pos[1], 'o', color='#2c3e50', markersize=10, zorder=zorder)
        # Small ground contact circle
        contact = plt.Circle(pos, 0.18, fill=True, facecolor='#2c3e50',
                             edgecolor='#2c3e50', alpha=0.15, zorder=zorder - 1)
        ax.add_patch(contact)
    else:
        ax.plot(pos[0], pos[1], 'o', color='white', markeredgecolor='#95a5a6',
                markeredgewidth=2, markersize=9, zorder=zorder)


def draw_body(ax, body_L, body_W, alpha=0.5):
    """Draw robot body as a rounded rectangle"""
    body_rect = patches.FancyBboxPatch(
        (-body_L, -body_W), 2 * body_L, 2 * body_W,
        boxstyle="round,pad=0.2", linewidth=2.0,
        edgecolor='#2c3e50', facecolor='#dfe6e9', alpha=alpha, zorder=4)
    ax.add_patch(body_rect)
    # Head indicator (front arrow)
    ax.annotate('', xy=(body_L + 0.15, 0), xytext=(body_L - 0.3, 0),
                arrowprops=dict(arrowstyle='->', color='#636e72', lw=1.5),
                zorder=5)
    ax.text(0, 0, 'Body', ha='center', va='center', fontsize=9,
            color='#636e72', fontstyle='italic', zorder=5)


# Label offsets
label_offsets = {
    'HL': (-0.2,  0.4),
    'FL': ( 0.2,  0.4),
    'HR': (-0.2, -0.55),
    'FR': ( 0.2, -0.55),
}

# --- Subplot (a): Walk Gait – 3 stance legs ---
ax = ax1
ax.set_xlim(-body_L - 2.8, body_L + 2.8)
ax.set_ylim(-body_W - 3.0, body_W + 3.0)
ax.set_aspect('equal')
ax.axis('off')
ax.set_title('(a) Walk Gait — 3 stance legs', fontsize=13, fontweight='bold', pad=14)

# Draw body
draw_body(ax, body_L, body_W)

stance_walk = ['FR', 'HL', 'HR']
swing_walk = ['FL']

# Draw support polygon (triangle for 3 stance legs)
poly_pts = np.array([feet[k] for k in stance_walk])
support_poly = patches.Polygon(poly_pts, closed=True, fill=True,
                                facecolor='#27ae60', alpha=0.15,
                                edgecolor='#27ae60', linewidth=2.2,
                                linestyle='-', zorder=1)
ax.add_patch(support_poly)

# Label support polygon
centroid = poly_pts.mean(axis=0)
ax.text(centroid[0] - 0.8, centroid[1] + 0.3, 'Support\nPolygon',
        ha='center', va='center', fontsize=10, color='#27ae60',
        fontstyle='italic', fontweight='bold')

# Draw legs and feet
for name in ['HL', 'HR', 'FL', 'FR']:
    is_stance = name in stance_walk
    leg_alpha = 1.0 if is_stance else 0.35
    leg_color = '#2c3e50' if is_stance else '#b2bec3'

    draw_robot_leg(ax, hips[name], knees[name], feet[name],
                   color=leg_color, alpha=leg_alpha, zorder=3 if is_stance else 2)
    draw_foot_marker(ax, feet[name], stance=is_stance)

    ox, oy = label_offsets[name]
    if is_stance:
        ax.text(feet[name][0] + ox, feet[name][1] + oy, name,
                fontsize=11, fontweight='bold', color='#2c3e50')
    else:
        ax.text(feet[name][0] + ox, feet[name][1] + oy, f'{name}\n(Swing)',
                fontsize=9, color='#95a5a6', fontstyle='italic', ha='center')

# Draw CoM projection
com = np.array([0.0, 0.0])
ax.plot(com[0], com[1], 'X', color='#e74c3c', markersize=16, markeredgewidth=2.5, zorder=8)
ax.text(com[0] + 0.35, com[1] + 0.35, 'CoM', fontsize=13,
        fontweight='bold', color='#e74c3c', zorder=8)

# Calculate SSM: min distance from CoM to each edge of support polygon
edges = [(poly_pts[i], poly_pts[(i + 1) % len(poly_pts)]) for i in range(len(poly_pts))]
min_dist = float('inf')
min_proj = None

for p1, p2 in edges:
    edge_vec = p2 - p1
    edge_len = np.linalg.norm(edge_vec)
    edge_unit = edge_vec / edge_len
    v = com - p1
    t = np.clip(np.dot(v, edge_unit), 0, edge_len)
    proj = p1 + t * edge_unit
    d = np.linalg.norm(com - proj)
    if d < min_dist:
        min_dist = d
        min_proj = proj

# Draw SSM line with double-headed arrow
ax.annotate('', xy=min_proj, xytext=com,
            arrowprops=dict(arrowstyle='<->', color='#2980b9', lw=2.2))
mid_ssm = (com + min_proj) / 2
ax.text(mid_ssm[0] - 0.8, mid_ssm[1], 'SSM', fontsize=14,
        fontweight='bold', color='#2980b9', ha='center', va='center',
        bbox=dict(boxstyle='round,pad=0.25', facecolor='white',
                  edgecolor='#2980b9', alpha=0.95))

# --- Subplot (b): Trot Gait – 2 diagonal stance legs ---
ax = ax2
ax.set_xlim(-body_L - 2.8, body_L + 2.8)
ax.set_ylim(-body_W - 3.0, body_W + 3.0)
ax.set_aspect('equal')
ax.axis('off')
ax.set_title('(b) Trot Gait — 2 diagonal stance legs', fontsize=13, fontweight='bold', pad=14)

# Draw body
draw_body(ax, body_L, body_W)

stance_trot = ['FL', 'HR']
swing_trot = ['FR', 'HL']

# Draw support line (degenerated polygon = diagonal line)
ax.plot([feet['FL'][0], feet['HR'][0]], [feet['FL'][1], feet['HR'][1]],
        color='#27ae60', linewidth=4.5, alpha=0.45, solid_capstyle='round', zorder=1)
# Dashed outline
ax.plot([feet['FL'][0], feet['HR'][0]], [feet['FL'][1], feet['HR'][1]],
        color='#27ae60', linewidth=1.5, linestyle='--', alpha=0.8, zorder=1)
ax.text(-1.2, 1.5, 'Support Line\n(Degenerated\n Polygon)',
        ha='center', va='center', fontsize=9, color='#27ae60',
        fontstyle='italic', fontweight='bold')

# Draw legs and feet
for name in ['HL', 'HR', 'FL', 'FR']:
    is_stance = name in stance_trot
    leg_alpha = 1.0 if is_stance else 0.35
    leg_color = '#2c3e50' if is_stance else '#b2bec3'

    draw_robot_leg(ax, hips[name], knees[name], feet[name],
                   color=leg_color, alpha=leg_alpha, zorder=3 if is_stance else 2)
    draw_foot_marker(ax, feet[name], stance=is_stance)

    ox, oy = label_offsets[name]
    if is_stance:
        ax.text(feet[name][0] + ox, feet[name][1] + oy, name,
                fontsize=11, fontweight='bold', color='#2c3e50')
    else:
        ax.text(feet[name][0] + ox, feet[name][1] + oy, f'{name}\n(Swing)',
                fontsize=9, color='#95a5a6', fontstyle='italic', ha='center')

# Draw CoM projection
ax.plot(com[0], com[1], 'X', color='#e74c3c', markersize=16, markeredgewidth=2.5, zorder=8)
ax.text(com[0] + 0.35, com[1] + 0.35, 'CoM', fontsize=13,
        fontweight='bold', color='#e74c3c', zorder=8)

# SSM ≈ 0 annotation with arrow
ax.annotate('SSM $\\approx$ 0', xy=(com[0], com[1]),
            xytext=(com[0] + 2.0, com[1] - 1.2),
            fontsize=13, fontweight='bold', color='#2980b9',
            arrowprops=dict(arrowstyle='->', color='#2980b9', lw=2.0),
            bbox=dict(boxstyle='round,pad=0.25', facecolor='white',
                      edgecolor='#2980b9', alpha=0.95))

plt.tight_layout(w_pad=2.5)
plt.savefig('img/fig_static_stability.pdf', dpi=300, bbox_inches='tight')
plt.savefig('img/fig_static_stability_preview.png', dpi=200, bbox_inches='tight')
plt.close()
print("Figure 1 (Static Stability / SSM) generated.")


# ==========================================
# Figure 2: Dynamic Stability (ZMP)
# Side view (Sagittal Plane) showing forces and ZMP
# ==========================================
fig, ax = plt.subplots(figsize=(10, 7))

# ---- Parameters ----
H_com = 4.0       # Height of CoM
ground_y = 0.0

# Leg geometry (side view)
hip_rear_x = -1.3
foot_rear_x = -2.0
hip_front_x = 1.3
foot_front_x = 2.0

# ---- Ground with hatching ----
ax.plot([-4.5, 5.0], [ground_y, ground_y], color='#2c3e50', linewidth=2.5, zorder=1)
for hx in np.arange(-4.5, 5.0, 0.25):
    ax.plot([hx, hx - 0.2], [ground_y, ground_y - 0.2],
            color='#7f8c8d', linewidth=0.8, zorder=0)

# ---- Robot body (side view rectangle) ----
body_w = 2.6  # body length
body_h = 0.55
body_rect = patches.FancyBboxPatch(
    (-body_w/2, H_com - body_h/2), body_w, body_h,
    boxstyle="round,pad=0.08", linewidth=2.5,
    edgecolor='#2c3e50', facecolor='#dfe6e9', zorder=4)
ax.add_patch(body_rect)

# ---- Legs ----
def draw_leg(ax, hip_x, foot_x, hip_y, foot_y, color='#2c3e50', lw=4, alpha=1.0, zorder=3):
    knee_x = (hip_x + foot_x) / 2
    knee_y = (hip_y + foot_y) / 2 + 0.2
    ax.plot([hip_x, knee_x], [hip_y, knee_y], color=color,
            linewidth=lw, solid_capstyle='round', alpha=alpha, zorder=zorder)
    ax.plot([knee_x, foot_x], [knee_y, foot_y], color=color,
            linewidth=lw - 0.5, solid_capstyle='round', alpha=alpha, zorder=zorder)
    ax.plot(hip_x, hip_y, 'o', color='#d63031', markersize=6,
            alpha=alpha, zorder=zorder+1)
    ax.plot(knee_x, knee_y, 'o', color='#d63031', markersize=5,
            alpha=alpha, zorder=zorder+1)
    ax.plot(foot_x, foot_y, 'o', color=color, markersize=6,
            alpha=alpha, zorder=zorder+1)

# Rear stance leg
draw_leg(ax, hip_rear_x, foot_rear_x, H_com - body_h/2, ground_y)
# Front stance leg
draw_leg(ax, hip_front_x, foot_front_x, H_com - body_h/2, ground_y)

# Swing leg (lighter, partially raised)
swing_knee_x = hip_front_x + 0.5
swing_knee_y = H_com/2 + 0.5
swing_foot_x = hip_front_x + 0.7
swing_foot_y = 0.9
ax.plot([hip_front_x, swing_knee_x], [H_com - body_h/2, swing_knee_y],
        color='#b2bec3', linewidth=3.5, solid_capstyle='round', alpha=0.5, zorder=2)
ax.plot([swing_knee_x, swing_foot_x], [swing_knee_y, swing_foot_y],
        color='#b2bec3', linewidth=3, solid_capstyle='round', alpha=0.5, zorder=2)
ax.plot(swing_foot_x, swing_foot_y, 'o', color='#b2bec3',
        markersize=6, alpha=0.5, zorder=3)
ax.text(swing_foot_x + 0.15, swing_foot_y + 0.15, 'Swing leg',
        fontsize=9, color='#636e72', fontstyle='italic')

# ---- CoM ----
com_pos = np.array([0.0, H_com])
ax.plot(com_pos[0], com_pos[1], 'o', color='#e74c3c', markersize=13, zorder=10)
ax.plot(com_pos[0], com_pos[1], '+', color='white', markersize=7,
        markeredgewidth=2, zorder=11)
ax.text(com_pos[0] + 0.15, com_pos[1] + 0.35, 'CoM',
        fontsize=13, fontweight='bold', color='#e74c3c', zorder=10)

# ---- Force vectors ----
g_vec = np.array([0, -1.8])
ax.annotate('', xy=com_pos + g_vec, xytext=com_pos,
            arrowprops=dict(arrowstyle='->', color='#0984e3', lw=2.5,
                            mutation_scale=18), zorder=8)
ax.text(com_pos[0] + 0.2, com_pos[1] - 1.0,
        '$F_g = m\\vec{g}$', fontsize=13, color='#0984e3', fontweight='bold')

inertia_vec = np.array([-1.5, 0])
ax.annotate('', xy=com_pos + inertia_vec, xytext=com_pos,
            arrowprops=dict(arrowstyle='->', color='#e17055', lw=2.5,
                            mutation_scale=18), zorder=8)
ax.text(com_pos[0] + inertia_vec[0] - 1.0, com_pos[1] + 0.3,
        '$F_{inertia} = -m\\vec{a}$', fontsize=12, color='#e17055', fontweight='bold')

res_vec = g_vec + inertia_vec
ax.annotate('', xy=com_pos + res_vec, xytext=com_pos,
            arrowprops=dict(arrowstyle='->', color='#6c5ce7', lw=3.0,
                            mutation_scale=20), zorder=9)
ax.plot([com_pos[0] + g_vec[0], com_pos[0] + res_vec[0]],
        [com_pos[1] + g_vec[1], com_pos[1] + res_vec[1]],
        color='#b2bec3', linestyle=':', linewidth=1.2, zorder=7)
ax.plot([com_pos[0] + inertia_vec[0], com_pos[0] + res_vec[0]],
        [com_pos[1] + inertia_vec[1], com_pos[1] + res_vec[1]],
        color='#b2bec3', linestyle=':', linewidth=1.2, zorder=7)
ax.text(com_pos[0] + res_vec[0]/2 - 0.6, com_pos[1] + res_vec[1]/2 - 0.3,
        '$F_{res}$', fontsize=13, color='#6c5ce7', fontweight='bold')

# ---- ZMP ----
t_zmp = -com_pos[1] / res_vec[1]
zmp_x = com_pos[0] + t_zmp * res_vec[0]
zmp_pos = np.array([zmp_x, ground_y])

ax.plot([com_pos[0] + res_vec[0], zmp_x],
        [com_pos[1] + res_vec[1], ground_y],
        color='#6c5ce7', linestyle='--', linewidth=1.5, zorder=6)

ax.plot(zmp_x, ground_y, 's', color='#00b894', markersize=11, zorder=10)
ax.text(zmp_x - 0.15, ground_y - 0.55, 'ZMP', fontsize=14,
        fontweight='bold', color='#00b894', ha='center')

# ---- Support Polygon ----
sp_left = foot_rear_x
sp_right = foot_front_x
ax.plot([sp_left, sp_right], [ground_y, ground_y],
        color='#00b894', linewidth=6, alpha=0.35, solid_capstyle='round', zorder=1)
for sx in [sp_left, sp_right]:
    ax.plot([sx, sx], [ground_y - 0.12, ground_y + 0.12],
            color='#00b894', linewidth=2, zorder=2)
ax.text((sp_left + sp_right)/2, ground_y - 0.55,
        'Support Polygon', fontsize=12, ha='center',
        color='#00b894', fontstyle='italic')

# ---- CoM vertical projection ----
ax.plot([com_pos[0], com_pos[0]], [com_pos[1] - 0.15, ground_y + 0.05],
        color='#e74c3c', linestyle=':', linewidth=1.2, zorder=5)
ax.plot(com_pos[0], ground_y, 'v', color='#e74c3c', markersize=7, zorder=10)
ax.text(com_pos[0] + 0.15, ground_y + 0.2, "CoM'", fontsize=10,
        color='#e74c3c', fontstyle='italic')

# ---- Ground reaction forces ----
for fx in [foot_rear_x, foot_front_x]:
    ax.annotate('', xy=(fx, ground_y + 0.7), xytext=(fx, ground_y),
                arrowprops=dict(arrowstyle='->', color='#f39c12', lw=2.0,
                                mutation_scale=14), zorder=8)
ax.text(foot_rear_x - 0.15, ground_y + 0.85, '$R_1$', fontsize=11,
        color='#f39c12', fontweight='bold', ha='center')
ax.text(foot_front_x + 0.15, ground_y + 0.85, '$R_2$', fontsize=11,
        color='#f39c12', fontweight='bold', ha='center')

# ---- Direction of motion ----
ax.annotate('', xy=(4.2, H_com + 0.7), xytext=(2.8, H_com + 0.7),
            arrowprops=dict(arrowstyle='->', color='#2d3436', lw=2.5,
                            mutation_scale=18))
ax.text(3.5, H_com + 1.0, 'Direction of motion ($\\vec{a}$)',
        fontsize=11, color='#2d3436', ha='center', fontweight='bold')

# ---- View label ----
ax.text(-4.3, H_com + 1.2, 'Sagittal Plane\n(Side View)',
        fontsize=10, color='#636e72', fontstyle='italic',
        bbox=dict(boxstyle='round,pad=0.3', facecolor='#f5f6fa',
                  edgecolor='#dcdde1', alpha=0.9))

ax.set_xlim(-4.5, 5.0)
ax.set_ylim(-1.0, H_com + 1.8)
ax.set_aspect('equal')
ax.axis('off')

plt.tight_layout()
plt.savefig('img/fig_zmp_stability.pdf', dpi=300, bbox_inches='tight')
plt.close()
print("Figure 2 (Dynamic Stability / ZMP) generated.")

print("\nAll figures generated successfully!")
