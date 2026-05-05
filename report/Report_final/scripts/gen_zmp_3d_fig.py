"""
Generate a 3D isometric-style ZMP stability figure for the quadruped robot thesis.
Based on the reference diagram showing CoM, ZMP, support polygon, and inverted pendulum model.
"""
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import os

os.makedirs('img', exist_ok=True)

plt.rcParams.update({
    'font.size': 12,
    'font.family': 'serif',
    'mathtext.fontset': 'cm',
})

# =============================================
# Isometric projection helpers
# =============================================
ISO_ANGLE = np.radians(30)

def iso(x, y, z):
    """Project 3D point to 2D isometric coordinates."""
    px = (x - y) * np.cos(ISO_ANGLE)
    py = (x + y) * np.sin(ISO_ANGLE) + z
    return px, py

def line3d(ax, p1, p2, **kwargs):
    x1, y1 = iso(*p1)
    x2, y2 = iso(*p2)
    return ax.plot([x1, x2], [y1, y2], **kwargs)

def arrow3d(ax, p_from, p_to, **kwargs):
    x1, y1 = iso(*p_from)
    x2, y2 = iso(*p_to)
    ax.annotate('', xy=(x2, y2), xytext=(x1, y1),
                arrowprops=dict(**kwargs))

# =============================================
fig, ax = plt.subplots(figsize=(9, 10))
ax.set_aspect('equal')
ax.axis('off')

# =============================================
# Robot dimensions
# =============================================
body_L = 1.8    # half-length (x)
body_W = 1.1    # half-width (y)
body_H = 0.55   # body height (z)
z_ground = 0.0
z_body = 3.2    # height of body bottom
z_com = z_body + body_H / 2

# Leg attachment points (bottom corners of body)
leg_attach = {
    'FL': ( body_L,  body_W, z_body),
    'FR': ( body_L, -body_W, z_body),
    'HL': (-body_L,  body_W, z_body),
    'HR': (-body_L, -body_W, z_body),
}

# Foot positions on ground
foot_pos = {
    'FL': ( body_L + 0.4,  body_W + 0.3, z_ground),
    'FR': ( body_L + 0.4, -body_W - 0.3, z_ground),
    'HL': (-body_L - 0.4,  body_W + 0.3, z_ground),
    'HR': (-body_L - 0.4, -body_W - 0.3, z_ground),
}

# Knee positions
knee_pos = {}
for leg in leg_attach:
    a = np.array(leg_attach[leg])
    f = np.array(foot_pos[leg])
    mid = (a + f) / 2
    # Bend knees outward
    if 'L' in leg:
        knee_pos[leg] = tuple(mid + np.array([0, 0.25, 0.15]))
    else:
        knee_pos[leg] = tuple(mid + np.array([0, -0.25, 0.15]))

# Which legs are stance (solid) vs swing (dashed)
stance_legs = ['FR', 'HL']   # right-front and left-hind: stance
swing_legs = ['FL', 'HR']    # left-front and right-hind: swing

# =============================================
# Draw order: back-to-front
# =============================================

# --- Coordinate axes ---
origin = (0, 0, z_ground)

# X axis
arrow3d(ax, origin, (4.0, 0, z_ground),
        arrowstyle='->', color='#555555', lw=1.5, mutation_scale=14)
ex, ey = iso(4.2, 0, z_ground)
ax.text(ex, ey - 0.05, '$x$', fontsize=15, color='#555555', fontweight='bold')

# Y axis
arrow3d(ax, origin, (0, 3.2, z_ground),
        arrowstyle='->', color='#555555', lw=1.5, mutation_scale=14)
ex, ey = iso(0, 3.4, z_ground)
ax.text(ex - 0.15, ey + 0.05, '$y$', fontsize=15, color='#555555', fontweight='bold')

# Z axis
arrow3d(ax, origin, (0, 0, z_com + 1.5),
        arrowstyle='->', color='#555555', lw=1.5, mutation_scale=14)
ex, ey = iso(0, 0, z_com + 1.7)
ax.text(ex + 0.08, ey + 0.05, '$z$', fontsize=15, color='#555555', fontweight='bold')

# --- Support polygon on ground (dashed green) ---
sp_order = ['FL', 'FR', 'HR', 'HL']
for i in range(len(sp_order)):
    p1 = foot_pos[sp_order[i]]
    p2 = foot_pos[sp_order[(i+1) % len(sp_order)]]
    line3d(ax, p1, p2, color='#1B7A2B', linewidth=2.2, linestyle='--', zorder=3)

# --- Foot contact points (green filled dots) ---
for leg, fpos in foot_pos.items():
    fx, fy = iso(*fpos)
    ax.plot(fx, fy, 'o', color='#27AE60', markersize=10, zorder=5,
            markeredgecolor='#1A5E1F', markeredgewidth=1.5)

# --- Draw legs ---
# Draw back legs first for proper occlusion
draw_order = ['HL', 'HR', 'FL', 'FR']
for leg in draw_order:
    a = leg_attach[leg]
    k = knee_pos[leg]
    f = foot_pos[leg]

    if leg in stance_legs:
        ls, lw, color = '-', 3.0, '#1C1C1C'
    else:
        ls, lw, color = '--', 2.5, '#1C1C1C'

    # Upper leg
    line3d(ax, a, k, color=color, linewidth=lw, linestyle=ls, zorder=4)
    # Lower leg
    line3d(ax, k, f, color=color, linewidth=lw, linestyle=ls, zorder=4)

# --- Robot body (3D box with shading) ---
c = {}
for sx, lx in [(-1, 'B'), (1, 'F')]:
    for sy, ly in [(-1, 'R'), (1, 'L')]:
        for sz, lz in [(0, 'b'), (1, 't')]:
            c[f'{lx}{ly}{lz}'] = (sx * body_L, sy * body_W, z_body + sz * body_H)

# Back face
face = [c['BRb'], c['BLb'], c['BLt'], c['BRt']]
xs = [iso(*p)[0] for p in face]
ys = [iso(*p)[1] for p in face]
ax.fill(xs, ys, color='#C8CCD0', edgecolor='#1C1C1C', linewidth=2, zorder=6)

# Left face
face = [c['BLb'], c['FLb'], c['FLt'], c['BLt']]
xs = [iso(*p)[0] for p in face]
ys = [iso(*p)[1] for p in face]
ax.fill(xs, ys, color='#DDE0E3', edgecolor='#1C1C1C', linewidth=2, zorder=6)

# Right face
face = [c['BRb'], c['FRb'], c['FRt'], c['BRt']]
xs = [iso(*p)[0] for p in face]
ys = [iso(*p)[1] for p in face]
ax.fill(xs, ys, color='#B0B5BA', edgecolor='#1C1C1C', linewidth=2, zorder=6)

# Front face
face = [c['FRb'], c['FLb'], c['FLt'], c['FRt']]
xs = [iso(*p)[0] for p in face]
ys = [iso(*p)[1] for p in face]
ax.fill(xs, ys, color='#D5D8DC', edgecolor='#1C1C1C', linewidth=2, zorder=7)

# Top face
face = [c['BLt'], c['FLt'], c['FRt'], c['BRt']]
xs = [iso(*p)[0] for p in face]
ys = [iso(*p)[1] for p in face]
ax.fill(xs, ys, color='#EBEDEF', edgecolor='#1C1C1C', linewidth=2, zorder=7)

# --- z₀ vertical line (blue, from CoM down to ground) ---
com_3d = (0, 0, z_com)
com_ground = (0, 0, z_ground)
line3d(ax, com_3d, com_ground, color='#2471A3', linewidth=2.5, linestyle='-', zorder=8)

# z₀ label with horizontal ticks
z0_mid = z_com / 2
z0x, z0y = iso(0, 0, z0_mid)
# Small horizontal ticks at top and bottom of z0
tick_len = 0.12
for z_tick in [z_com, z_ground + 0.05]:
    tx, ty = iso(0, 0, z_tick)
    ax.plot([tx - tick_len, tx + tick_len], [ty, ty],
            color='#555555', linewidth=1.2, zorder=9)

ax.text(z0x - 0.5, z0y + 0.1, '$z_0$', fontsize=15, color='#555555',
        fontweight='bold', zorder=15, ha='center')

# --- Red zigzag (inverted pendulum model from CoM to ZMP) ---
n_zags = 7
zag_amp = 0.2
zag_pts = []
for i in range(n_zags * 2 + 1):
    t = i / (n_zags * 2)
    z_val = z_com * (1 - t) + z_ground * t
    if i == 0 or i == n_zags * 2:
        x_off = 0
    else:
        x_off = zag_amp * (1 if i % 2 == 0 else -1)
    zag_pts.append((x_off, 0, z_val))

for i in range(len(zag_pts) - 1):
    x1, y1 = iso(*zag_pts[i])
    x2, y2 = iso(*zag_pts[i + 1])
    ax.plot([x1, x2], [y1, y2], color='#E74C3C', linewidth=3.0, zorder=9,
            solid_capstyle='round')

# --- CoM marker (half black/white circle like reference) ---
cx, cy = iso(*com_3d)
r = 0.15
# Full circle outline
circle = plt.Circle((cx, cy), r, fill=False, edgecolor='black', linewidth=2, zorder=14)
ax.add_patch(circle)
# Left half (black)
theta_left = np.linspace(np.pi/2, 3*np.pi/2, 50)
ax.fill([cx + r*np.cos(t) for t in theta_left],
        [cy + r*np.sin(t) for t in theta_left],
        color='black', zorder=14)
# Right half (white)
theta_right = np.linspace(-np.pi/2, np.pi/2, 50)
ax.fill([cx + r*np.cos(t) for t in theta_right],
        [cy + r*np.sin(t) for t in theta_right],
        color='white', zorder=14)

# CoM label
ax.text(cx + 0.35, cy + 0.2, 'CoM', fontsize=14, fontweight='bold',
        color='#1C1C1C', zorder=15)
ax.text(cx + 0.35, cy - 0.15, '$(x, y, z)$', fontsize=12,
        color='#555555', zorder=15)

# --- ZMP point ---
zmp_3d = (0.25, 0.15, z_ground)
zx, zy = iso(*zmp_3d)
ax.plot(zx, zy, 'o', color='black', markersize=10, zorder=10)
ax.text(zx - 0.15, zy - 0.5, 'ZMP', fontsize=14, fontweight='bold',
        color='#1C1C1C', zorder=15)
ax.text(zx - 0.35, zy - 0.9, '$(p_{x_k}, p_{y_k})$', fontsize=12,
        color='#555555', zorder=15)

# --- ZMP trajectory ellipse (small dashed blue on ground) ---
theta = np.linspace(0, 2*np.pi, 100)
erx, ery = 0.6, 0.35
ellipse_x = [iso(zmp_3d[0] + erx*np.cos(t), zmp_3d[1] + ery*np.sin(t), z_ground)[0] for t in theta]
ellipse_y = [iso(zmp_3d[0] + erx*np.cos(t), zmp_3d[1] + ery*np.sin(t), z_ground)[1] for t in theta]
# Draw as dashed
for i in range(0, len(theta)-1, 2):
    ax.plot([ellipse_x[i], ellipse_x[min(i+2, len(theta)-1)]],
            [ellipse_y[i], ellipse_y[min(i+2, len(theta)-1)]],
            color='#2471A3', linewidth=1.5, linestyle='--', zorder=3)

# Arrow on ellipse
arr_idx = 60
arrow3d(ax,
        (zmp_3d[0] + erx*np.cos(theta[arr_idx]),
         zmp_3d[1] + ery*np.sin(theta[arr_idx]), z_ground),
        (zmp_3d[0] + erx*np.cos(theta[arr_idx+3]),
         zmp_3d[1] + ery*np.sin(theta[arr_idx+3]), z_ground),
        arrowstyle='->', color='#2471A3', lw=1.5, mutation_scale=12)

# =============================================
ax.set_xlim(-5.5, 5.0)
ax.set_ylim(-2.5, 7.0)

plt.tight_layout()
plt.savefig('img/fig_zmp_stability.pdf', dpi=300, bbox_inches='tight')
plt.close()

print("ZMP 3D isometric figure generated successfully!")
