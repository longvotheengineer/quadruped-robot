#!/usr/bin/env python3
"""
Generate Simplified Figure 4.9 — Overall Control Architecture
Shows the bifurcation of PID output based on HOME vs GAIT modes.
"""

import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch

plt.rcParams['font.family'] = 'DejaVu Sans'

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'img')
os.makedirs(OUT, exist_ok=True)

# Colors
C_CMD    = '#BBDEFB'
C_PLAN   = '#C8E6C9'
C_IK     = '#FFF9C4'
C_BAL    = '#E1BEE7'
C_ROBOT  = '#FFCDD2'
C_IMU    = '#B3E5FC'
C_SWITCH = '#FFE0B2'
C_BORDER = '#37474F'

def draw_box(ax, cx, cy, w, h, text, color, boxstyle="round,pad=0.1"):
    x, y = cx - w/2, cy - h/2
    b = FancyBboxPatch((x, y), w, h, boxstyle=boxstyle,
                       facecolor=color, edgecolor=C_BORDER, lw=1.5, zorder=3)
    ax.add_patch(b)
    ax.text(cx, cy, text, ha='center', va='center',
            fontsize=10, fontweight='bold', zorder=5, linespacing=1.5)

def draw_circle(ax, cx, cy, r=0.25):
    c = plt.Circle((cx, cy), r, facecolor='white', edgecolor=C_BORDER, lw=1.5, zorder=3)
    ax.add_patch(c)
    ax.plot([cx - r/1.5, cx + r/1.5], [cy, cy], color=C_BORDER, lw=1.5, zorder=4)
    ax.plot([cx, cx], [cy - r/1.5, cy + r/1.5], color=C_BORDER, lw=1.5, zorder=4)

def draw_arrow(ax, x1, y1, x2, y2, label='', offset=0.2):
    ax.annotate('', xy=(x2, y2), xytext=(x1, y1),
                arrowprops=dict(arrowstyle='-|>', color='#455A64', lw=2), zorder=2)
    if label:
        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2 + offset
        ax.text(cx, cy, label, ha='center', va='center', fontsize=9,
                color='#111', style='italic', zorder=6,
                bbox=dict(boxstyle='round,pad=0.1', fc='white', ec='none', alpha=0.8))

def draw_path(ax, points, label='', label_idx=0, offset=0.2):
    x_coords = [p[0] for p in points]
    y_coords = [p[1] for p in points]
    ax.plot(x_coords[:-1], y_coords[:-1], color='#455A64', lw=2, zorder=2)
    draw_arrow(ax, points[-2][0], points[-2][1], points[-1][0], points[-1][1])
    if label:
        p1, p2 = points[label_idx], points[label_idx+1]
        cx, cy = (p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2 + offset
        ax.text(cx, cy, label, ha='center', va='center', fontsize=9,
                color='#111', style='italic', zorder=6,
                bbox=dict(boxstyle='round,pad=0.1', fc='white', ec='none', alpha=0.8))

def main():
    fig, ax = plt.subplots(figsize=(14, 8))
    ax.set_xlim(0, 14)
    ax.set_ylim(0, 8)
    ax.axis('off')

    # ax.text(7, 7.5, 'Kiến trúc điều khiển phân cấp (HOME vs GAIT)', ha='center', fontsize=15, fontweight='bold')

    bw, bh = 2.4, 1.2

    # Main path
    C_CMD_POS  = (1.5, 6.0)
    C_GAIT_POS = (4.5, 6.0)
    C_SUM1     = (6.5, 6.0) # Task space sum
    C_IK_POS   = (8.5, 6.0)
    C_SUM2     = (10.5, 6.0) # Joint space sum
    C_ROBOT_POS = (12.5, 4.5)

    # Feedback path
    C_IMU_POS  = (9.5, 1.5)
    C_PID_POS  = (6.5, 1.5)
    
    # Mode Switch
    C_SWITCH_POS = (6.5, 3.5)

    # Blocks
    draw_box(ax, C_CMD_POS[0], C_CMD_POS[1], bw, bh, 'Lệnh điều khiển\n(Vận tốc, Chế độ)', C_CMD)
    draw_box(ax, C_GAIT_POS[0], C_GAIT_POS[1], bw, bh, 'Bộ tạo quỹ đạo\n(Gait Generator)', C_PLAN)
    draw_circle(ax, C_SUM1[0], C_SUM1[1])
    draw_box(ax, C_IK_POS[0], C_IK_POS[1], bw, bh, 'Động học nghịch\n(IK)', C_IK)
    draw_circle(ax, C_SUM2[0], C_SUM2[1])
    draw_box(ax, C_ROBOT_POS[0], C_ROBOT_POS[1], 1.8, 3.5, 'Cơ cấu chấp hành\n(Động cơ / Gazebo)', C_ROBOT)
    
    draw_box(ax, C_IMU_POS[0], C_IMU_POS[1], bw, bh, 'Cảm biến IMU\n(Roll, Pitch)', C_IMU)
    draw_box(ax, C_PID_POS[0], C_PID_POS[1], bw, bh, 'Bộ điều khiển PID\n(Cân bằng)', C_BAL)

    draw_box(ax, C_SWITCH_POS[0], C_SWITCH_POS[1], 3.0, 1.0, 'Chuyển mạch Chế độ\n(HOME / GAIT)', C_SWITCH, "square,pad=0.1")

    # Path 1: Forward
    draw_arrow(ax, C_CMD_POS[0]+bw/2, C_CMD_POS[1], C_GAIT_POS[0]-bw/2, C_GAIT_POS[1], label='Lệnh (x,y,z)')
    draw_arrow(ax, C_GAIT_POS[0]+bw/2, C_GAIT_POS[1], C_SUM1[0]-0.25, C_SUM1[1], label=r'$P_{ref}$')
    draw_arrow(ax, C_SUM1[0]+0.25, C_SUM1[1], C_IK_POS[0]-bw/2, C_IK_POS[1], label=r'$P_{adj}$')
    draw_arrow(ax, C_IK_POS[0]+bw/2, C_IK_POS[1], C_SUM2[0]-0.25, C_SUM2[1], label=r'$\theta_{ik}$')
    draw_arrow(ax, C_SUM2[0]+0.25, C_SUM2[1], C_ROBOT_POS[0]-0.9, C_SUM2[1], label=r'$\theta_{cmd}$')

    # Feedback Path
    draw_path(ax, [(C_ROBOT_POS[0], 2.75), (C_ROBOT_POS[0], C_IMU_POS[1]), (C_IMU_POS[0]+bw/2, C_IMU_POS[1])], label='Trạng thái')
    draw_arrow(ax, C_IMU_POS[0]-bw/2, C_IMU_POS[1], C_PID_POS[0]+bw/2, C_PID_POS[1], label='Góc nghiêng')
    
    # PID to Switch
    draw_arrow(ax, C_PID_POS[0], C_PID_POS[1]+bh/2, C_SWITCH_POS[0], C_SWITCH_POS[1]-0.5, label='Tín hiệu bù (u)')

    # Switch to Sum1 (GAIT Mode)
    draw_path(ax, [(C_SWITCH_POS[0]-1.0, C_SWITCH_POS[1]+0.5), (C_SWITCH_POS[0]-1.0, C_SUM1[1]-0.25)], 
              label='GAIT\n(Task Space)', label_idx=0, offset=0.2)

    # Switch to Sum2 (HOME Mode)
    draw_path(ax, [(C_SWITCH_POS[0]+1.0, C_SWITCH_POS[1]+0.5), (C_SWITCH_POS[0]+1.0, 4.5), (C_SUM2[0], 4.5), (C_SUM2[0], C_SUM2[1]-0.25)], 
              label='HOME\n(Joint Space)', label_idx=1, offset=0.3)

    # Command to Switch (Mode selection)
    draw_path(ax, [(C_CMD_POS[0], C_CMD_POS[1]-bh/2), (C_CMD_POS[0], C_SWITCH_POS[1]), (C_SWITCH_POS[0]-1.5, C_SWITCH_POS[1])],
              label='Chọn chế độ', label_idx=1, offset=0.2)

    # Legends
    legend_items = [
        mpatches.Patch(color=C_PLAN, label='Không gian làm việc (Task Space)'),
        mpatches.Patch(color=C_IK,   label='Không gian khớp (Joint Space)'),
        mpatches.Patch(color=C_SWITCH, label='Phân luồng tín hiệu (Routing)'),
    ]
    ax.legend(handles=legend_items, loc='upper right', fontsize=10)

    # Plus signs
    ax.text(C_SUM1[0]-0.3, C_SUM1[1]+0.3, '+', fontsize=12, fontweight='bold', color='green')
    ax.text(C_SUM1[0]+0.3, C_SUM1[1]-0.3, '+', fontsize=12, fontweight='bold', color='green')
    ax.text(C_SUM2[0]-0.3, C_SUM2[1]+0.3, '+', fontsize=12, fontweight='bold', color='green')
    ax.text(C_SUM2[0]+0.3, C_SUM2[1]-0.3, '+', fontsize=12, fontweight='bold', color='green')

    out_path = os.path.join(OUT, 'fig28- Overall control architecture .png')
    fig.savefig(out_path, dpi=300, bbox_inches='tight', facecolor='white')
    print(f'Saved: {out_path}')
    plt.close()

if __name__ == '__main__':
    main()
