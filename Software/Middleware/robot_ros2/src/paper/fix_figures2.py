import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

def fix_image(filename, x_range, y_range, legend_items):
    print(f"Fixing {filename}...")
    img = cv2.imread(filename)
    if img is None: return
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # 1. Find grid bounds
    edges = cv2.Canny(gray, 50, 150)
    r_sum = np.sum(edges, axis=1)
    c_sum = np.sum(edges, axis=0)
    y1 = np.argmax(r_sum > np.max(r_sum)*0.5)
    y2 = len(r_sum) - 1 - np.argmax(r_sum[::-1] > np.max(r_sum)*0.5)
    x1 = np.argmax(c_sum > np.max(c_sum)*0.5)
    x2 = len(c_sum) - 1 - np.argmax(c_sum[::-1] > np.max(c_sum)*0.5)
    
    cropped = img[y1:y2, x1:x2]
    # Blank top right for old legend
    # Assume legend is max 800px wide, 350px tall
    cw, ch = 800, 350
    # Overwrite with white background grid
    cropped[0:ch, -cw:] = 255
    
    cropped_rgb = cv2.cvtColor(cropped, cv2.COLOR_BGR2RGB)
    
    # 2. Plot with Matplotlib
    fig, ax = plt.subplots(figsize=(10, 6), dpi=300)
    plt.subplots_adjust(left=0.10, right=0.97, top=0.95, bottom=0.15)
    
    ax.imshow(cropped_rgb, aspect='auto', extent=[x_range[0], x_range[1], y_range[0], y_range[1]])
    
    plt.xticks(fontsize=20)
    plt.yticks(fontsize=20)
    
    handles = []
    for label, color in legend_items:
        patch = mpatches.Patch(color=color, label=label)
        handles.append(patch)
    if handles:
        # For multiple columns if 6 items
        ncol = 2 if len(handles) > 4 else 1
        ax.legend(handles=handles, loc='upper right', fontsize=14, framealpha=0.95, ncol=ncol)
    
    plt.savefig(filename)
    plt.close()

# Vel Acc
fix_image('figures/offline_traj_vel_acc_left.png', [0, 5], [-2, 2], [
    ('LB Acc', 'green'),
    ('LB Vel', '#ff7f00'),  # orange
    ('LF Vel', '#9467bd'),  # purple
    ('LF Acc', '#17becf')   # cyan
])

# Angle
fix_image('figures/offline_traj_angle_left.png', [0, 5], [0, 4.2], [
    ('LB Joint 1', '#1f77b4'),
    ('LB Joint 2', '#17becf'),
    ('LB Joint 3', '#bcbd22'),
    ('LF Joint 1', '#1f77b4'), # some dark blue
    ('LF Joint 2', '#d62728'), # red
    ('LF Joint 3', '#2ca02c')  # green
])

