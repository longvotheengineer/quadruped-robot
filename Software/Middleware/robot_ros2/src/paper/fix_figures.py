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
    
    # Check if image is rgb and opencv loaded BGR, convert to RGB
    cropped_rgb = cv2.cvtColor(cropped, cv2.COLOR_BGR2RGB)
    
    # 2. Plot with Matplotlib
    fig, ax = plt.subplots(figsize=(10, 6), dpi=300)
    # Give axes some breathing room.
    plt.subplots_adjust(left=0.15, right=0.95, top=0.95, bottom=0.15)
    
    ax.imshow(cropped_rgb, aspect='auto', extent=[x_range[0], x_range[1], y_range[0], y_range[1]])
    
    # Set big fonts
    plt.xticks(fontsize=20)
    plt.yticks(fontsize=20)
    # Add a custom legend
    handles = []
    for label, color in legend_items:
        patch = mpatches.Patch(color=color, label=label)
        handles.append(patch)
    if handles:
        ax.legend(handles=handles, loc='upper right', fontsize=16, framealpha=0.95)
    
    # Save over the old file
    plt.savefig(filename)
    plt.close()

# For pid_error_home
fix_image('figures/pid_error_home.png', [0, 200], [-0.18, 0.18], [
    ('Roll Disturbance', '#c4c400'),
    ('Pitch Disturbance', '#0070c0'),
    ('Roll Error', 'red'),
    ('Pitch Error', 'green')
])

# For pid_error_gait
fix_image('figures/pid_error_gait.png', [0, 200], [-0.18, 0.18], [
    ('Roll Disturbance', '#c4c400'),
    ('Pitch Disturbance', '#0070c0'),
    ('Roll Error', 'red'),
    ('Pitch Error', 'green')
])

# For offline_traj_pos_left
fix_image('figures/offline_traj_pos_left.png', [0, 5], [-172, -128], [
    ('Left Behind Z', 'red'),
    ('Left Front Z', '#ff00ff') # magenta
])

# Let's see if there are other files
import os
print("Available PNGs:", [f for f in os.listdir('figures') if 'offline' in f or 'pid_error' in f])
