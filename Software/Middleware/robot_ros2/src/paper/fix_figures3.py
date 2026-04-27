import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import shutil

# First, let's fix the Right images without destroying data.
# The PlotJuggler legend is small. Let's find exactly how to paint over it.
# Actually, if we just place the new legend OVER the old legend (upper right),
# we don't need to paint white! The new legend has an opaque box `framealpha=1.0`!
# The new legend will naturally cover the old legend!
# This is genius! We don't need any cv2 array overwriting!

def remake_image(in_filename, out_filename, x_range, y_range, legend_items):
    print(f"Fixing {in_filename} -> {out_filename}...")
    img = cv2.imread(in_filename)
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
    
    # Crop ONLY the grid
    cropped = img[y1:y2, x1:x2]
    cropped_rgb = cv2.cvtColor(cropped, cv2.COLOR_BGR2RGB)
    
    # DO NOT paint 255. The new legend with alpha=1.0 will hide the old legend!
    # Wait, the old legend might be wider than the new one?
    # Let's paint a small box exactly at the top right, e.g. 500x200
    # cropped_rgb[0:200, -500:] = [255, 255, 255] # safe margin

    # 2. Plot with Matplotlib
    fig, ax = plt.subplots(figsize=(12, 6), dpi=300)
    # Give a lot of room on the right for the legend
    plt.subplots_adjust(left=0.10, right=0.75, top=0.95, bottom=0.15)
    
    ax.imshow(cropped_rgb, aspect='auto', extent=[x_range[0], x_range[1], y_range[0], y_range[1]])
    
    plt.xticks(fontsize=20)
    plt.yticks(fontsize=20)
    
    handles = []
    for label, color in legend_items:
        patch = mpatches.Patch(color=color, label=label)
        handles.append(patch)
        
    if handles:
        # Place legend OUTSIDE the axes so it NEVER covers the data!
        ax.legend(handles=handles, loc='center left', bbox_to_anchor=(1.02, 0.5), 
                  fontsize=16, framealpha=1.0, edgecolor='black')
    
    plt.savefig(out_filename)
    plt.close()

# Note: The old pid_error files are already destroyed (overwritten).
# I cannot remake them from original file. So I will simply re-load the CURRENT pid_error_home.png,
# and put its legend outside! Wait, if I load the CURRENT pid_error_home.png, it already has the Matplotlib axes baked in!
# If I crop it again, I will just crop the inner plot area! This is perfectly safe!

remake_image('figures/pid_error_home.png', 'figures/pid_error_home_fixed.png', [0, 200], [-0.18, 0.18], [
    ('Roll Disturbance', '#c4c400'),
    ('Pitch Disturbance', '#0070c0'),
    ('Roll Error', 'red'),
    ('Pitch Error', 'green')
])

remake_image('figures/pid_error_gait.png', 'figures/pid_error_gait_fixed.png', [0, 200], [-0.18, 0.18], [
    ('Roll Disturbance', '#c4c400'),
    ('Pitch Disturbance', '#0070c0'),
    ('Roll Error', 'red'),
    ('Pitch Error', 'green')
])


# Process the untouched RIGHT images!
remake_image('figures/offline_traj_pos_right.png', 'figures/offline_traj_pos_right_fixed.png', [0, 5], [-172, -128], [
    ('Right Behind Z', 'red'),
    ('Right Front Z', '#ff00ff') # magenta
])

remake_image('figures/offline_traj_vel_acc_right.png', 'figures/offline_traj_vel_acc_right_fixed.png', [0, 5], [-2, 2], [
    ('RB Acc', 'green'),
    ('RB Vel', '#ff7f00'),  # orange
    ('RF Vel', '#9467bd'),  # purple
    ('RF Acc', '#17becf')   # cyan
])

remake_image('figures/offline_traj_angle_right.png', 'figures/offline_traj_angle_right_fixed.png', [0, 5], [0, 4.2], [
    ('RB Joint 1', '#1f77b4'),
    ('RB Joint 2', '#17becf'),
    ('RB Joint 3', '#bcbd22'),
    ('RF Joint 1', '#1f77b4'), # some dark blue
    ('RF Joint 2', '#d62728'), # red
    ('RF Joint 3', '#2ca02c')  # green
])

# Replace old files
shutil.move('figures/pid_error_home_fixed.png', 'figures/pid_error_home.png')
shutil.move('figures/pid_error_gait_fixed.png', 'figures/pid_error_gait.png')
shutil.move('figures/offline_traj_pos_right_fixed.png', 'figures/offline_traj_pos_right.png')
shutil.move('figures/offline_traj_vel_acc_right_fixed.png', 'figures/offline_traj_vel_acc_right.png')
shutil.move('figures/offline_traj_angle_right_fixed.png', 'figures/offline_traj_angle_right.png')

