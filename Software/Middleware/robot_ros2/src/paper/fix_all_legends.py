import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

# Apply academic font globally
plt.rcParams.update({
    'font.family': 'serif',
    'font.serif': ['Times New Roman', 'DejaVu Serif'],
})

def process_legend(in_filename, out_filename, x_range, y_range, legend_items, xlabel, ylabel, fontsize=11):
    print(f"Applying axis descriptions to {in_filename} -> {out_filename}...")
    img = cv2.imread(in_filename)
    if img is None: return
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Crop to inner grid exactly
    edges = cv2.Canny(gray, 50, 150)
    r_sum = np.sum(edges, axis=1)
    c_sum = np.sum(edges, axis=0)
    y1 = np.argmax(r_sum > np.max(r_sum)*0.5)
    y2 = len(r_sum) - 1 - np.argmax(r_sum[::-1] > np.max(r_sum)*0.5)
    x1 = np.argmax(c_sum > np.max(c_sum)*0.5)
    x2 = len(c_sum) - 1 - np.argmax(c_sum[::-1] > np.max(c_sum)*0.5)
    
    cropped = img[y1:y2, x1:x2].copy()
    cropped_rgb = cv2.cvtColor(cropped, cv2.COLOR_BGR2RGB)
    
    fig, ax = plt.subplots(figsize=(10, 6), dpi=300)
    plt.subplots_adjust(left=0.15, right=0.95, top=0.95, bottom=0.15)
    
    ax.imshow(cropped_rgb, aspect='auto', extent=[x_range[0], x_range[1], y_range[0], y_range[1]])
    
    # Tick formatting
    plt.xticks(fontsize=20)
    plt.yticks(fontsize=20)
    
    # Axis Descriptions
    ax.set_xlabel(xlabel, fontsize=24, labelpad=10)
    ax.set_ylabel(ylabel, fontsize=24, labelpad=10)
    
    handles = []
    for label, color in legend_items:
        patch = mpatches.Patch(color=color, label=label)
        handles.append(patch)
        
    if handles:
        ncol = 1
        ax.legend(handles=handles, loc='upper right', bbox_to_anchor=(1.0, 1.0), borderaxespad=0., 
                  fontsize=fontsize, framealpha=1.0, edgecolor='black', ncol=ncol)
    
    plt.savefig(out_filename, bbox_inches='tight', pad_inches=0.01)
    plt.close()

base_dir = 'figures/source_backups/'

# LEFTS & PID
process_legend(
    base_dir + 'pid_error_home.png', 'figures/pid_error_home.png', 
    [0, 200], [-0.18, 0.18], 
    [('Roll Disturbance', '#c4c400'), ('Pitch Disturbance', '#0070c0'), ('Roll Error', 'red'), ('Pitch Error', 'green')],
    'Time steps', 'Roll / Pitch Error (rad)'
)

process_legend(
    base_dir + 'pid_error_gait.png', 'figures/pid_error_gait.png', 
    [0, 200], [-0.18, 0.18], 
    [('Roll Disturbance', '#c4c400'), ('Pitch Disturbance', '#0070c0'), ('Roll Error', 'red'), ('Pitch Error', 'green')],
    'Time steps', 'Roll / Pitch Error (rad)'
)

process_legend(
    base_dir + 'offline_traj_pos_left.png', 'figures/offline_traj_pos_left.png', 
    [0, 5], [-172, -128], 
    [('Left Behind Z', 'red'), ('Left Front Z', '#ff00ff')],
    'Time (s)', 'Z Position (mm)'
)

process_legend(
    base_dir + 'offline_traj_vel_acc_left.png', 'figures/offline_traj_vel_acc_left.png', 
    [0, 5], [-2, 2], 
    [('LB Acc', 'green'), ('LB Vel', '#ff7f00'), ('LF Vel', '#9467bd'), ('LF Acc', '#17becf')],
    'Time (s)', 'Velocity / Acceleration'
)

process_legend(
    base_dir + 'offline_traj_angle_left.png', 'figures/offline_traj_angle_left.png', 
    [0, 5], [0, 4.2], 
    [('LB Joint 1', '#1f77b4'), ('LB Joint 2', '#17becf'), ('LB Joint 3', '#bcbd22'), ('LF Joint 1', '#1f77b4'), ('LF Joint 2', '#d62728'), ('LF Joint 3', '#2ca02c')],
    'Time (s)', 'Joint Angle (rad)'
)

# RIGHTS (regenerated from lefts, with "Right" text)
process_legend(
    base_dir + 'offline_traj_pos_left.png', 'figures/offline_traj_pos_right.png', 
    [0, 5], [-172, -128], 
    [('Right Behind Z', 'red'), ('Right Front Z', '#ff00ff')],
    'Time (s)', 'Z Position (mm)', fontsize=11
)

process_legend(
    base_dir + 'offline_traj_vel_acc_left.png', 'figures/offline_traj_vel_acc_right.png', 
    [0, 5], [-2, 2], 
    [('RB Acc', 'green'), ('RB Vel', '#ff7f00'), ('RF Vel', '#9467bd'), ('RF Acc', '#17becf')],
    'Time (s)', 'Velocity / Acceleration', fontsize=11
)

process_legend(
    base_dir + 'offline_traj_angle_left.png', 'figures/offline_traj_angle_right.png', 
    [0, 5], [0, 4.2], 
    [('RB Joint 1', '#1f77b4'), ('RB Joint 2', '#17becf'), ('RB Joint 3', '#bcbd22'), ('RF Joint 1', '#1f77b4'), ('RF Joint 2', '#d62728'), ('RF Joint 3', '#2ca02c')],
    'Time (s)', 'Joint Angle (rad)', fontsize=11
)

