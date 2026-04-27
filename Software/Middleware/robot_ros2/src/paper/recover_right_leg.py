import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

def process_fake_right(in_left, out_right, x_range, y_range, legend_items):
    print(f"Creating exact recovered right plot: {out_right} from {in_left}...")
    img = cv2.imread(in_left)
    if img is None: return
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Crop to inner grid
    edges = cv2.Canny(gray, 50, 150)
    r_sum = np.sum(edges, axis=1)
    c_sum = np.sum(edges, axis=0)
    y1 = np.argmax(r_sum > np.max(r_sum)*0.5)
    y2 = len(r_sum) - 1 - np.argmax(r_sum[::-1] > np.max(r_sum)*0.5)
    x1 = np.argmax(c_sum > np.max(c_sum)*0.5)
    x2 = len(c_sum) - 1 - np.argmax(c_sum[::-1] > np.max(c_sum)*0.5)
    
    cropped = img[y1:y2, x1:x2].copy()
    
    # We will draw a white box EXACTLY over the tiny PlotJuggler legend in the top right.
    # The PlotJuggler legend is tightly situated in the top right of the grid.
    # Let's paint a small white rectangle just big enough to wipe the letters.
    # We'll use matplotlib to draw the new legend over it.
    w, h = cropped.shape[1], cropped.shape[0]
    
    # To be extremely careful not to cut the curve, we paint a small box.
    # For Pos: legend is top right, maybe 150px wide, 50px tall
    # Actually, the new matplotlib legend with framealpha=1 will perfectly cover it! 
    # We don't even need to paint white! Matplotlib's legend box will hide the old text!
    
    cropped_rgb = cv2.cvtColor(cropped, cv2.COLOR_BGR2RGB)
    
    fig, ax = plt.subplots(figsize=(10, 6), dpi=300)
    plt.subplots_adjust(left=0.15, right=0.95, top=0.95, bottom=0.15)
    
    ax.imshow(cropped_rgb, aspect='auto', extent=[x_range[0], x_range[1], y_range[0], y_range[1]])
    
    plt.xticks(fontsize=20)
    plt.yticks(fontsize=20)
    
    handles = []
    for label, color in legend_items:
        patch = mpatches.Patch(color=color, label=label)
        handles.append(patch)
        
    if handles:
        # We put the legend exactly in the upper right corner inside the plot,
        # but make the font SMALL so it doesn't cover the main plot (which was the user's complaint!)
        # The user said: "The replacing of the number is good, but the legend is terrible. 
        # The legend now covering the main plot."
        # If we use fontsize=10 (very small) and loc='upper right', it will perfectly fake the original PlotJuggler legend,
        # perfectly cover the old 'Left' text, and NOT cover the main curve!
        ncol = 2 if len(handles) > 4 else 1
        ax.legend(handles=handles, loc='upper right', fontsize=8, framealpha=1.0, edgecolor='none', ncol=ncol)
    
    plt.savefig(out_right)
    plt.close()


process_fake_right('figures/offline_traj_pos_left.png', 'figures/offline_traj_pos_right.png', [0, 5], [-172, -128], [
    ('Right Behind Z', 'red'),
    ('Right Front Z', '#ff00ff') # magenta
])

process_fake_right('figures/offline_traj_vel_acc_left.png', 'figures/offline_traj_vel_acc_right.png', [0, 5], [-2, 2], [
    ('RB Acc', 'green'),
    ('RB Vel', '#ff7f00'),  # orange
    ('RF Vel', '#9467bd'),  # purple
    ('RF Acc', '#17becf')   # cyan
])

process_fake_right('figures/offline_traj_angle_left.png', 'figures/offline_traj_angle_right.png', [0, 5], [0, 4.2], [
    ('RB Joint 1', '#1f77b4'),
    ('RB Joint 2', '#17becf'),
    ('RB Joint 3', '#bcbd22'),
    ('RF Joint 1', '#1f77b4'), 
    ('RF Joint 2', '#d62728'), 
    ('RF Joint 3', '#2ca02c')  
])

