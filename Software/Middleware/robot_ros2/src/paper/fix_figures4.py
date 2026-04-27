import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def replace_numbers_only(filename, out_filename, x_range, y_range):
    print(f"Enhancing axes numbers for {filename}...")
    img = cv2.imread(filename)
    if img is None: 
        print(f"Could not load {filename}")
        return
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # 1. Find grid bounds
    edges = cv2.Canny(gray, 50, 150)
    r_sum = np.sum(edges, axis=1)
    c_sum = np.sum(edges, axis=0)
    y1 = np.argmax(r_sum > np.max(r_sum)*0.5)
    y2 = len(r_sum) - 1 - np.argmax(r_sum[::-1] > np.max(r_sum)*0.5)
    x1 = np.argmax(c_sum > np.max(c_sum)*0.5)
    x2 = len(c_sum) - 1 - np.argmax(c_sum[::-1] > np.max(c_sum)*0.5)
    
    # Crop exactly the inner grid (which contains the original data AND the original legend)
    cropped = img[y1:y2, x1:x2]
    # No white-out! Keep the original legend exactly as is!
    
    cropped_rgb = cv2.cvtColor(cropped, cv2.COLOR_BGR2RGB)
    
    # 2. Plot with Matplotlib
    fig, ax = plt.subplots(figsize=(10, 6), dpi=300)
    plt.subplots_adjust(left=0.15, right=0.95, top=0.95, bottom=0.15)
    
    ax.imshow(cropped_rgb, aspect='auto', extent=[x_range[0], x_range[1], y_range[0], y_range[1]])
    
    # ENLARGE THE NUMBERS ONLY!
    plt.xticks(fontsize=20)
    plt.yticks(fontsize=20)
    
    # NO LEGEND ADDED HERE! The original legend remains inside the image!
    
    plt.savefig(out_filename)
    plt.close()

# Process all 5 restored images perfectly
replace_numbers_only('figures/pid_error_home.png', 'figures/pid_error_home.png', [0, 200], [-0.18, 0.18])
replace_numbers_only('figures/pid_error_gait.png', 'figures/pid_error_gait.png', [0, 200], [-0.18, 0.18])
replace_numbers_only('figures/offline_traj_pos_left.png', 'figures/offline_traj_pos_left.png', [0, 5], [-172, -128])
replace_numbers_only('figures/offline_traj_vel_acc_left.png', 'figures/offline_traj_vel_acc_left.png', [0, 5], [-2, 2])
replace_numbers_only('figures/offline_traj_angle_left.png', 'figures/offline_traj_angle_left.png', [0, 5], [0, 4.2])

