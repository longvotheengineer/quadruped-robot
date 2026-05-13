import cv2
import numpy as np
import glob

for f in glob.glob('thesis/presentation/images/results/trot_pid_*_zoom.png'):
    if "backup" in f: continue
    img = cv2.imread(f)
    if img is None: continue
    
    # Calculate color variance across RGB channels.
    # Gray/black/white pixels have R ~ G ~ B. Colored pixels have high variance.
    b, g, r = cv2.split(img.astype(float))
    variance = np.var([b, g, r], axis=0)
    
    # Threshold for colored pixels
    colored_mask = variance > 100
    
    y_idx, x_idx = np.where(colored_mask)
    if len(y_idx) > 0:
        ymin, ymax = y_idx.min(), y_idx.max()
        xmin, xmax = x_idx.min(), x_idx.max()
        print(f"{f}: X=[{xmin}, {xmax}], Y=[{ymin}, {ymax}], W={xmax-xmin}, H={ymax-ymin}")
    else:
        print(f"{f}: No colored pixels found")
