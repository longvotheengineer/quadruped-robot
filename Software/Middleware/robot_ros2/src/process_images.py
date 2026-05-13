import cv2
import numpy as np

img = cv2.imread('thesis/presentation/images/results/trot_pid_pitch_controller_meas_avg_zoom.png')
print("Image shape:", img.shape)
print("Corners colors:")
print("Top-left:", img[10, 10])
print("Bottom-right:", img[-10, -10])
print("Center:", img[img.shape[0]//2, img.shape[1]//2])

# Find non-background pixels to get the bounding box of the actual plot
# Assuming background is the color of top-left
bg_color = img[10, 10]
diff = np.abs(img.astype(int) - bg_color.astype(int))
mask = np.sum(diff, axis=2) > 20
y_indices, x_indices = np.where(mask)
if len(y_indices) > 0:
    print(f"Content bbox: x=[{x_indices.min()}, {x_indices.max()}], y=[{y_indices.min()}, {y_indices.max()}]")
