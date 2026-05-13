import cv2
import numpy as np

def analyze_colors(image_path):
    img = cv2.imread(image_path)
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    
    # Reshape to a list of pixels
    pixels = img_rgb.reshape(-1, 3)
    
    # Filter out white (background) and black/gray (axes, text)
    mask = (np.min(pixels, axis=1) < 200) & (np.max(pixels, axis=1) > 50) & (np.max(pixels, axis=1) - np.min(pixels, axis=1) > 50)
    colored_pixels = pixels[mask]
    
    if len(colored_pixels) > 0:
        # Simple quantization to 16 colors (4 bits per channel)
        quantized = (colored_pixels // 32) * 32
        
        # Count frequencies
        unique, counts = np.unique(quantized, axis=0, return_counts=True)
        
        # Sort by frequency
        sorted_indices = np.argsort(-counts)
        print(f"Colors in {image_path}:")
        for i in range(min(5, len(sorted_indices))):
            idx = sorted_indices[i]
            print(f"  RGB: {unique[idx]}, count: {counts[idx]}")
    else:
        print("No dominant colors found")

analyze_colors('thesis/presentation/images/results/trot_pid_roll_controller_meas_avg_zoom_backup.png')
analyze_colors('thesis/presentation/images/results/trot_pid_roll_controller_p_i_d_zoom_backup.png')
analyze_colors('thesis/presentation/images/results/trot_pid_roll_controller_sat_lpf_ff_zoom_backup.png')

