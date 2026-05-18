import cv2
import numpy as np

# Load the image
img = cv2.imread('/home/an/quadruped-robot/Software/Middleware/robot_ros2/src/thesis/Images/img_matlab/fig-step Kp=1 Ki=0 Kd=0.png')
if img is None:
    print("Error loading image")
    exit()

# Convert to HSV to easily isolate the blue/orange/yellow line
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# Assuming the line is blue (typical for matlab plots)
lower_blue = np.array([100, 50, 50])
upper_blue = np.array([130, 255, 255])
mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

# Assuming the line is red/orange (another typical color)
lower_red1 = np.array([0, 50, 50])
upper_red1 = np.array([10, 255, 255])
mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
lower_red2 = np.array([170, 50, 50])
upper_red2 = np.array([180, 255, 255])
mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
mask_red = mask_red1 + mask_red2

# Yellow (another common color)
lower_yellow = np.array([20, 50, 50])
upper_yellow = np.array([40, 255, 255])
mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

for mask_name, mask in [('Blue', mask_blue), ('Red', mask_red), ('Yellow', mask_yellow)]:
    points = np.column_stack(np.where(mask > 0))
    if len(points) > 100:
        print(f"Found {len(points)} points in {mask_name} line")
        min_y = np.min(points[:, 0])
        max_y = np.max(points[:, 0])
        min_x = np.min(points[:, 1])
        max_x = np.max(points[:, 1])
        print(f"Y range (pixels): {min_y} to {max_y}")
        print(f"X range (pixels): {min_x} to {max_x}")
        
        # approximate the values assuming some scale, but it's hard without OCR
        # let's just get the final Y value (steady state) compared to the start Y value (0)
        start_y = points[points[:, 1] < min_x + 50][:, 0].mean()
        end_y = points[points[:, 1] > max_x - 50][:, 0].mean()
        print(f"Start Y (mean pixel): {start_y}")
        print(f"End Y (mean pixel): {end_y}")
        print(f"Max deviation Y (pixel): {max_y if max_y - start_y > start_y - min_y else min_y}")
        
