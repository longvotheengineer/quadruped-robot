import cv2
import numpy as np

img = cv2.imread('thesis/presentation/images/results/trot_pid_pitch_controller_meas_avg_zoom.png')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
edges = cv2.Canny(gray, 50, 150)
contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Find the largest rectangular contour
max_area = 0
best_rect = None
for c in contours:
    x, y, w, h = cv2.boundingRect(c)
    area = w * h
    if area > max_area and area < img.shape[0]*img.shape[1] * 0.95:
        max_area = area
        best_rect = (x, y, w, h)

print(f"Detected plot box: {best_rect}")
if best_rect:
    x, y, w, h = best_rect
    cropped = img[y:y+h, x:x+w]
    cv2.imwrite('test_cropped.png', cropped)
