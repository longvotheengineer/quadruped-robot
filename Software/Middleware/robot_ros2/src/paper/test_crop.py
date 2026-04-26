import cv2
import numpy as np

img = cv2.imread('figures/pid_error_home.png')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# The grid border is a solid dark gray line.
# Let's find the first and last column that has a very long vertical line.
# And the first and last row that has a very long horizontal line.

# Apply edge detection
edges = cv2.Canny(gray, 50, 150)

# Sum edges along axes
row_edges = np.sum(edges, axis=1)
col_edges = np.sum(edges, axis=0)

y1 = np.argmax(row_edges > np.max(row_edges)*0.5)
y2 = len(row_edges) - 1 - np.argmax(row_edges[::-1] > np.max(row_edges)*0.5)
x1 = np.argmax(col_edges > np.max(col_edges)*0.5)
x2 = len(col_edges) - 1 - np.argmax(col_edges[::-1] > np.max(col_edges)*0.5)

print(f"Crop bounds: x1={x1}, x2={x2}, y1={y1}, y2={y2}")

# Let's also check if there's a legend box we should crop out.
# Legend is usually top right.
# We can just blank out the top right corner if there's a legend.
# Or better, just crop the image to the grid, AND THEN in matplotlib,
# add white rectangle over the old legend!
