from PIL import Image
import numpy as np

# Load image
img = Image.open("/home/an/quadruped-robot/report/Report_final/img/fig14- Single leg assembly..jpg").convert("RGBA")
data = np.array(img)

# Simple color threshold to remove near-white background
# Background is roughly > 230 on all channels
r, g, b, a = data[:,:,0], data[:,:,1], data[:,:,2], data[:,:,3]
mask = (r > 210) & (g > 210) & (b > 210)
data[mask, 3] = 0

# Crop to the central leg area to remove text
# The leg is roughly between x: 200 to 650, y: 150 to 800 (from visual estimate of 1024x768)
# Let's just find the bounding box of non-transparent pixels after a tighter crop
crop_box = (250, 100, 650, 850) # approximate
img_cropped = Image.fromarray(data).crop(crop_box)

img_cropped.save("/home/an/quadruped-robot/report/Report_final/img/cropped_leg.png")
print("Saved cropped_leg.png")
