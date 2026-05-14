from PIL import Image, ImageDraw, ImageFont
import os

img_path = '/home/an/quadruped-robot/Software/Middleware/robot_ros2/src/thesis/Images/architecture/system_overview.jpeg'
out_path = '/home/an/quadruped-robot/Software/Middleware/robot_ros2/src/thesis/Images/architecture/system_overview_with_specs_v2.jpeg'

if not os.path.exists(img_path):
    print("Image not found")
    exit(1)

# Open original image
img = Image.open(img_path)
width, height = img.size

# Specifications to add (matching the detailed fields of the reference image but adapted for this thesis)
specs = [
    "Degrees of freedom: 12",
    "Weight: 3.5 kg",
    "Payload: 1.5 kg",
    "Run time (DC Wired): Unlimited",
    "Max speed: 0.05 m/sec",
    "Max height: 18.5 cm",
    "Min height(walk): 12 cm",
    "Width: 19.1 cm",
    "Length: 20.9 cm",
    "Materials: PLA (3D Printed)"
]

# Create new image with extra width
extra_width = int(width * 0.45)  # Add 45% width for the specs
new_width = width + extra_width
new_img = Image.new('RGB', (new_width, height), (255, 255, 255))
new_img.paste(img, (0, 0))

draw = ImageDraw.Draw(new_img)

# Try to load a font, otherwise use default
try:
    title_font = ImageFont.truetype("/usr/share/fonts/truetype/freefont/FreeSerifItalic.ttf", int(height * 0.04))
    text_font = ImageFont.truetype("/usr/share/fonts/truetype/freefont/FreeSansBold.ttf", int(height * 0.03))
except IOError:
    title_font = ImageFont.load_default()
    text_font = ImageFont.load_default()

# Draw rounded rectangle (pale yellow)
padding_x = int(extra_width * 0.08)
padding_y = int(height * 0.05)
box_x0 = width + padding_x
box_y0 = padding_y
box_x1 = new_width - padding_x
box_y1 = height - padding_y

def draw_rounded_rectangle(draw, xy, rad, fill):
    x0, y0, x1, y1 = xy
    draw.rectangle([x0, y0 + rad, x1, y1 - rad], fill=fill)
    draw.rectangle([x0 + rad, y0, x1 - rad, y1], fill=fill)
    draw.pieslice([x0, y0, x0 + rad * 2, y0 + rad * 2], 180, 270, fill=fill)
    draw.pieslice([x1 - rad * 2, y0, x1, y0 + rad * 2], 270, 360, fill=fill)
    draw.pieslice([x0, y1 - rad * 2, x0 + rad * 2, y1], 90, 180, fill=fill)
    draw.pieslice([x1 - rad * 2, y1 - rad * 2, x1, y1], 0, 90, fill=fill)
    # Add border
    draw.arc([x0, y0, x0 + rad * 2, y0 + rad * 2], 180, 270, fill='black', width=3)
    draw.arc([x1 - rad * 2, y0, x1, y0 + rad * 2], 270, 360, fill='black', width=3)
    draw.arc([x0, y1 - rad * 2, x0 + rad * 2, y1], 90, 180, fill='black', width=3)
    draw.arc([x1 - rad * 2, y1 - rad * 2, x1, y1], 0, 90, fill='black', width=3)
    draw.line([x0 + rad, y0, x1 - rad, y0], fill='black', width=3)
    draw.line([x0 + rad, y1, x1 - rad, y1], fill='black', width=3)
    draw.line([x0, y0 + rad, x0, y1 - rad], fill='black', width=3)
    draw.line([x1, y0 + rad, x1, y1 - rad], fill='black', width=3)

radius = int(min(box_x1 - box_x0, box_y1 - box_y0) * 0.05)
fill_color = (255, 255, 204)  # pale yellow
draw_rounded_rectangle(draw, [box_x0, box_y0, box_x1, box_y1], radius, fill_color)

# Draw Title
title = "Specifications of the robot"
try:
    bbox = draw.textbbox((0, 0), title, font=title_font)
    title_w = bbox[2] - bbox[0]
    title_h = bbox[3] - bbox[1]
except AttributeError:
    title_w, title_h = draw.textsize(title, font=title_font)

title_x = box_x0 + (box_x1 - box_x0 - title_w) / 2
title_y = box_y0 + padding_y * 0.5
draw.text((title_x, title_y), title, fill='black', font=title_font)
draw.line((title_x, title_y + title_h + 5, title_x + title_w, title_y + title_h + 5), fill='black', width=2)

# Draw Specs
text_y = title_y + title_h + padding_y * 1.5
line_spacing = int((box_y1 - text_y - padding_y) / len(specs))
if line_spacing > height * 0.06:
    line_spacing = int(height * 0.06)

for spec in specs:
    parts = spec.split(":", 1)
    if len(parts) == 2:
        key = parts[0] + ":"
        val = parts[1].strip()
        draw.text((box_x0 + padding_x, text_y), key, fill='black', font=text_font)
        draw.text((box_x0 + padding_x + int(extra_width*0.55), text_y), val, fill='black', font=text_font)
    else:
        draw.text((box_x0 + padding_x, text_y), spec, fill='black', font=text_font)
    text_y += line_spacing

new_img.save(out_path, quality=95)
print(f"Saved to {out_path}")
