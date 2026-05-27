from PIL import Image

img = Image.open('quadrupedrobot.jpeg')
w, h = img.size

# The watermark is in the bottom right corner.
# Let's crop 40 pixels from the right and 40 from the bottom.
img_cropped1 = img.crop((0, 0, w - 50, h - 50))
img_cropped1.save('quadrupedrobot_crop50.png')

img_cropped2 = img.crop((0, 0, w - 75, h - 75))
img_cropped2.save('quadrupedrobot_crop75.png')

# Alternatively, let's take a 80x80 patch from bottom-left (0, h-80), flip it, and paste it at bottom-right (w-80, h-80)
patch = img.crop((0, h-100, 100, h))
patch = patch.transpose(Image.FLIP_LEFT_RIGHT)
img_patched = img.copy()
img_patched.paste(patch, (w-100, h-100))
img_patched.save('quadrupedrobot_patched.png')

