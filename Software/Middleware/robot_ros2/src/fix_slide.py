import os

with open('thesis/presentation/slide.tex', 'r') as f:
    current_lines = f.readlines()

with open('old_slide.tex', 'r') as f:
    old_lines = f.readlines()

# find \section{Thiết kế và thực hiện phần mềm} in both
current_start = -1
for i, line in enumerate(current_lines):
    if r'\section{Thiết kế và thực hiện phần mềm}' in line:
        current_start = i
        break

old_start = -1
for i, line in enumerate(old_lines):
    if r'\section{Thiết kế và thực hiện phần mềm}' in line:
        old_start = i
        break

if current_start != -1 and old_start != -1:
    new_content = current_lines[:current_start] + old_lines[old_start:]
    with open('thesis/presentation/slide.tex', 'w') as f:
        f.writelines(new_content)
    print("Replaced section 4 onwards successfully.")
else:
    print(f"Error finding section 4. current_start={current_start}, old_start={old_start}")

