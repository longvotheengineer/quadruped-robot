import re

with open('balance_pid_diagram.tex', 'r') as f:
    text = f.read()

# 1. Update TL Box bounds and Title
text = re.sub(r'\\coordinate \(TL_topleft\) at \(0, \d+\);', r'\\coordinate (TL_topleft) at (0, 15.5);', text)
text = re.sub(r'\\coordinate \(TL_botright\) at \(31\.5, [\d\.\-]+\);', r'\\coordinate (TL_botright) at (31.5, 9.5);', text)
text = re.sub(r'\\node \[below=20pt, font=\\fontsize\{35\}\{40\}\\selectfont\\bfseries\] at \(15\.75, \d+\) \{Signal Pre-Conditioning\};', r'\\node [below=20pt, font=\\fontsize{35}{40}\\selectfont\\bfseries] at (15.75, 15.5) {Signal Pre-Conditioning};', text)

# 2. Update TR Box bounds and Title
text = re.sub(r'\\coordinate \(TR_topleft\) at \(32\.5, \d+\);', r'\\coordinate (TR_topleft) at (32.5, 15.5);', text)
text = re.sub(r'\\coordinate \(TR_botright\) at \(55, [\d\.\-]+\);', r'\\coordinate (TR_botright) at (55, 5.5);', text)
text = re.sub(r'\\node \[below=20pt, font=\\fontsize\{35\}\{40\}\\selectfont\\bfseries\] at \(43\.75, \d+\) \{Kinematics \\\& Execution\};', r'\\node [below=20pt, font=\\fontsize{35}{40}\\selectfont\\bfseries] at (43.75, 15.5) {Kinematics \\& Execution};', text)

# 3. Elevate TL internal nodes from Y=8 to Y=12
text = re.sub(r'\\node \[input, name=plant_out\] at \(-2, 8\) \{\};', r'\\node [input, name=plant_out] at (-2, 12) {};', text)
text = re.sub(r'\\node \[top_block\] \(imu\) at \(4\.5, 8\) \{IMU Sensor\\\\ \\textit\{sensor\\_msgs\}\};', r'\\node [top_block] (imu) at (4.5, 12) {IMU Sensor\\\\ \\textit{sensor\\_msgs}};' , text)
text = re.sub(r'\\node \[top_block\] \(quat\) at \(12\.5, 8\) \{quaternion\\_to\\_rp\\\\ \\textit\{Euler Conversion\}\};', r'\\node [top_block] (quat) at (12.5, 12) {quaternion\\_to\\_rp\\\\ \\textit{Euler Conversion}};' , text)
text = re.sub(r'\\node \[top_block\] \(ema\) at \(20\.5, 8\) \{EMA Filter\\\\ \$\backslash\backslashalpha=0\.4\$\};', r'\\node [top_block] (ema) at (20.5, 12) {EMA Filter\\\\ $\\alpha=0.4$};' , text)
text = re.sub(r'\\node \[top_block\] \(ma\) at \(28\.5, 8\) \{Moving Avg Buffer\\\\ \\textit\{len=270 frames\}\};', r'\\node [top_block] (ma) at (28.5, 12) {Moving Avg Buffer\\\\ \\textit{len=270 frames}};' , text)

# 4. Routing paths fixes
text = text.replace(r'\draw [thick] (ma.east) -- (32, 8)', r'\draw [thick] (ma.east) -- (32, 12)')
text = re.sub(r'\\draw \[line\] \(robot\.north\) -- \(52, \d+\) -- node \[above, font=\\Huge, pos=0\.5\] \{IMU Physics / Pose Update\} \(-2, \d+\) -- \(plant_out\);', r'\\draw [line] (robot.north) -- (52, 17.5) -- node [above, font=\\Huge, pos=0.5] {IMU Physics / Pose Update} (-2, 17.5) -- (plant_out);', text)

with open('balance_pid_diagram.tex', 'w') as f:
    f.write(text)
