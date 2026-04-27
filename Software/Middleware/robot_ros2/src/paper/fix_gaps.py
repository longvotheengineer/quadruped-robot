import re

with open('balance_pid_diagram.tex', 'r') as f:
    text = f.read()

# 1. Expand the boxes to 57.5
text = re.sub(r'\\coordinate \(TR_botright\) at \(55, 5\.5\);', r'\\coordinate (TR_botright) at (57.5, 5.5);', text)
text = re.sub(r'\\node \[below=20pt, font=\\fontsize\{35\}\{40\}\\selectfont\\bfseries\] at \(43\.75, 15\.5\) \{Kinematics \\\& Execution\};', r'\\node [below=20pt, font=\\fontsize{35}{40}\\selectfont\\bfseries] at (45.0, 15.5) {Kinematics \\& Execution};', text)

text = re.sub(r'\\coordinate \(BOT_botright\) at \(55, -22\);', r'\\coordinate (BOT_botright) at (57.5, -22);', text)
text = re.sub(r'\\node \[below=20pt, font=\\fontsize\{35\}\{40\}\\selectfont\\bfseries\] at \(27\.5, 3\) \{Adaptive Attitude PID Controller\};', r'\\node [below=20pt, font=\\fontsize{35}{40}\\selectfont\\bfseries] at (28.75, 3) {Adaptive Attitude PID Controller};', text)


# 2. Spread the Kinematic nodes out to give 3.5cm horizontal gaps
text = re.sub(r'\\node \[top_block\] \(gait\) at \(36\.5, 12\) \{Offline Gait Plan\\\\ \\textit\{Target Feet \(x,y,z\)\}\};', r'\\node [top_block] (gait) at (36.0, 12) {Offline Gait Plan\\\\ \\textit{Target Feet (x,y,z)}};' , text)
text = re.sub(r'\\node \[top_block\] \(rot\) at \(36\.5, 8\) \{Rotation Matrix\\\\ \$R_y\(p\)\\times R_x\(r\)\$\};', r'\\node [top_block] (rot) at (36.0, 8) {Rotation Matrix\\\\ $R_y(p)\\times R_x(r)$};' , text)
text = re.sub(r'\\node \[top_block\] \(ik\) at \(44\.5, 8\) \{Inverse\\\\ Kinematics\};', r'\\node [top_block] (ik) at (45.0, 8) {Inverse\\\\ Kinematics};' , text)
text = re.sub(r'\\node \[top_block\] \(robot\) at \(52, 8\) \{Quadruped\\\\ Robot Plant\};', r'\\node [top_block] (robot) at (54.0, 8) {Quadruped\\\\ Robot Plant};' , text)

# 3. Fix associated paths
text = re.sub(r'\\draw \[line\] \(lpf\.north\) -- \(50, 3\.75\) -- \(36\.5, 3\.75\) -- node \[right, font=\\SuperHuge\] \{Correction\} \(rot\.south\);', r'\\draw [line] (lpf.north) -- (50, 3.75) -- (36.0, 3.75) -- node [right, font=\\SuperHuge] {Correction} (rot.south);', text)

text = re.sub(r'\\draw \[line\] \(robot\.north\) -- \(52, 17\.5\) -- node \[above, font=\\Huge, pos=0\.5\] \{IMU Physics / Pose Update\} \(-2, 17\.5\) -- \(plant_out\);', r'\\draw [line] (robot.north) -- (54.0, 17.5) -- node [above, font=\\Huge, pos=0.5] {IMU Physics / Pose Update} (-2, 17.5) -- (plant_out);', text)

with open('balance_pid_diagram.tex', 'w') as f:
    f.write(text)
