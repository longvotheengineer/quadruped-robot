import re

with open('balance_pid_diagram.tex', 'r') as f:
    content = f.read()

# Lift TL_topleft and Title
content = re.sub(
    r'\\coordinate\s*\(TL_topleft\)\s*at\s*\(0,\s*15\);',
    r'\\coordinate (TL_topleft) at (0, 17);',
    content
)
content = re.sub(
    r'\\node\s*\[below=20pt.*at\s*\(15.75,\s*15\) \{Signal Pre-Conditioning\};',
    r'\\node [below=20pt, font=\\fontsize{35}{40}\\selectfont\\bfseries] at (15.75, 17) {Signal Pre-Conditioning};',
    content
)

# Lift TR_topleft and Title
content = re.sub(
    r'\\coordinate\s*\(TR_topleft\)\s*at\s*\(32.5,\s*15\);',
    r'\\coordinate (TR_topleft) at (32.5, 17);',
    content
)
content = re.sub(
    r'\\node\s*\[below=20pt.*at\s*\(43.75,\s*15\) \{Kinematics \\\& Execution\};',
    r'\\node [below=20pt, font=\\fontsize{35}{40}\\selectfont\\bfseries] at (43.75, 17) {Kinematics \\\& Execution};',
    content
)

# Lift Feedback loop
content = re.sub(
    r'\\draw\s*\[line\]\s*\(robot\.north\)\s*--\s*\(52,\s*16\.5\)\s*--\s*node.*?pos=0\.5\]\s*\{.*?\}\s*\(-2,\s*16\.5\)\s*--\s*\(plant_out\);',
    r'\\draw [line] (robot.north) -- (52, 19) -- node [above, font=\\Huge, pos=0.5] {IMU Physics / Pose Update} (-2, 19) -- (plant_out);',
    content
)

with open('balance_pid_diagram.tex', 'w') as f:
    f.write(content)
