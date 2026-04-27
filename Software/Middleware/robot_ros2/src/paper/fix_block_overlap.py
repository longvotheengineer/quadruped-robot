import re

with open('balance_pid_diagram.tex', 'r') as f:
    text = f.read()

# 1. Expand the boxes to 60.5
text = re.sub(r'\\coordinate \(TR_botright\) at \(57\.5, 5\.5\);', r'\\coordinate (TR_botright) at (60.5, 5.5);', text)
text = re.sub(r'\\coordinate \(BOT_botright\) at \(57\.5, -22\);', r'\\coordinate (BOT_botright) at (60.5, -22);', text)

text = re.sub(r'\\node \[(below=20pt, font=\\fontsize\{35\}\{40\}\\selectfont\\bfseries)\] at \([0-9\.]+, 15\.5\) \{Kinematics \\\& Execution\};', r'\\node [\1] at (46.5, 15.5) {Kinematics \\& Execution};', text)
text = re.sub(r'\\node \[(below=20pt, font=\\fontsize\{35\}\{40\}\\selectfont\\bfseries)\] at \([0-9\.]+, 3\) \{Adaptive Attitude PID Controller\};', r'\\node [\1] at (30.25, 3) {Adaptive Attitude PID Controller};', text)

# 2. Spread the Kinematic nodes to fill the expanded TR box
text = re.sub(r'\\node \[top_block\] \(gait\) at \(36\.0, 12\)', r'\\node [top_block] (gait) at (36.5, 12)', text)
text = re.sub(r'\\node \[top_block\] \(rot\) at \(36\.0, 8\)', r'\\node [top_block] (rot) at (36.5, 8)', text)
text = re.sub(r'\\node \[top_block\] \(ik\) at \(45\.0, 8\)', r'\\node [top_block] (ik) at (46.5, 8)', text)
text = re.sub(r'\\node \[top_block\] \(robot\) at \(54\.0, 8\)', r'\\node [top_block] (robot) at (56.5, 8)', text)

# 3. Shift PID components strictly right by 3.0cm
text = re.sub(r'\(adaptive\) at \(17\.0, -7\)', r'(adaptive) at (20.0, -7)', text)
text = re.sub(r'\(integ\) at \(17\.0, -12\)', r'(integ) at (20.0, -12)', text)
text = re.sub(r'\(deriv\) at \(17\.0, -18\.5\)', r'(deriv) at (20.0, -18.5)', text)

text = re.sub(r'\(p\) at \(26\.5, -2\)', r'(p) at (29.5, -2)', text)
text = re.sub(r'\(i\) at \(26\.5, -12\)', r'(i) at (29.5, -12)', text)
text = re.sub(r'\(d\) at \(26\.5, -18\.5\)', r'(d) at (29.5, -18.5)', text)

text = re.sub(r'\(pid_sum\) at \(34\.5, -12\)', r'(pid_sum) at (37.5, -12)', text)
text = re.sub(r'\(sat\) at \(42\.5, -12\)', r'(sat) at (45.5, -12)', text)
text = re.sub(r'\(lpf\) at \(52\.5, -12\)', r'(lpf) at (55.5, -12)', text)

# 4. Update the hardcoded X routing values
text = re.sub(r'\(lpf\.north\) -- \(52\.5, 3\.75\) -- \(36\.0, 3\.75\) --', r'(lpf.north) -- (55.5, 3.75) -- (36.5, 3.75) --', text)
text = re.sub(r'\(robot\.north\) -- \(54\.0, 17\.5\) -- node', r'(robot.north) -- (56.5, 17.5) -- node', text)

with open('balance_pid_diagram.tex', 'w') as f:
    f.write(text)
