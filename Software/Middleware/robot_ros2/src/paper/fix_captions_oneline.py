import re

with open('balance_pid_diagram.tex', 'r') as f:
    text = f.read()

# Replace all literal backslashes used for newlines in node text
text = text.replace('Offline Gait Plan\\\\ \\textit{Target Feet (x,y,z)}', 'Offline Gait Plan: \\textit{Target Feet (x,y,z)}')
text = text.replace('Rotation Matrix\\\\ $R_y(p)\\times R_x(r)$', 'Rotation Matrix: $R_y(p)\\times R_x(r)$')
text = text.replace('Inverse\\\\ Kinematics', 'Inverse Kinematics')
text = text.replace('Quadruped\\\\ Robot Plant', 'Quadruped Robot Plant')

text = text.replace('Deadzone\\\\ $\\pm 0.03 \\text{ rad}$', 'Deadzone $\\pm 0.03 \\text{ rad}$')
text = text.replace('Adaptive\\\\ Gain Scale', 'Adaptive Gain Scale')
# For integral, use a paranthesis for the saturation text
text = text.replace('$\\displaystyle\\int dt$\\\\ \\textit{Sat $\\pm 0.3$}', '$\\displaystyle\\int dt$ \\textit{(Sat $\\pm 0.3$)}')

text = text.replace('Saturation\\\\ $\\pm 0.3$', 'Saturation $\\pm 0.3$')
text = text.replace('Low Pass\\\\ Filter $\\alpha=0.97$', 'Low Pass Filter $\\alpha=0.97$')

# Are there any other wrapping captions?
text = text.replace('Adjusted\\\\XYZ', 'Adjusted XYZ')

with open('balance_pid_diagram.tex', 'w') as f:
    f.write(text)
