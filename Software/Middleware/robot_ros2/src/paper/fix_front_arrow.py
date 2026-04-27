import re

with open('balance_pid_diagram.tex', 'r') as f:
    text = f.read()

# Shift deadzone right by 1.0cm
text = re.sub(r'\\node \[bot_block\] \(deadzone\) at \(5, -12\) \{Deadzone\\\\ \$\\pm 0\.03 \\text\{ rad\}\$\};', r'\\node [bot_block] (deadzone) at (6.0, -12) {Deadzone\\\\ $\\pm 0.03 \\text{ rad}$};', text)

# Shift err_out from 9.6 -> 10.6, err_out_scale from 10.0 -> 11.0
text = re.sub(r'\\draw \[very thick\] \(deadzone\.east\) -- \(9\.6, -12\) coordinate \(err_out\) -- \(10\.0, -12\) coordinate \(err_out_scale\);', r'\\draw [very thick] (deadzone.east) -- (10.6, -12) coordinate (err_out) -- (11.0, -12) coordinate (err_out_scale);', text)

with open('balance_pid_diagram.tex', 'w') as f:
    f.write(text)
