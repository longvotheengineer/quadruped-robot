import re

with open('balance_pid_diagram.tex', 'r') as f:
    text = f.read()

# Shift P, I, D components right by 2.5cm
text = re.sub(r'\\node \[bot_block\] \(adaptive\) at \(14\.5, -7\) \{Adaptive\\\\ Gain Scale\};', r'\\node [bot_block] (adaptive) at (17.0, -7) {Adaptive\\\\ Gain Scale};', text)
text = re.sub(r'\\node \[bot_block\] \(integ\) at \(14\.5, -12\) \{\$\backslashdisplaystyle\\int dt\$\\\\ \\textit\{Sat \$\\pm 0\.3\$\}\};', r'\\node [bot_block] (integ) at (17.0, -12) {$\\displaystyle\\int dt$\\\\ \\textit{Sat $\\pm 0.3$}};' , text)
text = re.sub(r'\\node \[bot_block\] \(deriv\) at \(14\.5, -18\.5\) \{\$\backslashdisplaystyle\\frac\{d\}\{dt\}\$\};', r'\\node [bot_block] (deriv) at (17.0, -18.5) {$\\displaystyle\\frac{d}{dt}$};' , text)

text = re.sub(r'\\node \[bot_gain\] \(p\) at \(24, -2\) \{\$K_p\$\};', r'\\node [bot_gain] (p) at (26.5, -2) {$K_p$};' , text)
text = re.sub(r'\\node \[bot_gain\] \(i\) at \(24, -12\) \{\$K_i\$\};', r'\\node [bot_gain] (i) at (26.5, -12) {$K_i$};' , text)
text = re.sub(r'\\node \[bot_gain\] \(d\) at \(24, -18\.5\) \{\$K_d\$\};', r'\\node [bot_gain] (d) at (26.5, -18.5) {$K_d$};' , text)

# Shift downstream components right by 2.5cm
text = re.sub(r'\\node \[sum\] \(pid_sum\) at \(32, -12\) \{-\};', r'\\node [sum] (pid_sum) at (34.5, -12) {-};' , text)
text = re.sub(r'\\node \[bot_block\] \(sat\) at \(40, -12\) \{Saturation\\\\ \$\\pm 0\.3\$\};', r'\\node [bot_block] (sat) at (42.5, -12) {Saturation\\\\ $\\pm 0.3$};' , text)
text = re.sub(r'\\node \[bot_block\] \(lpf\) at \(50, -12\) \{Low Pass\\\\ Filter \$\\alpha=0\.97\$\};', r'\\node [bot_block] (lpf) at (52.5, -12) {Low Pass\\\\ Filter $\\alpha=0.97$};' , text)

# Update LPF routing
text = re.sub(r'\\draw \[line\] \(lpf\.north\) -- \(50, 3\.75\) -- \(36\.0, 3\.75\) -- node \[right, font=\\SuperHuge\] \{Correction\} \(rot\.south\);', r'\\draw [line] (lpf.north) -- (52.5, 3.75) -- (36.0, 3.75) -- node [right, font=\\SuperHuge] {Correction} (rot.south);', text)

with open('balance_pid_diagram.tex', 'w') as f:
    f.write(text)
