import re

with open('balance_pid_diagram.tex', 'r') as f:
    text = f.read()

# Blocks
text = text.replace('thick, rounded corners=2pt', 'line width=3.0pt, rounded corners=12pt')
text = text.replace('align=center, thick, font=\\MegaHuge]', 'align=center, line width=3.0pt, font=\\MegaHuge, rounded corners=8pt]')
text = text.replace('font=\\MegaHuge, thick]', 'font=\\MegaHuge, line width=3.0pt]')

# Dashed background boundary boxes
text = text.replace('dashed, very thick, inner sep=0pt', 'dashed, line width=4pt, rounded corners=25pt, inner sep=0pt')

# Main routing lines
text = text.replace('draw, -latex\', very thick', 'draw, -latex\', line width=4.0pt')

# Explicitly drawn partial segments without the 'line' style
text = text.replace('\\draw [thick] (ma.east)', '\\draw [line width=4.0pt] (ma.east)')
text = text.replace('\\draw [very thick] (deadzone.east)', '\\draw [line width=4.0pt] (deadzone.east)')

with open('balance_pid_diagram.tex', 'w') as f:
    f.write(text)
