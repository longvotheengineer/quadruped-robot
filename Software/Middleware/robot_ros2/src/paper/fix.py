with open("balance_pid_diagram.tex", "r") as f:
    text = f.read()

text = text.replace(
    "\\coordinate (TL_topleft) at (0, 15);",
    "\\coordinate (TL_topleft) at (0, 17);"
)
text = text.replace(
    "at (15.75, 15) {Signal Pre-Conditioning};",
    "at (15.75, 17) {Signal Pre-Conditioning};"
)

text = text.replace(
    "\\coordinate (TR_topleft) at (32.5, 15);",
    "\\coordinate (TR_topleft) at (32.5, 17);"
)
text = text.replace(
    "\\&", "&" # clean up the broken one
)
text = text.replace(
    "at (43.75, 17) {Kinematics \\& Execution};",
    "at (43.75, 17) {Kinematics \\& Execution};"
)
# Wait, let's just do it cleanly by searching the broken block
