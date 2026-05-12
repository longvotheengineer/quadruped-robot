import graphviz
import os

# Create directed graph
dot = graphviz.Digraph(comment='ROS 2 Architecture', format='png')
dot.attr(rankdir='TB', size='12,8')
dot.attr(nodesep='1.5', ranksep='1.5')

# Set global node attributes
dot.attr('node', fontname='Helvetica', fontsize='18', margin='0.2')
dot.attr('edge', fontname='Helvetica', fontsize='16', penwidth='2.0')

# Define nodes with HTML-like labels for better formatting
dot.node('A', '''<<TABLE BORDER="0" CELLBORDER="1" CELLSPACING="0" CELLPADDING="4">
  <TR><TD BGCOLOR="#E6F2FF"><B>/teleop_command_node</B></TD></TR>
  <TR><TD BGCOLOR="#FFFFFF"><FONT POINT-SIZE="10">User Interface</FONT></TD></TR>
</TABLE>>''', shape='none')

dot.node('B', '''<<TABLE BORDER="0" CELLBORDER="1" CELLSPACING="0" CELLPADDING="4">
  <TR><TD BGCOLOR="#E6FFE6"><B>/node_leg_controller</B></TD></TR>
  <TR><TD BGCOLOR="#FFFFFF"><FONT POINT-SIZE="10">Gait Generator &amp; IK</FONT></TD></TR>
</TABLE>>''', shape='none')

dot.node('C', '''<<TABLE BORDER="0" CELLBORDER="1" CELLSPACING="0" CELLPADDING="4">
  <TR><TD BGCOLOR="#E6FFE6"><B>/nodeBalanceController</B></TD></TR>
  <TR><TD BGCOLOR="#FFFFFF"><FONT POINT-SIZE="10">Attitude PID Controller</FONT></TD></TR>
</TABLE>>''', shape='none')

dot.node('D', '''<<TABLE BORDER="0" CELLBORDER="1" CELLSPACING="0" CELLPADDING="4">
  <TR><TD BGCOLOR="#FFE6E6"><B>/imu_plugin</B></TD></TR>
  <TR><TD BGCOLOR="#FFFFFF"><FONT POINT-SIZE="10">Gazebo Sensor</FONT></TD></TR>
</TABLE>>''', shape='none')

dot.node('E', '''<<TABLE BORDER="0" CELLBORDER="1" CELLSPACING="0" CELLPADDING="4">
  <TR><TD BGCOLOR="#FFE6E6"><B>/controller_manager</B></TD></TR>
  <TR><TD BGCOLOR="#FFFFFF"><FONT POINT-SIZE="10">Gazebo Effort Interfaces</FONT></TD></TR>
</TABLE>>''', shape='none')

# Define ranks to force a balanced, squarish 2D layout
with dot.subgraph() as s:
    s.attr(rank='same')
    s.node('A')
    s.node('D')

with dot.subgraph() as s:
    s.attr(rank='same')
    s.node('B')
    s.node('C')

# Define edges
dot.edge('A', 'B', label=' /gait_control')
dot.edge('A', 'C', label=' /balance_mode')
dot.edge('D', 'C', label=' /imu/data')
dot.edge('C', 'B', label=' /posture/gait_correction')
dot.edge('B', 'E', label=' /leg_controller/commands')

# Determine output directory
script_dir = os.path.dirname(os.path.abspath(__file__))
output_dir = os.path.join(script_dir, '..', 'Images', 'architecture')
os.makedirs(output_dir, exist_ok=True)
output_path = os.path.join(output_dir, 'simplified_ros2_graph')

# Render graph
dot.render(output_path, view=False)
print("Graph generated successfully at:", output_path + ".png")
