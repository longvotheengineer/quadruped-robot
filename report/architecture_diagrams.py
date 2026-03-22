#!/usr/bin/env python3
"""
architecture_diagrams.py
========================
Generate architecture diagrams for the Quadruped Robot thesis:
  Fig 9  — Gazebo ROS2 Simulation System Architecture
  Fig 10 — Software Control Architecture (block diagram)

Dependencies: matplotlib, numpy (same as simulation_report.py)
"""

import os
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, ArrowStyle
import numpy as np

FIGURE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'figures')
os.makedirs(FIGURE_DIR, exist_ok=True)

# ─────────────────────────────────────────────────────────────────────────
#  Color palette
# ─────────────────────────────────────────────────────────────────────────
C_GAZEBO   = '#E8F5E9'   # light green
C_ROS2     = '#E3F2FD'   # light blue
C_CTRL     = '#FFF3E0'   # light orange
C_USER     = '#F3E5F5'   # light purple
C_HW       = '#FFEBEE'   # light red
C_TOPIC    = '#FFD54F'   # amber
C_NODE     = '#42A5F5'   # blue
C_BORDER   = '#37474F'   # dark gray
C_ARROW    = '#455A64'   # gray-blue
C_DATA     = '#1B5E20'   # dark green - data flow
C_CMD      = '#B71C1C'   # dark red - command flow
C_SW_BG    = '#ECEFF1'   # software background
C_PLAN     = '#E1F5FE'   # planning modules
C_IK       = '#FFF9C4'   # IK module
C_PD       = '#FFCCBC'   # PD controller
C_PUB      = '#C8E6C9'   # publish module


def _fancy_box(ax, xy, w, h, text, facecolor, fontsize=10,
               edgecolor=C_BORDER, lw=1.5, fontstyle='normal',
               fontweight='normal', textcolor='black', alpha=1.0,
               rounded=True, zorder=2):
    """Draw a rounded rectangle with centered text."""
    style = "round,pad=0.015" if rounded else "square,pad=0"
    box = FancyBboxPatch(xy, w, h, boxstyle=style,
                         facecolor=facecolor, edgecolor=edgecolor,
                         linewidth=lw, alpha=alpha, zorder=zorder)
    ax.add_patch(box)
    cx, cy = xy[0] + w/2, xy[1] + h/2
    ax.text(cx, cy, text, ha='center', va='center',
            fontsize=fontsize, fontweight=fontweight,
            fontstyle=fontstyle, color=textcolor, zorder=zorder+1)
    return box


def _arrow(ax, xy_start, xy_end, color=C_ARROW, lw=1.8, style='->', 
           connectionstyle='arc3,rad=0', zorder=3):
    """Draw an arrow between two points."""
    arrow = FancyArrowPatch(xy_start, xy_end,
                            arrowstyle=ArrowStyle(style, head_length=8, head_width=5),
                            connectionstyle=connectionstyle,
                            color=color, linewidth=lw, zorder=zorder)
    ax.add_patch(arrow)
    return arrow


def _label_on_arrow(ax, xy_start, xy_end, text, fontsize=7.5,
                    offset=(0, 8), color='#333333', bg='white'):
    """Place a label near the midpoint of an arrow."""
    mx = (xy_start[0] + xy_end[0]) / 2 + offset[0]
    my = (xy_start[1] + xy_end[1]) / 2 + offset[1]
    ax.text(mx, my, text, ha='center', va='center', fontsize=fontsize,
            color=color, zorder=5,
            bbox=dict(boxstyle='round,pad=0.15', facecolor=bg,
                      edgecolor='#BDBDBD', alpha=0.92, lw=0.6))


# ═══════════════════════════════════════════════════════════════════════════
#  FIGURE 9 — Gazebo ROS2 Simulation System Architecture
# ═══════════════════════════════════════════════════════════════════════════

def fig9_system_architecture():
    fig, ax = plt.subplots(1, 1, figsize=(16, 10))
    ax.set_xlim(0, 160)
    ax.set_ylim(0, 100)
    ax.set_aspect('equal')
    ax.axis('off')

    # ── Background regions ──────────────────────────────────────────────
    # User/Operator region (top)
    _fancy_box(ax, (3, 85), 154, 12, '', C_USER, rounded=True, lw=2,
               edgecolor='#9C27B0', alpha=0.25, zorder=0)
    ax.text(80, 95.5, 'User / Operator', ha='center', va='center',
            fontsize=11, fontweight='bold', color='#6A1B9A', zorder=1)

    # ROS2 Middleware region (middle)
    _fancy_box(ax, (3, 32), 154, 50, '', C_ROS2, rounded=True, lw=2,
               edgecolor='#1565C0', alpha=0.2, zorder=0)
    ax.text(80, 80, 'ROS 2 Humble — Middleware Layer', ha='center', va='center',
            fontsize=11, fontweight='bold', color='#0D47A1', zorder=1)

    # Gazebo region (bottom)
    _fancy_box(ax, (3, 3), 154, 26, '', C_GAZEBO, rounded=True, lw=2,
               edgecolor='#2E7D32', alpha=0.25, zorder=0)
    ax.text(80, 27.5, 'Gazebo Classic — Physics Simulation Engine', ha='center', va='center',
            fontsize=11, fontweight='bold', color='#1B5E20', zorder=1)

    # ── User nodes ──────────────────────────────────────────────────────
    _fancy_box(ax, (15, 87), 30, 8, 'Terminal\n(ros2 topic pub)', '#CE93D8',
               fontsize=9, fontweight='bold')
    _fancy_box(ax, (65, 87), 30, 8, 'RViz2\nVisualization', '#CE93D8',
               fontsize=9, fontweight='bold')
    _fancy_box(ax, (115, 87), 30, 8, 'Custom GUI\n(optional)', '#CE93D8',
               fontsize=9, fontweight='bold')

    # ── ROS2 Nodes ──────────────────────────────────────────────────────
    # Main controller node
    _fancy_box(ax, (10, 62), 38, 14, '', C_CTRL, fontsize=9,
               edgecolor='#E65100', lw=2, alpha=0.6)
    ax.text(29, 74, 'node_leg_controller', ha='center', va='center',
            fontsize=10, fontweight='bold', color='#BF360C', zorder=3)
    # Sub-modules inside controller
    _fancy_box(ax, (12, 63.5), 16, 5, 'Gait\nGenerator', '#FFE0B2',
               fontsize=7.5, fontweight='bold', lw=1)
    _fancy_box(ax, (30, 63.5), 16, 5, 'Inverse\nKinematics', '#FFE0B2',
               fontsize=7.5, fontweight='bold', lw=1)
    _fancy_box(ax, (12, 69), 16, 4, 'Quintic\nPlanning', '#FFE0B2',
               fontsize=7, fontweight='bold', lw=1)
    _fancy_box(ax, (30, 69), 16, 4, 'PD Controller\n(ControllerSim)', '#FFCCBC',
               fontsize=6.5, fontweight='bold', lw=1)

    # Robot State Publisher
    _fancy_box(ax, (55, 62), 30, 8, 'robot_state_\npublisher', '#90CAF9',
               fontsize=9, fontweight='bold')

    # Controller Manager
    _fancy_box(ax, (95, 62), 30, 8, 'controller_manager\n(ros2_control)', '#90CAF9',
               fontsize=9, fontweight='bold')

    # Joint State Broadcaster
    _fancy_box(ax, (60, 44), 30, 8, 'joint_state_\nbroadcaster', '#A5D6A7',
               fontsize=9, fontweight='bold')

    # Effort Controller
    _fancy_box(ax, (100, 44), 30, 8, 'leg_controller\n(JointGroupEffort\nController)', '#EF9A9A',
               fontsize=8, fontweight='bold')

    # ── ROS2 Topics ─────────────────────────────────────────────────────
    # /gait_control topic
    _fancy_box(ax, (15, 78), 28, 4, '/gait_control', C_TOPIC,
               fontsize=8, fontweight='bold', lw=1.2, edgecolor='#F57F17')
    ax.text(35, 80, 'String', ha='left', va='center', fontsize=7,
            color='#666', fontstyle='italic', zorder=4)

    # /joint_states topic
    _fancy_box(ax, (55, 35), 28, 4, '/joint_states', C_TOPIC,
               fontsize=8, fontweight='bold', lw=1.2, edgecolor='#F57F17')
    ax.text(75, 37, 'JointState', ha='left', va='center', fontsize=7,
            color='#666', fontstyle='italic', zorder=4)

    # /leg_controller/commands topic
    _fancy_box(ax, (98, 35), 34, 4, '/leg_controller/commands', C_TOPIC,
               fontsize=7.5, fontweight='bold', lw=1.2, edgecolor='#F57F17')
    ax.text(123, 37, 'Float64\nMultiArray', ha='left', va='center', fontsize=6.5,
            color='#666', fontstyle='italic', zorder=4)

    # /robot_description topic
    _fancy_box(ax, (55, 55), 28, 4, '/robot_description', C_TOPIC,
               fontsize=7.5, fontweight='bold', lw=1.2, edgecolor='#F57F17')

    # /tf topic
    _fancy_box(ax, (95, 55), 28, 4, '/tf, /tf_static', C_TOPIC,
               fontsize=8, fontweight='bold', lw=1.2, edgecolor='#F57F17')

    # ── Gazebo internals ─────────────────────────────────────────────────
    _fancy_box(ax, (15, 8), 28, 10, 'Physics Engine\n(ODE)\nCollision Detection', '#A5D6A7',
               fontsize=8.5, fontweight='bold')
    _fancy_box(ax, (50, 8), 28, 10, 'URDF Model\n(test.urdf)\n12 Revolute Joints', '#A5D6A7',
               fontsize=8.5, fontweight='bold')
    _fancy_box(ax, (85, 8), 28, 10, 'ros2_control\nHardware Interface\n(gazebo_ros2_control)', '#A5D6A7',
               fontsize=8, fontweight='bold')
    _fancy_box(ax, (120, 8), 28, 10, '3D Rendering\nSensor Simulation\nWorld Environment', '#A5D6A7',
               fontsize=8.5, fontweight='bold')

    # ── Arrows (data flow) ──────────────────────────────────────────────
    # Terminal → /gait_control
    _arrow(ax, (30, 87), (29, 82), color=C_CMD, lw=1.5)

    # /gait_control → node_leg_controller
    _arrow(ax, (29, 78), (29, 76), color=C_CMD, lw=1.5)

    # node_leg_controller → /leg_controller/commands
    _arrow(ax, (48, 66), (98, 37), color=C_CMD, lw=2,
           connectionstyle='arc3,rad=-0.15')
    _label_on_arrow(ax, (48, 66), (98, 37), 'Effort\ncommands\n(12 torques)',
                    offset=(5, 5), fontsize=7)

    # /leg_controller/commands → effort controller
    _arrow(ax, (115, 39), (115, 44), color=C_CMD, lw=1.5)

    # Effort controller → Gazebo hardware interface
    _arrow(ax, (115, 44), (99, 18), color=C_CMD, lw=1.5,
           connectionstyle='arc3,rad=0.1')

    # Gazebo → joint_state_broadcaster
    _arrow(ax, (99, 18), (75, 44), color=C_DATA, lw=1.5,
           connectionstyle='arc3,rad=-0.15')
    _label_on_arrow(ax, (99, 18), (75, 44), 'Joint positions\n& velocities',
                    offset=(-5, -3), fontsize=7)

    # joint_state_broadcaster → /joint_states
    _arrow(ax, (69, 44), (69, 39), color=C_DATA, lw=1.5)

    # /joint_states → node_leg_controller (feedback)
    _arrow(ax, (55, 37), (29, 62), color=C_DATA, lw=1.5,
           connectionstyle='arc3,rad=0.15')
    _label_on_arrow(ax, (55, 37), (29, 62), 'PD feedback',
                    offset=(-5, 3), fontsize=7)

    # robot_state_publisher → /tf
    _arrow(ax, (85, 66), (109, 59), color=C_DATA, lw=1.2,
           connectionstyle='arc3,rad=0.1')

    # /tf → RViz
    _arrow(ax, (109, 59), (80, 87), color=C_DATA, lw=1.2,
           connectionstyle='arc3,rad=-0.15')

    # robot_state_publisher → /robot_description
    _arrow(ax, (70, 62), (69, 59), color=C_DATA, lw=1.2)

    # Controller manager ↔ spawners
    _arrow(ax, (110, 62), (115, 52), color='#546E7A', lw=1.2)
    _arrow(ax, (110, 62), (75, 52), color='#546E7A', lw=1.2,
           connectionstyle='arc3,rad=0.1')

    # ── Legend ──────────────────────────────────────────────────────────
    legend_elements = [
        mpatches.Patch(facecolor=C_TOPIC, edgecolor='#F57F17', label='ROS2 Topic'),
        mpatches.Patch(facecolor='#90CAF9', edgecolor=C_BORDER, label='ROS2 System Node'),
        mpatches.Patch(facecolor=C_CTRL, edgecolor='#E65100', label='Custom Control Node'),
        mpatches.Patch(facecolor='#A5D6A7', edgecolor=C_BORDER, label='Gazebo Component'),
        plt.Line2D([0], [0], color=C_CMD, lw=2, label='Command flow'),
        plt.Line2D([0], [0], color=C_DATA, lw=2, label='Data/Feedback flow'),
    ]
    ax.legend(handles=legend_elements, loc='lower left', fontsize=8,
              framealpha=0.9, edgecolor='#BDBDBD',
              bbox_to_anchor=(0.0, -0.02))

    fig.suptitle('Figure 9 -- Gazebo ROS2 Simulation System Architecture',
                 fontsize=15, fontweight='bold', y=0.98)

    fig.tight_layout(rect=[0, 0.02, 1, 0.96])
    fig.savefig(os.path.join(FIGURE_DIR, 'fig9_system_architecture.png'),
                dpi=200, bbox_inches='tight')
    plt.close(fig)
    print('  [OK] Figure 9 saved')


# ═══════════════════════════════════════════════════════════════════════════
#  FIGURE 10 — Software Control Architecture (Block Diagram)
# ═══════════════════════════════════════════════════════════════════════════

def fig10_control_architecture():
    fig, ax = plt.subplots(1, 1, figsize=(16, 9))
    ax.set_xlim(0, 160)
    ax.set_ylim(0, 90)
    ax.set_aspect('equal')
    ax.axis('off')

    # ── Background regions ──────────────────────────────────────────────
    # High-level planning region
    _fancy_box(ax, (2, 52), 90, 35, '', C_PLAN, rounded=True, lw=2,
               edgecolor='#0277BD', alpha=0.3, zorder=0)
    ax.text(47, 85.5, 'Trajectory Planning Layer', ha='center', va='center',
            fontsize=11, fontweight='bold', color='#01579B', zorder=1)

    # Low-level control region
    _fancy_box(ax, (96, 52), 60, 35, '', C_PD, rounded=True, lw=2,
               edgecolor='#BF360C', alpha=0.2, zorder=0)
    ax.text(126, 85.5, 'Low-Level Control Layer', ha='center', va='center',
            fontsize=11, fontweight='bold', color='#BF360C', zorder=1)

    # Plant (Gazebo/Robot) region
    _fancy_box(ax, (2, 3), 154, 42, '', C_GAZEBO, rounded=True, lw=2,
               edgecolor='#2E7D32', alpha=0.2, zorder=0)
    ax.text(79, 43.5, 'Plant — Gazebo Simulation / Physical Robot', ha='center', va='center',
            fontsize=11, fontweight='bold', color='#1B5E20', zorder=1)

    # ── High-level planning blocks ─────────────────────────────────────
    # Gait command input
    _fancy_box(ax, (4, 68), 20, 10, 'Gait Command\n(FORWARD,\nTURN, HOMING)', '#B39DDB',
               fontsize=8, fontweight='bold', edgecolor='#4527A0', lw=1.5)

    # Gait Generator
    _fancy_box(ax, (28, 68), 24, 10, 'Gait Generator\n(gaitGenerator.py)\n• Trot gait pattern\n• Phase scheduling',
               '#81D4FA', fontsize=7.5, fontweight='bold', edgecolor='#0277BD', lw=1.5)

    # Quintic Planner
    _fancy_box(ax, (28, 55), 24, 10, 'Quintic Polynomial\nTrajectory Planner\n(quinticPlanning.py)\n• Smooth S-curve',
               '#80DEEA', fontsize=7.5, fontweight='bold', edgecolor='#00838F', lw=1.5)

    # Inverse Kinematics
    _fancy_box(ax, (58, 55), 30, 23, '', C_IK, fontsize=8,
               edgecolor='#F9A825', lw=1.8)
    ax.text(73, 76, 'Inverse Kinematics', ha='center', va='center',
            fontsize=9.5, fontweight='bold', color='#F57F17', zorder=3)
    ax.text(73, 72.5, '(kinematics.py)', ha='center', va='center',
            fontsize=7.5, fontstyle='italic', color='#666', zorder=3)
    ax.text(73, 68, '• Body → Leg frame\n  transform\n• Geometric IK\n  (3-DOF per leg)\n• θ₁, θ₂, θ₃ output\n  (degrees)', 
            ha='center', va='center',
            fontsize=7, color='#333', zorder=3)

    # ── Low-level control blocks ─────────────────────────────────────────
    # SerialPublish / angle conversion
    _fancy_box(ax, (98, 68), 26, 10, 'Angle Conversion\n(serialPublish.py)\n• deg → rad\n• θ₃ + 180° offset',
               '#C8E6C9', fontsize=7.5, fontweight='bold', edgecolor='#2E7D32', lw=1.5)

    # PD Controller
    _fancy_box(ax, (128, 55), 26, 23, '', '#FFCDD2', fontsize=8,
               edgecolor='#C62828', lw=1.8)
    ax.text(141, 76, 'PD Controller', ha='center', va='center',
            fontsize=9.5, fontweight='bold', color='#C62828', zorder=3)
    ax.text(141, 72.5, '(controllerSim.py)', ha='center', va='center',
            fontsize=7.5, fontstyle='italic', color='#666', zorder=3)
    ax.text(141, 65, 'τ = Kp·e − Kd·ω\n\ne = θ_target − θ_actual\n\nKp = 20.0 Nm/rad\nKd = 0.5 Nm·s/rad', 
            ha='center', va='center',
            fontsize=7.5, color='#333', zorder=3, family='monospace')

    # ── Plant blocks ─────────────────────────────────────────────────────
    # Effort Controller (ros2_control)
    _fancy_box(ax, (100, 30), 30, 8, 'ros2_control\nJointGroupEffortController\n(12 joints × torque)',
               '#EF9A9A', fontsize=8, fontweight='bold', edgecolor='#C62828', lw=1.5)

    # Robot Model
    _fancy_box(ax, (35, 10), 40, 15, '', '#A5D6A7',
               fontsize=9, edgecolor='#2E7D32', lw=1.8)
    ax.text(55, 22, 'Quadruped Robot Model', ha='center', va='center',
            fontsize=9.5, fontweight='bold', color='#1B5E20', zorder=3)
    ax.text(55, 15.5, 'URDF: 4 legs × 3 joints = 12 DOF\n'
            'Links: coxa (20mm), femur (80mm), tibia (80mm)\n'
            'Sensors: joint_states (position + velocity)',
            ha='center', va='center', fontsize=7.5, color='#333', zorder=3)

    # Physics Engine
    _fancy_box(ax, (85, 10), 30, 15, 'Gazebo Physics\n(ODE Engine)\n\n• Gravity: 9.81 m/s²\n• Friction model\n• Contact dynamics',
               '#A5D6A7', fontsize=7.5, fontweight='bold', edgecolor='#2E7D32', lw=1.5)

    # Encoder output
    _fancy_box(ax, (10, 30), 25, 8, 'Joint State\nBroadcaster\n(Encoder)', '#A5D6A7',
               fontsize=8, fontweight='bold', edgecolor='#2E7D32', lw=1.5)

    # ── Arrows ──────────────────────────────────────────────────────────
    # Gait command → Gait Generator
    _arrow(ax, (24, 73), (28, 73), color=C_CMD, lw=2)

    # Gait Generator → Quintic Planner
    _arrow(ax, (40, 68), (40, 65), color=C_CMD, lw=2)
    _label_on_arrow(ax, (40, 68), (40, 65), 'Foot\nendpoints', offset=(10, 0), fontsize=7)

    # Quintic Planner → IK
    _arrow(ax, (52, 63), (58, 63), color=C_CMD, lw=2)
    _label_on_arrow(ax, (52, 63), (58, 63), 'Waypoints\n[x,y,z] (mm)', offset=(0, -6), fontsize=7)

    # Gait Generator → IK (D-shape path)
    _arrow(ax, (52, 73), (58, 73), color=C_CMD, lw=2)
    _label_on_arrow(ax, (52, 73), (58, 73), 'D-shape\npath', offset=(0, -5), fontsize=7)

    # IK → SerialPublish
    _arrow(ax, (88, 73), (98, 73), color=C_CMD, lw=2)
    _label_on_arrow(ax, (88, 73), (98, 73), 'θ₁,θ₂,θ₃\n(degrees)', offset=(0, -5), fontsize=7)

    # SerialPublish → PD Controller
    _arrow(ax, (124, 73), (128, 68), color=C_CMD, lw=2)
    _label_on_arrow(ax, (124, 73), (128, 68), 'Target\n(radians)', offset=(0, 5), fontsize=7)

    # PD Controller → Effort controller
    _arrow(ax, (141, 55), (125, 38), color=C_CMD, lw=2.5,
           connectionstyle='arc3,rad=0.1')
    _label_on_arrow(ax, (141, 55), (125, 38), 'τ (Nm)\n12 torques', offset=(5, 3), fontsize=7.5)

    # Effort controller → Robot model
    _arrow(ax, (100, 34), (75, 20), color=C_CMD, lw=2,
           connectionstyle='arc3,rad=0.1')

    # Robot model → Physics engine
    _arrow(ax, (75, 17), (85, 17), color='#546E7A', lw=1.5)

    # Robot → encoder (feedback)
    _arrow(ax, (35, 17), (22, 30), color=C_DATA, lw=2,
           connectionstyle='arc3,rad=-0.1')

    # Encoder → PD controller (feedback loop)
    _arrow(ax, (22, 38), (22, 50), color=C_DATA, lw=2)
    # Continue feedback up and right to PD controller
    ax.annotate('', xy=(128, 60), xytext=(22, 50),
                arrowprops=dict(arrowstyle='->', color=C_DATA, lw=2,
                               connectionstyle='arc3,rad=-0.2'),
                zorder=3)
    _label_on_arrow(ax, (22, 50), (128, 60), 'θ_actual, ω_actual\n(feedback)',
                    offset=(15, 5), fontsize=7.5, bg='#E8F5E9')

    # ── Reference setpoint annotation ───────────────────────────────────
    # Summing junction symbol
    circle = plt.Circle((96, 60), 2.5, fill=False, edgecolor=C_CMD,
                         linewidth=2, zorder=4)
    ax.add_patch(circle)
    ax.text(96, 60, 'Σ', ha='center', va='center', fontsize=10,
            fontweight='bold', color=C_CMD, zorder=5)

    # SerialPublish → Σ 
    _arrow(ax, (111, 68), (96, 62.5), color=C_CMD, lw=1.5)

    # Σ → PD Controller
    _arrow(ax, (98.5, 60), (128, 66), color=C_CMD, lw=1.5)
    _label_on_arrow(ax, (98.5, 60), (128, 66), 'error (e)', offset=(3, 4), fontsize=7)

    # Feedback into Σ
    # Already connected from encoder loop above

    # ── Legend ──────────────────────────────────────────────────────────
    legend_elements = [
        mpatches.Patch(facecolor=C_PLAN, edgecolor='#0277BD', label='Trajectory Planning'),
        mpatches.Patch(facecolor='#FFCCBC', edgecolor='#BF360C', label='Low-Level Control'),
        mpatches.Patch(facecolor='#A5D6A7', edgecolor='#2E7D32', label='Plant (Gazebo/Robot)'),
        plt.Line2D([0], [0], color=C_CMD, lw=2, label='Command flow (feedforward)'),
        plt.Line2D([0], [0], color=C_DATA, lw=2, label='Feedback flow (sensor data)'),
    ]
    ax.legend(handles=legend_elements, loc='lower right', fontsize=8,
              framealpha=0.9, edgecolor='#BDBDBD',
              bbox_to_anchor=(1.0, -0.02))

    fig.suptitle('Figure 10 -- Software Control Architecture\n'
                 '(Closed-Loop PD Effort Control with Trajectory Planning)',
                 fontsize=14, fontweight='bold', y=0.99)

    fig.tight_layout(rect=[0, 0.02, 1, 0.95])
    fig.savefig(os.path.join(FIGURE_DIR, 'fig10_control_architecture.png'),
                dpi=200, bbox_inches='tight')
    plt.close(fig)
    print('  [OK] Figure 10 saved')


# ═══════════════════════════════════════════════════════════════════════════
#  Flowchart drawing utilities
# ═══════════════════════════════════════════════════════════════════════════

# Colors for flowchart elements
FC_START   = '#A5D6A7'  # green - start/end terminals
FC_PROCESS = '#90CAF9'  # blue - process boxes
FC_DECIDE  = '#FFE082'  # amber - decision diamonds
FC_IO      = '#CE93D8'  # purple - I/O
FC_SUB     = '#80DEEA'  # cyan - subroutine
FC_ARROW   = '#37474F'  # dark gray


def _fc_terminal(ax, cx, cy, w, h, text, color=FC_START, fontsize=9):
    """Draw a rounded terminal (start/end) block."""
    box = FancyBboxPatch((cx - w/2, cy - h/2), w, h,
                         boxstyle="round,pad=0.3",
                         facecolor=color, edgecolor='#2E7D32',
                         linewidth=2, zorder=2)
    ax.add_patch(box)
    ax.text(cx, cy, text, ha='center', va='center',
            fontsize=fontsize, fontweight='bold', zorder=3)


def _fc_process(ax, cx, cy, w, h, text, color=FC_PROCESS, fontsize=8.5,
                edgecolor='#1565C0', lw=1.8):
    """Draw a rectangular process block."""
    box = FancyBboxPatch((cx - w/2, cy - h/2), w, h,
                         boxstyle="square,pad=0.15",
                         facecolor=color, edgecolor=edgecolor,
                         linewidth=lw, zorder=2)
    ax.add_patch(box)
    ax.text(cx, cy, text, ha='center', va='center',
            fontsize=fontsize, fontweight='normal', zorder=3)


def _fc_decision(ax, cx, cy, size, text, color=FC_DECIDE, fontsize=7.5):
    """Draw a diamond decision block."""
    s = size
    diamond = plt.Polygon([(cx, cy+s), (cx+s*1.3, cy), (cx, cy-s), (cx-s*1.3, cy)],
                          closed=True, facecolor=color, edgecolor='#F57F17',
                          linewidth=2, zorder=2)
    ax.add_patch(diamond)
    ax.text(cx, cy, text, ha='center', va='center',
            fontsize=fontsize, fontweight='bold', zorder=3)


def _fc_arrow(ax, x1, y1, x2, y2, label='', label_side='right',
              color=FC_ARROW, lw=1.8):
    """Draw a flowchart arrow with optional label."""
    arrow = FancyArrowPatch((x1, y1), (x2, y2),
                            arrowstyle=ArrowStyle('->', head_length=7, head_width=4),
                            color=color, linewidth=lw, zorder=1,
                            connectionstyle='arc3,rad=0')
    ax.add_patch(arrow)
    if label:
        mx, my = (x1+x2)/2, (y1+y2)/2
        offset = (6, 0) if label_side == 'right' else (-6, 0)
        if abs(y1 - y2) < 1:  # horizontal arrow
            offset = (0, 4)
        ax.text(mx + offset[0], my + offset[1], label, ha='center', va='center',
                fontsize=7, color='#333', fontweight='bold', zorder=4,
                bbox=dict(boxstyle='round,pad=0.1', facecolor='white',
                          edgecolor='#CCC', alpha=0.9, lw=0.5))


def _fc_arrow_bend(ax, x1, y1, x_mid, y_mid, x2, y2, label='',
                   color=FC_ARROW, lw=1.5):
    """Draw a bent arrow through a midpoint (right-angle connector)."""
    # Draw line segments
    ax.plot([x1, x_mid], [y1, y_mid], color=color, lw=lw, zorder=1,
            solid_capstyle='round')
    arrow = FancyArrowPatch((x_mid, y_mid), (x2, y2),
                            arrowstyle=ArrowStyle('->', head_length=7, head_width=4),
                            color=color, linewidth=lw, zorder=1)
    ax.add_patch(arrow)
    if label:
        ax.text(x_mid + 4, (y_mid + y2)/2, label, ha='left', va='center',
                fontsize=7, color='#333', zorder=4)


# ═══════════════════════════════════════════════════════════════════════════
#  FIGURE 11 — Main Control Loop Flowchart
# ═══════════════════════════════════════════════════════════════════════════

def fig11_main_control_flowchart():
    """Flowchart of the main control loop in nodeLegController.py."""
    fig, ax = plt.subplots(1, 1, figsize=(12, 16))
    ax.set_xlim(0, 120)
    ax.set_ylim(0, 175)
    ax.set_aspect('equal')
    ax.axis('off')

    # ── Column positions ──
    cx = 60   # center column
    bw = 36   # box width
    bh = 8    # box height
    ds = 7    # diamond size

    # ── START ──
    y = 170
    _fc_terminal(ax, cx, y, 30, 6, 'START', FC_START, fontsize=10)

    # ── Init ROS2 node ──
    y -= 12
    _fc_process(ax, cx, y, bw, bh,
                'Khởi tạo ROS2 Node\nnode_leg_controller\n(Timer 3ms callback)')
    _fc_arrow(ax, cx, 170-3, cx, y+bh/2)

    # ── Subscribe /gait_control ──
    y -= 12
    _fc_process(ax, cx, y, bw, bh,
                'Subscribe topic\n/gait_control (String)\nĐợi lệnh dáng đi', FC_IO)
    _fc_arrow(ax, cx, y+12-bh/2, cx, y+bh/2)

    # ── Timer callback entry ──
    y -= 12
    _fc_terminal(ax, cx, y, 34, 5, 'Timer Callback (3ms)', '#BBDEFB', fontsize=9)
    _fc_arrow(ax, cx, y+12-bh/2, cx, y+5/2)

    # ── Decision: có lệnh mới? ──
    y -= 12
    _fc_decision(ax, cx, y, ds, 'Có lệnh\nmới?')
    _fc_arrow(ax, cx, y+12-5/2, cx, y+ds)

    # YES branch: INIT state
    y_init = y
    y -= 14
    _fc_process(ax, cx, y, bw, bh,
                'state = "INIT"\nGọi gait.control_init()\n→ Tạo quỹ đạo cho 4 chân')
    _fc_arrow(ax, cx, y_init-ds, cx, y+bh/2, label='Có')

    # → Set state = READY
    y -= 11
    _fc_process(ax, cx, y, bw, 6,
                'state = "READY"')
    _fc_arrow(ax, cx, y+11-bh/2, cx, y+3)

    # ── Decision: state == READY? ──
    y -= 11
    y_ready = y
    _fc_decision(ax, cx, y, ds, 'state ==\n"READY"?')
    _fc_arrow(ax, cx, y+11-3, cx, y+ds)

    # NO from "Có lệnh mới?" → goes to state == READY check
    _fc_arrow_bend(ax, cx+ds*1.3, y_init, cx+ds*1.3+15, y_init,
                   cx+ds*1.3+15, y_ready, color='#888', lw=1.2)
    ax.text(cx+ds*1.3+3, y_init+3, 'Không', fontsize=7, color='#666',
            fontweight='bold')

    # ── YES: control_tick ──
    y -= 13
    _fc_decision(ax, cx, y, ds, 'cmd ==\n"ZERO"?')
    _fc_arrow(ax, cx, y_ready-ds, cx, y+ds, label='Có')

    # NO from READY: return (wait)
    ax.text(cx-ds*1.3-3, y_ready+3, 'Không', fontsize=7, color='#666',
            fontweight='bold', ha='right')
    _fc_process(ax, 18, y_ready, 18, 5, 'Return\n(đợi timer)', '#FFCDD2',
                fontsize=7.5, edgecolor='#C62828')
    _fc_arrow(ax, cx-ds*1.3, y_ready, 27, y_ready, color='#888', lw=1.2)

    # ── ZERO branch: _tick_homing ──
    y_cmd = y
    _fc_process(ax, 22, y-2, 28, 10,
                '_tick_homing()\n• Đọc homing targets\n• PD torque\n• Publish effort',
                '#C8E6C9', fontsize=7.5, edgecolor='#2E7D32')
    _fc_arrow(ax, cx-ds*1.3, y, 36, y-2, color=FC_ARROW, lw=1.5)
    ax.text(cx-ds*1.3-3, y+3, 'Có\n(Homing)', fontsize=6.5, color='#333',
            fontweight='bold', ha='right')

    # ── Gait branch: _tick_gait ──
    y -= 16
    _fc_process(ax, cx, y, bw, 12,
                '_tick_gait()\n• Đọc góc khớp θ[frame]\n• Gọi publish_message(θ)\n• frame += 1',
                '#BBDEFB', fontsize=8)
    _fc_arrow(ax, cx, y_cmd-ds, cx, y+6, label='Không\n(Gait)')

    # ── Decision: frame >= total? ──
    y -= 13
    _fc_decision(ax, cx, y, ds, 'frame >=\nT_total?')
    _fc_arrow(ax, cx, y+13-6, cx, y+ds)

    # YES: step completed
    y_cyc = y
    y -= 12
    _fc_process(ax, cx, y, bw, 7,
                'frame = 0\ncomplete_step += 1')
    _fc_arrow(ax, cx, y_cyc-ds, cx, y+3.5, label='Có')

    # ── Decision: step >= max_step? ──
    y -= 11
    _fc_decision(ax, cx, y, ds, 'step >=\nmax_step?')
    _fc_arrow(ax, cx, y+11-3.5, cx, y+ds)

    # YES: hold position
    _fc_process(ax, 22, y, 22, 6, 'Giữ vị trí\n(Hold pose)', '#FFCDD2',
                fontsize=7.5, edgecolor='#C62828')
    _fc_arrow(ax, cx-ds*1.3, y, 33, y, color=FC_ARROW, lw=1.5)
    ax.text(cx-ds*1.3-2, y+3, 'Có', fontsize=7, color='#333',
            fontweight='bold', ha='right')

    # NO: continue loop
    y_loop = y
    # Arrow going right, then up back to timer callback
    ax.text(cx+ds*1.3+3, y+3, 'Không', fontsize=7, color='#333',
            fontweight='bold')
    # Draw loop-back arrow
    loop_x = cx + ds*1.3 + 22
    ax.plot([cx+ds*1.3, loop_x], [y, y], color='#1565C0', lw=1.5, zorder=1)
    ax.plot([loop_x, loop_x], [y, 134], color='#1565C0', lw=1.5, zorder=1)
    loop_arrow = FancyArrowPatch((loop_x, 134), (cx+bw/2, 134),
                                 arrowstyle=ArrowStyle('->', head_length=7, head_width=4),
                                 color='#1565C0', linewidth=1.5, zorder=1)
    ax.add_patch(loop_arrow)
    ax.text(loop_x+2, (y+134)/2, 'Lặp lại\nchu kỳ', fontsize=7,
            color='#1565C0', fontweight='bold', ha='left', va='center')

    # NO from frame >= total: also loops back
    ax.text(cx+ds*1.3+3, y_cyc+3, 'Không', fontsize=7, color='#333',
            fontweight='bold')
    ax.plot([cx+ds*1.3, loop_x-8], [y_cyc, y_cyc], color='#1565C0', lw=1.2, zorder=1)
    ax.plot([loop_x-8, loop_x-8], [y_cyc, 134], color='#1565C0', lw=1.2, zorder=1)
    loop_arrow2 = FancyArrowPatch((loop_x-8, 134), (cx+bw/2+1, 134),
                                  arrowstyle='->', color='#1565C0',
                                  linewidth=1.2, zorder=1)
    ax.add_patch(loop_arrow2)

    # ── Inside _tick_gait: publish subroutine ──
    y_pub = y - 15
    _fc_process(ax, cx, y_pub, bw+4, 12,
                'publish_simulation(θ)\n① deg → rad cho 12 khớp\n'
                '② Wrap targets ±π\n③ PD: τ = Kp·e − Kd·ω\n④ Clamp |τ| ≤ τ_max',
                FC_SUB, fontsize=7.5, edgecolor='#00838F')
    _fc_arrow(ax, 22, y-6, 22, y_pub+6, color='#2E7D32', lw=1.5)
    _fc_arrow(ax, cx, y_loop-ds, cx, y_pub+6, color='#1565C0', lw=1.2)

    # ── Publish to Gazebo ──
    y_pub2 = y_pub - 12
    _fc_process(ax, cx, y_pub2, bw, 7,
                'Publish Float64MultiArray\n→ /leg_controller/commands',
                FC_IO, fontsize=8, edgecolor='#7B1FA2')
    _fc_arrow(ax, cx, y_pub-6, cx, y_pub2+3.5)

    # ── END (return to timer) ──
    y_end = y_pub2 - 10
    _fc_terminal(ax, cx, y_end, 28, 5, 'Return → Timer', '#FFCDD2', fontsize=9)
    _fc_arrow(ax, cx, y_pub2-3.5, cx, y_end+2.5)

    fig.suptitle('Hình 11 -- Lưu đồ giải thuật vòng điều khiển chính\n'
                 '(Main Control Loop — nodeLegController)',
                 fontsize=14, fontweight='bold', y=0.99)

    fig.tight_layout(rect=[0, 0.01, 1, 0.97])
    fig.savefig(os.path.join(FIGURE_DIR, 'fig11_main_control_flowchart.png'),
                dpi=200, bbox_inches='tight')
    plt.close(fig)
    print('  [OK] Figure 11 saved')


# ═══════════════════════════════════════════════════════════════════════════
#  FIGURE 12 — Gait Generation Algorithm Flowchart
# ═══════════════════════════════════════════════════════════════════════════

def fig12_gait_algorithm_flowchart():
    """Flowchart of the gait generation algorithm."""
    fig, ax = plt.subplots(1, 1, figsize=(14, 18))
    ax.set_xlim(0, 140)
    ax.set_ylim(0, 195)
    ax.set_aspect('equal')
    ax.axis('off')

    cx = 70
    bw = 42
    bh = 8
    ds = 7

    # ── START ──
    y = 190
    _fc_terminal(ax, cx, y, 30, 6, 'BẮT ĐẦU', FC_START, fontsize=10)

    # ── Nhận lệnh ──
    y -= 12
    _fc_process(ax, cx, y, bw, bh,
                'Nhận lệnh dáng đi\n(cmd, step) từ /gait_control',
                FC_IO, fontsize=9, edgecolor='#7B1FA2')
    _fc_arrow(ax, cx, 190-3, cx, y+bh/2)

    # ── Decision: loại lệnh ──
    y -= 13
    _fc_decision(ax, cx, y, ds+1, 'cmd =\n?')
    _fc_arrow(ax, cx, y+13-bh/2, cx, y+ds+1)

    # ── HOMING branch (left) ──
    _fc_process(ax, 20, y, 26, 10,
                'generate_home()\n• Tính home pose\n  cho 4 chân\n• Quỹ đạo homing',
                '#C8E6C9', fontsize=7.5, edgecolor='#2E7D32')
    _fc_arrow(ax, cx-(ds+1)*1.3, y, 33, y, color=FC_ARROW, lw=1.5)
    ax.text(cx-(ds+1)*1.3-2, y+5, 'ZERO\n(Homing)', fontsize=7, color='#333',
            fontweight='bold', ha='right')

    # ── GAIT branches (down) ──
    y_gait = y
    y -= 15
    _fc_process(ax, cx, y, bw+4, 9,
                'Chọn bộ tham số dáng đi\nFORWARD → PARAMS_GAIT_TROT\n'
                'TURN → PARAMS_GAIT_TURN',
                FC_PROCESS, fontsize=8)
    _fc_arrow(ax, cx, y_gait-(ds+1), cx, y+9/2, label='FORWARD\nTURN_RIGHT\n...')

    # ── For each leg ──
    y -= 13
    _fc_terminal(ax, cx, y, 38, 5, 'FOR leg ∈ {LF, LB, RF, RB}', '#BBDEFB', fontsize=9)
    _fc_arrow(ax, cx, y+13-9/2, cx, y+5/2)

    # ── Step 1: Define endpoints ──
    y -= 12
    _fc_process(ax, cx, y, bw+2, 10,
                'Bước 1: Xác định điểm đầu/cuối\n'
                '  x_center, y_val ← tham số chân\n'
                '  pos_A = [x_backward, y, z_stance]\n'
                '  pos_D = [x_forward, y, z_stance]',
                FC_PROCESS, fontsize=7.5)
    _fc_arrow(ax, cx, y+12-5/2, cx, y+5)

    # ── Decision: CONTROL_VELOCITY? ──
    y -= 13
    _fc_decision(ax, cx, y, ds, 'CONTROL\n_VELOCITY\n?')
    _fc_arrow(ax, cx, y+13-5, cx, y+ds)

    # ── YES: Quintic planning ──
    y_vel = y
    _fc_process(ax, 20, y-2, 30, 10,
                'Quintic Polynomial\nPlanning\ns(τ) = 10τ³−15τ⁴+6τ⁵\n'
                'T_swing=30, T_stance=310',
                '#80DEEA', fontsize=7.5, edgecolor='#00838F')
    _fc_arrow(ax, cx-ds*1.3, y, 35, y-2, color=FC_ARROW, lw=1.5)
    ax.text(cx-ds*1.3-2, y+4, 'TRUE', fontsize=7, color='#2E7D32',
            fontweight='bold', ha='right')

    # ── NO: Sine wave ──
    _fc_process(ax, 120, y-2, 28, 10,
                'Sine-wave\nSwing: z = h·sin(πτ)\n'
                'Stance: z = z_stance\n'
                'Nội suy tuyến tính',
                '#FFCCBC', fontsize=7.5, edgecolor='#BF360C')
    _fc_arrow(ax, cx+ds*1.3, y, 106, y-2, color=FC_ARROW, lw=1.5)
    ax.text(cx+ds*1.3+2, y+4, 'FALSE', fontsize=7, color='#C62828',
            fontweight='bold')

    # ── Merge: waypoints output ──
    y -= 16
    _fc_process(ax, cx, y, bw, 7,
                'waypoint[T_total, 3]\n= [x, y, z] cho mỗi waypoint',
                '#B3E5FC', fontsize=8)
    # Connect from both branches
    _fc_arrow(ax, 35, y_vel-2-5, cx-bw/2, y, color=FC_ARROW, lw=1.2)
    _fc_arrow(ax, 106, y_vel-2-5, cx+bw/2, y, color=FC_ARROW, lw=1.2)

    # ── Step 2: IK ──
    y -= 13
    _fc_process(ax, cx, y, bw+4, 12,
                'Bước 2: Inverse Kinematics\nFOR i = 0 → T_total:\n'
                '  θ₁,θ₂,θ₃ = IK(x[i], y[i], z[i])\n'
                '  • Body → Leg frame transform\n'
                '  • Geometric 3-DOF solution',
                C_IK, fontsize=7.5, edgecolor='#F9A825', lw=2)
    _fc_arrow(ax, cx, y+13-7/2, cx, y+6)

    # ── Step 3: Phase shift ──
    y -= 14
    _fc_process(ax, cx, y, bw+2, 10,
                'Bước 3: Dịch pha (Phase Shift)\n'
                'shift = T_total × phase_offset\n'
                'θ_shifted = np.roll(θ, shift)\n'
                '(Trot: LF=0, LB=0.5, RF=0.5, RB=0)',
                '#B39DDB', fontsize=7.5, edgecolor='#4527A0')
    _fc_arrow(ax, cx, y+14-6, cx, y+5)

    # ── END FOR ──
    y -= 11
    _fc_terminal(ax, cx, y, 34, 5, 'END FOR (4 chân)', '#BBDEFB', fontsize=9)
    _fc_arrow(ax, cx, y+11-5, cx, y+5/2)

    # ── Assemble ──
    y -= 12
    _fc_process(ax, cx, y, bw+4, 9,
                'Ghép dữ liệu 4 chân\ngait_angle_data[4][T_total, 3]\n'
                'Sẵn sàng cho control_tick()',
                FC_PROCESS, fontsize=8)
    _fc_arrow(ax, cx, y+12-5/2, cx, y+9/2)

    # ── Homing branch merge ──
    # Draw arrow from homing back to this point
    _fc_arrow_bend(ax, 20, y_gait-5, 8, y_gait-5, 8, y, color='#2E7D32', lw=1.2)
    ax.plot([8, cx-bw/2-2], [y, y], color='#2E7D32', lw=1.2, zorder=1)

    # ── Tick loop: publish ──
    y -= 13
    _fc_process(ax, cx, y, bw+4, 10,
                'control_tick() — Mỗi 3ms:\nĐọc θ[frame] cho 4 chân\n'
                'Chuyển đổi deg→rad, +offset\n'
                'PD torque → Publish effort',
                FC_SUB, fontsize=7.5, edgecolor='#00838F')
    _fc_arrow(ax, cx, y+13-9/2, cx, y+5)

    # ── Decision: hoàn thành? ──
    y -= 12
    _fc_decision(ax, cx, y, ds, 'Hoàn\nthành?')
    _fc_arrow(ax, cx, y+12-5, cx, y+ds)

    # NO: loop back
    ax.text(cx+ds*1.3+2, y+4, 'Chưa', fontsize=7, color='#333',
            fontweight='bold')
    loop_x = cx + ds*1.3 + 18
    ax.plot([cx+ds*1.3, loop_x], [y, y], color='#1565C0', lw=1.5, zorder=1)
    ax.plot([loop_x, loop_x], [y, y+12+5], color='#1565C0', lw=1.5, zorder=1)
    loop_a = FancyArrowPatch((loop_x, y+12+5), (cx+(bw+4)/2, y+12+5),
                             arrowstyle='->', color='#1565C0',
                             linewidth=1.5, zorder=1)
    ax.add_patch(loop_a)

    # YES: end
    y -= 12
    _fc_terminal(ax, cx, y, 30, 6, 'KẾT THÚC', '#FFCDD2', fontsize=10)
    _fc_arrow(ax, cx, y+12-ds, cx, y+3, label='Rồi')

    fig.suptitle('Hình 12 -- Lưu đồ giải thuật tạo dáng đi\n'
                 '(Gait Generation Algorithm)',
                 fontsize=14, fontweight='bold', y=0.995)

    fig.tight_layout(rect=[0, 0.01, 1, 0.97])
    fig.savefig(os.path.join(FIGURE_DIR, 'fig12_gait_algorithm_flowchart.png'),
                dpi=200, bbox_inches='tight')
    plt.close(fig)
    print('  [OK] Figure 12 saved')


# ═══════════════════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════════════════

if __name__ == '__main__':
    print('=' * 60)
    print('  QUADRUPED ROBOT -- ARCHITECTURE & FLOWCHART DIAGRAMS')
    print(f'  Output: {FIGURE_DIR}')
    print('=' * 60)

    fig9_system_architecture()
    fig10_control_architecture()
    fig11_main_control_flowchart()
    fig12_gait_algorithm_flowchart()

    print('=' * 60)
    print(f'  All diagrams saved to: {FIGURE_DIR}/')
    print('=' * 60)
