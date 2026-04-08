#!/usr/bin/env python3
"""
Check if forward gait has any foot dragging (z above ground during swing/stance transition)
and analyze the actual velocity profiles in the stance phase.
"""
import math
import numpy as np

def quintic(pos_start, pos_end, T_swing=30, T_stance=150, lift_height=40):
    T_total = T_swing + T_stance
    waypoint = np.zeros((T_total, 3))
    z_ground = min(pos_start[2], pos_end[2])
    for i in range(T_swing):
        tau = i / (T_swing - 1)
        s = 10*tau**3 - 15*tau**4 + 6*tau**5
        x = pos_start[0] + s * (pos_end[0] - pos_start[0])
        y = pos_start[1] + s * (pos_end[1] - pos_start[1])
        z = z_ground + lift_height * 4 * s * (1 - s)
        waypoint[i, :] = [x, y, z]
    for i in range(T_stance):
        tau = i / (T_stance - 1)
        s = 10*tau**3 - 15*tau**4 + 6*tau**5
        x = pos_end[0] + s * (pos_start[0] - pos_end[0])
        y = pos_end[1] + s * (pos_start[1] - pos_end[1])
        z = z_ground
        waypoint[T_swing + i, :] = [x, y, z]
    return waypoint


# Forward gait: reverse=False
pos_A_fwd = [102.5, 135, -170]  # x_backward
pos_D_fwd = [147.5, 135, -170]  # x_forward
waypoints_fwd = quintic(pos_A_fwd, pos_D_fwd, T_swing=30, T_stance=150, lift_height=40)

# Backward gait: reverse=True
pos_A_bwd = [147.5, 135, -170]  # x_forward
pos_D_bwd = [102.5, 135, -170]  # x_backward
waypoints_bwd = quintic(pos_A_bwd, pos_D_bwd, T_swing=30, T_stance=150, lift_height=40)

# Phase shift for trot: LF at 0.0, so starts at waypoints[0]
# Check velocity profile
dt = 0.003  # timer period

print("=== FORWARD - Swing phase velocity (first 30 steps) ===")
for i in range(1, 30):
    dx = (waypoints_fwd[i, 0] - waypoints_fwd[i-1, 0]) / dt
    dz = (waypoints_fwd[i, 2] - waypoints_fwd[i-1, 2]) / dt
    if i < 5 or i > 25 or i % 5 == 0:
        print(f"  Step {i:3d}: x={waypoints_fwd[i,0]:7.2f} z={waypoints_fwd[i,2]:7.2f} dx/dt={dx:8.1f} dz/dt={dz:8.1f} mm/s")

print("\n=== FORWARD - Stance->Swing transition ===")
for i in range(28, 33):
    idx = i
    if idx >= 30:
        idx = idx  # enters stance
    print(f"  Step {idx:3d}: x={waypoints_fwd[idx,0]:7.2f} z={waypoints_fwd[idx,2]:7.2f}")

print("\n=== FORWARD - Stance phase velocity (steps 30-180) ===")
for i in range(31, 180):
    dx = (waypoints_fwd[i, 0] - waypoints_fwd[i-1, 0]) / dt
    if i < 35 or i > 175 or i % 20 == 0:
        print(f"  Step {i:3d}: x={waypoints_fwd[i,0]:7.2f} z={waypoints_fwd[i,2]:7.2f} dx/dt={dx:8.1f} mm/s")

print("\n=== Stance phase total displacement ===")
print(f"  Forward stance: x goes from {waypoints_fwd[30,0]:.1f} to {waypoints_fwd[179,0]:.1f} = {waypoints_fwd[179,0] - waypoints_fwd[30,0]:.1f} mm")
print(f"  Backward stance: x goes from {waypoints_bwd[30,0]:.1f} to {waypoints_bwd[179,0]:.1f} = {waypoints_bwd[179,0] - waypoints_bwd[30,0]:.1f} mm")

# Key: what's the effective forward push velocity (mm/s) at the ground?
stance_dur = 150 * 0.003
print(f"\n  Stance duration: {stance_dur:.3f}s")
print(f"  Average stance velocity: {45 / stance_dur:.1f} mm/s forward push")
print(f"  But quintic s-curve has peak velocity at midpoint")

# Calculate peak stance velocity
t_peak = 75  # midpoint of stance
tau = t_peak / (150 - 1)
dsdt = 30*tau**2 - 60*tau**3 + 30*tau**4
peak_vel = 45 * dsdt / dt  # mm/s
print(f"  Peak stance velocity: {peak_vel:.1f} mm/s")

# KEY ANALYSIS: During the trot, while LF+RB are in swing (lifting forward),
# LB+RF are in stance (pushing backward). But the SWING legs have NO ground contact.
# So during swing phase, the body is only supported by 2 diagonal legs.
# The forward momentum comes entirely from the stance legs.
# Both forward and backward gaits have the same support pattern.
print("\n=== TROT PHASE ANALYSIS ===")
total = 180
phases = {
    "left-front":   0.0,
    "right-behind": 0.0,
    "left-behind":  0.5,
    "right-front":  0.5,
}
print("Time step analysis (which legs are in swing vs stance):")
for step in [0, 15, 30, 60, 90, 120, 150, 179]:
    legs_state = {}
    for leg, phase in phases.items():
        shifted = (step + round(total * phase)) % total
        if shifted < 30:
            legs_state[leg] = "SWING"
        else:
            legs_state[leg] = "STANCE"
    n_stance = sum(1 for s in legs_state.values() if s == "STANCE")
    print(f"  Step {step:3d}: {legs_state}  ({n_stance} on ground)")
