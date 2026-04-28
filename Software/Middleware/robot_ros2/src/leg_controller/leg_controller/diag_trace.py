#!/usr/bin/env python3
"""Diagnostic: full gait cycle trace for RF leg.
Shows servo_2 value at EVERY frame (swing + stance) to find the max.
"""
import math, sys, os, numpy as np
sys.path.insert(0, os.path.dirname(__file__))

# ─── Minimal Kinematics ──────────────────────────────────────────────
class RL:
    L=250; W=193; l1=45; l2=107; l3=116

def inverse(x, y, z, leg_type):
    px, py, pz = x, y, z
    match leg_type:
        case "left-front":
            x, y, z = pz, RL.W/2-py, -RL.L/2+px
            theta1 = math.atan2(-x,y)+math.atan2(-math.sqrt(x**2+y**2-RL.l1**2),-RL.l1)
            sign_s3, sign_p2 = -1, 1
        case "left-behind":
            x, y, z = pz, RL.W/2-py, RL.L/2+px
            theta1 = math.atan2(-x,y)+math.atan2(-math.sqrt(x**2+y**2-RL.l1**2),-RL.l1)
            sign_s3, sign_p2 = -1, 1
        case "right-front":
            x, y, z = pz, -RL.W/2-py, -RL.L/2+px
            theta1 = math.atan2(-x,y)+math.atan2(-math.sqrt(x**2+y**2-RL.l1**2),RL.l1)
            sign_s3, sign_p2 = 1, -1
        case "right-behind":
            x, y, z = pz, -RL.W/2-py, RL.L/2+px
            theta1 = math.atan2(-x,y)+math.atan2(-math.sqrt(x**2+y**2-RL.l1**2),RL.l1)
            sign_s3, sign_p2 = 1, -1
    p1 = x*math.cos(theta1)+y*math.sin(theta1)
    p2 = sign_p2*z
    c3 = (p1**2+p2**2-RL.l2**2-RL.l3**2)/(2*RL.l2*RL.l3)
    s3 = sign_s3*math.sqrt(max(0, 1-c3**2))
    theta3 = math.atan2(s3, c3)
    theta2 = math.atan2(p2, p1)-math.atan2(RL.l3*s3, RL.l2+RL.l3*c3)
    theta1 = round(math.degrees(theta1),1)
    theta2 = round(math.degrees(theta2),1)
    if leg_type in ("left-front","right-behind"):
        if theta2 < 0: theta2 += 360
    elif leg_type in ("left-behind","right-front"):
        if theta2 > 0: theta2 -= 360
    theta3 = round(math.degrees(theta3),1)
    return theta1, theta2, theta3

def ik_to_servo_right(t1, t2, t3):
    if t2 < -180: t2 += 360
    return t1, t2, t3-90.0

TICKS = 4096.0/360.0

def trajectory_moving(x_center, y_val, reverse=False):
    stride=45; x_f=x_center+stride/2; x_b=x_center-stride/2
    z_stance=-170; z_swing=-130; lift=z_swing-z_stance
    T_swing=70; T_stance=200
    if reverse: pos_A,pos_D = [x_f,y_val,z_stance],[x_b,y_val,z_stance]
    else: pos_A,pos_D = [x_b,y_val,z_stance],[x_f,y_val,z_stance]
    # Simple trajectory (matches non-quintic path)
    swing = np.zeros((T_swing, 3))
    swing[:,0] = np.linspace(pos_A[0], pos_D[0], T_swing)
    swing[:,1] = y_val
    swing[:,2] = z_stance + lift*np.sin(np.linspace(0, np.pi, T_swing))
    stance = np.zeros((T_stance, 3))
    stance[:,0] = np.linspace(pos_D[0], pos_A[0], T_stance)
    stance[:,1] = y_val
    stance[:,2] = z_stance
    return np.vstack([swing, stance])

def main():
    print("="*80)
    print("FULL GAIT CYCLE TRACE — RF vs RB servo_2 comparison")
    print("="*80)

    for leg, x_center in [("right-front", 125), ("right-behind", -125)]:
        waypoints = trajectory_moving(x_center, -135, reverse=False)
        N = waypoints.shape[0]
        s2_vals = []
        tick_vals = []
        for i in range(N):
            t1, t2, t3 = inverse(*waypoints[i], leg)
            s1, s2, s3 = ik_to_servo_right(t1, t2, t3)
            tick2 = int(s2 * TICKS + 2048)
            s2_vals.append(s2)
            tick_vals.append(tick2)

        phase = "swing" if True else "stance"
        print(f"\n── {leg.upper()} ──")
        print(f"  Total frames: {N} (swing=70, stance=200)")
        print(f"  servo_2 range: {min(s2_vals):.1f}° to {max(s2_vals):.1f}°")
        print(f"  tick_2  range: {min(tick_vals)} to {max(tick_vals)}")

        emin, emax = (2048, 3754) if leg == "right-front" else (2048, 3764)
        exceeds = [i for i,t in enumerate(tick_vals) if t < emin or t > emax]
        if exceeds:
            print(f"  ✗ {len(exceeds)} frames EXCEED EEPROM limits [{emin}, {emax}]!")
            for idx in exceeds[:5]:
                wp = waypoints[idx]
                print(f"    frame {idx}: foot=({wp[0]:.1f}, {wp[1]:.1f}, {wp[2]:.1f}) "
                      f"→ servo_2={s2_vals[idx]:.1f}° → tick={tick_vals[idx]}")
        else:
            print(f"  ✓ All frames within EEPROM limits [{emin}, {emax}]")

        # Show key frames
        print(f"\n  Key frames (servo_2 / tick):")
        print(f"  {'Frame':>6}  {'Phase':>8}  {'foot_x':>7}  {'foot_z':>7}  {'servo_2':>8}  {'tick':>6}")
        for i in [0, 17, 35, 52, 69, 70, 135, 200, 269]:
            if i >= N: continue
            wp = waypoints[i]
            ph = "swing" if i < 70 else "stance"
            print(f"  {i:6d}  {ph:>8}  {wp[0]:7.1f}  {wp[2]:7.1f}  {s2_vals[i]:7.1f}°  {tick_vals[i]:6d}")

    # Also check with phase shift applied (RF has phase=0.50)
    print("\n\n── PHASE SHIFT EFFECT ─────────────────────────────────────────")
    waypoints = trajectory_moving(125, -135, reverse=False)
    N = waypoints.shape[0]
    shift = round(N * 0.50)
    print(f"  RF phase shift: 0.50 → roll by {shift} frames")
    print(f"  Frame 0 after shift = original frame {shift}")
    print(f"  This means RF STARTS in the middle of the cycle")

    shifted_wp = np.roll(waypoints, shift, axis=0)
    s2_shifted = []
    for i in range(N):
        t1, t2, t3 = inverse(*shifted_wp[i], "right-front")
        s1, s2, s3 = ik_to_servo_right(t1, t2, t3)
        s2_shifted.append(s2)

    print(f"  First 5 frames after phase shift: servo_2 = {[f'{v:.1f}°' for v in s2_shifted[:5]]}")
    print(f"  Standing position servo_2: 137.0°")
    jump = abs(s2_shifted[0] - 137.0)
    print(f"  Jump from homing to first trot frame: {jump:.1f}°")

if __name__ == "__main__":
    main()
