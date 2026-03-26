"""
Main Simulation Runner
======================
Ties all blocks together into a time-stepping simulation loop.

Runs multiple scenarios for comparison:
1. No control (open-loop, disturbance only)
2. P-only controller
3. PD controller
4. Full PID controller

Saves all signal data and calls the visualization module to produce plots.

Usage:
    cd src/simulation/sim_imu_controller
    python3 run_simulation.py
"""

import os
import sys
import copy
import numpy as np

# Ensure we can import from this directory
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robot_params import RobotParams
from pid_controller import PIDController, PIDGains
from imu_simulator import IMUSimulator
from body_posture import BodyPosture
from plant_model import PlantModel
from disturbance import Disturbance


# ══════════════════════════════════════════════════════════════════
#  Simulation Configuration
# ══════════════════════════════════════════════════════════════════

# Simulation time
T_TOTAL   = 10.0    # seconds
DT        = 0.001   # time step (s), 1 kHz like Gazebo

# PID gains (tuned for this robot)
# These are typical gains for a small quadruped (~1.5 kg)
ROLL_GAINS_PID  = PIDGains(Kp=8.0, Ki=2.0, Kd=0.3)
PITCH_GAINS_PID = PIDGains(Kp=8.0, Ki=2.0, Kd=0.3)

# PD gains (no integral)
ROLL_GAINS_PD   = PIDGains(Kp=8.0, Ki=0.0, Kd=0.3)
PITCH_GAINS_PD  = PIDGains(Kp=8.0, Ki=0.0, Kd=0.3)

# P-only gains
ROLL_GAINS_P    = PIDGains(Kp=8.0, Ki=0.0, Kd=0.0)
PITCH_GAINS_P   = PIDGains(Kp=8.0, Ki=0.0, Kd=0.0)

# PID output limits (max correction angle in rad, ~15°)
PID_OUTPUT_LIMIT   = 0.2618
PID_INTEGRAL_LIMIT = 0.5

# IMU noise
IMU_NOISE_STD = 0.002  # rad (~0.1°)

# Disturbance: sinusoidal platform sway
DIST_ROLL_AMP   = np.radians(5)    # 5° amplitude
DIST_PITCH_AMP  = np.radians(3)    # 3° amplitude
DIST_ROLL_FREQ  = 0.5              # Hz
DIST_PITCH_FREQ = 0.3              # Hz


# ══════════════════════════════════════════════════════════════════
#  Single Scenario Simulation
# ══════════════════════════════════════════════════════════════════

def run_scenario(name: str, roll_gains: PIDGains, pitch_gains: PIDGains,
                 enable_control: bool = True,
                 dist_type: str = "sinusoidal") -> dict:
    """
    Run one simulation scenario.

    Args:
        name:           scenario label (e.g., "PID", "No Control")
        roll_gains:     PID gains for roll channel
        pitch_gains:    PID gains for pitch channel
        enable_control: if False, no correction is applied (open-loop)
        dist_type:      disturbance type ("sinusoidal", "step", "combined")

    Returns:
        dict with all component histories
    """
    print(f"\n{'='*60}")
    print(f"  Running scenario: {name}")
    print(f"  Duration: {T_TOTAL}s | dt: {DT}s | Steps: {int(T_TOTAL/DT)}")
    if enable_control:
        print(f"  Roll  PID: Kp={roll_gains.Kp}, Ki={roll_gains.Ki}, Kd={roll_gains.Kd}")
        print(f"  Pitch PID: Kp={pitch_gains.Kp}, Ki={pitch_gains.Ki}, Kd={pitch_gains.Kd}")
    else:
        print(f"  Control: DISABLED (open-loop)")
    print(f"{'='*60}")

    params = RobotParams(dt=DT)

    # Create components
    pid_roll = PIDController(roll_gains, DT, PID_OUTPUT_LIMIT, PID_INTEGRAL_LIMIT, "Roll PID")
    pid_pitch = PIDController(pitch_gains, DT, PID_OUTPUT_LIMIT, PID_INTEGRAL_LIMIT, "Pitch PID")
    imu = IMUSimulator(noise_std_roll=IMU_NOISE_STD, noise_std_pitch=IMU_NOISE_STD, dt=DT)
    posture = BodyPosture(params)
    plant = PlantModel(params)
    dist = Disturbance(dist_type=dist_type,
                       roll_amplitude=DIST_ROLL_AMP,
                       pitch_amplitude=DIST_PITCH_AMP,
                       roll_frequency=DIST_ROLL_FREQ,
                       pitch_frequency=DIST_PITCH_FREQ,
                       step_time=3.0,
                       step_roll=np.radians(3),
                       step_pitch=np.radians(2))

    # ── Time loop ─────────────────────────────────────────────────
    n_steps = int(T_TOTAL / DT)
    for step in range(n_steps):
        t = step * DT

        # 1. Get platform disturbance
        dist_roll, dist_pitch = dist.get(t)

        # 2. Read IMU (measures actual body orientation)
        meas_roll, meas_pitch = imu.measure(plant.roll, plant.pitch, t)

        # 3. PID compute
        if enable_control:
            # Setpoint = 0 (we want the body level)
            corr_roll  = pid_roll.compute(0.0, meas_roll, t)
            corr_pitch = pid_pitch.compute(0.0, meas_pitch, t)
        else:
            corr_roll = 0.0
            corr_pitch = 0.0
            # Still record for consistent history
            pid_roll.compute(0.0, meas_roll, t)
            pid_pitch.compute(0.0, meas_pitch, t)

        # 4. Body posture adjustment (compute foot adjustments)
        posture.compute_foot_adjustments(corr_roll, corr_pitch, t)

        # 5. Plant dynamics step
        plant.step(corr_roll, corr_pitch, dist_roll, dist_pitch, t)

        # Progress reporting
        if step % (n_steps // 10) == 0:
            pct = 100 * step / n_steps
            print(f"  [{pct:5.1f}%]  t={t:.2f}s  roll={np.degrees(plant.roll):+.3f}°  "
                  f"pitch={np.degrees(plant.pitch):+.3f}°")

    print(f"  [100.0%]  Simulation complete.")

    # ── Collect results ───────────────────────────────────────────
    return {
        'name':       name,
        'pid_roll':   pid_roll.get_history_arrays(),
        'pid_pitch':  pid_pitch.get_history_arrays(),
        'imu':        imu.get_history_arrays(),
        'posture':    posture.get_history_arrays(),
        'plant':      plant.get_history_arrays(),
        'disturbance': dist.get_history_arrays(),
        'params':     params,
        'gains_roll': roll_gains,
        'gains_pitch': pitch_gains,
    }


# ══════════════════════════════════════════════════════════════════
#  Main entry point
# ══════════════════════════════════════════════════════════════════

def main():
    print("╔══════════════════════════════════════════════════════════╗")
    print("║   IMU PID Posture Stabilization Simulation              ║")
    print("║   Based on: Li et al. (IEEE CASE 2022)                  ║")
    print("║   Robot: Quadruped, 12 DOF, position-feedback servos    ║")
    print("╚══════════════════════════════════════════════════════════╝")

    results = {}

    # ── Scenario 1: No control ────────────────────────────────────
    results['no_control'] = run_scenario(
        "No Control (Open-Loop)",
        PIDGains(0, 0, 0), PIDGains(0, 0, 0),
        enable_control=False)

    # ── Scenario 2: P-only ────────────────────────────────────────
    results['p_only'] = run_scenario(
        "P-Only Controller",
        ROLL_GAINS_P, PITCH_GAINS_P)

    # ── Scenario 3: PD controller ─────────────────────────────────
    results['pd'] = run_scenario(
        "PD Controller",
        ROLL_GAINS_PD, PITCH_GAINS_PD)

    # ── Scenario 4: Full PID ──────────────────────────────────────
    results['pid'] = run_scenario(
        "Full PID Controller",
        ROLL_GAINS_PID, PITCH_GAINS_PID)

    # ── Scenario 5: PID with step disturbance ─────────────────────
    results['pid_step'] = run_scenario(
        "PID – Step Disturbance",
        ROLL_GAINS_PID, PITCH_GAINS_PID,
        dist_type="step")

    # ── Scenario 6: PID with combined disturbance ─────────────────
    results['pid_combined'] = run_scenario(
        "PID – Combined Disturbance",
        ROLL_GAINS_PID, PITCH_GAINS_PID,
        dist_type="combined")

    # ── Generate visualizations ───────────────────────────────────
    print("\n\nGenerating visualizations...")
    from visualize import generate_all_plots
    output_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")
    generate_all_plots(results, output_dir)

    print(f"\n✅  All done! Check the results in: {output_dir}/")
    print("Generated files:")
    for f in sorted(os.listdir(output_dir)):
        fpath = os.path.join(output_dir, f)
        size_kb = os.path.getsize(fpath) / 1024
        print(f"  📊 {f}  ({size_kb:.1f} KB)")


if __name__ == "__main__":
    main()
