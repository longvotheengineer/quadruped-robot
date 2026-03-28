"""
Posture Stabilization Simulation
=================================
Simulates the IMU-based PID posture stabilization controller for a
quadruped robot, with parameters matched to the working Gazebo simulation.

Control-theory pipeline:
    r=0 → ⊕ → [PID] → [Saturation] → [LPF] → [Dead Zone] → [Gain] → Plant
          ↑                                                              │
          └──────────────────── [IMU Sensor] ←───────────────────────────┘

Scenarios:
    1. No Control      — open-loop, ramp disturbance only
    2. HOME Balance    — θ₃ offset correction (standing)
    3. GAIT Balance    — rotation-formula correction (trotting)

Usage:
    cd src/simulation/sim_imu_controller
    python3 run_simulation.py
"""

import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robot_params import RobotParams
from pid_controller import PIDController, PIDGains
from imu_simulator import IMUSimulator
from body_posture import BodyPosture
from plant_model import PlantModel
from disturbance import Disturbance


# ══════════════════════════════════════════════════════════════════
#  Parameters (matched to Gazebo physics simulation)
# ══════════════════════════════════════════════════════════════════

T_TOTAL = 30.0       # s — simulation duration (long enough for steady-state)
DT      = 0.001      # s — time step (1 kHz, matches Gazebo)

# PID gains — nodeBalanceController.py
PID_GAINS = PIDGains(Kp=1.5, Ki=1.5, Kd=0.3)

# Saturation — nodeBalanceController.py
PID_SAT     = 0.5    # rad — symmetric output clamp
PID_WINDUP  = 0.8    # rad·s — integral anti-windup

# Low-pass filter — nodeBalanceController.py
LPF_ALPHA = 0.85     # IIR coefficient (0=no filter, 1=frozen)

# HOME mode — gaitGenerator.py
HOME_DEAD_ZONE = 0.005   # rad — noise rejection threshold
HOME_GAIN      = 1.8     # PID output × gain → θ₃ offset
HOME_SAT       = 0.5     # rad — max θ₃ offset per joint
HOME_STARTUP   = 3.0     # s — IMU settling delay

# GAIT mode — gaitGenerator.py
GAIT_DEAD_ZONE = 0.08    # rad (~4.6°) — ignore trot bounce
GAIT_GAIN      = 0.5     # attenuation: rotation formula is stronger

# Ramp disturbance (step input at t=3s)
RAMP_ROLL  = np.radians(5)
RAMP_PITCH = np.radians(8)


# ══════════════════════════════════════════════════════════════════
#  Simulation Loop
# ══════════════════════════════════════════════════════════════════

def run_scenario(name, mode="home", enable_control=True):
    """
    Run one scenario through the full control pipeline.

    Args:
        name:           scenario label
        mode:           "home" or "gait"
        enable_control: False for open-loop baseline
    """
    print(f"\n{'─'*60}")
    print(f"  {name}")
    print(f"  T={T_TOTAL}s | dt={DT}s | mode={mode.upper()}")
    print(f"{'─'*60}")

    params = RobotParams(dt=DT)
    pid_r = PIDController(PID_GAINS, DT, PID_SAT, PID_WINDUP, "Roll")
    pid_p = PIDController(PID_GAINS, DT, PID_SAT, PID_WINDUP, "Pitch")
    imu   = IMUSimulator(noise_std_roll=0.002, noise_std_pitch=0.002, dt=DT)
    plant = PlantModel(params)
    posture = BodyPosture(params)
    dist  = Disturbance(dist_type="step", step_time=3.0,
                        step_roll=RAMP_ROLL, step_pitch=RAMP_PITCH)

    # Low-pass filter state
    filt_r, filt_p = 0.0, 0.0

    # History
    hist = {'time': [], 'raw_r': [], 'raw_p': [],
            'filt_r': [], 'filt_p': [], 'corr_r': [], 'corr_p': []}

    n_steps = int(T_TOTAL / DT)
    for step in range(n_steps):
        t = step * DT

        # 1. Disturbance
        d_r, d_p = dist.get(t)

        # 2. IMU measurement
        m_r, m_p = imu.measure(plant.roll, plant.pitch, t)

        # 3. PID
        raw_r = pid_r.compute(0.0, m_r, t) if enable_control else 0.0
        raw_p = pid_p.compute(0.0, m_p, t) if enable_control else 0.0
        if not enable_control:
            pid_r.compute(0.0, m_r, t)
            pid_p.compute(0.0, m_p, t)

        # 4. Low-pass filter
        filt_r = LPF_ALPHA * filt_r + (1 - LPF_ALPHA) * raw_r
        filt_p = LPF_ALPHA * filt_p + (1 - LPF_ALPHA) * raw_p

        # 5. Dead zone + gain + saturation (mode-dependent)
        corr_r, corr_p = 0.0, 0.0
        if enable_control and t > HOME_STARTUP:
            if mode == "home":
                if abs(filt_r) > HOME_DEAD_ZONE or abs(filt_p) > HOME_DEAD_ZONE:
                    corr_r = np.clip(filt_r * HOME_GAIN, -HOME_SAT, HOME_SAT)
                    corr_p = np.clip(filt_p * HOME_GAIN, -HOME_SAT, HOME_SAT)
            elif mode == "gait":
                if abs(filt_r) > GAIT_DEAD_ZONE or abs(filt_p) > GAIT_DEAD_ZONE:
                    corr_r = filt_r * GAIT_GAIN
                    corr_p = filt_p * GAIT_GAIN

        # 6. Apply to plant
        posture.compute_foot_adjustments(corr_r, corr_p, t)
        plant.step(corr_r, corr_p, d_r, d_p, t)

        # Record
        hist['time'].append(t)
        hist['raw_r'].append(raw_r)
        hist['raw_p'].append(raw_p)
        hist['filt_r'].append(filt_r)
        hist['filt_p'].append(filt_p)
        hist['corr_r'].append(corr_r)
        hist['corr_p'].append(corr_p)

        if step % (n_steps // 5) == 0:
            print(f"  t={t:5.1f}s  roll={np.degrees(plant.roll):+6.2f}°  "
                  f"pitch={np.degrees(plant.pitch):+6.2f}°")

    print(f"  t={T_TOTAL:5.1f}s  DONE")

    return {
        'name': name, 'mode': mode,
        'pid_roll': pid_r.get_history_arrays(),
        'pid_pitch': pid_p.get_history_arrays(),
        'imu': imu.get_history_arrays(),
        'plant': plant.get_history_arrays(),
        'posture': posture.get_history_arrays(),
        'disturbance': dist.get_history_arrays(),
        'filter': {k: np.array(v) for k, v in hist.items()},
    }


# ══════════════════════════════════════════════════════════════════
#  Main
# ══════════════════════════════════════════════════════════════════

def main():
    print("╔══════════════════════════════════════════════════════════╗")
    print("║   Posture Stabilization Simulation                      ║")
    print("║   Parameters: Gazebo-matched | T=30s | 3 scenarios      ║")
    print("╚══════════════════════════════════════════════════════════╝")

    results = {}
    results['no_control'] = run_scenario("No Control (Open-Loop)",
                                          enable_control=False)
    results['home']       = run_scenario("HOME Balance (θ₃ Offsets)",
                                          mode="home")
    results['gait']       = run_scenario("GAIT Balance (Rotation Formula)",
                                          mode="gait")

    # Generate outputs
    print("\nGenerating outputs...")
    from visualize import generate_all
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")
    generate_all(results, out)

    print(f"\n✅  Done. Files in {out}/:")
    for f in sorted(os.listdir(out)):
        kb = os.path.getsize(os.path.join(out, f)) / 1024
        print(f"  {'📊' if f.endswith('.png') else '🎬'} {f}  ({kb:.0f} KB)")


if __name__ == "__main__":
    main()
