"""
Posture Stabilization Simulation
=================================
Simulates the IMU-based PID posture stabilization controller for a
quadruped robot, with parameters matched to the actual ROS 2 controller
code in nodeBalanceController.py and gaitGenerator.py.

Control-theory pipeline (HOME mode):
    r=0 → ⊕ → [EMA] → [Dead Zone] → [Gain Schedule] → [PID] → [Sat] → [LPF] → Apply
          ↑                                                                        │
          └────────────────────── [IMU Sensor] ←───────────────────────────────────┘

Control-theory pipeline (GAIT mode):
    IMU → [EMA] → [Moving Avg] → [Dead Zone] → [Adaptive PID] → [Sat] → [LPF] → Apply
                                                                                    │
                       [Baseline Learning] ←────────────────────────────────────────┘

Scenarios:
    1. No Control      — open-loop, step disturbance only
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
from pid_controller import PIDGains
from imu_simulator import IMUSimulator
from body_posture import BodyPosture
from plant_model import PlantModel
from disturbance import Disturbance


# ══════════════════════════════════════════════════════════════════
#  Parameters (matched to nodeBalanceController.py & gaitGenerator.py)
# ══════════════════════════════════════════════════════════════════

T_TOTAL = 30.0       # s — simulation duration (long enough for steady-state)
DT      = 0.001      # s — time step (1 kHz, matches Gazebo)

# ── Home PID — from PidConfig in nodeBalanceController.py ─────────
HOME_PID_GAINS = PIDGains(Kp=2.0, Ki=0.30, Kd=0.15)

# Saturation — PidConfig
HOME_PID_SAT     = 1.5    # rad — symmetric output clamp (SAT_CORR)
HOME_PID_WINDUP  = 1.5    # rad·s — integral anti-windup (SAT_INTEG)

# Gain scheduling — PidConfig
HOME_GAIN_SCALE_PID     = 0.08  # threshold for gain ramp
HOME_SAT_GAIN_SCALE_PID = 0.5   # minimum gain multiplier
HOME_RAMP_DERIV         = 0.005 # derivative ramp threshold

# Integral decay — PidConfig
HOME_DECAY_INTEG = 0.9995

# Filtering — from FilterConfig in nodeBalanceController.py
HOME_LPF_RAW   = 0.4    # EMA pre-filter on IMU measurement
HOME_LPF_DERIV = 0.7    # derivative low-pass filter
HOME_LPF_CORR  = 0.15   # output LPF (IIR coefficient)

# Deadzone — PidConfig
HOME_DEADZONE_MEAS = 0.02  # rad — noise rejection on measurement

# Home application — from ImuHomeConfig in gaitGenerator.py
HOME_APPLY_GAIN    = 7.0     # PID output × gain → θ₃ offset
HOME_APPLY_DEADZONE = 0.02   # rad — deadzone on applied correction
HOME_APPLY_SAT     = 0.5     # rad — max θ₃ offset per joint
HOME_APPLY_LPF     = 0.95   # offset smoothing
HOME_STARTUP       = 3.0     # s — IMU settling delay

# ── Gait PID — from GaitPidConfig in nodeBalanceController.py ─────
GAIT_PID_KP       = 3.0
GAIT_PID_KI       = 0.005
GAIT_PID_KD       = 1.2
GAIT_GAIN_PROP    = 10.0    # adaptive proportional gain denominator
GAIT_DEADZONE_AVG = 0.03    # rad — deadzone on moving average
GAIT_SAT_CORR     = 0.30    # rad — output saturation
GAIT_SAT_INTEG    = 0.3     # integral anti-windup
GAIT_LPF_CORR     = 0.97    # output LPF coefficient
GAIT_BASELINE_LEARN_CYCLES = 2
GAIT_BASELINE_ADAPT_RATE   = 0.05
GAIT_CYCLE_LEN    = 270     # stance(200) + swing(70)

# ── Disturbance ───────────────────────────────────────────────────
RAMP_ROLL  = np.radians(5)
RAMP_PITCH = np.radians(8)


# ══════════════════════════════════════════════════════════════════
#  Home PID Simulation (matches nodeBalanceController._run_home_pid)
# ══════════════════════════════════════════════════════════════════

class HomePIDSim:
    """Replicates the home PID pipeline from nodeBalanceController.py."""

    def __init__(self, dt):
        self.dt = dt
        # Per-axis state
        self.roll  = _HomeAxisState()
        self.pitch = _HomeAxisState()
        self.meas_initialized = False

    def filter_meas(self, roll_raw, pitch_raw):
        """EMA pre-filter on raw IMU measurement (FilterConfig.LPF_RAW)."""
        if not self.meas_initialized:
            self.roll.meas = roll_raw
            self.pitch.meas = pitch_raw
            self.meas_initialized = True
        else:
            a = HOME_LPF_RAW
            self.roll.meas  = a * self.roll.meas  + (1 - a) * roll_raw
            self.pitch.meas = a * self.pitch.meas + (1 - a) * pitch_raw

    def compute(self, dt):
        """Run one PID step for both axes."""
        for axis in [self.roll, self.pitch]:
            # Deadzone on measurement
            meas_dz = _deadzone_linear(axis.meas, HOME_DEADZONE_MEAS)
            error = 0.0 - meas_dz
            axis.error = error

            # Gain scheduling
            ratio = abs(error) / HOME_GAIN_SCALE_PID if HOME_GAIN_SCALE_PID > 0 else 1.0
            gs = max(HOME_SAT_GAIN_SCALE_PID, min(1.0, ratio))
            axis.gs = gs

            # Proportional
            axis.prop = HOME_PID_GAINS.Kp * gs * error

            # Integral with leakage
            axis.integ *= HOME_DECAY_INTEG
            axis.integ += error * dt
            axis.integ = np.clip(axis.integ, -HOME_PID_WINDUP, HOME_PID_WINDUP)

            # Derivative with LPF
            if dt > 0:
                deriv_raw = HOME_PID_GAINS.Kd * (error - axis.error_prev) / dt
            else:
                deriv_raw = 0.0
            axis.deriv = HOME_LPF_DERIV * axis.deriv + (1 - HOME_LPF_DERIV) * deriv_raw
            if abs(error) < HOME_RAMP_DERIV:
                axis.deriv = 0.0

            P = axis.prop
            I = HOME_PID_GAINS.Ki * gs * axis.integ
            D = axis.deriv
            axis.p_out = P
            axis.i_out = I
            axis.d_out = D
            axis.pid_sum = P + I + D
            axis.corr_sat = np.clip(axis.pid_sum, -HOME_PID_SAT, HOME_PID_SAT)

            # Back-calculation anti-windup
            if abs(axis.pid_sum) > HOME_PID_SAT and HOME_PID_GAINS.Ki > 0:
                excess = axis.pid_sum - axis.corr_sat
                axis.integ -= excess / HOME_PID_GAINS.Ki * 0.5
                axis.integ = np.clip(axis.integ, -HOME_PID_WINDUP, HOME_PID_WINDUP)

            # Output LPF
            axis.corr_lpf = (HOME_LPF_CORR * axis.corr_lpf
                             + (1 - HOME_LPF_CORR) * axis.corr_sat)

            axis.error_prev = error


class _HomeAxisState:
    def __init__(self):
        self.meas = 0.0
        self.error = 0.0
        self.gs = 0.5
        self.prop = 0.0
        self.integ = 0.0
        self.deriv = 0.0
        self.p_out = 0.0
        self.i_out = 0.0
        self.d_out = 0.0
        self.pid_sum = 0.0
        self.error_prev = 0.0
        self.corr_sat = 0.0
        self.corr_lpf = 0.0


# ══════════════════════════════════════════════════════════════════
#  Gait PID Simulation (matches nodeBalanceController._run_gait_pid)
# ══════════════════════════════════════════════════════════════════

class GaitPIDSim:
    """Replicates the gait PID pipeline from nodeBalanceController.py."""

    def __init__(self, cycle_len=GAIT_CYCLE_LEN):
        from collections import deque
        self.cycle_len = cycle_len
        self.roll_buf = deque(maxlen=cycle_len)
        self.pitch_buf = deque(maxlen=cycle_len)

        # Baseline
        self.roll_baseline = [0.0] * cycle_len
        self.pitch_baseline = [0.0] * cycle_len
        self.count_baseline = [0.0] * cycle_len
        self.baseline_cycles = 0
        self.baseline_ready = False

        # PID state
        self.roll_avg_prev = 0.0
        self.pitch_avg_prev = 0.0
        self.roll_integ = 0.0
        self.pitch_integ = 0.0

        # Output
        self.roll_corr_lpf = 0.0
        self.pitch_corr_lpf = 0.0

    def compute(self, roll_meas, pitch_meas, frame):
        """Run one gait PID step."""
        # Moving average
        self.roll_buf.append(roll_meas)
        self.pitch_buf.append(pitch_meas)
        roll_avg = sum(self.roll_buf) / len(self.roll_buf)
        pitch_avg = sum(self.pitch_buf) / len(self.pitch_buf)

        # Baseline update
        if 0 <= frame < self.cycle_len:
            if not self.baseline_ready:
                n = self.count_baseline[frame]
                self.roll_baseline[frame] = (
                    self.roll_baseline[frame] * n + roll_meas) / (n + 1)
                self.pitch_baseline[frame] = (
                    self.pitch_baseline[frame] * n + pitch_meas) / (n + 1)
                self.count_baseline[frame] += 1
            else:
                ar = GAIT_BASELINE_ADAPT_RATE
                self.roll_baseline[frame] += ar * (
                    roll_meas - self.roll_baseline[frame])
                self.pitch_baseline[frame] += ar * (
                    pitch_meas - self.pitch_baseline[frame])

        # Error with deadzone
        roll_error = _deadzone_linear(roll_avg, GAIT_DEADZONE_AVG)
        pitch_error = _deadzone_linear(pitch_avg, GAIT_DEADZONE_AVG)

        # Adaptive P term
        tilt_mag = max(abs(roll_error), abs(pitch_error))
        gain_scale = 1.0 + min(1.0, tilt_mag / GAIT_GAIN_PROP)
        kp_eff = GAIT_PID_KP * gain_scale

        roll_p = roll_error * kp_eff
        pitch_p = pitch_error * kp_eff

        # I term with anti-windup + zero-crossing reset
        self.roll_integ += roll_error
        self.pitch_integ += pitch_error
        self.roll_integ = np.clip(self.roll_integ, -GAIT_SAT_INTEG, GAIT_SAT_INTEG)
        self.pitch_integ = np.clip(self.pitch_integ, -GAIT_SAT_INTEG, GAIT_SAT_INTEG)
        if roll_error * self.roll_integ < 0:
            self.roll_integ = 0.0
        if pitch_error * self.pitch_integ < 0:
            self.pitch_integ = 0.0
        roll_i = self.roll_integ * GAIT_PID_KI
        pitch_i = self.pitch_integ * GAIT_PID_KI

        # D term
        roll_d = (roll_avg - self.roll_avg_prev) * GAIT_PID_KD
        pitch_d = (pitch_avg - self.pitch_avg_prev) * GAIT_PID_KD
        self.roll_avg_prev = roll_avg
        self.pitch_avg_prev = pitch_avg

        # PID sum (negative = oppose the tilt)
        roll_pid_sum = -(roll_p + roll_i + roll_d)
        pitch_pid_sum = -(pitch_p + pitch_i + pitch_d)

        # Saturate
        roll_corr = np.clip(roll_pid_sum, -GAIT_SAT_CORR, GAIT_SAT_CORR)
        pitch_corr = np.clip(pitch_pid_sum, -GAIT_SAT_CORR, GAIT_SAT_CORR)

        # LPF
        self.roll_corr_lpf = (GAIT_LPF_CORR * self.roll_corr_lpf
                              + (1 - GAIT_LPF_CORR) * roll_corr)
        self.pitch_corr_lpf = (GAIT_LPF_CORR * self.pitch_corr_lpf
                               + (1 - GAIT_LPF_CORR) * pitch_corr)

        return {
            'roll_avg': roll_avg, 'pitch_avg': pitch_avg,
            'roll_error': roll_error, 'pitch_error': pitch_error,
            'roll_p': roll_p, 'pitch_p': pitch_p,
            'roll_i': roll_i, 'pitch_i': pitch_i,
            'roll_d': roll_d, 'pitch_d': pitch_d,
            'gain_scale': gain_scale, 'kp_eff': kp_eff,
            'roll_pid_sum': roll_pid_sum, 'pitch_pid_sum': pitch_pid_sum,
            'roll_corr': roll_corr, 'pitch_corr': pitch_corr,
            'roll_corr_lpf': self.roll_corr_lpf,
            'pitch_corr_lpf': self.pitch_corr_lpf,
        }


def _deadzone_linear(value, deadzone):
    if abs(value) <= deadzone:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - deadzone)


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
    imu   = IMUSimulator(noise_std_roll=0.002, noise_std_pitch=0.002, dt=DT)
    plant = PlantModel(params)
    posture = BodyPosture(params)
    dist  = Disturbance(dist_type="step", step_time=3.0,
                        step_roll=RAMP_ROLL, step_pitch=RAMP_PITCH)

    # Mode-specific PID
    home_pid = HomePIDSim(DT) if mode == "home" else None
    gait_pid = GaitPIDSim() if mode == "gait" else None

    # Comprehensive history — every signal in the pipeline
    HIST_KEYS = [
        'time', 'meas_r', 'meas_p',
        'error_r', 'error_p',
        'gs_r', 'gs_p',
        'p_out_r', 'p_out_p',
        'i_out_r', 'i_out_p',
        'd_out_r', 'd_out_p',
        'pid_sum_r', 'pid_sum_p',
        'corr_sat_r', 'corr_sat_p',
        'corr_lpf_r', 'corr_lpf_p',
        'corr_r', 'corr_p',
    ]
    hist = {k: [] for k in HIST_KEYS}

    # Gait-specific history
    GAIT_KEYS = [
        'roll_avg', 'pitch_avg',
        'roll_error', 'pitch_error',
        'gain_scale', 'kp_eff',
        'roll_p', 'pitch_p',
        'roll_i', 'pitch_i',
        'roll_d', 'pitch_d',
        'roll_pid_sum', 'pitch_pid_sum',
        'roll_corr', 'pitch_corr',
        'roll_corr_lpf', 'pitch_corr_lpf',
    ]
    gait_hist = {k: [] for k in GAIT_KEYS}

    # Home application state
    home_apply_lpf = {'left-front': 0.0, 'left-behind': 0.0,
                      'right-front': 0.0, 'right-behind': 0.0}

    n_steps = int(T_TOTAL / DT)
    for step in range(n_steps):
        t = step * DT

        # 1. Disturbance
        d_r, d_p = dist.get(t)

        # 2. IMU measurement
        m_r, m_p = imu.measure(plant.roll, plant.pitch, t)

        # 3. Control pipeline
        corr_r, corr_p = 0.0, 0.0

        if mode == "home" and enable_control:
            # EMA filter
            home_pid.filter_meas(m_r, m_p)
            # PID compute
            home_pid.compute(DT)

            r, p = home_pid.roll, home_pid.pitch
            hist['meas_r'].append(r.meas)
            hist['meas_p'].append(p.meas)
            hist['error_r'].append(r.error)
            hist['error_p'].append(p.error)
            hist['gs_r'].append(r.gs)
            hist['gs_p'].append(p.gs)
            hist['p_out_r'].append(r.p_out)
            hist['p_out_p'].append(p.p_out)
            hist['i_out_r'].append(r.i_out)
            hist['i_out_p'].append(p.i_out)
            hist['d_out_r'].append(r.d_out)
            hist['d_out_p'].append(p.d_out)
            hist['pid_sum_r'].append(r.pid_sum)
            hist['pid_sum_p'].append(p.pid_sum)
            hist['corr_sat_r'].append(r.corr_sat)
            hist['corr_sat_p'].append(p.corr_sat)
            hist['corr_lpf_r'].append(r.corr_lpf)
            hist['corr_lpf_p'].append(p.corr_lpf)

            # Home application: deadzone → gain → sat → LPF
            if t > HOME_STARTUP:
                roll_out = r.corr_lpf
                pitch_out = p.corr_lpf
                roll_dz = _deadzone_linear(roll_out, HOME_APPLY_DEADZONE)
                pitch_dz = _deadzone_linear(pitch_out, HOME_APPLY_DEADZONE)
                in_deadzone = (roll_dz == 0.0 and pitch_dz == 0.0)

                roll_scaled = roll_dz * HOME_APPLY_GAIN
                pitch_scaled = pitch_dz * HOME_APPLY_GAIN
                sat = HOME_APPLY_SAT

                offsets = {
                    'left-front':  np.clip(-(roll_scaled - pitch_scaled), -sat, sat),
                    'left-behind': np.clip(-(roll_scaled + pitch_scaled), -sat, sat),
                    'right-front': np.clip(-(roll_scaled + pitch_scaled), -sat, sat),
                    'right-behind': np.clip(-(roll_scaled - pitch_scaled), -sat, sat),
                }

                if in_deadzone:
                    for leg in home_apply_lpf:
                        home_apply_lpf[leg] *= HOME_APPLY_LPF
                else:
                    alpha = HOME_APPLY_LPF
                    for leg in offsets:
                        home_apply_lpf[leg] = (alpha * home_apply_lpf[leg]
                                               + (1 - alpha) * offsets[leg])

                corr_r = r.corr_lpf
                corr_p = p.corr_lpf

            hist['corr_r'].append(corr_r)
            hist['corr_p'].append(corr_p)

        elif mode == "gait" and enable_control:
            # EMA pre-filter for gait mode
            if not hasattr(run_scenario, '_gait_meas_init'):
                run_scenario._gait_meas_r = m_r
                run_scenario._gait_meas_p = m_p
                run_scenario._gait_meas_init = True
            else:
                a = HOME_LPF_RAW
                run_scenario._gait_meas_r = a * run_scenario._gait_meas_r + (1 - a) * m_r
                run_scenario._gait_meas_p = a * run_scenario._gait_meas_p + (1 - a) * m_p
            meas_r = run_scenario._gait_meas_r
            meas_p = run_scenario._gait_meas_p

            # Simulate gait frame cycling
            frame = step % GAIT_CYCLE_LEN
            gait_out = gait_pid.compute(meas_r, meas_p, frame)

            # Check baseline cycle
            if frame == 0 and step > 0:
                gait_pid.baseline_cycles += 1
                if (gait_pid.baseline_cycles >= GAIT_BASELINE_LEARN_CYCLES
                        and not gait_pid.baseline_ready):
                    gait_pid.baseline_ready = True

            hist['meas_r'].append(meas_r)
            hist['meas_p'].append(meas_p)
            hist['error_r'].append(gait_out['roll_error'])
            hist['error_p'].append(gait_out['pitch_error'])
            hist['gs_r'].append(gait_out['gain_scale'])
            hist['gs_p'].append(gait_out['gain_scale'])
            hist['p_out_r'].append(gait_out['roll_p'])
            hist['p_out_p'].append(gait_out['pitch_p'])
            hist['i_out_r'].append(gait_out['roll_i'])
            hist['i_out_p'].append(gait_out['pitch_i'])
            hist['d_out_r'].append(gait_out['roll_d'])
            hist['d_out_p'].append(gait_out['pitch_d'])
            hist['pid_sum_r'].append(gait_out['roll_pid_sum'])
            hist['pid_sum_p'].append(gait_out['pitch_pid_sum'])
            hist['corr_sat_r'].append(gait_out['roll_corr'])
            hist['corr_sat_p'].append(gait_out['pitch_corr'])
            hist['corr_lpf_r'].append(gait_out['roll_corr_lpf'])
            hist['corr_lpf_p'].append(gait_out['pitch_corr_lpf'])

            corr_r = gait_out['roll_corr_lpf']
            corr_p = gait_out['pitch_corr_lpf']
            hist['corr_r'].append(corr_r)
            hist['corr_p'].append(corr_p)

            for k in gait_hist:
                gait_hist[k].append(gait_out[k])

        else:
            # No control — zero-fill all signals
            hist['meas_r'].append(m_r)
            hist['meas_p'].append(m_p)
            for k in HIST_KEYS:
                if k not in ('time', 'meas_r', 'meas_p'):
                    if k.startswith('gs'):
                        hist[k].append(1.0)
                    else:
                        hist[k].append(0.0)

        # 4. Apply to plant
        posture.compute_foot_adjustments(corr_r, corr_p, t)
        plant.step(corr_r, corr_p, d_r, d_p, t)

        # Record time
        hist['time'].append(t)

        if step % (n_steps // 5) == 0:
            print(f"  t={t:5.1f}s  roll={np.degrees(plant.roll):+6.2f}°  "
                  f"pitch={np.degrees(plant.pitch):+6.2f}°")

    print(f"  t={T_TOTAL:5.1f}s  DONE")

    # Clean up gait static state
    if hasattr(run_scenario, '_gait_meas_init'):
        del run_scenario._gait_meas_init
        del run_scenario._gait_meas_r
        del run_scenario._gait_meas_p

    return {
        'name': name, 'mode': mode,
        'imu': imu.get_history_arrays(),
        'plant': plant.get_history_arrays(),
        'posture': posture.get_history_arrays(),
        'disturbance': dist.get_history_arrays(),
        'filter': {k: np.array(v) for k, v in hist.items()},
        'gait': {k: np.array(v) for k, v in gait_hist.items()},
    }


# ══════════════════════════════════════════════════════════════════
#  Main
# ══════════════════════════════════════════════════════════════════

def main():
    print("╔══════════════════════════════════════════════════════════╗")
    print("║   Posture Stabilization Simulation                      ║")
    print("║   Parameters: Controller-matched | T=30s | 3 scenarios  ║")
    print("╚══════════════════════════════════════════════════════════╝")

    results = {}
    results['no_control'] = run_scenario("No Control (Open-Loop)",
                                          enable_control=False)
    results['home']       = run_scenario("HOME Balance (θ₃ Offsets, K=7.0)",
                                          mode="home")
    results['gait']       = run_scenario("GAIT Balance (Adaptive PID)",
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
