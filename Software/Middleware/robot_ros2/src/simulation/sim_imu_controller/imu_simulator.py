"""
IMU Sensor Simulator
====================
Simulates an IMU (Inertial Measurement Unit) mounted at the robot's
center of gravity, providing roll and pitch measurements.

Based on the paper's approach: the IMU at the robot's CoG measures
body orientation (roll, pitch) without needing vision or force sensors.

Features:
- Gaussian noise model (configurable standard deviation)
- Optional low-pass filter (first-order)
- Configurable sample rate
"""

import numpy as np


class IMUSimulator:
    """Simulated IMU sensor producing roll and pitch readings."""

    def __init__(self, noise_std_roll: float = 0.001,
                       noise_std_pitch: float = 0.001,
                       lpf_cutoff: float = None,
                       dt: float = 0.001,
                       seed: int = 42):
        """
        Args:
            noise_std_roll:  Gaussian noise σ for roll (rad)
            noise_std_pitch: Gaussian noise σ for pitch (rad)
            lpf_cutoff:      low-pass filter cutoff frequency (Hz), None = disabled
            dt:              time step (s)
            seed:            random seed for reproducibility
        """
        self.noise_std_roll = noise_std_roll
        self.noise_std_pitch = noise_std_pitch
        self.dt = dt
        self.rng = np.random.default_rng(seed)

        # Low-pass filter
        self.lpf_enabled = lpf_cutoff is not None
        if self.lpf_enabled:
            alpha_denom = 1.0 / (2.0 * np.pi * lpf_cutoff * dt) + 1.0
            self.lpf_alpha = 1.0 / alpha_denom
        else:
            self.lpf_alpha = 1.0

        self._prev_roll = 0.0
        self._prev_pitch = 0.0

        # ── Signal history ────────────────────────────────────────
        self.history = {
            'time':          [],
            'true_roll':     [],
            'true_pitch':    [],
            'noisy_roll':    [],
            'noisy_pitch':   [],
            'filtered_roll': [],
            'filtered_pitch':[],
        }

    def reset(self):
        self._prev_roll = 0.0
        self._prev_pitch = 0.0
        for key in self.history:
            self.history[key].clear()

    def measure(self, true_roll: float, true_pitch: float, t: float):
        """
        Simulate an IMU measurement.

        Args:
            true_roll:  actual body roll angle (rad)
            true_pitch: actual body pitch angle (rad)
            t:          current time (s)

        Returns:
            (measured_roll, measured_pitch) in radians
        """
        # Add Gaussian noise
        noisy_roll  = true_roll  + self.rng.normal(0, self.noise_std_roll)
        noisy_pitch = true_pitch + self.rng.normal(0, self.noise_std_pitch)

        # Low-pass filter (first-order IIR)
        filtered_roll  = self.lpf_alpha * noisy_roll  + (1 - self.lpf_alpha) * self._prev_roll
        filtered_pitch = self.lpf_alpha * noisy_pitch + (1 - self.lpf_alpha) * self._prev_pitch
        self._prev_roll = filtered_roll
        self._prev_pitch = filtered_pitch

        # Record history
        self.history['time'].append(t)
        self.history['true_roll'].append(true_roll)
        self.history['true_pitch'].append(true_pitch)
        self.history['noisy_roll'].append(noisy_roll)
        self.history['noisy_pitch'].append(noisy_pitch)
        self.history['filtered_roll'].append(filtered_roll)
        self.history['filtered_pitch'].append(filtered_pitch)

        return filtered_roll, filtered_pitch

    def get_history_arrays(self) -> dict:
        return {k: np.array(v) for k, v in self.history.items()}
