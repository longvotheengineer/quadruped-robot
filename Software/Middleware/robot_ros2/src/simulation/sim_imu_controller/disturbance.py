"""
Platform Disturbance Generator
==============================
Generates platform sway disturbances (roll, pitch) as described in the paper.
The paper tests with sinusoidal swaying platforms.

Disturbance types:
1. Sinusoidal (the paper's primary test case)
2. Step (for transient/step response analysis)
3. Combined (sin + step for robustness testing)
"""

import numpy as np


class Disturbance:
    """Platform sway disturbance generator."""

    def __init__(self, dist_type: str = "sinusoidal",
                 roll_amplitude:  float = 0.0873,   # 5 degrees in rad
                 pitch_amplitude: float = 0.0524,   # 3 degrees in rad
                 roll_frequency:  float = 0.5,      # Hz
                 pitch_frequency: float = 0.3,      # Hz
                 roll_phase:      float = 0.0,      # rad
                 pitch_phase:     float = np.pi/4,  # rad
                 step_time:       float = 2.0,      # s – when step kicks in
                 step_roll:       float = 0.0524,   # 3 deg step in roll
                 step_pitch:      float = 0.0349):  # 2 deg step in pitch
        """
        Args:
            dist_type:        "sinusoidal", "step", or "combined"
            roll_amplitude:   sinusoidal roll amplitude (rad)
            pitch_amplitude:  sinusoidal pitch amplitude (rad)
            roll_frequency:   sinusoidal roll frequency (Hz)
            pitch_frequency:  sinusoidal pitch frequency (Hz)
            roll_phase:       sinusoidal roll phase offset (rad)
            pitch_phase:      sinusoidal pitch phase offset (rad)
            step_time:        time at which step disturbance occurs (s)
            step_roll:        step magnitude in roll (rad)
            step_pitch:       step magnitude in pitch (rad)
        """
        self.dist_type = dist_type
        self.roll_amplitude = roll_amplitude
        self.pitch_amplitude = pitch_amplitude
        self.roll_freq = roll_frequency
        self.pitch_freq = pitch_frequency
        self.roll_phase = roll_phase
        self.pitch_phase = pitch_phase
        self.step_time = step_time
        self.step_roll = step_roll
        self.step_pitch = step_pitch

        # History
        self.history = {'time': [], 'roll': [], 'pitch': []}

    def reset(self):
        for key in self.history:
            self.history[key].clear()

    def get(self, t: float):
        """
        Get disturbance at time t.

        Returns:
            (roll_disturbance, pitch_disturbance) in radians
        """
        roll_d = 0.0
        pitch_d = 0.0

        if self.dist_type in ("sinusoidal", "combined"):
            roll_d  += self.roll_amplitude  * np.sin(2*np.pi*self.roll_freq*t  + self.roll_phase)
            pitch_d += self.pitch_amplitude * np.sin(2*np.pi*self.pitch_freq*t + self.pitch_phase)

        if self.dist_type in ("step", "combined"):
            if t >= self.step_time:
                roll_d  += self.step_roll
                pitch_d += self.step_pitch

        self.history['time'].append(t)
        self.history['roll'].append(roll_d)
        self.history['pitch'].append(pitch_d)

        return roll_d, pitch_d

    def get_history_arrays(self) -> dict:
        return {k: np.array(v) for k, v in self.history.items()}
