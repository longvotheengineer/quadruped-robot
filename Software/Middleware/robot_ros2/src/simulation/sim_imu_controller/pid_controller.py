"""
PID Controller
==============
Based on the 2022 paper: "Posture Stabilization Control for a Quadruped Robot
Walking on Swaying Platforms" (Li et al., IEEE CASE 2022).

Standard PID with anti-windup and output saturation.
Separate instances are used for roll and pitch channels.

Equation:
    u(t) = Kp * e(t) + Ki * ∫e(t)dt + Kd * de(t)/dt
"""

import numpy as np
from dataclasses import dataclass, field


@dataclass
class PIDGains:
    """PID gain set."""
    Kp: float = 0.0
    Ki: float = 0.0
    Kd: float = 0.0


class PIDController:
    """Discrete-time PID controller with anti-windup."""

    def __init__(self, gains: PIDGains, dt: float,
                 output_limit: float = np.inf,
                 integral_limit: float = np.inf,
                 name: str = "PID"):
        """
        Args:
            gains:          Kp, Ki, Kd
            dt:             time step (s)
            output_limit:   symmetric saturation on output (rad)
            integral_limit: anti-windup clamp on integral term
            name:           label for logging/plotting
        """
        self.gains = gains
        self.dt = dt
        self.output_limit = output_limit
        self.integral_limit = integral_limit
        self.name = name

        # Internal state
        self._integral = 0.0
        self._prev_error = 0.0
        self._first_step = True

        # ── Signal history for plotting ───────────────────────────
        self.history = {
            'time':       [],
            'error':      [],
            'p_term':     [],
            'i_term':     [],
            'd_term':     [],
            'output':     [],
            'setpoint':   [],
            'measurement':[],
        }

    def reset(self):
        """Reset internal state and history."""
        self._integral = 0.0
        self._prev_error = 0.0
        self._first_step = True
        for key in self.history:
            self.history[key].clear()

    def compute(self, setpoint: float, measurement: float, t: float) -> float:
        """
        Compute one PID step.

        Args:
            setpoint:    desired value (e.g., 0 rad for level body)
            measurement: current measured value (e.g., roll angle)
            t:           current simulation time (s)

        Returns:
            control output u(t)
        """
        error = setpoint - measurement

        # ── Proportional ──────────────────────────────────────────
        p_term = self.gains.Kp * error

        # ── Integral (trapezoidal rule + anti-windup) ─────────────
        self._integral += error * self.dt
        self._integral = np.clip(self._integral,
                                 -self.integral_limit,
                                  self.integral_limit)
        i_term = self.gains.Ki * self._integral

        # ── Derivative (backward difference) ──────────────────────
        if self._first_step:
            d_term = 0.0
            self._first_step = False
        else:
            d_term = self.gains.Kd * (error - self._prev_error) / self.dt

        self._prev_error = error

        # ── Total output with saturation ──────────────────────────
        output = p_term + i_term + d_term
        output = np.clip(output, -self.output_limit, self.output_limit)

        # ── Record history ────────────────────────────────────────
        self.history['time'].append(t)
        self.history['error'].append(error)
        self.history['p_term'].append(p_term)
        self.history['i_term'].append(i_term)
        self.history['d_term'].append(d_term)
        self.history['output'].append(output)
        self.history['setpoint'].append(setpoint)
        self.history['measurement'].append(measurement)

        return output

    def get_history_arrays(self) -> dict:
        """Return history as numpy arrays for convenient plotting."""
        return {k: np.array(v) for k, v in self.history.items()}
