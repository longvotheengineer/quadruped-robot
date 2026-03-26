"""
Plant Model (Simplified Rigid-Body Dynamics)
=============================================
Models the quadruped robot body as a rigid body supported by 4 legs.
The platform sway acts as an external disturbance that tilts the body.

The plant model includes:
1. Rigid-body rotational dynamics (Newton-Euler for roll/pitch)
2. Servo motor dynamics (first-order lag representing servo bandwidth)
3. Joint PD controller (your existing Kp=20, Kd=0.5)
4. Gravity restoring torque
5. Platform disturbance coupling

Simplifications (appropriate for this level of simulation):
- No leg swing dynamics (all feet assumed in stance / ground contact)
- Decoupled roll and pitch channels (valid for small angles)
- Servo modeled as first-order system (bandwidth from URDF specs)
"""

import numpy as np
from robot_params import RobotParams, get_total_mass


class PlantModel:
    """Simplified rigid-body plant for body posture simulation."""

    def __init__(self, params: RobotParams):
        self.p = params
        self.total_mass = get_total_mass(params)

        # ── State variables ───────────────────────────────────────
        # Body orientation
        self.roll  = 0.0    # φ (rad)
        self.pitch = 0.0    # θ (rad)
        self.roll_dot  = 0.0  # dφ/dt (rad/s)
        self.pitch_dot = 0.0  # dθ/dt (rad/s)

        # Servo state (actual joint positions track commanded positions
        # through first-order dynamics)
        # For simplicity, we track an effective "body correction" through
        # the servo lag rather than all 12 individual servos
        self.servo_roll_actual  = 0.0
        self.servo_pitch_actual = 0.0

        # ── History ───────────────────────────────────────────────
        self.history = {
            'time':             [],
            'roll':             [],
            'pitch':            [],
            'roll_dot':         [],
            'pitch_dot':        [],
            'roll_torque':      [],
            'pitch_torque':     [],
            'servo_roll':       [],
            'servo_pitch':      [],
            'platform_roll':    [],
            'platform_pitch':   [],
        }

    def reset(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.roll_dot = 0.0
        self.pitch_dot = 0.0
        self.servo_roll_actual = 0.0
        self.servo_pitch_actual = 0.0
        for key in self.history:
            self.history[key].clear()

    def step(self, commanded_roll_correction: float,
                   commanded_pitch_correction: float,
                   platform_roll: float,
                   platform_pitch: float,
                   t: float) -> tuple:
        """
        Advance the plant by one time step.

        The body orientation is the sum of:
        - Platform disturbance (external)
        - Gravity-induced dynamics
        - Servo correction (compensating for tilt)

        Args:
            commanded_roll_correction:  desired correction from PID (rad)
            commanded_pitch_correction: desired correction from PID (rad)
            platform_roll:   current platform roll disturbance (rad)
            platform_pitch:  current platform pitch disturbance (rad)
            t:               current time (s)

        Returns:
            (actual_roll, actual_pitch) – current body orientation
        """
        dt = self.p.dt

        # ── 1. Servo dynamics (first-order lag) ───────────────────
        # The servo can't instantly reach the commanded correction.
        # Model: τ·dy/dt + y = u  →  y += (u - y) * (dt * bandwidth)
        servo_bw = self.p.servo_bandwidth_12  # use the slower servo BW
        alpha = min(1.0, dt * servo_bw)
        self.servo_roll_actual  += alpha * (commanded_roll_correction  - self.servo_roll_actual)
        self.servo_pitch_actual += alpha * (commanded_pitch_correction - self.servo_pitch_actual)

        # ── 2. Effective torques on body ──────────────────────────
        # Disturbance torque: platform sway tries to tilt the body
        # The effective roll/pitch of the body = body_own_tilt + platform_tilt
        # Gravity restoring torque: -m*g*h*sin(angle) ≈ -m*g*h*angle
        h = self.p.nominal_height
        g = self.p.gravity
        m = self.total_mass

        # Restoring spring effect from servo correction
        # The servo correction acts like a virtual spring - when the body
        # tilts by 'roll' and servo commands correction 'servo_roll_actual',
        # the effective net tilt error drives the restoring force
        body_roll_from_platform = platform_roll
        body_pitch_from_platform = platform_pitch

        # Roll dynamics: I_xx * d²φ/dt² = τ_disturbance - τ_restore - τ_damping
        # τ_disturbance from platform
        # τ_restore from servo correction (acts to counter body tilt)
        # τ_damping from joint friction

        # Effective roll error (what the body "feels")
        roll_error = body_roll_from_platform - self.servo_roll_actual
        pitch_error = body_pitch_from_platform - self.servo_pitch_actual

        # Gravity restoring torque (pendulum-like)
        tau_gravity_roll  = -m * g * h * np.sin(self.roll)
        tau_gravity_pitch = -m * g * h * np.sin(self.pitch)

        # Disturbance coupling: platform tilt creates a torque
        tau_dist_roll  = m * g * h * np.sin(platform_roll)
        tau_dist_pitch = m * g * h * np.sin(platform_pitch)

        # Servo correction torque (dominant restoring force)
        # The PD servo creates a virtual spring-damper at the body level
        k_eff = self.p.joint_Kp * 4  # 4 legs contributing
        d_eff = self.p.joint_Kd * 4 + self.p.joint_damping * 12  # damping from all joints

        tau_servo_roll  = -k_eff * (self.roll - self.servo_roll_actual + platform_roll)
        tau_servo_pitch = -k_eff * (self.pitch - self.servo_pitch_actual + platform_pitch)

        tau_damping_roll  = -d_eff * self.roll_dot
        tau_damping_pitch = -d_eff * self.pitch_dot

        # Total torques
        tau_roll  = tau_dist_roll  + tau_servo_roll  + tau_gravity_roll  + tau_damping_roll
        tau_pitch = tau_dist_pitch + tau_servo_pitch + tau_gravity_pitch + tau_damping_pitch

        # ── 3. Integrate rotational dynamics ──────────────────────
        # I * α = τ  →  α = τ / I
        roll_ddot  = tau_roll  / self.p.body_ixx
        pitch_ddot = tau_pitch / self.p.body_iyy

        # Semi-implicit Euler integration
        self.roll_dot  += roll_ddot  * dt
        self.pitch_dot += pitch_ddot * dt
        self.roll  += self.roll_dot  * dt
        self.pitch += self.pitch_dot * dt

        # Numerical stability: clamp angles and velocities
        max_angle = np.radians(30)   # ±30° physical limit
        max_rate  = 50.0             # ±50 rad/s
        self.roll      = np.clip(self.roll,      -max_angle, max_angle)
        self.pitch     = np.clip(self.pitch,     -max_angle, max_angle)
        self.roll_dot  = np.clip(self.roll_dot,  -max_rate,  max_rate)
        self.pitch_dot = np.clip(self.pitch_dot, -max_rate,  max_rate)

        # ── 4. Record history ─────────────────────────────────────
        self.history['time'].append(t)
        self.history['roll'].append(self.roll)
        self.history['pitch'].append(self.pitch)
        self.history['roll_dot'].append(self.roll_dot)
        self.history['pitch_dot'].append(self.pitch_dot)
        self.history['roll_torque'].append(tau_roll)
        self.history['pitch_torque'].append(tau_pitch)
        self.history['servo_roll'].append(self.servo_roll_actual)
        self.history['servo_pitch'].append(self.servo_pitch_actual)
        self.history['platform_roll'].append(platform_roll)
        self.history['platform_pitch'].append(platform_pitch)

        return self.roll, self.pitch

    def get_history_arrays(self) -> dict:
        return {k: np.array(v) for k, v in self.history.items()}
