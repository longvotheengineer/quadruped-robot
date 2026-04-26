"""
Posture Stabilizer Node.

PID-based balance controller for quadruped robot.

Supports two operating modes:
  HOME — Continuous PID on filtered IMU, publishes /posture/home_correction
  GAIT — Gait-cycle-aware PID with baseline learning, publishes
         /posture/gait_correction

Subscribes:  /imu/data, /balance/enable, /balance/mode, /balance/gait_frame
Publishes:   /posture/home_correction, /posture/measurement,
             /posture/gait_correction
PlotJuggler: /diag/home/{roll,pitch}_error
             /diag/trotting/{roll,pitch}_error
Diagnostics: /diag/home/pid/{roll,pitch}/01_meas .. 09_dt
             /diag/gait/pid/{roll,pitch}/01_meas .. 09_ff_ready
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64, Bool, String, Int32
from collections import deque

# Home PID diagnostic signals — per axis under /diag/home/pid/{roll,pitch}/
# Pipeline: raw IMU → EMA → deadzone → error → P,I,D → sum → sat → LPF
_HOME_PID_SIGNALS = [
    's01_meas',      # EMA-filtered IMU measurement
    's02_sp',        # setpoint (target = 0.0)
    's03_err',       # error = sp − deadzone(meas)
    's04_p',         # proportional term
    's05_i',         # integral state (accumulator)
    's06_d',         # derivative term (LPF'd)
    's07_pid_sat',   # P+I+D sum, saturated
    's08_pid_lpf',   # after final low-pass filter → published
    's09_dt',        # timestep (seconds)
]

# Gait PID diagnostic signals — per axis under /diag/gait/pid/{roll,pitch}/
# Pipeline: meas → moving avg → deadzone → error → P,I,D → sum → sat → LPF
_GAIT_PID_SIGNALS = [
    's01_meas',      # EMA-filtered IMU measurement
    's02_avg',       # moving average over gait cycle
    's03_err',       # error after deadzone on avg
    's04_p',         # proportional term (adaptive gain)
    's05_i',         # integral state (with zero-crossing reset)
    's06_d',         # derivative term (of undeadzoned avg)
    's07_pid_sat',   # P+I+D sum, saturated
    's08_pid_lpf',   # after final low-pass filter → published
    's09_ff_ready',  # baseline learning complete (1.0/0.0)
]


def quaternion_to_rp(x, y, z, w):
    roll  = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sinp  = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    return roll, pitch


def deadzone_linear(value, deadzone):
    if abs(value) <= deadzone:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - deadzone)


# ── Axis / IMU State (Home PID) ──────────────────────────────────────


class AxisState:
    def __init__(self):
        self.prop       = 0.0
        self.integ      = 0.0
        self.deriv      = 0.0
        self.error_prev = 0.0
        self.meas       = 0.0
        self.corr_lpf   = 0.0
        self.corr_sat   = 0.0


class ImuState:
    def __init__(self):
        self.roll  = AxisState()
        self.pitch = AxisState()
        self.meas_initialized = False


# ── Home PID Configuration ───────────────────────────────────────────


class PidConfig:
    KP = 2.0
    KI = 0.30
    KD = 0.15
    SAT_CORR           = 1.5
    SAT_INTEG          = 1.5
    SAT_GAIN_SCALE_PID = 0.5
    DECAY_INTEG        = 0.9995
    DEADZONE_MEAS      = 0.02
    GAIN_SCALE_PID     = 0.08
    RAMP_DERIV         = 0.005


class FilterConfig:
    LPF_RAW   = 0.4
    LPF_DERIV = 0.7
    LPF_CORR  = 0.15


# ── Gait PID Configuration (moved from gaitGenerator) ────────────────


class GaitPidConfig:
    # PID gains
    KP = 3.0
    KI = 0.005
    KD = 1.2
    GAIN_PROP = 10.0

    # Thresholds & limits
    DEADZONE_AVG = 0.03
    SAT_CORR = 0.30
    SAT_INTEG = 0.3

    # Filtering
    LPF_CORR = 0.97

    # Baseline learning
    BASELINE_LEARN_CYCLES = 2
    BASELINE_ADAPT_RATE = 0.05


# ── Gait PID State (moved from gaitGenerator) ────────────────────────


class GaitPidState:
    def __init__(self, cycle_len):
        # Moving-average buffers
        self.roll_buf = deque(maxlen=cycle_len)
        self.pitch_buf = deque(maxlen=cycle_len)

        # Per-frame baseline
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

        # Output (smoothed correction)
        self.roll_corr_lpf = 0.0
        self.pitch_corr_lpf = 0.0

        # Cycle length
        self.cycle_len = cycle_len


# ── Main Node ─────────────────────────────────────────────────────────


class BalanceController(Node):
    def __init__(self):
        super().__init__('nodeBalanceController')

        self._imu = ImuState()
        self._prev_time = None
        self._enabled = False

        # Mode: "HOME" or "GAIT"
        self._mode = "HOME"

        # Gait PID state (initialized with a default cycle length;
        # will be reset when mode switches to GAIT)
        self._default_cycle_len = 270  # stance(200) + swing(70)
        self._gait = GaitPidState(self._default_cycle_len)
        self._gait_frame = 0

        # ── ROS 2 Subscriptions ──────────────────────────────────
        self._sub_imu = self.create_subscription(
            Imu, '/imu/data', self._imu_callback, 10)
        self._sub_enable = self.create_subscription(
            Bool, '/balance/enable', self._enable_callback, 10)
        self._sub_mode = self.create_subscription(
            String, '/balance/mode', self._mode_callback, 10)
        self._sub_gait_frame = self.create_subscription(
            Int32, '/balance/gait_frame', self._frame_callback, 10)

        # ── ROS 2 Publishers ─────────────────────────────────────
        self._pub_corr = self.create_publisher(
            Vector3, '/posture/home_correction', 10)
        self._pub_meas = self.create_publisher(
            Vector3, '/posture/measurement', 10)
        self._pub_gait_corr = self.create_publisher(
            Vector3, '/posture/gait_correction', 10)

        # ── Diagnostic publishers (home PID) ─────────────────────
        self._diag_home_pubs = {}
        for axis in ('roll', 'pitch'):
            self._diag_home_pubs[axis] = {}
            for sig in _HOME_PID_SIGNALS:
                self._diag_home_pubs[axis][sig] = self.create_publisher(
                    Float64, f'/diag/home/pid/{axis}/{sig}', 10)

        # ── Diagnostic publishers (gait PID) ─────────────────────
        self._diag_gait_pubs = {}
        for axis in ('roll', 'pitch'):
            self._diag_gait_pubs[axis] = {}
            for sig in _GAIT_PID_SIGNALS:
                self._diag_gait_pubs[axis][sig] = self.create_publisher(
                    Float64, f'/diag/gait/pid/{axis}/{sig}', 10)

        # ── PlotJuggler error publishers (clean names) ─────────────
        self._pub_home_error = {
            'roll':  self.create_publisher(Float64, '/diag/home/roll_error',  10),
            'pitch': self.create_publisher(Float64, '/diag/home/pitch_error', 10),
        }
        self._pub_trot_error = {
            'roll':  self.create_publisher(Float64, '/diag/trotting/roll_error',  10),
            'pitch': self.create_publisher(Float64, '/diag/trotting/pitch_error', 10),
        }

        self.get_logger().info(
            f'Posture Stabilizer started '
            f'(Kp={PidConfig.KP}, Ki={PidConfig.KI}, Kd={PidConfig.KD}, '
            f'deadzone={PidConfig.DEADZONE_MEAS}, '
            f'gs_thresh={PidConfig.GAIN_SCALE_PID})')

    # ── Mode / Enable Callbacks ──────────────────────────────────

    def _enable_callback(self, msg: Bool):
        """Enable/disable PID. On rising edge, reset all state."""
        if msg.data and not self._enabled:
            self._imu = ImuState()
            self._prev_time = None
            self._gait = GaitPidState(self._default_cycle_len)
            self.get_logger().info('Balance PID ENABLED — all state reset')
        elif not msg.data and self._enabled:
            self.get_logger().info('Balance PID DISABLED')
        self._enabled = msg.data

    def _mode_callback(self, msg: String):
        """Switch between HOME and GAIT PID modes."""
        new_mode = msg.data.strip().upper()
        if new_mode not in ("HOME", "GAIT"):
            return
        if new_mode != self._mode:
            self._mode = new_mode
            if new_mode == "GAIT":
                self._gait = GaitPidState(self._default_cycle_len)
            self.get_logger().info(f'Balance mode → {self._mode}')

    def _frame_callback(self, msg: Int32):
        """Receive current gait frame index from gait generator."""
        self._gait_frame = msg.data

    # ── Main IMU Callback ────────────────────────────────────────

    def _imu_callback(self, msg: Imu):
        """Process IMU data: pre-filter, deadzone, PID, LPF, publish."""
        if not self._enabled:
            return

        dt = self._compute_dt()
        if dt is None:
            return

        q = msg.orientation
        roll_raw, pitch_raw = quaternion_to_rp(q.x, q.y, q.z, q.w)

        self._filter_meas(roll_raw, pitch_raw)

        # Always publish filtered measurement
        msg_meas   = Vector3()
        msg_meas.x = self._imu.roll.meas
        msg_meas.y = self._imu.pitch.meas
        msg_meas.z = 0.0
        self._pub_meas.publish(msg_meas)

        if self._mode == "GAIT":
            self._run_gait_pid()
        else:
            self._run_home_pid(dt)

    # ── Home PID Pipeline ────────────────────────────────────────

    def _run_home_pid(self, dt):
        """Run the home-mode PID and publish correction."""
        # PID compute + LPF for both axes
        errors = [
            0.0 - deadzone_linear(self._imu.roll.meas,  PidConfig.DEADZONE_MEAS),
            0.0 - deadzone_linear(self._imu.pitch.meas, PidConfig.DEADZONE_MEAS),]
        for axis, error in zip([self._imu.roll, self._imu.pitch], errors):
            self._compute_pid(error, axis, dt)
            axis.corr_lpf = (FilterConfig.LPF_CORR * axis.corr_lpf
                          + (1 - FilterConfig.LPF_CORR) * axis.corr_sat)

        msg_corr   = Vector3()
        msg_corr.x = self._imu.roll.corr_lpf
        msg_corr.y = self._imu.pitch.corr_lpf
        msg_corr.z = 0.0
        self._pub_corr.publish(msg_corr)

        self._publish_diag(dt)

        # PlotJuggler: publish PID error (after deadzone) for home
        msg_err = Float64()
        msg_err.data = self._imu.roll.error_prev
        self._pub_home_error['roll'].publish(msg_err)
        msg_err = Float64()
        msg_err.data = self._imu.pitch.error_prev
        self._pub_home_error['pitch'].publish(msg_err)

    # ── Gait PID Pipeline (moved from gaitGenerator) ─────────────

    def _run_gait_pid(self):
        """Run the gait-mode PID and publish gait correction."""
        g = self._gait
        roll_meas = self._imu.roll.meas
        pitch_meas = self._imu.pitch.meas
        frame = self._gait_frame

        roll_avg, pitch_avg = self._update_moving_average(
            roll_meas, pitch_meas)
        self._update_gait_baseline(frame, roll_meas, pitch_meas)

        pid_out = self._compute_gait_pid_correction(
            roll_avg, pitch_avg)

        # Publish gait correction
        msg_corr = Vector3()
        msg_corr.x = g.roll_corr_lpf
        msg_corr.y = g.pitch_corr_lpf
        msg_corr.z = 0.0
        self._pub_gait_corr.publish(msg_corr)

        self._publish_gait_diag(roll_meas, pitch_meas, pid_out)

        # PlotJuggler: publish PID error (after deadzone) for trotting
        msg_err = Float64()
        msg_err.data = pid_out['roll'][0]
        self._pub_trot_error['roll'].publish(msg_err)
        msg_err = Float64()
        msg_err.data = pid_out['pitch'][0]
        self._pub_trot_error['pitch'].publish(msg_err)

    def _update_moving_average(self, roll_meas, pitch_meas):
        """Append IMU measurement to gait-cycle deque, return averages."""
        g = self._gait
        g.roll_buf.append(roll_meas)
        g.pitch_buf.append(pitch_meas)
        roll_avg = sum(g.roll_buf) / len(g.roll_buf)
        pitch_avg = sum(g.pitch_buf) / len(g.pitch_buf)
        return roll_avg, pitch_avg

    def _update_gait_baseline(self, frame, roll_meas, pitch_meas):
        """Update per-frame feed-forward baseline."""
        g = self._gait
        if frame < 0 or frame >= g.cycle_len:
            return
        if not g.baseline_ready:
            n = g.count_baseline[frame]
            g.roll_baseline[frame] = (g.roll_baseline[frame] * n
                                + roll_meas) / (n + 1)
            g.pitch_baseline[frame] = (g.pitch_baseline[frame] * n
                                 + pitch_meas) / (n + 1)
            g.count_baseline[frame] += 1
        else:
            adapt_rate = GaitPidConfig.BASELINE_ADAPT_RATE
            g.roll_baseline[frame]  += adapt_rate * (roll_meas  - g.roll_baseline[frame])
            g.pitch_baseline[frame] += adapt_rate * (pitch_meas - g.pitch_baseline[frame])

    def _compute_gait_pid_correction(self, roll_avg, pitch_avg):
        """Compute adaptive PID correction from compensated tilt.

        Pipeline:  avg → deadzone → error → P/I/D → sum → saturate → LPF
        Returns dict with per-axis intermediate values for diagnostics.
        """
        g = self._gait

        # ── Error (with deadzone) ─────────────────────────────────
        roll_error  = deadzone_linear(roll_avg,  GaitPidConfig.DEADZONE_AVG)
        pitch_error = deadzone_linear(pitch_avg, GaitPidConfig.DEADZONE_AVG)

        # ── P term (adaptive proportional gain) ───────────────────
        tilt_mag   = max(abs(roll_error), abs(pitch_error))
        gain_scale = 1.0 + min(1.0, tilt_mag / GaitPidConfig.GAIN_PROP)
        kp_eff     = GaitPidConfig.KP * gain_scale

        roll_p  = roll_error  * kp_eff
        pitch_p = pitch_error * kp_eff

        # ── I term (integral with anti-windup + zero-crossing reset)
        g.roll_integ  += roll_error
        g.pitch_integ += pitch_error

        sat_integ = GaitPidConfig.SAT_INTEG
        g.roll_integ  = max(-sat_integ, min(sat_integ, g.roll_integ))
        g.pitch_integ = max(-sat_integ, min(sat_integ, g.pitch_integ))

        if roll_error * g.roll_integ < 0:
            g.roll_integ = 0.0
        if pitch_error * g.pitch_integ < 0:
            g.pitch_integ = 0.0

        roll_i  = g.roll_integ  * GaitPidConfig.KI
        pitch_i = g.pitch_integ * GaitPidConfig.KI

        # ── D term (derivative of undeadzoned signal) ─────────────
        roll_d  = (roll_avg  - g.roll_avg_prev)  * GaitPidConfig.KD
        pitch_d = (pitch_avg - g.pitch_avg_prev) * GaitPidConfig.KD
        g.roll_avg_prev  = roll_avg
        g.pitch_avg_prev = pitch_avg

        # ── PID sum (negative = oppose the tilt) ─────────────────
        roll_corr  = -(roll_p  + roll_i  + roll_d)
        pitch_corr = -(pitch_p + pitch_i + pitch_d)

        # ── Saturate output ──────────────────────────────────────
        sat = GaitPidConfig.SAT_CORR
        roll_corr  = max(-sat, min(sat, roll_corr))
        pitch_corr = max(-sat, min(sat, pitch_corr))

        # ── Low-pass filter ──────────────────────────────────────
        alpha = GaitPidConfig.LPF_CORR
        g.roll_corr_lpf  = alpha * g.roll_corr_lpf  + (1 - alpha) * roll_corr
        g.pitch_corr_lpf = alpha * g.pitch_corr_lpf + (1 - alpha) * pitch_corr

        return {
            'roll':  (roll_error, roll_p,  g.roll_integ,  roll_d,
                      roll_corr,  g.roll_corr_lpf),
            'pitch': (pitch_error, pitch_p, g.pitch_integ, pitch_d,
                      pitch_corr, g.pitch_corr_lpf),
        }

    def _publish_gait_diag(self, roll_meas, pitch_meas, pid_out):
        """Publish per-axis gait PID diagnostic signals."""
        g = self._gait
        ff_ready = 1.0 if g.baseline_ready else 0.0
        meas = {'roll': roll_meas, 'pitch': pitch_meas}
        avg  = {'roll': sum(g.roll_buf) / max(1, len(g.roll_buf)),
                'pitch': sum(g.pitch_buf) / max(1, len(g.pitch_buf))}

        msg = Float64()
        for axis in ('roll', 'pitch'):
            err, p, i, d, pid_sat, pid_lpf = pid_out[axis]
            values = [
                meas[axis],   # 01_meas
                avg[axis],    # 02_avg
                err,          # 03_err
                p,            # 04_p
                i,            # 05_i
                d,            # 06_d
                pid_sat,      # 07_pid_sat
                pid_lpf,      # 08_pid_lpf
                ff_ready,     # 09_ff_ready
            ]
            for sig, val in zip(_GAIT_PID_SIGNALS, values):
                msg.data = val
                self._diag_gait_pubs[axis][sig].publish(msg)

    # ── Home PID Helpers ─────────────────────────────────────────

    def _compute_dt(self):
        """Return dt in seconds, or None on first call / invalid dt."""
        now = self.get_clock().now()
        if self._prev_time is None:
            self._prev_time = now
            return None
        dt = (now - self._prev_time).nanoseconds * 1e-9
        self._prev_time = now
        if dt <= 0 or dt > 0.1:
            return None
        return dt

    def _filter_meas(self, roll_raw, pitch_raw):
        """EMA pre-filter on raw IMU measurement."""
        roll  = self._imu.roll
        pitch = self._imu.pitch
        if not self._imu.meas_initialized:
            roll.meas  = roll_raw
            pitch.meas = pitch_raw
            self._imu.meas_initialized = True
        else:
            a = FilterConfig.LPF_RAW
            roll.meas  = a * roll.meas  + (1 - a) * roll_raw
            pitch.meas = a * pitch.meas + (1 - a) * pitch_raw

    def _compute_gain_scale(self, error):
        """Gain multiplier in [SAT_GAIN_SCALE_PID, 1.0]."""
        ratio = abs(error) / PidConfig.GAIN_SCALE_PID
        return max(PidConfig.SAT_GAIN_SCALE_PID, min(1.0, ratio))

    def _compute_pid(self, error_curr, axis, dt):
        """Run one PID step in-place on axis state."""
        gs = self._compute_gain_scale(error_curr)

        # Proportional
        axis.prop = PidConfig.KP * gs * error_curr
    
        # Integral with leakage
        axis.integ *= PidConfig.DECAY_INTEG
        axis.integ += error_curr * dt
        axis.integ  = max(-PidConfig.SAT_INTEG,
                      min( PidConfig.SAT_INTEG, axis.integ))

        # Derivative with low-pass filter
        deriv_raw = (PidConfig.KD * (error_curr - axis.error_prev) / dt
                     if dt > 0 else 0.0)
        axis.deriv = (FilterConfig.LPF_DERIV * axis.deriv
                   + (1 - FilterConfig.LPF_DERIV) * deriv_raw)
        if abs(error_curr) < PidConfig.RAMP_DERIV:
            axis.deriv = 0.0

        Prop  = axis.prop
        Integ = PidConfig.KI * gs * axis.integ
        Deriv = axis.deriv
        corr_sat_raw  = Prop + Integ + Deriv
        axis.corr_sat = max(-PidConfig.SAT_CORR,
                        min( PidConfig.SAT_CORR, corr_sat_raw))

        # Back-calculation anti-windup
        if abs(corr_sat_raw) > PidConfig.SAT_CORR and PidConfig.KI > 0:
            excess = corr_sat_raw - axis.corr_sat
            axis.integ -= excess / PidConfig.KI * 0.5
            axis.integ  = max(-PidConfig.SAT_INTEG,
                          min( PidConfig.SAT_INTEG, axis.integ))

        axis.error_prev = error_curr

    def _publish_diag(self, dt):
        """Publish per-axis home PID diagnostic signals."""
        msg = Float64()
        for axis_name, axis in [('roll', self._imu.roll),
                                ('pitch', self._imu.pitch)]:
            values = [
                axis.meas,       # 01_meas
                0.0,             # 02_sp
                axis.error_prev, # 03_err
                axis.prop,       # 04_p
                axis.integ,      # 05_i
                axis.deriv,      # 06_d
                axis.corr_sat,   # 07_pid_sat
                axis.corr_lpf,   # 08_pid_lpf
                dt,              # 09_dt
            ]
            for sig, val in zip(_HOME_PID_SIGNALS, values):
                msg.data = val
                self._diag_home_pubs[axis_name][sig].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BalanceController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()