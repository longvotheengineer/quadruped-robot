"""Gait controller: orchestrates trajectory generation, balance, and actuation.

This is the main entry point for the gait system. It manages:
  - ROS 2 publishers/subscribers for balance and diagnostics
  - Gait command dispatch and timing selection
  - Tick-by-tick execution of homing, locomotion, and shutdown
  - Foot position tracking for PlotJuggler diagnostics
"""

import numpy as np
from std_msgs.msg import Float64MultiArray, Float64, Bool, String, Int32
from geometry_msgs.msg import Vector3

from leg_controller.gaitConfig import (
    ROBOT, LEG_NAMES, LEG_ABBREVIATIONS, INIT_POSE, JOINT_NAMES,
    SAFE_OFF_ANGLES, SHUTDOWN_FRAMES_PER_PHASE,
    BODY_COMMANDS, BODY_BLEND_FRAMES, HOME_DIAGNOSTIC_SIGNALS,
    TIMING_TROT_FORWARD, TIMING_TROT_BACKWARD, TIMING_WALK,
    TIMING_WAVE, TIMING_TURN, TIMING_STRAFE, TIMING_BODY,
)
from leg_controller.gaitBalance import (
    HomeBalanceState, StanceCorrectionInput,
    applyHomeOffset, applyStanceCorrection,
)
from leg_controller.gaitTrajectory import generateLegCycle, generateWaveCycle
from leg_controller.kinematics import Kinematics
from leg_controller.actuator import Actuator


class GaitController:
    """Main gait orchestrator for the quadruped robot.

    Public API (called by nodeLegController):
        tick()          — advance one frame
        startCommand()  — initialize trajectory for a new gait command
        initPose()      — hold at initial pose (idle state)
    """

    def __init__(self, node, command):
        self._node = node
        self._logger = node.get_logger()
        self._command = command
        self._useReal = node.use_real

        # Hardware abstraction
        self.actuator = Actuator(node)

        # Kinematics
        self._kinematics = Kinematics(node, ROBOT, use_real=self._useReal)

        # Timing (overridden per gait command)
        self._timing = TIMING_TROT_FORWARD

        # Trajectory state
        self._jointTrajectory = None
        self._footTrajectory = None
        self._frameIndex = 0
        self._cycleCount = 0

        # Homing targets (set by buildHomingTrajectory)
        self._homingTargets = None

        # Balance state
        self._homeBalance = HomeBalanceState()
        self._stanceCorrection = StanceCorrectionInput()
        self._balanceEnabled = False

        # Shutdown state
        self._shutdownComplete = False
        self._shutdownTargets = None

        # Body posture blend tracking
        self._lastFootPos = None
        self._blendEndFrame = 0

        # Foot P-V-A tracking for PlotJuggler
        self._footZPrev = {leg: 0.0 for leg in LEG_NAMES}
        self._footVzPrev = {leg: 0.0 for leg in LEG_NAMES}

        # ── ROS 2 Subscriptions ──────────────────────────────────────
        node.create_subscription(
            Vector3, '/posture/home_correction', self._homeCorrCb, 10)
        node.create_subscription(
            Vector3, '/posture/gait_correction', self._gaitCorrCb, 10)

        # ── ROS 2 Publishers ─────────────────────────────────────────
        self._pubBalanceEnable = node.create_publisher(Bool, '/balance/enable', 10)
        self._pubBalanceMode = node.create_publisher(String, '/balance/mode', 10)
        self._pubGaitFrame = node.create_publisher(Int32, '/balance/gait_frame', 10)

        # Home offset diagnostic publishers
        self._diagHomePubs = {}
        for sig in HOME_DIAGNOSTIC_SIGNALS:
            self._diagHomePubs[sig] = node.create_publisher(
                Float64, f'/diag/home/apply/{sig}', 10)

        # Foot P-V-A diagnostic publishers
        self._pubFoot = {}
        for leg, abbr in LEG_ABBREVIATIONS.items():
            self._pubFoot[leg] = {
                'pos': node.create_publisher(Float64, f'/diag/foot/{abbr}/pos_z', 10),
                'vel': node.create_publisher(Float64, f'/diag/foot/{abbr}/vel_z', 10),
                'acc': node.create_publisher(Float64, f'/diag/foot/{abbr}/acc_z', 10),
            }

    # ── 1. Public Interface ──────────────────────────────────────────────

    def tick(self):
        """Advance one frame. Returns True if still running."""
        if self._jointTrajectory is None:
            return False
        cmd = self._command.cmd
        if cmd == "ZERO":
            return self._tickHoming()
        elif cmd == "ROBOTOFF":
            return self._tickShutdown()
        else:
            return self._tickLocomotion()

    def startCommand(self):
        """Initialize trajectory for the current gait command."""
        self._jointTrajectory = self._buildTrajectory()
        self._frameIndex = 0
        self._cycleCount = 0
        self._stanceCorrection = StanceCorrectionInput()

    def initPose(self):
        """Hold at initial pose (called every tick while IDLE)."""
        self.actuator.holdInitPose(INIT_POSE)

    # ── 2. Tick Orchestration (Frame Advance) ────────────────────────────

    def _tickHoming(self):
        """Advance homing by one frame."""
        traj = self._jointTrajectory
        total = traj.shape[0]

        if self._frameIndex < total:
            self.actuator.sendHomingFrame(traj[self._frameIndex].tolist())
            self._frameIndex += 1
            return True

        # Homing complete — enable balance PID (sim only) and hold
        if not self._balanceEnabled and not self._useReal:
            self._pubBalanceEnable.publish(Bool(data=True))
            self._pubBalanceMode.publish(String(data='HOME'))
            self._balanceEnabled = True
            self._logger.info('Homing complete — enabling balance PID (HOME mode)')

        if self._useReal:
            self.actuator.holdHomingTarget(self._homingTargets)
        else:
            # Apply IMU home correction before holding
            names = self.actuator._simController.joint_names
            targets = [self._homingTargets[n] for n in names]
            applyHomeOffset(targets, self._homeBalance, self._diagHomePubs)
            # Send once with corrected targets
            self.actuator.holdHomingTarget(self._homingTargets, sim_target_list=targets)

        return False

    def _tickLocomotion(self):
        """Advance gait by one frame."""
        traj = self._jointTrajectory
        frame = self._frameIndex

        if self._useReal:
            pos = np.array([traj[i][frame] for i in range(4)])
        else:
            self._pubGaitFrame.publish(Int32(data=frame))
            pos = applyStanceCorrection(
                frame, traj, self._footTrajectory,
                self._stanceCorrection, self._kinematics)

        # Check step limit
        if self._command.step > 0 and self._cycleCount >= self._command.step:
            self.actuator.send(pos)
            return False

        self.actuator.send(pos)

        # Track last foot position for smooth body transitions
        if self._footTrajectory is not None:
            self._lastFootPos = {}
            for i, leg in enumerate(LEG_NAMES):
                self._lastFootPos[leg] = self._footTrajectory[i][frame].copy()

        self._frameIndex += 1
        if self._frameIndex >= traj[0].shape[0]:
            self._frameIndex = self._blendEndFrame
            self._cycleCount += 1

        self._publishFootDiagnostics(frame)
        return True

    def _tickShutdown(self):
        """Advance safe shutdown by one frame (real only, no-op for sim)."""
        if not self._useReal:
            return False

        traj = self._jointTrajectory
        total = traj.shape[0]

        if self._frameIndex < total:
            self.actuator.sendShutdownFrame(traj[self._frameIndex].tolist())
            self._frameIndex += 1
            return True

        self.actuator.sendShutdownFrame(self._shutdownTargets)
        if not self._shutdownComplete:
            self._shutdownComplete = True
            self._logger.info('╔══════════════════════════════════════════╗')
            self._logger.info('║   ROBOT OFF COMPLETE — SAFE TO Ctrl+C   ║')
            self._logger.info('╚══════════════════════════════════════════╝')
        return False

    # ── 3. Trajectory Generation (Build Handlers) ────────────────────────

    def _buildTrajectory(self):
        """Dispatch trajectory generation based on current command."""
        cmd = self._command.cmd

        # Select per-gait timing
        match cmd:
            case "TROT_FORWARD":
                self._timing = TIMING_TROT_FORWARD
            case "TROT_BACKWARD":
                self._timing = TIMING_TROT_BACKWARD
            case "WALK_FORWARD" | "WALK_BACKWARD":
                self._timing = TIMING_WALK
            case "WAVE_FORWARD" | "WAVE_BACKWARD":
                self._timing = TIMING_WAVE
            case "TURN_RIGHT" | "TURN_LEFT":
                self._timing = TIMING_TURN
            case "STRAFE_RIGHT" | "STRAFE_LEFT":
                self._timing = TIMING_STRAFE
            case "BODY_HEAVE" | "BODY_ROLL" | "BODY_PITCH" | "BODY_CIRCLE":
                self._timing = TIMING_BODY

        # Build trajectory
        match cmd:
            case "ZERO":
                return self._buildHomingTrajectory()
            case "ROBOTOFF":
                return self._buildShutdownTrajectory()
            case "WAVE_FORWARD" | "WAVE_BACKWARD":
                return self._buildWaveTrajectory(cmd)
            case _:
                return self._buildStandardTrajectory(cmd)

    def _buildStandardTrajectory(self, cmd):
        """Build standard gait trajectory (trot, walk, turn, strafe, body)."""
        angles = np.empty(4, dtype=object)
        feet = np.empty(4, dtype=object)

        for idx, leg in enumerate(LEG_NAMES):
            a, f = generateLegCycle(
                cmd, leg, self._timing, self._kinematics, self._useReal)
            angles[idx] = a
            feet[idx] = f

        # Smooth blend for body posture transitions
        if cmd in BODY_COMMANDS and self._lastFootPos is not None:
            blend_n = BODY_BLEND_FRAMES
            for idx, leg in enumerate(LEG_NAMES):
                start = self._lastFootPos[leg]
                end = feet[idx][0]
                blend_foot = np.zeros((blend_n, 3))
                blend_ang = np.zeros((blend_n, 3))
                for f in range(blend_n):
                    alpha = 0.5 - 0.5 * np.cos(np.pi * f / blend_n)
                    ft = start + alpha * (end - start)
                    blend_foot[f] = ft
                    blend_ang[f] = self._kinematics.inverse(*ft, leg)
                angles[idx] = np.vstack([blend_ang, angles[idx]])
                feet[idx] = np.vstack([blend_foot, feet[idx]])
            self._blendEndFrame = blend_n
        else:
            self._blendEndFrame = 0

        self._footTrajectory = feet
        self._pubBalanceMode.publish(String(data='GAIT'))
        return angles

    def _buildWaveTrajectory(self, cmd):
        """Build sequential (wave) gait trajectory."""
        angles, feet = generateWaveCycle(
            cmd, self._timing, self._kinematics, self._useReal)
        self._footTrajectory = feet
        self._pubBalanceMode.publish(String(data='GAIT'))
        return angles

    def _buildHomingTrajectory(self):
        """Build homing trajectory (delegates to actuator)."""
        self._footTrajectory = None
        self._pubBalanceMode.publish(String(data='HOME'))

        traj, targets = self.actuator.buildHomingTrajectory(
            self._kinematics, self._timing)
        self._homingTargets = targets
        return traj

    def _buildShutdownTrajectory(self):
        """Build three-phase safe-off trajectory (real hardware only)."""
        self._footTrajectory = None
        self._shutdownComplete = False

        if not self._useReal:
            self._logger.info('ROBOTOFF: simulation mode — nothing to do.')
            return None

        # Use homing targets as start position (feedback is disabled)
        if hasattr(self, '_homingTargets') and self._homingTargets:
            current = np.array(self._homingTargets, dtype=float)
            self._logger.info('ROBOTOFF: using homing targets as start position')
        else:
            if not self.actuator._realPositions:
                self._logger.warn('No servo positions — waiting for feedback...')
                return None
            current = np.array([self.actuator._realPositions.get(n, 0.0)
                                for n in JOINT_NAMES])
            self._logger.info('ROBOTOFF: using feedback positions as start')

        safe = SAFE_OFF_ANGLES
        N = SHUTDOWN_FRAMES_PER_PHASE
        legs = LEG_NAMES

        j3_idx = [2, 5, 8, 11]
        j2_idx = [1, 4, 7, 10]
        j1_idx = [0, 3, 6, 9]

        j3_targets = [safe['joint3'][leg] for leg in legs]
        j2_targets = [safe['joint2'][leg] for leg in legs]
        j1_targets = [safe['joint1'][leg] for leg in legs]

        # Phase 1: collapse knee
        phase1 = np.tile(current, (N, 1))
        for k, idx in enumerate(j3_idx):
            phase1[:, idx] = np.linspace(current[idx], j3_targets[k], N)
        mid1 = current.copy()
        for k, idx in enumerate(j3_idx):
            mid1[idx] = j3_targets[k]

        # Phase 2: fold shoulder
        phase2 = np.tile(mid1, (N, 1))
        for k, idx in enumerate(j2_idx):
            phase2[:, idx] = np.linspace(mid1[idx], j2_targets[k], N)
        mid2 = mid1.copy()
        for k, idx in enumerate(j2_idx):
            mid2[idx] = j2_targets[k]

        # Phase 3: splay hip
        phase3 = np.tile(mid2, (N, 1))
        for k, idx in enumerate(j1_idx):
            phase3[:, idx] = np.linspace(mid2[idx], j1_targets[k], N)

        final = mid2.copy()
        for k, idx in enumerate(j1_idx):
            final[idx] = j1_targets[k]
        self._shutdownTargets = final.tolist()

        self._logger.info(
            f'ROBOTOFF: 3 phases × {N} frames = {3*N} total '
            f'({3*N*7/1000:.1f} s)')
        return np.vstack([phase1, phase2, phase3])

    # ── 4. Telemetry & Diagnostics ───────────────────────────────────────

    def _publishFootDiagnostics(self, frame):
        """Publish foot Z position, velocity, acceleration per leg."""
        if self._footTrajectory is None:
            return

        msg = Float64()
        for i, leg in enumerate(LEG_NAMES):
            z = float(self._footTrajectory[i][frame][2])
            vz = z - self._footZPrev[leg]
            az = vz - self._footVzPrev[leg]

            self._footZPrev[leg] = z
            self._footVzPrev[leg] = vz

            pubs = self._pubFoot[leg]
            msg.data = z;  pubs['pos'].publish(msg)
            msg.data = vz; pubs['vel'].publish(msg)
            msg.data = az; pubs['acc'].publish(msg)

    # ── 5. ROS 2 Callbacks ───────────────────────────────────────────────

    def _homeCorrCb(self, msg):
        self._homeBalance.roll = msg.x
        self._homeBalance.pitch = msg.y

    def _gaitCorrCb(self, msg):
        self._stanceCorrection.roll = msg.x
        self._stanceCorrection.pitch = msg.y
