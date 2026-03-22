import numpy as np
from dataclasses import dataclass
from leg_controller.kinematics import Kinematics
from leg_controller.serialPublish import SerialPublish
from leg_controller.quinticPlanning import quintic_planning
from std_msgs.msg import Float64MultiArray

@dataclass(frozen=True)
class Waypoint:
    zero   : int
    stance : int
    swing  : int

@dataclass(frozen=True)
class RobotLength:
    L : float
    W : float
    l1: float   
    l2: float
    l3: float

class Gait:
    def __init__(self, node, gait_msg):
        self.node = node
        self.get_logger = node.get_logger()

        robot_length = RobotLength(L = 120, W = 90, l1 = 20, l2 = 80, l3 = 80)
        self.kinematics = Kinematics(self.node, robot_length)
        self.serial_publish = SerialPublish(self.node)
        self.gait_msg = gait_msg

        self.waypoint = Waypoint(20, 300, 30)

        # Trajectory state
        self.gait_angle_data = None
        self.current_frame = 0
        self.complete_step = 0

    # ── Leg configuration ────────────────────────────────────────

    PARAMS_GAIT_TROT = {
        "left-front":    {"x_center":  60, "y_val":  60},
        "left-behind":   {"x_center": -60, "y_val":  60},
        "right-front":   {"x_center":  60, "y_val": -60},
        "right-behind":  {"x_center": -60, "y_val": -60},
    }

    PARAMS_GAIT_BACKWARD = {
        "left-front":    {"x_center":  60, "y_val":  60, "reverse": True},
        "left-behind":   {"x_center": -60, "y_val":  60, "reverse": True},
        "right-front":   {"x_center":  60, "y_val": -60, "reverse": True},
        "right-behind":  {"x_center": -60, "y_val": -60, "reverse": True},
    }

    PARAMS_GAIT_TURN_RIGHT = {
        "left-front":    {"x_center":  60, "y_val":  60, "reverse": False},
        "left-behind":   {"x_center": -60, "y_val":  60, "reverse": False},
        "right-front":   {"x_center":  60, "y_val": -60, "reverse": True},
        "right-behind":  {"x_center": -60, "y_val": -60, "reverse": True},
    }

    PARAMS_GAIT_TURN_LEFT = {
        "left-front":    {"x_center":  60, "y_val":  60, "reverse": True},
        "left-behind":   {"x_center": -60, "y_val":  60, "reverse": True},
        "right-front":   {"x_center":  60, "y_val": -60, "reverse": False},
        "right-behind":  {"x_center": -60, "y_val": -60, "reverse": False},
    }

    PARAMS_PHASESHIFT_TROT = {
        "left-front":   0.00,
        "right-behind": 0.00,
        "left-behind":  0.50,
        "right-front":  0.50,
    }

    CONTROL_VELOCITY = True

    # ── Trajectory generation ───────────────────────────────────

    def trajectory_foot(self, x_center, y_val, reverse=False):
        """Build D-shape foot path in Cartesian space (x, y, z).
        If reverse=True, swap swing direction (for turning)."""
        stride_length = 45
        x_forward  = x_center + stride_length / 2
        x_backward = x_center - stride_length / 2
        z_stance = -150
        z_swing  = -110
        lift_height = z_swing - z_stance

        if reverse:
            pos_A = [x_forward,  y_val, z_stance]
            pos_D = [x_backward, y_val, z_stance]
        else:
            pos_A = [x_backward, y_val, z_stance]
            pos_D = [x_forward,  y_val, z_stance]

        if self.CONTROL_VELOCITY:
            return quintic_planning(pos_A, pos_D,
                                  T_swing     = self.waypoint.swing,
                                  T_stance    = self.waypoint.stance,
                                  lift_height = lift_height)
        else:
            # Swing: sine-wave lift from backward to forward
            swing = np.zeros((self.waypoint.swing, 3))
            swing[:, 0] = np.linspace(x_backward, x_forward, self.waypoint.swing)
            swing[:, 1] = y_val
            swing[:, 2] = z_stance + lift_height * np.sin(np.linspace(0, np.pi, self.waypoint.swing))

            # Stance: slide on ground from forward to backward
            stance = np.zeros((self.waypoint.stance, 3))
            stance[:, 0] = np.linspace(x_forward, x_backward, self.waypoint.stance)
            stance[:, 1] = y_val
            stance[:, 2] = z_stance

            return np.vstack([swing, stance])

    def generate_gait(self, leg_type):
        """Generate one leg's full gait cycle: foot path → IK → phase shift."""
        # Select params based on command
        match self.gait_msg.cmd:
            case "FORWARD":
                params = self.PARAMS_GAIT_TROT.get(leg_type)
            case "BACKWARD":
                params = self.PARAMS_GAIT_BACKWARD.get(leg_type)
            case "TURN_RIGHT":
                params = self.PARAMS_GAIT_TURN_RIGHT.get(leg_type)
            case "TURN_LEFT":
                params = self.PARAMS_GAIT_TURN_LEFT.get(leg_type)
            case _:
                return None

        if not params:
            return None

        # 1. Foot trajectory in Cartesian space
        reverse = params.get("reverse", False)
        waypoint = self.trajectory_foot(params["x_center"], params["y_val"], reverse)

        # 2. Convert to joint angles via IK
        theta_i = np.zeros_like(waypoint)
        for i in range(waypoint.shape[0]):
            theta_i[i] = self.kinematics.inverse(*waypoint[i], leg_type)

        # 3. Apply phase shift (trot pattern)
        shift = round(waypoint.shape[0] * self.PARAMS_PHASESHIFT_TROT.get(leg_type, 0))
        return np.roll(theta_i, shift, axis=0)

    def generate_home(self):
        """Precalculate homing trajectory: back legs first, then front legs."""
        th1, th2, th3 = self.kinematics.inverse(60, 60, -150, "left-front")

        if th2 > 180:
            th2 -= 360

        # ± sign pattern with 2π-complement (inverted rotation)
        homing_targets = {
            'joint_lf_1': 0.0,  'joint_lf_2':  np.radians(th2) + 2 * np.pi,  'joint_lf_3':  np.radians(th3) + np.pi,
            'joint_lb_1': 0.0,  'joint_lb_2': -np.radians(th2) - 2 * np.pi,  'joint_lb_3': -np.radians(th3) - np.pi,
            'joint_rf_1': 0.0,  'joint_rf_2': -np.radians(th2) - 2 * np.pi,  'joint_rf_3': -np.radians(th3) - np.pi,
            'joint_rb_1': 0.0,  'joint_rb_2':  np.radians(th2) + 2 * np.pi,  'joint_rb_3':  np.radians(th3) + np.pi,
        }

        # Read current encoder positions
        positions = self.serial_publish.controller_sim.actual_positions
        if not positions:
            return None

        joint_names = self.serial_publish.controller_sim.joint_names
        back_joints  = [n for n in joint_names if '_lb_' in n or '_rb_' in n]
        front_joints = [n for n in joint_names if '_lf_' in n or '_rf_' in n]

        total_steps = 2000

        # Build frame-by-frame targets: [total_frames x 12] array
        # Phase 1: back legs move (0 to total_steps)
        # Phase 2: front legs move (total_steps to 2*total_steps)
        theta_i = []
        for phase in range(2):
            for step in range(total_steps):
                alpha = step / total_steps
                targets = []
                for name in joint_names:
                    start = positions[name]
                    end = homing_targets[name]
                    if phase == 0:  # Back legs move, front hold
                        if name in back_joints:
                            targets.append(start + alpha * (end - start))
                        else:
                            targets.append(start)
                    else:  # Front legs move, back hold
                        if name in front_joints:
                            targets.append(start + alpha * (end - start))
                        else:
                            targets.append(end)
                theta_i.append(targets)

        # Store homing targets for READY state hold
        self.homing_targets = homing_targets
        return np.array(theta_i)  # shape: [4000, 12]

    def change(self):        
        match self.gait_msg.cmd:
            case "ZERO":
                return self.generate_home()
            case _:
                theta_i     = np.empty(4, dtype=object) 
                theta_i[0]  = self.generate_gait("left-front")
                theta_i[1]  = self.generate_gait("left-behind")
                theta_i[2]  = self.generate_gait("right-front")
                theta_i[3]  = self.generate_gait("right-behind")
                return theta_i   

    def control_init(self):
        self.gait_angle_data = self.change()
        self.current_frame = 0
        self.complete_step = 0

    def control_tick(self):
        if self.gait_angle_data is None:
            return False

        if self.gait_msg.cmd == "ZERO":
            return self._tick_homing()
        else:
            return self._tick_gait()

    def _tick_homing(self):
        """Advance homing by one frame: publish joint targets via PID."""
        total_frames = self.gait_angle_data.shape[0]
        if self.current_frame < total_frames:
            targets = self.gait_angle_data[self.current_frame].tolist()
            torques = self.serial_publish.controller_sim.compute_torques(targets)
            msg = Float64MultiArray()
            msg.data = torques
            self.serial_publish.pub_sim_gazebo.publish(msg)
            self.current_frame += 1
            return True
        else:
            # Hold at homing targets
            joint_names = self.serial_publish.controller_sim.joint_names
            targets = [self.homing_targets[name] for name in joint_names]
            torques = self.serial_publish.controller_sim.compute_torques(targets)
            msg = Float64MultiArray()
            msg.data = torques
            self.serial_publish.pub_sim_gazebo.publish(msg)
            return False

    def _tick_gait(self):
        """Advance gait by one frame. step=0 means run continuously."""
        theta_i = self.gait_angle_data

        # If finite steps completed, hold the last frame position
        if self.gait_msg.step > 0 and self.complete_step >= self.gait_msg.step:
            pos = np.array([theta_i[i][self.current_frame] for i in range(4)])
            self.serial_publish.publish_message(pos)
            return False

        pos = np.array([theta_i[i][self.current_frame] for i in range(4)])        
        self.serial_publish.publish_message(pos)
        
        # Advance frame
        self.current_frame += 1
        if self.current_frame >= theta_i[0].shape[0]:
            self.current_frame = 0
            self.complete_step += 1
            
        return True