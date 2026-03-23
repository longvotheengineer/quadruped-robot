from sensor_msgs.msg import JointState

class ControllerSim():
    def __init__(self, node):
        self.node = node

        # PID gains for effort control
        self.Kp = 20.0   # Nm/rad
        self.Kd = 0.5    # Nm·s/rad

        # Effort clamp limit (matches URDF effort limit)
        self.effort_limit = 29.0

        # Joint name order (must match controller config)
        self.joint_names = ['joint_lf_1', 'joint_lf_2', 'joint_lf_3',
                            'joint_lb_1', 'joint_lb_2', 'joint_lb_3',
                            'joint_rf_1', 'joint_rf_2', 'joint_rf_3',
                            'joint_rb_1', 'joint_rb_2', 'joint_rb_3']

        # Actual joint states from Gazebo
        self.actual_positions = {}
        self.actual_velocities = {}
        self.sub_joint_states = node.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10)

    def joint_states_callback(self, msg):
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.actual_positions[name] = msg.position[i]
            if i < len(msg.velocity):
                self.actual_velocities[name] = msg.velocity[i]

    def compute_torques(self, targets):
        """Compute PD torques from target positions (rad).
        
        Args:
            targets: list of 12 target joint positions in radians
        Returns:
            list of 12 clamped torque values
        """
        torques = []
        for i, name in enumerate(self.joint_names):
            target = targets[i]
            actual_pos = self.actual_positions.get(name, target)
            actual_vel = self.actual_velocities.get(name, 0.0)

            # PD control: torque = Kp * position_error - Kd * velocity
            error = target - actual_pos
            torque = self.Kp * error - self.Kd * actual_vel

            # Clamp to URDF effort limit
            torque = max(-self.effort_limit, min(self.effort_limit, torque))

            torques.append(torque)

        return torques
