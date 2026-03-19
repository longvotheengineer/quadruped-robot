import math
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from rclpy.node import Node

class SerialPublish():
    def __init__(self, node):
        self.node = node
        self.get_logger = node.get_logger()

        # Publisher for effort commands to Gazebo
        self.pub_sim_gazebo = node.create_publisher(Float64MultiArray, '/leg_controller/commands', 10)

        # Subscribe to actual joint states from Gazebo for PID feedback
        self.actual_positions = {}
        self.actual_velocities = {}
        self.sub_joint_states = node.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10)

        # Software PID gains for effort control
        self.Kp = 50.0   # Proportional gain (Nm/rad)
        self.Kd = 0.5    # Derivative gain (Nm·s/rad)

        # Joint name order (must match controller config)
        self.joint_names = [
            'joint_lf_1', 'joint_lf_2', 'joint_lf_3',
            'joint_lb_1', 'joint_lb_2', 'joint_lb_3',
            'joint_rf_1', 'joint_rf_2', 'joint_rf_3',
            'joint_rb_1', 'joint_rb_2', 'joint_rb_3'
        ]

        # Store last commanded for diagnostics
        self.last_commanded = {}

    def joint_states_callback(self, msg):
        """Read actual joint positions/velocities from Gazebo."""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.actual_positions[name] = msg.position[i]
            if i < len(msg.velocity):
                self.actual_velocities[name] = msg.velocity[i]

    def publish_simulation(self, theta):
        """Compute target angles, then publish torques via software PID."""
        # Convert degrees to radians (with +180° offset on theta3 for Gazebo)
        theta_lf_1 = math.radians(theta[0, 0])
        theta_lf_2 = math.radians(theta[0, 1])
        theta_lf_3 = math.radians(theta[0, 2])
        theta_lb_1 = math.radians(theta[1, 0])
        theta_lb_2 = math.radians(theta[1, 1])
        theta_lb_3 = math.radians(theta[1, 2])
        theta_rf_1 = math.radians(theta[2, 0])
        theta_rf_2 = math.radians(theta[2, 1])
        theta_rf_3 = math.radians(theta[2, 2])
        theta_rb_1 = math.radians(theta[3, 0])
        theta_rb_2 = math.radians(theta[3, 1])
        theta_rb_3 = math.radians(theta[3, 2])

        # Target positions (radians) with +180° offset on theta3
        targets = [
            theta_lf_1, theta_lf_2, theta_lf_3 + math.radians(180),
            theta_lb_1, theta_lb_2, theta_lb_3 + math.radians(180),
            theta_rf_1, theta_rf_2, theta_rf_3 + math.radians(180),
            theta_rb_1, theta_rb_2, theta_rb_3 + math.radians(180)
        ]

        # Store for diagnostics
        self.last_commanded = {'joint_lf_2': theta_lf_2}

        # Compute torques using software PID
        torques = []
        for i, name in enumerate(self.joint_names):
            target = targets[i]
            actual_pos = self.actual_positions.get(name, target)  # Default to target if no feedback yet
            actual_vel = self.actual_velocities.get(name, 0.0)

            # PD control: torque = Kp * position_error - Kd * velocity
            error = target - actual_pos
            torque = self.Kp * error - self.Kd * actual_vel
            
            # Clamp to URDF effort limit
            torque = max(-29.0, min(29.0, torque))

            torques.append(torque)

        # Publish torques to Gazebo effort controller
        msg_gazebo = Float64MultiArray()
        msg_gazebo.data = torques
        self.pub_sim_gazebo.publish(msg_gazebo)

    def publish_message(self, theta):
        self.publish_simulation(theta)