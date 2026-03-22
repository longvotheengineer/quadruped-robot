import math
from std_msgs.msg import Float64MultiArray
from rclpy.node import Node
from leg_controller.controllerSim import ControllerSim

class SerialPublish():
    def __init__(self, node):
        self.node = node
        self.get_logger = node.get_logger()

        # Publisher for effort commands to Gazebo
        self.pub_sim_gazebo = node.create_publisher(Float64MultiArray, '/leg_controller/commands', 10)

        # PID controller for torque computation
        self.controller_sim = ControllerSim(node)

    def publish_simulation(self, theta):
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

        targets = [theta_lf_1, theta_lf_2, theta_lf_3 + math.radians(180),
                   theta_lb_1, theta_lb_2, theta_lb_3 + math.radians(180),
                   theta_rf_1, theta_rf_2, theta_rf_3 + math.radians(180),
                   theta_rb_1, theta_rb_2, theta_rb_3 + math.radians(180)]

        # Wrap targets to nearest ±π of actual position (prevents 2π jumps)
        if self.controller_sim.actual_positions:
            joint_names = self.controller_sim.joint_names
            for i, name in enumerate(joint_names):
                actual = self.controller_sim.actual_positions.get(name, 0.0)
                diff = targets[i] - actual
                # Wrap diff to [-π, π]
                diff = (diff + math.pi) % (2 * math.pi) - math.pi
                targets[i] = actual + diff

        # Compute torques via PID and publish
        torques = self.controller_sim.compute_torques(targets)
        msg_gazebo = Float64MultiArray()
        msg_gazebo.data = torques
        self.pub_sim_gazebo.publish(msg_gazebo)

    def publish_message(self, theta):
        self.publish_simulation(theta)