"""Leg controller ROS 2 node: receives gait commands and drives the robot.

Subscribes to /gait_control for command strings (e.g., 'TROT_FORWARD 3').
Runs a 3 ms timer loop that dispatches to the GaitController.
"""

import rclpy
from std_msgs.msg import String
from rclpy.node import Node
from leg_controller.gaitController import GaitController


class GaitCommand:
    """Parsed gait command with name and step count."""

    def __init__(self, cmd=None, step=0):
        self.cmd = cmd
        self.step = step


class LegController(Node):
    """Main ROS 2 node for quadruped leg control."""

    def __init__(self):
        super().__init__('nodeLegController')

        self.declare_parameter('use_real_hardware', False)
        self.use_real = self.get_parameter('use_real_hardware').value

        self._command = GaitCommand()
        self._gait = GaitController(self, self._command)

        self._sub = self.create_subscription(
            String, '/gait_control', self._commandCb, 10)
        self._timer = self.create_timer(0.003, self._timerCb)

        if self.use_real:
            self._state = "IDLE"
            self.get_logger().info('REAL HARDWARE mode — ready for commands.')
        else:
            self._state = "AUTO_INIT"

        self.get_logger().info('Leg controller node started.')

    def _timerCb(self):
        """Main control loop (called every 3 ms)."""
        if self._state == "AUTO_INIT":
            if self._gait.actuator.hasPositionData():
                self._state = "IDLE"
                self.get_logger().info('Init pose reached. Waiting for commands.')
        elif self._state == "IDLE":
            self._gait.initPose()
        elif self._state == "INIT":
            self._gait.startCommand()
            self._state = "READY"
        elif self._state == "READY":
            self._gait.tick()

    def _commandCb(self, msg):
        """Parse incoming gait command string."""
        parts = msg.data.split()
        if len(parts) == 2:
            try:
                self._command.cmd = parts[0]
                self._command.step = float(parts[1])
                self._state = "INIT"
            except ValueError:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = LegController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()