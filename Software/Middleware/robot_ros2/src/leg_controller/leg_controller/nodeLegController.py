import rclpy
from std_msgs.msg import String
from rclpy.node import Node
from leg_controller.gaitGenerator import Gait

class GaitMsg:
    def __init__(self, cmd, step):
        self.cmd = cmd
        self.step = step

class LegController(Node):
    def __init__(self):
        super().__init__('node_leg_controller')

        self.gait_msg = GaitMsg(None, 0)
        self.gait = Gait(self, self.gait_msg)
        
        self.subscription_ = self.create_subscription(
            String, '/gait_control', self.listener_callback, 10)
        
        self.timer = self.create_timer(0.003, self.timer_callback)
        self.state_gait = "AUTO_INIT"

        self.get_logger().info('The main Node has started.')    

    def timer_callback(self):
        if self.state_gait == "AUTO_INIT":
            # Wait for encoder data, then hold at init pose
            if self.gait.serial_publish.controller_sim.actual_positions:
                self.state_gait = "IDLE"
                self.get_logger().info('Init pose reached. Waiting for commands.')
        elif self.state_gait == "IDLE":
            self.gait.init_pose_tick()
        elif self.state_gait == "INIT":
            self.gait.control_init()
            self.state_gait = "READY"
        elif self.state_gait == "READY":
            self.gait.control_tick()

    def listener_callback(self, msg):
        parts = msg.data.split()
        if len(parts) == 2:
            try:
                self.gait_msg.cmd = parts[0]
                self.gait_msg.step = float(parts[1])
                self.state_gait = "INIT"
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