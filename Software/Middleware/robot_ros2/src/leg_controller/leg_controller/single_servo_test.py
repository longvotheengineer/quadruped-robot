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
        super().__init__('leg_controller_node')

        self.gait_msg  = GaitMsg("ZERO", 0)
        self.gait = Gait(self, self.gait_msg)
        
        self.subscription_ = self.create_subscription(
            String,
            '/gait_control',
            self.listener_callback,
            10)
        self.get_logger().info('The main Node has started.')

    def parse_command(self, msg):
        parts = msg.split()
        if len(parts) == 2:
            gait_cmd = parts[0]
            try:
                gait_value = float(parts[1])
            except ValueError:
                gait_value = None
            return gait_cmd, gait_value
        else:
            return None, None

    def listener_callback(self, msg):
        self.get_logger().info(f'[Sub]: {msg.data}')   
        self.gait_msg.cmd, self.gait_msg.step = self.parse_command(msg.data)
        self.gait.control()

def main(args=None):
    rclpy.init(args=args)
    node = LegController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()