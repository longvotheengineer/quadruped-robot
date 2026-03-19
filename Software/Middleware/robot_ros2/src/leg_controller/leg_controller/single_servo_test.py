import rclpy
from std_msgs.msg import String
from rclpy.node import Node
from leg_controller.gaitGenerator import Gait
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import LinkStates
import numpy as np
import math

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
        
        # Subscribe to Gazebo joint states for diagnostics
        self.gazebo_joint_positions = {}
        self.body_z = 0.0
        self.last_commanded = {}
        self.diag_counter = 0
        
        self.sub_joint_states = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10)
        
        self.sub_link_states = self.create_subscription(
            LinkStates,
            '/gazebo/link_states',
            self.link_states_callback,
            10)
        
        self.timer = self.create_timer(0.003, self.timer_callback)

        self.get_logger().info('The main Node has started.')

    def timer_callback(self):
        if self.gait_msg.cmd == "ZERO":
            theta_zero = np.array([[0,  200, -60],   
                                   [0, -200,  60],    
                                   [0, -200,  60],   
                                   [0,  200, -60]])  
            self.gait.serial_publish.publish_simulation(theta_zero)
        else:
            # Tick the trajectory for forward motion
            self.gait.tick_trajectory()
        
        # Diagnostic logging every ~1 second
        self.diag_counter += 1
        if self.diag_counter % 333 == 0:
            sp = self.gait.serial_publish
            cmd_dict = getattr(sp, 'last_commanded', {})
            cmd_lf2 = cmd_dict.get('joint_lf_2', 0.0)
            act_lf2 = sp.actual_positions.get('joint_lf_2', 0.0)
            error = cmd_lf2 - act_lf2
            self.get_logger().info(
                f'[DIAG] body_z={self.body_z:.4f}m | '
                f'joint_lf_2: cmd={math.degrees(cmd_lf2):.1f}° act={math.degrees(act_lf2):.1f}° err={math.degrees(error):.2f}° | '
                f'mode={self.gait_msg.cmd}')

    def joint_states_callback(self, msg):
        pass  # Handled by serialPublish now
    
    def link_states_callback(self, msg):
        for i, name in enumerate(msg.name):
            if 'body_link' in name:
                self.body_z = msg.pose[i].position.z

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
        # self.get_logger().info(f'[Sub]: {msg.data}')   
        self.gait_msg.cmd, self.gait_msg.step = self.parse_command(msg.data)
        self.gait.init_trajectory()

def main(args=None):
    rclpy.init(args=args)
    node = LegController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()