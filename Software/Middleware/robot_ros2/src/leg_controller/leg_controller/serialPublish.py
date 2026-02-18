import struct
import math
import rclpy
from std_msgs.msg import String
from sensor_msgs.msg import JointState

class SerialPublish:
    def __init__(self, node):
        self.pub_serial = node.create_publisher(String, '/angle_servo', 10)
        self.pub_simulation = node.create_publisher(JointState, '/joint_states', 10)
        self.get_logger = node.get_logger()
                
    def convert_to_serial(self, theta):        
        theta1 = -theta.th1 + 90
        theta2 =  theta.th2 - 90
        theta3 =  theta.th3 + 90

        header = [0xAA, 0x55]   
        footer = [0xFF]         
        
        # Convert Dec - Float to Hex - Int
        angle = [int(theta1), int(theta2), int(theta3)]
        # Use struct to pack the integers into bytes
        angle = struct.pack('<HHH', *angle)
        # Combine the parts into a full message
        msg = bytes(header) + angle + bytes(footer)
        # Convert to space-separated hex string
        msg = ' '.join(f'{byte:02X}' for byte in msg)
        self.get_logger.info(f'[serialPublish] Converted message: {msg}')
        return msg

    def publish_simulation(self, theta):
        msg = JointState()
        msg.header.stamp = rclpy.clock.Clock().now().to_msg()
        msg.name = ['joint_lf_1', 'joint_lf_2', 'joint_lf_3', 
                    'joint_lb_1', 'joint_lb_2', 'joint_lb_3',
                    'joint_rf_1', 'joint_rf_2', 'joint_rf_3',
                    'joint_rb_1', 'joint_rb_2', 'joint_rb_3'] 
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
        msg.position = [theta_lf_1, theta_lf_2, theta_lf_3 + math.radians(180), 
                        theta_lb_1, theta_lb_2, theta_lb_3 + math.radians(180),
                        theta_rf_1, theta_rf_2, theta_rf_3 + math.radians(180),
                        theta_rb_1, theta_rb_2, theta_rb_3 + math.radians(180)]
        # msg.position = [math.radians(0), math.radians(90), math.radians(180+0),
        #                 math.radians(0), math.radians(90), math.radians(180+0),
        #                 math.radians(0), math.radians(90), math.radians(180+0),
        #                 math.radians(0), math.radians(90), math.radians(180+0)] # Zero pose testing
        self.pub_simulation.publish(msg)
        # self.get_logger.info(
        #     f'[serialPublish] Published to simulation: '
        #     f'joint_lf_1={theta_lf_1}, '
        #     f'joint_lf_2={theta_lf_2}, '
        #     f'joint_lf_3={theta_lf_3}, '
        #     f'joint_lb_1={theta_lb_1}, '
        #     f'joint_lb_2={theta_lb_2}, '
        #     f'joint_lb_3={theta_lb_3}, '
        #     f'joint_rf_1={theta_rf_1}, '
        #     f'joint_rf_2={theta_rf_2}, '
        #     f'joint_rf_3={theta_rf_3}, '
        #     f'joint_rb_1={theta_rb_1}, '
        #     f'joint_rb_2={theta_rb_2}, '
        #     f'joint_rb_3={theta_rb_3}'
        # )

    def publish_message(self, theta):
        # msg = String()
        # msg.data = self.convert_to_serial(theta)
        # self.pub_serial.publish(msg)   

        self.publish_simulation(theta)     