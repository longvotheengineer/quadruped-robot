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
        msg.name = ['joint1', 'joint2', 'joint3']
        th1 = math.radians(theta.th1)
        th2 = math.radians(theta.th2)
        th3 = math.radians(theta.th3)
        msg.position = [th1, th2, th3 + math.radians(180)] 
        # msg.position = [math.radians(0), math.radians(90), math.radians(180+90)] # For zero pose testing
        self.pub_simulation.publish(msg)
        self.get_logger.info(
            f'[serialPublish] Published to simulation: '
            f'joint1={th1}, '
            f'joint2={th2}, '
            f'joint3={th3 + math.radians(180)}'
        )

    def publish_message(self, theta):
        msg = String()
        msg.data = self.convert_to_serial(theta)
        self.pub_serial.publish(msg)   

        self.publish_simulation(theta)     