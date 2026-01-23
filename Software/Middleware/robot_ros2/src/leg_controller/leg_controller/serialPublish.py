import struct
from std_msgs.msg import String

class SerialPublish:
    def __init__(self, node, theta1, theta2, theta3):
        self.publisher_ = node.create_publisher(String, '/angle_servo', 10)
        self.get_logger = node.get_logger()
        self.theta1 = -theta1 + 90
        self.theta2 =  theta2 - 90
        self.theta3 =  theta3 + 90
                
    def convert_to_serial(self):
        self.theta1 = -self.theta1 + 90
        self.theta2 =  self.theta2 - 90
        self.theta3 =  self.theta3 + 90

        header = [0xAA, 0x55]   
        footer = [0xFF]         
        
        # Convert Dec - Float to Hex - Int
        angle = [int(self.theta1), int(self.theta2), int(self.theta3)]
        # Use struct to pack the integers into bytes
        angle = struct.pack('<HHH', *angle)
        # Combine the parts into a full message
        msg = bytes(header) + angle + bytes(footer)
        # Convert to space-separated hex string
        msg = ' '.join(f'{byte:02X}' for byte in msg)
        self.get_logger.info(f'[serialPublish] Converted message: {msg}')
        return msg

    def publish_message(self):
        msg = String()
        msg.data = self.convert_to_serial()
        self.publisher_.publish(msg)        