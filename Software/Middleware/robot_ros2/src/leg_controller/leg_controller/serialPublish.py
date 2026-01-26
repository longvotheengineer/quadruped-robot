import struct
from std_msgs.msg import String

class SerialPublish:
    def __init__(self, node):
        self.publisher_ = node.create_publisher(String, '/angle_servo', 10)
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

    def publish_message(self, theta):
        msg = String()
        msg.data = self.convert_to_serial(theta)
        self.publisher_.publish(msg)        