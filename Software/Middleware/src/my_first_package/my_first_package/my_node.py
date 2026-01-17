import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Float32

class MyNode(Node):
    def __init__(self):
        super().__init__('first_node')
        self.publisher_ = self.create_publisher(Float32, '/motor_angle', 10)
        self.create_timer(0.1, self.timer_callback)
        self.time_counter = 0.0
    
    def timer_callback(self):
        angle = 10.0 * math.sin(self.time_counter * 2)

        msg = Float32()
        msg.data = angle
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing Angle: {angle}')
        self.time_counter += 0.1

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()