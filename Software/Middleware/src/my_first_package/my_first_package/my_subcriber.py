import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class MySubcriber(Node):
    def __init__(self):
        super().__init__('listener_node')
        self.subcription = self.create_subscription(
            Float32,
            '/motor_angle',
            self.listener_callback,
            10
        )
        self.subcription
    
    def listener_callback(self, msg):
        angle = msg.data
        self.get_logger().info(f'I heard: {angle}')

def main(args=None):
    rclpy.init(args=args)
    node = MySubcriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()