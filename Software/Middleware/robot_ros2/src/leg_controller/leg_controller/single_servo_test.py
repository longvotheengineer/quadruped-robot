import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SingleServoTestNode(Node):
    def __init__(self):
        super().__init__('single_servo_test_node')
        self.publisher_ = self.create_publisher(String, '/angle_servo', 10)
        self.angle = ["AA 55 00 00 00 00 00 00 FF", 
                      "AA 55 03 84 03 84 03 84 FF",
                      "AA 55 05 05 05 05 05 05 FF",]
        self.counter = 0
        
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.get_logger().info('SingleServoTestNode has started.')

    def timer_callback(self):
        current_angle = self.angle[self.counter]
        msg = String()
        msg.data = current_angle
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {current_angle} degrees')
        self.counter += 1
        if self.counter >= len(self.angle):
            self.counter = 0

def main(args=None):
    rclpy.init(args=args)
    node = SingleServoTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()