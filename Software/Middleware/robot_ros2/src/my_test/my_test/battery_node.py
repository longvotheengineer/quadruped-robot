import rclpy
from rclpy.node import Node
from my_test.battery_utils import convert_to_percentage

class BatteryNode(Node):
    def __init__(self):
        super().__init__('battery_node')
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Battery node started')

    def timer_callback(self):
        voltage = 10.5  # Simulated voltage reading
        percentage = convert_to_percentage(voltage)
        self.get_logger().info(f'Battery Voltage: {voltage}V -> Status: {percentage:.1f}%')

def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
