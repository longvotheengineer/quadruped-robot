import rclpy
from rclpy.node import Node
from leg_controller.kinematics import Kinematics
from leg_controller.serialPublish import SerialPublish

class LegController(Node):
    def __init__(self):
        super().__init__('leg_controller_node')
        
        self.kinematics = Kinematics(self, L = 120, W = 90, l1 = 20, l2 = 80, l3 = 80)
        self.serial_publish = SerialPublish(self, 0, 0, 0)
        
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.get_logger().info('The main Node has started.')

        self.cnt = 0

    def timer_callback(self):
        if self.cnt == 0:    
            # theta1, theta2, theta3 = 0, 0, 0
            # x, y, z = self.kinematics.forward_kinematics(theta1, theta2, theta3)
            # self.serial_publish.theta1 = theta1
            # self.serial_publish.theta2 = theta2
            # self.serial_publish.theta3 = theta3    

            x, y, z = 60, 45, -140         
            theta1, theta2, theta3 = self.kinematics.inverse_kinematics(x, y, z)
            self.serial_publish.theta1 = theta1
            self.serial_publish.theta2 = theta2
            self.serial_publish.theta3 = theta3
            self.serial_publish.publish_message()
        # elif self.cnt == 1:
        #     theta1, theta2, theta3 = 90, 90, 90
        #     x, y, z = self.kinematics.forward_kinematics(theta1, theta2, theta3)
        #     self.serial_publish.theta1 = theta1
        #     self.serial_publish.theta2 = theta2
        #     self.serial_publish.theta3 = theta3
        #     self.serial_publish.publish_message()
        # elif self.cnt == 2:
        #     x, y, z = self.kinematics.forward_kinematics(123.4, 123.4, 123.4)
        
        self.cnt += 1
        if self.cnt > 2:
            self.cnt = 0

def main(args=None):
    rclpy.init(args=args)
    node = LegController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()