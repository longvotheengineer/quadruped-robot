import math

class Kinematics:
    def __init__(self, node, L, W, l1, l2, l3):
        self.get_logger = node.get_logger()
        self.L  = L
        self.W  = W
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
    
    def forward_kinematics(self, theta1, theta2, theta3):
        # Convert angles from degrees to radians
        theta1 = math.radians(theta1)
        theta2 = math.radians(theta2)
        theta3 = math.radians(theta3)
        
        # Tip orientation relative to the leg base frame
        px = self.l1 * math.sin(theta1) - self.l3 * (math.cos(theta1) * math.sin(theta2) * math.sin(theta3) - math.cos(theta1) * math.cos(theta2) * math.cos(theta3)) + self.l2 * math.cos(theta1) * math.cos(theta2)
        py = self.l2 * math.cos(theta2) * math.sin(theta1) - self.l1 * math.cos(theta1) - self.l3 * (math.sin(theta1) * math.sin(theta2) * math.sin(theta3) - math.cos(theta2) * math.cos(theta3) * math.sin(theta1))
        pz = self.l3 * math.sin(theta2 + theta3) + self.l2 * math.sin(theta2)

        # Tip orientation relative to the body center frame
        x = self.L / 2 + pz
        y = self.W / 2 - py
        z = px

        self.get_logger().info(f'[kinematics] FK Orientation: x={x}, y={y}, z={z}')

        return x, y, z