import math

class Kinematics:
    def __init__(self, node, robot_length):
        self.get_logger = node.get_logger()
        self.L  = robot_length.L
        self.W  = robot_length.W
        self.l1 = robot_length.l1
        self.l2 = robot_length.l2
        self.l3 = robot_length.l3
    
    def forward(self, theta1, theta2, theta3):
        # Convert angles from degree to radian
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
        self.get_logger.info(
            f'[kinematics] FK Orientation: '
            f'x={x},'
            f'y={y},'
            f'z={z}')

        return x, y, z
    
    def inverse(self, x, y, z, leg_type):
        px = x
        py = y
        pz = z

        match leg_type:
            case "left-front":
                x       =  pz
                y       =  self.W / 2 - py
                z       = -self.L / 2 + px
                theta1  =  math.atan2(-x, y) + math.atan2(-math.sqrt(x**2 + y**2 - self.l1**2), -self.l1)
                sign_s3 = -1
                sign_p2 =  1
            case "left-behind":
                x       =  pz
                y       =  self.W / 2 - py
                z       =  self.L / 2 + px
                theta1  =  math.atan2(-x, y) + math.atan2(-math.sqrt(x**2 + y**2 - self.l1**2), -self.l1)
                sign_s3 =  1
                sign_p2 =  1
            case "right-front":
                x       =  pz
                y       = -self.W / 2 - py
                z       = -self.L / 2 + px
                theta1  =  math.atan2(-x, y) + math.atan2(-math.sqrt(x**2 + y**2 - self.l1**2), self.l1)
                sign_s3 =  1
                sign_p2 = -1
            case "right-behind":
                x       =  pz
                y       = -self.W / 2 - py
                z       =  self.L / 2 + px
                theta1  =  math.atan2(-x, y) + math.atan2(-math.sqrt(x**2 + y**2 - self.l1**2), self.l1)
                sign_s3 = -1
                sign_p2 = -1
            case _:
                return None

        p1      = x * math.cos(theta1) + y * math.sin(theta1)
        p2      = sign_p2 * z  
        c3      = (p1**2 + p2**2 - self.l2**2 - self.l3**2) / (2 * self.l2 * self.l3)
        s3      = sign_s3 * math.sqrt(1 - c3**2)
        theta3  = math.atan2(s3, c3)
        theta2  = math.atan2(p2, p1) - math.atan2(self.l3 * s3, self.l2 + self.l3 * c3)
        
        theta1  = round(math.degrees(theta1), 1)        
        theta2  = round(math.degrees(theta2), 1)
        theta2  = theta2 % 360
        theta3  = round(math.degrees(theta3), 1)
        self.get_logger.info(
            f'[kinematics] IK Angles: '
            f'leg_type={leg_type}, '
            f'theta1={theta1},' 
            f'theta2={theta2},'
            f'theta3={theta3}')                 

        return theta1, theta2, theta3