import math

class Kinematics:
    def __init__(self, node, robot_length):
        self.get_logger = node.get_logger()
        self.robot_length = robot_length
    
    def forward(self, theta1, theta2, theta3):
        # Convert angles from degree to radian
        theta1 = math.radians(theta1)
        theta2 = math.radians(theta2)
        theta3 = math.radians(theta3)
        
        # Tip orientation relative to the leg base frame
        px =  self.robot_length.l1 * math.sin(theta1) \
            - self.robot_length.l3 * (math.cos(theta1) * math.sin(theta2) * math.sin(theta3) \
            - math.cos(theta1) * math.cos(theta2) * math.cos(theta3)) \
            + self.robot_length.l2 * math.cos(theta1) * math.cos(theta2)
        py =  self.robot_length.l2 * math.cos(theta2) * math.sin(theta1) \
            - self.robot_length.l1 * math.cos(theta1) \
            - self.robot_length.l3 * (math.sin(theta1) * math.sin(theta2) * math.sin(theta3) 
            - math.cos(theta2) * math.cos(theta3) * math.sin(theta1))
        pz =  self.robot_length.l3 * math.sin(theta2 + theta3) + self.robot_length.l2 * math.sin(theta2)

        # Tip orientation relative to the body center frame
        x = self.robot_length.L / 2 + pz
        y = self.robot_length.W / 2 - py
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
                y       =  self.robot_length.W / 2 - py
                z       = -self.robot_length.L / 2 + px
                theta1  =  math.atan2(-x, y) \
                         + math.atan2(-math.sqrt(x**2 + y**2 - self.robot_length.l1**2), -self.robot_length.l1)
                theta1  = theta1  # negate for URDF left joint frame convention
                sign_s3 = -1
                sign_p2 =  1
            case "left-behind":
                x       =  pz
                y       =  self.robot_length.W / 2 - py
                z       =  self.robot_length.L / 2 + px
                theta1  =  math.atan2(-x, y) \
                         + math.atan2(-math.sqrt(x**2 + y**2 - self.robot_length.l1**2), -self.robot_length.l1)
                theta1  = theta1  # negate for URDF left joint frame convention
                sign_s3 =  -1
                sign_p2 =  1
            case "right-front":
                x       =  pz
                y       = -self.robot_length.W / 2 - py
                z       = -self.robot_length.L / 2 + px
                theta1  =  math.atan2(-x, y) \
                         + math.atan2(-math.sqrt(x**2 + y**2 - self.robot_length.l1**2), self.robot_length.l1)
                sign_s3 =  1
                sign_p2 = -1
            case "right-behind":
                x       =  pz
                y       = -self.robot_length.W / 2 - py
                z       =  self.robot_length.L / 2 + px
                theta1  =  math.atan2(-x, y) \
                         + math.atan2(-math.sqrt(x**2 + y**2 - self.robot_length.l1**2), self.robot_length.l1)
                sign_s3 = 1
                sign_p2 = -1
            case _:
                return None

        p1      = x * math.cos(theta1) + y * math.sin(theta1)
        p2      = sign_p2 * z  
        c3      = (p1**2 + p2**2 - self.robot_length.l2**2 - self.robot_length.l3**2) / (2 * self.robot_length.l2 * self.robot_length.l3)
        s3      = sign_s3 * math.sqrt(1 - c3**2)
        theta3  = math.atan2(s3, c3)
        theta2  = math.atan2(p2, p1) \
                - math.atan2(self.robot_length.l3 * s3, self.robot_length.l2 + self.robot_length.l3 * c3)
        
        theta1  = round(math.degrees(theta1), 1)        
        theta2  = round(math.degrees(theta2), 1)
        # Normalize theta2 based on each leg's physical rotation direction
        # if leg_type in ("right-behind"):
        #     if theta2 < 0:
        #         theta2 += 360       # e.g. -152° → 208°
        # elif leg_type in ("right-front"):
        #     if theta2 > 0:
        #         theta2 -= 360       # e.g. 152° → -208°
        theta3  = round(math.degrees(theta3), 1)                

        return theta1, theta2, theta3