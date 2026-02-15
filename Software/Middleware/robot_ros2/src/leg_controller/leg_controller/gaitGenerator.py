import numpy as np
import time
from dataclasses import dataclass
from leg_controller.kinematics import Kinematics
from leg_controller.serialPublish import SerialPublish

@dataclass(frozen=True)
class Waypoint:
    zero   : int
    stance : int
    swing  : int

@dataclass(frozen=True)
class RobotLength:
    L : float
    W : float
    l1: float   
    l2: float
    l3: float

@dataclass
class Theta:
    th1: float
    th2: float
    th3: float

class Gait:
    def __init__(self, node, gait_msg):
        self.node = node
        self.get_logger = node.get_logger()

        robot_length = RobotLength(L = 120, W = 90, l1 = 20, l2 = 80, l3 = 80)
        self.kinematics = Kinematics(self.node, robot_length)

        self.serial_publish = SerialPublish(self.node)

        self.gait_msg  = gait_msg

        self.waypoint  = Waypoint(20, 30, 20)

    def generate(self, leg_type):
        match self.gait_msg.cmd:
            # case "ZERO":
            case "FORWARD":
                match leg_type:
                    case "left-front":
                        pos_A = [40, 60, -150]
                        pos_B = [40, 60, -130]
                        pos_C = [70, 60, -130]
                        pos_D = [70, 60, -150]
                    # case "left-behind":
                    #     pos_A = [-40, 60, -150]
                    #     pos_B = [-40, 60, -130]
                    #     pos_C = [-10, 60, -130]
                    #     pos_D = [-10, 60, -150]
                    # case "right-front":
                    #     pos_A = [40, -60, -150]
                    #     pos_B = [40, -60, -130]
                    #     pos_C = [70, -60, -130]
                    #     pos_D = [70, -60, -150]
                    # case "right-behind":
                    #     pos_A = [-40, -60, -150]
                    #     pos_B = [-40, -60, -130]
                    #     pos_C = [-10, -60, -130]
                    #     pos_D = [-10, -60, -150]
                    case _:
                        return None
            # case "BACKWARD":
            # case "LEFT":
            # case "RIGHT":
            case _: 
                return None
        
        waypoint_AB = np.linspace(pos_A, pos_B, num=self.waypoint.swing, axis=0)
        waypoint_BC = np.linspace(pos_B, pos_C, num=self.waypoint.swing, axis=0)
        waypoint_CD = np.linspace(pos_C, pos_D, num=self.waypoint.swing, axis=0)
        waypoint_DA = np.linspace(pos_D, pos_A, num=self.waypoint.stance, axis=0)
        waypoint    = np.vstack([waypoint_AB, waypoint_BC, waypoint_CD, waypoint_DA])

        waypoint_row = waypoint.shape[0]
        waypoint_col = waypoint.shape[1]
        theta_i = np.zeros((waypoint_row, waypoint_col))
        for i in range(waypoint_row):
            px = waypoint[i, 0]
            py = waypoint[i, 1]
            pz = waypoint[i, 2]
            self.get_logger.info(
                f'[gaitGenerator] px={px}, '
                f'py={py}, '
                f'pz={pz}')
            theta_i[i, :] = self.kinematics.inverse(px, py, pz, leg_type)
        
        match leg_type:
            case "left-front":
                shift = round(waypoint_row * 0.00)
            case "right-behind":
                shift = round(waypoint_row * 0.00)
            case "left-behind":
                shift = round(waypoint_row * 0.50)
            case "right-front":
                shift = round(waypoint_row * 0.50)
            case _:
                return None
            
        theta_i = np.roll(theta_i, shift, axis=1)       
        return theta_i   

    def change(self):        
        match self.gait_msg.cmd:
            case "ZERO":                
                # leg_type = "left-front"
                # theta = self.generate(leg_type)
                # theta_i[0] = theta[0]

                # leg_type = "left-behind"
                # theta = self.generate(leg_type)
                # theta_i[1] = theta[1]

                # leg__type = "right-front"
                # theta = self.generate(leg_type)
                # theta_i[2] = theta[2]

                # leg_type = "right-behind"
                # theta = self.generate(leg_type)
                # theta_i[3] = theta[3]

                # return theta_i
                return None
            case _:
                theta_i     = np.empty(4, dtype=object) 

                leg_type    = "left-front"
                theta_i[0]  = self.generate(leg_type)
                # leg_type    = "left-behind"
                # theta_i[1]  = self.generate(leg_type)
                # leg_type    = "right-front"
                # theta_i[2]  = self.generate(leg_type)
                # leg_type    = "right-behind"   
                # theta_i[3]  = self.generate(leg_type)   

                return theta_i   
    
    def control(self):
        theta_i = self.change()

        match self.gait_msg.cmd:
            case "ZERO":
                return None
            case _:
                gait_step = 0

                while gait_step < self.gait_msg.step:
                    for i in range(theta_i[0].shape[0]):
                        pos_LF = [theta_i[0][i, 0], theta_i[0][i, 1], theta_i[0][i, 2]]
                        # pos_LB = [theta_i[1][i, 0], theta_i[1][i, 1], theta_i[1][i, 2]]
                        # pos_RF = [theta_i[2][i, 0], theta_i[2][i, 1], theta_i[2][i, 2]]
                        # pos_RB = [theta_i[3][i, 0], theta_i[3][i, 1], theta_i[3][i, 2]]
                        # pos    = np.vstack([pos_LF, pos_LB, pos_RF, pos_RB])
                        
                        theta = Theta(pos_LF[0], pos_LF[1], pos_LF[2])
                        self.serial_publish.publish_message(theta)  
                        time.sleep(0.05)                      
                   
                    gait_step += 1