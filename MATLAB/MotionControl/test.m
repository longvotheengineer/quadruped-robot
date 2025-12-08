clear; close all; clc;

sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);

robot_length = struct('base_length', 0.5,...
                'base_width', 0.2,...
                'L1', 0.05,...
                'L2', 0.15,...
                'L3', 0.1);

robot_config = struct('leg_type', "",...
                      'joint_angle', "");
robot_config.robot_length = robot_length;

robot_motion.gait = "ZERO";
% robot_motion.gait = "FORWARD";

robot_config.leg_type = "left-front";
theta_i = Gait(robot_motion, robot_config);
theta_i_LF = theta_i;

robot_config.leg_type = "left-behind";
theta_i = Gait(robot_motion, robot_config);
theta_i_LB = theta_i;

robot_config.leg_type = "right-front";
theta_i = Gait(robot_motion, robot_config);
theta_i_RF = theta_i;

robot_config.leg_type = "right-behind";
theta_i = Gait(robot_motion, robot_config);
theta_i_RB = theta_i;

if (clientID>-1)
    disp('Connected to remote API server');

    % joint handle
    h_LF = [0, 0, 0];
    h_LB = [0, 0, 0];
    h_RF = [0, 0, 0];
    h_RB = [0, 0, 0];
    [r_LF, h_LF(1)] = sim.simxGetObjectHandle(clientID, 'Revolute_joint_20', sim.simx_opmode_blocking);
    [r_LF, h_LF(2)] = sim.simxGetObjectHandle(clientID, 'Revolute_joint_21', sim.simx_opmode_blocking);
    [r_LF, h_LF(3)] = sim.simxGetObjectHandle(clientID, 'Revolute_joint_22', sim.simx_opmode_blocking);
    [r_LB, h_LB(1)] = sim.simxGetObjectHandle(clientID, 'Revolute_joint_30', sim.simx_opmode_blocking);
    [r_LB, h_LB(2)] = sim.simxGetObjectHandle(clientID, 'Revolute_joint_31', sim.simx_opmode_blocking);
    [r_LB, h_LB(3)] = sim.simxGetObjectHandle(clientID, 'Revolute_joint_32', sim.simx_opmode_blocking);
    [r_RF, h_RF(1)] = sim.simxGetObjectHandle(clientID, 'Revolute_joint_10', sim.simx_opmode_blocking);
    [r_RF, h_RF(2)] = sim.simxGetObjectHandle(clientID, 'Revolute_joint_11', sim.simx_opmode_blocking);
    [r_RF, h_RF(3)] = sim.simxGetObjectHandle(clientID, 'Revolute_joint_12', sim.simx_opmode_blocking);
    [r_RB, h_RB(1)] = sim.simxGetObjectHandle(clientID, 'Revolute_joint_00', sim.simx_opmode_blocking);
    [r_RB, h_RB(2)] = sim.simxGetObjectHandle(clientID, 'Revolute_joint_01', sim.simx_opmode_blocking);
    [r_RB, h_RB(3)] = sim.simxGetObjectHandle(clientID, 'Revolute_joint_02', sim.simx_opmode_blocking);

    for j = 1 : size(theta_i,1)
        joint_pos_left_front = [theta_i_LF(j,1), theta_i_LF(j,2), theta_i_LF(j,3)]; 
        joint_pos_right_behind = [theta_i_RB(j,1), theta_i_RB(j,2), theta_i_RB(j,3)];
        joint_pos_left_behind = [theta_i_LB(j,1), theta_i_LB(j,2), theta_i_LB(j,3)];
        joint_pos_right_front = [theta_i_RF(j,1), theta_i_RF(j,2), theta_i_RF(j,3)]; 
        for i = 1 : 3
            sim.simxSetJointTargetPosition(clientID, h_LF(i), joint_pos_left_front(i), sim.simx_opmode_streaming);
            sim.simxSetJointTargetPosition(clientID, h_RB(i), joint_pos_right_behind(i), sim.simx_opmode_streaming);                  
            sim.simxSetJointTargetPosition(clientID, h_LB(i), joint_pos_left_behind(i), sim.simx_opmode_streaming);
            sim.simxSetJointTargetPosition(clientID, h_RF(i), joint_pos_right_front(i), sim.simx_opmode_streaming);          
            pause(0.315);
        end
    end

    robot_motion.gait = "FORWARD";

    robot_config.leg_type = "left-front";
    theta_i = Gait(robot_motion, robot_config);
    theta_i_LF = theta_i;
    
    robot_config.leg_type = "left-behind";
    theta_i = Gait(robot_motion, robot_config);
    theta_i_LB = theta_i;
    
    robot_config.leg_type = "right-front";
    theta_i = Gait(robot_motion, robot_config);
    theta_i_RF = theta_i;
    
    robot_config.leg_type = "right-behind";
    theta_i = Gait(robot_motion, robot_config);
    theta_i_RB = theta_i;
    while true
        for j = 1 : size(theta_i,1)
            joint_pos_left_front = [theta_i_LF(j,1), theta_i_LF(j,2), theta_i_LF(j,3)]; 
            joint_pos_right_behind = [theta_i_RB(j,1), theta_i_RB(j,2), theta_i_RB(j,3)];
            for i = 1 : 3
                sim.simxSetJointTargetPosition(clientID, h_LF(i), joint_pos_left_front(i), sim.simx_opmode_streaming);
                sim.simxSetJointTargetPosition(clientID, h_RB(i), joint_pos_right_behind(i), sim.simx_opmode_streaming);
            end
            pause(0.005);             
        end
        for j = 1 : size(theta_i,1)
            joint_pos_left_behind = [theta_i_LB(j,1), theta_i_LB(j,2), theta_i_LB(j,3)];
            joint_pos_right_front = [theta_i_RF(j,1), theta_i_RF(j,2), theta_i_RF(j,3)];
            for i = 1 : 3
                sim.simxSetJointTargetPosition(clientID, h_LB(i), joint_pos_left_behind(i), sim.simx_opmode_streaming);
                sim.simxSetJointTargetPosition(clientID, h_RF(i), joint_pos_right_front(i), sim.simx_opmode_streaming);
            end
            pause(0.005);
        end
    end
else
    disp('Failed connecting to remote API server');
end
    sim.delete(); % call the destructor!
    
    disp('Program ended');