function control_gait(robot_config, robot_motion, sim, clientID, sensor_data, slamObj, axMap, state)
    step_time_zero = 0.001;
    step_time      = 0.01;
   
    persistent update_cnt
    if isempty(update_cnt)
        update_cnt = 0;
    end
    

    % joint handle
    h_LF = [0, 0, 0];
    h_LB = [0, 0, 0];
    h_RF = [0, 0, 0];
    h_RB = [0, 0, 0];
    [~, h_LF(1)] = sim.simxGetObjectHandle(clientID, 'Revolute_joint_20', sim.simx_opmode_blocking);
    [~, h_LF(2)] = sim.simxGetObjectHandle(clientID, 'Revolute_joint_21', sim.simx_opmode_blocking);
    [~, h_LF(3)] = sim.simxGetObjectHandle(clientID, 'Revolute_joint_22', sim.simx_opmode_blocking);
    [~, h_LB(1)] = sim.simxGetObjectHandle(clientID, 'Revolute_joint_30', sim.simx_opmode_blocking);
    [~, h_LB(2)] = sim.simxGetObjectHandle(clientID, 'Revolute_joint_31', sim.simx_opmode_blocking);
    [~, h_LB(3)] = sim.simxGetObjectHandle(clientID, 'Revolute_joint_32', sim.simx_opmode_blocking);
    [~, h_RF(1)] = sim.simxGetObjectHandle(clientID, 'Revolute_joint_10', sim.simx_opmode_blocking);
    [~, h_RF(2)] = sim.simxGetObjectHandle(clientID, 'Revolute_joint_11', sim.simx_opmode_blocking);
    [~, h_RF(3)] = sim.simxGetObjectHandle(clientID, 'Revolute_joint_12', sim.simx_opmode_blocking);
    [~, h_RB(1)] = sim.simxGetObjectHandle(clientID, 'Revolute_joint_00', sim.simx_opmode_blocking);
    [~, h_RB(2)] = sim.simxGetObjectHandle(clientID, 'Revolute_joint_01', sim.simx_opmode_blocking);
    [~, h_RB(3)] = sim.simxGetObjectHandle(clientID, 'Revolute_joint_02', sim.simx_opmode_blocking);
    
    [theta_i_LF, theta_i_LB, theta_i_RF, theta_i_RB, theta_i] = change_gait(robot_config, robot_motion);

    if ismember(robot_motion.gait, ["TURN_LEFT", "TURN_RIGHT"])
        robot_motion.step = round(robot_motion.step / 25);
    end
    switch robot_motion.gait
        case "ZERO"  
            for j = 1 : size(theta_i{1},1)
                joint_pos_left_front   = [theta_i_LF(j,1), theta_i_LF(j,2), theta_i_LF(j,3)]; 
                joint_pos_right_behind = [theta_i_RB(j,1), theta_i_RB(j,2), theta_i_RB(j,3)];
                joint_pos_left_behind  = [theta_i_LB(j,1), theta_i_LB(j,2), theta_i_LB(j,3)];
                joint_pos_right_front  = [theta_i_RF(j,1), theta_i_RF(j,2), theta_i_RF(j,3)]; 
                for i = 1 : 3    
                    sim.simxSetJointTargetPosition(clientID, h_LB(i), joint_pos_left_behind (i), sim.simx_opmode_streaming);
                    sim.simxSetJointTargetPosition(clientID, h_RB(i), joint_pos_right_behind(i), sim.simx_opmode_streaming);       
                    sim.simxSetJointTargetPosition(clientID, h_LF(i), joint_pos_left_front  (i), sim.simx_opmode_streaming);
                    sim.simxSetJointTargetPosition(clientID, h_RF(i), joint_pos_right_front (i), sim.simx_opmode_streaming);          
                    pause(step_time_zero);     
                end
            end  
        otherwise
            gait_step = 0;
            while (gait_step < robot_motion.step)
                for j = 1 : size(theta_i,1)
                    joint_pos_left_front   = [theta_i_LF(j,1), theta_i_LF(j,2), theta_i_LF(j,3)]; 
                    joint_pos_right_behind = [theta_i_RB(j,1), theta_i_RB(j,2), theta_i_RB(j,3)];
                    joint_pos_left_behind  = [theta_i_LB(j,1), theta_i_LB(j,2), theta_i_LB(j,3)];
                    joint_pos_right_front  = [theta_i_RF(j,1), theta_i_RF(j,2), theta_i_RF(j,3)];
                    for i = 1 : 3
                        sim.simxSetJointTargetPosition(clientID, h_LF(i), joint_pos_left_front  (i), sim.simx_opmode_streaming);
                        sim.simxSetJointTargetPosition(clientID, h_RB(i), joint_pos_right_behind(i), sim.simx_opmode_streaming);
                        sim.simxSetJointTargetPosition(clientID, h_LB(i), joint_pos_left_behind (i), sim.simx_opmode_streaming);
                        sim.simxSetJointTargetPosition(clientID, h_RF(i), joint_pos_right_front (i), sim.simx_opmode_streaming);

                        % sensor_data = read_sensor_data(clientID, sim, sensor_data);                        
                        % disp( sensor_data.ax);
                    end
                    update_cnt = update_cnt + 1;
                    [state, sensor_data] = process_map(clientID, sim, sensor_data, slamObj, axMap, update_cnt, state);
                    pause(step_time);   
                end  
                gait_step = gait_step + 1;
            end
    end   
end

function [theta_i_LF, theta_i_LB, theta_i_RF, theta_i_RB, theta_i] = change_gait(robot_config, robot_motion)
    switch robot_motion.gait
        case "ZERO"           
            robot_config.leg_type = "left-front";
            theta_i = Gait(robot_motion, robot_config);
            theta_i_LF = theta_i{1};
            
            robot_config.leg_type = "left-behind";
            theta_i = Gait(robot_motion, robot_config);
            theta_i_LB = theta_i{2};
            
            robot_config.leg_type = "right-front";
            theta_i = Gait(robot_motion, robot_config);
            theta_i_RF = theta_i{3};
            
            robot_config.leg_type = "right-behind";
            theta_i = Gait(robot_motion, robot_config);
            theta_i_RB = theta_i{4};
        otherwise
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
    end
end