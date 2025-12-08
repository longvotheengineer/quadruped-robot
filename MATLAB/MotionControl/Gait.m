function theta_i = Gait(robot_motion, robot_config)
    switch robot_motion.gait
        case "ZERO"
            [x, y, z] = ForwardKinematics(0, 0, 0, robot_config);
            switch robot_config.leg_type
                case "left-front"
                    pos_A = [x, y, z];
                    pos_B = [0.25, 0.15, -0.24];
                case "left-behind"
                    pos_A = [x, y, z];
                    pos_B = [-0.25, 0.15, -0.24];
                case "right-front"
                    pos_A = [x, y, z];
                    pos_B = [0.25, -0.15, -0.24];
                case "right-behind"
                    pos_A = [x, y, z];
                    pos_B = [-0.25, -0.15, -0.24];
                otherwise
            end
        case "FORWARD"
            switch robot_config.leg_type
                case "left-front"
                    pos_A = [0.25, 0.15, -0.24];
                    pos_B = [0.25, 0.15, -0.20];
                    pos_C = [0.29, 0.15, -0.20];
                    pos_D = [0.29, 0.15, -0.24];
                case "left-behind"
                    pos_A = [-0.25, 0.15, -0.24];
                    pos_B = [-0.25, 0.15, -0.20];
                    pos_C = [-0.21, 0.15, -0.20];
                    pos_D = [-0.21, 0.15, -0.24];
                case "right-front"
                    pos_A = [0.25, -0.15, -0.24];
                    pos_B = [0.25, -0.15, -0.20];
                    pos_C = [0.29, -0.15, -0.20];
                    pos_D = [0.29, -0.15, -0.24];
                case "right-behind"
                    pos_A = [-0.25, -0.15, -0.24];
                    pos_B = [-0.25, -0.15, -0.20];
                    pos_C = [-0.21, -0.15, -0.20];
                    pos_D = [-0.21, -0.15, -0.24];
                otherwise
            end
        case "BACKWARD"
        case "TURN_LEFT"
        case "TURN_RIGHT"
        otherwise
    end
    
    waypoint_n = 10;
    if robot_motion.gait == "ZERO"
        [theta1, theta2, theta3] = InverseKinematics(pos_A(1), pos_A(2), pos_A(3), robot_config);
        theta_iA = [theta1, theta2, theta3];
        [theta1, theta2, theta3] = InverseKinematics(pos_B(1), pos_B(2), pos_B(3), robot_config);
        theta_iB = [theta1, theta2, theta3];
        waypoint_jointspace_AB1 = linspace(theta_iA(1), theta_iB(1), waypoint_n);
        waypoint_jointspace_AB2 = linspace(theta_iA(2), theta_iB(2), waypoint_n);
        waypoint_jointspace_AB3 = linspace(theta_iA(3), theta_iB(3), waypoint_n);
        
        theta_i = zeros(waypoint_n, 3);
        for i = 1 : waypoint_n
            theta_i(i, :) = [waypoint_jointspace_AB1(i), waypoint_jointspace_AB2(i), waypoint_jointspace_AB3(i)];
        end
    else         
        waypoint_AB1 = linspace(pos_A(1), pos_B(1), waypoint_n);
        waypoint_AB2 = linspace(pos_A(2), pos_B(2), waypoint_n);
        waypoint_AB3 = linspace(pos_A(3), pos_B(3), waypoint_n);
        waypoint_BC1 = linspace(pos_B(1), pos_C(1), waypoint_n);
        waypoint_BC2 = linspace(pos_B(2), pos_C(2), waypoint_n);
        waypoint_BC3 = linspace(pos_B(3), pos_C(3), waypoint_n);
        waypoint_CD1 = linspace(pos_C(1), pos_D(1), waypoint_n);
        waypoint_CD2 = linspace(pos_C(2), pos_D(2), waypoint_n);
        waypoint_CD3 = linspace(pos_C(3), pos_D(3), waypoint_n);
        waypoint_DA1 = linspace(pos_D(1), pos_A(1), waypoint_n);
        waypoint_DA2 = linspace(pos_D(2), pos_A(2), waypoint_n);
        waypoint_DA3 = linspace(pos_D(3), pos_A(3), waypoint_n);
        waypoint_AB = [waypoint_AB1' waypoint_AB2' waypoint_AB3'];
        waypoint_BC = [waypoint_BC1' waypoint_BC2' waypoint_BC3'];
        waypoint_CD = [waypoint_CD1' waypoint_CD2' waypoint_CD3'];
        waypoint_DA = [waypoint_DA1' waypoint_DA2' waypoint_DA3'];
        waypoint = [waypoint_AB; waypoint_BC; waypoint_CD; waypoint_DA];

        theta_i = zeros(size(waypoint, 1), 3);
        for i = 1 : size(waypoint, 1)
            px = waypoint(i, 1);
            py = waypoint(i, 2);
            pz = waypoint(i, 3);
            [theta1, theta2, theta3] = InverseKinematics(px, py, pz, robot_config);
            theta_i(i, :) = [theta1 theta2 theta3]; 
        end
    end         
end