function [x, y, z] = ForwardKinematics(theta1, theta2, theta3, robot_config)
    L  = robot_config.length.base_length;
    W  = robot_config.length.base_width;
    L1 = robot_config.length.L1;
    L2 = robot_config.length.L2;
    L3 = robot_config.length.L3;

    % syms L1 L2 L3 theta1 theta2 theta3 a_i_minus_1 alpha_i_minus_1 di thetai r11 r12 r13 r21 r22 r23 r31 r32 r33 px py pz;
    % 
    % T_4_0 = [r11 r12 r13 px;...
    %          r21 r22 r23 py;...
    %          r31 r32 r33 pz;...
    %            0   0   0  1];
    % 
    % A = [cos(thetai)                           -sin(thetai)                          0                        a_i_minus_1               ;...
    %      sin(thetai)*cos(alpha_i_minus_1)       cos(thetai)*cos(alpha_i_minus_1)    -sin(alpha_i_minus_1)    -di*sin(alpha_i_minus_1)   ;...
    %      sin(thetai)*sin(alpha_i_minus_1)       cos(thetai)*sin(alpha_i_minus_1)     cos(alpha_i_minus_1)     di*cos(alpha_i_minus_1)   ;...         
    %      0                                      0                                    0                        1                         ];  
    % 
    % A_1_0 = subs(A, [a_i_minus_1, alpha_i_minus_1, di, thetai], [0, 0, 0, theta1]);
    % A_2_1 = subs(A, [a_i_minus_1, alpha_i_minus_1, di, thetai], [0, -pi/2, L1, theta2]);
    % A_3_2 = subs(A, [a_i_minus_1, alpha_i_minus_1, di, thetai], [L2, 0, 0, theta3]);
    % A_4_3 = subs(A, [a_i_minus_1, alpha_i_minus_1, di, thetai], [L3, 0, 0, 0]);
    % 
    % A_4_0 = simplify(A_1_0 * A_2_1 * A_3_2 * A_4_3);
    % px = A_4_0(1,4);
    % py = A_4_0(2,4);
    % pz = A_4_0(3,4);
    % 
    % A_4_1 = simplify(inv(A_1_0) * A_4_0);
    % T_4_1 = simplify(inv(A_4_1) * T_4_0);

    switch robot_config.leg_type
        case {"left-front", "left-behind" }
            px =   L1*sin(theta1) - L3*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + L2*cos(theta1)*cos(theta2);
            py =   L2*cos(theta2)*sin(theta1) - L1*cos(theta1) - L3*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1));
            pz =   L3*sin(theta2 + theta3) + L2*sin(theta2);
        case {"right-front", "right-behind"}
            px = - L1*sin(theta1) - L3*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + L2*cos(theta1)*cos(theta2);
            py =   L2*cos(theta2)*sin(theta1) + L1*cos(theta1) - L3*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1));
            pz = - L3*sin(theta2 + theta3) - L2*sin(theta2);
        otherwise
    end
    switch robot_config.leg_type
        case "left-front"
            x = L/2 + pz;
            y = W/2 - py;
            z = px;
        case "left-behind"
            x = -L/2 + pz;
            y = W/2 - py;
            z = px;
        case "right-front"
            x = L/2 + pz;
            y = -W/2 - py;
            z = px;
        case "right-behind"
            x = -L/2 + pz;
            y = -W/2 - py;
            z = px;
        otherwise
    end
end