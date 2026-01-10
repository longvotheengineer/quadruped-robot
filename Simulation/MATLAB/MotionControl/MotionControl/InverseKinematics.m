function [theta1, theta2, theta3] = InverseKinematics(x, y, z, robot_config)
    L   = robot_config.length.base_length;
    W   = robot_config.length.base_width;
    L1  = robot_config.length.L1;
    L2  = robot_config.length.L2;
    L3  = robot_config.length.L3;
    % theta1 = atan2(-z,W/2 - y) + atan2(sqrt((W/2 - y)^2 + z^2 - L1^2), -L1);
    % p1 = (W/2 - y + L1*cos(theta1)) / sin(theta1)
    % p2 = x - L/2

    x_temp = x;
    y_temp = y;
    z_temp = z;
    
    switch robot_config.leg_type
        case "left-front"
            x       = z_temp;
            y       = W/2 - y_temp;
            z       = -L/2 + x_temp;
            theta1  = atan2(-x, y) + atan2(-sqrt(x^2 + y^2 - L1^2), -L1);
            sign_s3 = -1;
            sign_b2 = 1;
        case "left-behind"
            x       = z_temp;
            y       = W/2 - y_temp;
            z       = L/2 + x_temp;
            theta1  = atan2(-x, y) + atan2(-sqrt(x^2 + y^2 - L1^2), -L1);
            sign_s3 = 1;
            sign_b2 = 1;
        case "right-front"
            x       = z_temp;
            y       = -W/2 - y_temp;
            z       = -L/2 + x_temp;
            theta1  = atan2(-x, y) + atan2(-sqrt(x^2 + y^2 - L1^2), L1);
            sign_s3 = 1;
            sign_b2 = -1;
        case "right-behind"
            x       = z_temp;
            y       = -W/2 - y_temp;
            z       = L/2 + x_temp;
            theta1  = atan2(-x, y) + atan2(-sqrt(x^2 + y^2 - L1^2), L1);
            sign_s3 = -1;
            sign_b2 = -1;
        otherwise
    end

    p1      = x*cos(theta1) + y*sin(theta1);
    p2      = sign_b2 * z;
    c3      = (p1^2 + p2^2 - L2^2 - L3^2) / (2*L2*L3);
    s3      = sign_s3 * sqrt(1 - c3^2);
    theta3  = atan2(s3, c3);
    theta2  = atan2(p2, p1) - atan2(L3*s3, L2 + L3*c3);
end