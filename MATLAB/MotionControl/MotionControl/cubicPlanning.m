function [theta] = cubicPlanning(theta_start, theta_end)
    T_swing     = 200;
    T_ground    = 300;
    T_total     = T_swing + T_ground;
    lift_height = 0.05; % How high to lift foot (in meters, positive = up)

    theta = zeros(T_total, 3);

    x_start  = theta_start(1);
    x_end    = theta_end  (1);
    y_start  = theta_start(2);
    y_end    = theta_end  (2);
    z_ground = min(theta_start(3), theta_end(3));  

    % Parabola path planning (swing phase)
    for i = 1 : T_swing
        tau = (i-1) / (T_swing - 1);       % tau belongs to [0, 1]
        s   = 3*tau^2 - 2*tau^3;           %   s belongs to [0, 1] 
        
        x   = x_start  + s * (x_end - x_start);
        y   = y_start  + s * (y_end - y_start);
        z   = z_ground + 4 * lift_height * tau * (1 - tau);        
        theta(i,:) = [x, y, z];    
    end

    % Linear path planning (ground phase)
    for i = 1 : T_ground
        tau = (i-1) / (T_ground - 1);      % tau belongs to [0, 1]
        s   = 3*tau^2 - 2*tau^3;           %   s belongs to [0, 1]
        
        x   = x_end + s * (x_start - x_end);
        y   = y_end + s * (y_start - y_end);
        z   = z_ground;         
        theta(T_swing + i, :) = [x, y, z];
    end
end