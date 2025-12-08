% This code just describe ONLY ONE left-front leg of the robot
function WorkSpace(robot_config)
    L = robot_config.robot_length.base_length;
    W = robot_config.robot_length.base_width;
    L1 = robot_config.robot_length.L1;
    L2 = robot_config.robot_length.L2;
    L3 = robot_config.robot_length.L3;
    
    link1 = Link([0 0 0 0], 'modified');
    link2 = Link([0 L1 0 pi/2], 'modified');
    link3 = Link([0 0 L2 0], 'modified');
    % link4 = Link([0 0 L3 0], 'modified');
    
    leg = SerialLink([link1 link2 link3]);
    leg.tool = transl(L3, 0, 0);
    % leg.base = trotz(0) * troty(pi/2) * trotx(-pi/2);
    leg.base = trotz(-pi) * troty(-pi/2) * trotx(0);
    leg.base = transl(L/2, W/2, 0) * leg.base;
    
    q = [0, 0, 0];
    %figure; 
    
    joint_limit_1 = linspace(pi/2, 3*pi/2, 30);
    joint_limit_2 = linspace(-pi/2, pi/2, 30);
    joint_limit_3 = linspace(-pi/2, pi/2, 30);
    
    points = [];
    
    for t1 = joint_limit_1
        for t2 = joint_limit_2
            for t3 = joint_limit_3
                [x, y, z] = ForwardKinematics(t1, t2, t3, robot_config);
                p = [x, y, z]; %T(1:3, 4)';
                points = [points; p];
            end
        end
    end
    
    scatter3(points(:, 1), points(:, 2), points(:, 3), 8, 'filled');
    grid on; hold on;
    leg.plot(q);
    
    axis equal;
end