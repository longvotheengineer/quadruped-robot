function theta_i = TrajectoryPlanning(robot_config)   
    leg = InitModel(robot_config);
    switch robot_config.leg_type
        case "left-front"
            % pos_A = [0.25, 0.15, -0.24];
            % pos_B = [0.25, 0.15, -0.20];
            % pos_C = [0.29, 0.15, -0.20];
            % pos_D = [0.29, 0.15, -0.24];
            pos_A = [0.25, 0.15, -0.24];
            pos_B = [0.25, 0.15, -0.10];
            pos_C = [0.29, 0.15, -0.10];
            pos_D = [0.29, 0.15, -0.24];
        case "left-behind"
            % pos_A = [-0.25, 0.15, -0.24];
            % pos_B = [-0.25, 0.15, -0.20];
            % pos_C = [-0.21, 0.15, -0.20];
            % pos_D = [-0.21, 0.15, -0.24];
            pos_A = [-0.25, 0.15, -0.24];
            pos_B = [-0.25, 0.15, -0.10];
            pos_C = [-0.21, 0.15, -0.10];
            pos_D = [-0.21, 0.15, -0.24];
        case "right-front"
            % pos_A = [0.25, -0.15, -0.24];
            % pos_B = [0.25, -0.15, -0.20];
            % pos_C = [0.29, -0.15, -0.20];
            % pos_D = [0.29, -0.15, -0.24];
            pos_A = [0.25, -0.15, -0.24];
            pos_B = [0.25, -0.15, -0.10];
            pos_C = [0.29, -0.15, -0.10];
            pos_D = [0.29, -0.15, -0.24];
        case "right-behind"
            % pos_A = [-0.25, -0.15, -0.24];
            % pos_B = [-0.25, -0.15, -0.20];
            % pos_C = [-0.21, -0.15, -0.20];
            % pos_D = [-0.21, -0.15, -0.24];
            pos_A = [-0.25, -0.15, -0.24];
            pos_B = [-0.25, -0.15, -0.10];
            pos_C = [-0.21, -0.15, -0.10];
            pos_D = [-0.21, -0.15, -0.24];
        otherwise
    end

    waypoint_n = 10;
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

    figure; grid on; axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z'); 
    legend('FK of q(t)', 'waypoints'); 
    plot3(waypoint(:,1), waypoint(:,2), waypoint(:,3), '-');
    hold on;
    P = [pos_A' pos_B' pos_C' pos_D'];
    scatter3(P(1,:), P(2,:), P(3,:), 50, 'r', 'filled');

    hold on;
    while true
        leg.plot(theta_i, 'workspace', [-0.1 0.6 -0.1 0.6 -0.25 0.25],...
                     'view', [45,30], 'delay', 0.005);
    end
end