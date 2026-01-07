%% Debug: Compare front vs behind leg trajectories
clear; close all; clc;
setup();

%% Robot config
robot_config = struct();
robot_config.length = struct('base_length', 0.50, ...
                             'base_width',  0.20, ...
                             'L1',          0.05, ...
                             'L2',          0.15, ...
                             'L3',          0.10);

%% Test positions (FORWARD gait)
legs = {
    'left-front',   [0.25,  0.15, -0.19], [0.29,  0.15, -0.19];
    'left-behind',  [-0.25, 0.15, -0.19], [-0.21, 0.15, -0.19];
    'right-front',  [0.25, -0.15, -0.19], [0.29, -0.15, -0.19];
    'right-behind', [-0.25,-0.15, -0.19], [-0.21,-0.15, -0.19];
};

figure('Name', 'Leg Trajectories Comparison', 'Position', [100 100 1200 800]);

for leg_idx = 1:4
    leg_name = legs{leg_idx, 1};
    pos_A = legs{leg_idx, 2};
    pos_D = legs{leg_idx, 3};
    
    % Generate trajectory
    waypoint = cubicPlanning(pos_A, pos_D);
    
    % Get joint angles
    robot_config.leg_type = leg_name;
    theta_all = zeros(size(waypoint, 1), 3);
    for i = 1:size(waypoint, 1)
        [theta1, theta2, theta3] = InverseKinematics(waypoint(i,1), waypoint(i,2), waypoint(i,3), robot_config);
        theta_all(i,:) = [theta1, theta2, theta3];
    end
    
    % Plot trajectory (x-z plane)
    subplot(2, 4, leg_idx);
    plot(waypoint(:,1), waypoint(:,3), 'b-', 'LineWidth', 2);
    hold on;
    plot(pos_A(1), pos_A(3), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    plot(pos_D(1), pos_D(3), 'rs', 'MarkerSize', 10, 'LineWidth', 2);
    xlabel('X (m)'); ylabel('Z (m)');
    title(sprintf('%s - Trajectory', leg_name));
    grid on;
    
    % Plot joint angles
    subplot(2, 4, leg_idx + 4);
    plot(rad2deg(theta_all(:,1)), 'r-', 'LineWidth', 1.5); hold on;
    plot(rad2deg(theta_all(:,2)), 'g-', 'LineWidth', 1.5);
    plot(rad2deg(theta_all(:,3)), 'b-', 'LineWidth', 1.5);
    xlabel('Waypoint'); ylabel('Angle (deg)');
    title(sprintf('%s - Joint Angles', leg_name));
    legend('θ1', 'θ2', 'θ3', 'Location', 'best');
    grid on;
    
    % Print statistics
    fprintf('\n=== %s ===\n', upper(leg_name));
    fprintf('Trajectory: x=[%.3f to %.3f], z=[%.3f to %.3f]\n', ...
        min(waypoint(:,1)), max(waypoint(:,1)), min(waypoint(:,3)), max(waypoint(:,3)));
    fprintf('Z lift: %.3f m (ground=%.3f, peak=%.3f)\n', ...
        max(waypoint(:,3)) - min(waypoint(:,3)), min(waypoint(:,3)), max(waypoint(:,3)));
    fprintf('θ2 range: [%.1f° to %.1f°]\n', rad2deg(min(theta_all(:,2))), rad2deg(max(theta_all(:,2))));
    fprintf('θ3 range: [%.1f° to %.1f°]\n', rad2deg(min(theta_all(:,3))), rad2deg(max(theta_all(:,3))));
    
    % Check for NaN
    if any(isnan(theta_all(:)))
        fprintf('⚠️  WARNING: NaN values in joint angles!\n');
    end
end

%% Compare theta2 changes (this controls the lift)
fprintf('\n\n=== COMPARISON ===\n');
fprintf('If behind legs have SMALLER θ2 change than front legs,\n');
fprintf('they wont lift as much visually.\n');
