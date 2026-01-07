%% Test Trajectory Planning - Debug Script
% This script visualizes the trajectory and logs all values for debugging
clear; close all; clc;

% Add paths
setup();

%% Test Parameters (same as in Gait.m for FORWARD left-front)
pos_A = [0.25, 0.15, -0.19];   % Start (ground)
pos_D = [0.29, 0.15, -0.19];   % End (ground)
waypoint_n = 200;

%% Generate trajectory
waypoint = cubicPlanning(pos_A, pos_D, waypoint_n);

%% Display key info
fprintf('========== TRAJECTORY DEBUG INFO ==========\n');
fprintf('Start (A): x=%.4f, y=%.4f, z=%.4f\n', pos_A(1), pos_A(2), pos_A(3));
fprintf('End   (D): x=%.4f, y=%.4f, z=%.4f\n', pos_D(1), pos_D(2), pos_D(3));
fprintf('Total waypoints: %d (swing: %d, stance: %d)\n', size(waypoint, 1), waypoint_n, size(waypoint,1)-waypoint_n);
fprintf('\n');

% Check z values
z_min = min(waypoint(:,3));
z_max = max(waypoint(:,3));
fprintf('Z range: min=%.4f (ground), max=%.4f (peak)\n', z_min, z_max);
fprintf('Lift height: %.4f m\n', z_max - z_min);

%% Plot X-Z plane (side view - most important)
figure('Name', 'Foot Trajectory', 'Position', [100 100 1000 500]);

subplot(2,2,1);
plot(waypoint(1:waypoint_n,1), waypoint(1:waypoint_n,3), 'b-', 'LineWidth', 2);
hold on;
plot(waypoint(waypoint_n+1:end,1), waypoint(waypoint_n+1:end,3), 'r-', 'LineWidth', 2);
plot(pos_A(1), pos_A(3), 'go', 'MarkerSize', 12, 'LineWidth', 3);
plot(pos_D(1), pos_D(3), 'ms', 'MarkerSize', 12, 'LineWidth', 3);
xlabel('X (m)'); ylabel('Z (m)');
title('X-Z Plane (Side View)');
legend('Swing (air)', 'Stance (ground)', 'Start A', 'End D', 'Location', 'best');
grid on;
axis equal;

subplot(2,2,2);
t_swing = linspace(0, 1, waypoint_n);
t_stance = linspace(1, 2, size(waypoint,1)-waypoint_n);
plot(t_swing, waypoint(1:waypoint_n,3), 'b-', 'LineWidth', 2);
hold on;
plot(t_stance, waypoint(waypoint_n+1:end,3), 'r-', 'LineWidth', 2);
xlabel('Normalized Time'); ylabel('Z (m)');
title('Z Height over Time');
legend('Swing', 'Stance');
grid on;

subplot(2,2,3);
plot(t_swing, waypoint(1:waypoint_n,1), 'b-', 'LineWidth', 2);
hold on;
plot(t_stance, waypoint(waypoint_n+1:end,1), 'r-', 'LineWidth', 2);
xlabel('Normalized Time'); ylabel('X (m)');
title('X Position over Time');
legend('Swing (A→D)', 'Stance (D→A)');
grid on;

subplot(2,2,4);
plot3(waypoint(:,1), waypoint(:,2), waypoint(:,3), 'b-', 'LineWidth', 2);
hold on;
plot3(pos_A(1), pos_A(2), pos_A(3), 'go', 'MarkerSize', 12, 'LineWidth', 3);
plot3(pos_D(1), pos_D(2), pos_D(3), 'ms', 'MarkerSize', 12, 'LineWidth', 3);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('3D Trajectory');
grid on; axis equal;
view(45, 20);

%% Test with all 4 legs
fprintf('\n========== ALL LEGS TEST ==========\n');

legs = {
    'left-front',   [0.25,  0.15, -0.19], [0.29,  0.15, -0.19];
    'left-behind',  [-0.25, 0.15, -0.19], [-0.21, 0.15, -0.19];
    'right-front',  [0.25, -0.15, -0.19], [0.29, -0.15, -0.19];
    'right-behind', [-0.25,-0.15, -0.19], [-0.21,-0.15, -0.19];
};

figure('Name', 'All Legs Trajectory', 'Position', [150 150 800 600]);
hold on;
colors = {'b', 'r', 'g', 'm'};

for leg = 1:4
    leg_name = legs{leg, 1};
    leg_A = legs{leg, 2};
    leg_D = legs{leg, 3};
    
    wp = cubicPlanning(leg_A, leg_D, waypoint_n);
    
    plot3(wp(:,1), wp(:,2), wp(:,3), colors{leg}, 'LineWidth', 1.5, 'DisplayName', leg_name);
    plot3(leg_A(1), leg_A(2), leg_A(3), [colors{leg} 'o'], 'MarkerSize', 10, 'LineWidth', 2, 'HandleVisibility', 'off');
end

xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('All 4 Legs Trajectories');
legend('Location', 'best');
grid on; axis equal;
view(30, 25);

fprintf('✓ All legs plotted. Check if trajectories look correct.\n');
fprintf('  - Swing phase should arc UP (z increases then decreases)\n');
fprintf('  - Stance phase should stay on ground (z constant)\n');