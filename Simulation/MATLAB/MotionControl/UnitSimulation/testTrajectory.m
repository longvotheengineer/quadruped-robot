%% Test Trajectory Planning - Debug Script
% This script visualizes the trajectory and logs all values for debugging
clear; close all; clc;

% Add paths
setup();

%% Test Parameters (same as in Gait.m for FORWARD left-front)
pos_A = [0.25, 0.15, -0.19];   % Start (ground)
pos_D = [0.29, 0.15, -0.19];   % End (ground)
waypoint_n = 500;

%% Generate trajectory
waypoint = cubicPlanning(pos_A, pos_D);

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

% Compute velocity and acceleration (finite difference)
dt = 1; % normalized time step (since t is linspace)
vel = diff(waypoint) / dt;
vel = [vel; vel(end,:)]; % pad to match size
acc = diff(vel) / dt;
acc = [acc; acc(end,:)];

% Normalized time vector
N = size(waypoint,1);
t = linspace(0,1,N);

% Plot S-curve for position, velocity, acceleration (2D)
figure('Name','Cubic S-curve Trajectory (2D)','Position',[300 300 1200 600]);
subplot(3,1,1);
plot(t, waypoint(:,1), 'r', 'LineWidth', 2); hold on;
plot(t, waypoint(:,2), 'g', 'LineWidth', 2);
plot(t, waypoint(:,3), 'b', 'LineWidth', 2);
legend('X','Y','Z'); ylabel('Position (m)'); grid on;
title('5th-order S-curve: Position vs. Time');

subplot(3,1,2);
plot(t, vel(:,1), 'r', 'LineWidth', 2); hold on;
plot(t, vel(:,2), 'g', 'LineWidth', 2);
plot(t, vel(:,3), 'b', 'LineWidth', 2);
legend('Vx','Vy','Vz'); ylabel('Velocity (m/step)'); grid on;
title('4th-order curve: Velocity vs. Time');

subplot(3,1,3);
plot(t, acc(:,1), 'r', 'LineWidth', 2); hold on;
plot(t, acc(:,2), 'g', 'LineWidth', 2);
plot(t, acc(:,3), 'b', 'LineWidth', 2);
legend('Ax','Ay','Az'); xlabel('Normalized Time'); ylabel('Acceleration (m/step^2)'); grid on;
title('3rd-order curve: Acceleration vs. Time');

saveas(gcf, 'scurve_2d_pos_vel_acc.png');

% 3D S-curve trajectory
figure('Name','Cubic S-curve Trajectory (3D)','Position',[400 400 700 500]);
plot3(waypoint(:,1), waypoint(:,2), waypoint(:,3), 'k-', 'LineWidth', 2);
hold on;
plot3(waypoint(1,1), waypoint(1,2), waypoint(1,3), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot3(waypoint(end,1), waypoint(end,2), waypoint(end,3), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)'); grid on; axis equal;
title('3D Cubic S-curve Trajectory');
legend('Trajectory','Start','End');
view(45,25);
saveas(gcf, 'scurve_3d_traj.png');

% Plot velocity and acceleration profiles
figure('Name','Velocity and Acceleration Profiles','Position',[200 200 1000 400]);
subplot(1,2,1);
plot(vel(:,1), 'b', 'LineWidth', 1.5); hold on;
plot(vel(:,2), 'r', 'LineWidth', 1.5);
plot(vel(:,3), 'g', 'LineWidth', 1.5);
legend('Vx','Vy','Vz'); xlabel('Waypoint'); ylabel('Velocity (m/step)'); grid on;
title('Velocity Profile');
subplot(1,2,2);
plot(acc(:,1), 'b', 'LineWidth', 1.5); hold on;
plot(acc(:,2), 'r', 'LineWidth', 1.5);
plot(acc(:,3), 'g', 'LineWidth', 1.5);
legend('Ax','Ay','Az'); xlabel('Waypoint'); ylabel('Acceleration (m/step^2)'); grid on;
title('Acceleration Profile');

% Save plots as images
saveas(gcf, 'velocity_acceleration_profile.png');
figHandles = findall(0,'Type','figure');
for k = 1:length(figHandles)
    saveas(figHandles(k), sprintf('trajectory_plot_%d.png', k));
end

% Export waypoint data
save('waypoint_single_leg.mat','waypoint','vel','acc');

% Print summary statistics
fprintf('Step length: %.4f m\n', norm(pos_D - pos_A));
fprintf('Max velocity: %.4f m/step\n', max(vecnorm(vel,2,2)));
fprintf('Max acceleration: %.4f m/step^2\n', max(vecnorm(acc,2,2)));

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
    
    wp = cubicPlanning(leg_A, leg_D);
    
    plot3(wp(:,1), wp(:,2), wp(:,3), colors{leg}, 'LineWidth', 1.5, 'DisplayName', leg_name);
    plot3(leg_A(1), leg_A(2), leg_A(3), [colors{leg} 'o'], 'MarkerSize', 10, 'LineWidth', 2, 'HandleVisibility', 'off');

    % Compute velocity/acceleration for each leg
    v = diff(wp); v = [v; v(end,:)];
    a = diff(v); a = [a; a(end,:)];
    % Export data
    save(sprintf('waypoint_%s.mat',leg_name),'wp','v','a');
    % Print stats
    fprintf('Leg: %s\n', leg_name);
    fprintf('  Step length: %.4f m\n', norm(leg_D - leg_A));
    fprintf('  Max height: %.4f m\n', max(wp(:,3)));
    fprintf('  Max velocity: %.4f m/step\n', max(vecnorm(v,2,2)));
    fprintf('  Max acceleration: %.4f m/step^2\n', max(vecnorm(a,2,2)));
end

xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('All 4 Legs Trajectories');
legend('Location', 'best');
grid on; axis equal;
view(30, 25);

fprintf('✓ All legs plotted. Check if trajectories look correct.\n');
fprintf('  - Swing phase should arc UP (z increases then decreases)\n');
fprintf('  - Stance phase should stay on ground (z constant)\n');