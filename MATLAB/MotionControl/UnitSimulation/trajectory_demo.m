% Trajectory Planning and Simulation Demo
% This script demonstrates trajectory planning, simulation, and visualization
% for a quadruped robot. It is self-contained and does not depend on other files.

% Parameters
T = 5;              % Total time (s)
N = 100;            % Number of time steps
L = 1.0;            % Path length (m)
start_pos = [0, 0]; % Start position (x, y)
end_pos   = [L, 0]; % End position (x, y)

% Time vector
traj.t = linspace(0, T, N);

% Linear interpolation for position
traj.x = linspace(start_pos(1), end_pos(1), N);
traj.y = linspace(start_pos(2), end_pos(2), N);

% Velocity (constant for linear path)
traj.vx = diff([traj.x, traj.x(end)]) ./ diff([traj.t, traj.t(end)]);
traj.vy = diff([traj.y, traj.y(end)]) ./ diff([traj.t, traj.t(end)]);

% Acceleration (zero for linear path)
traj.ax = zeros(1, N);
traj.ay = zeros(1, N);

% Simulate noisy measurement (for reporting)
noise_level = 0.01;
traj.x_meas = traj.x + noise_level*randn(1, N);
traj.y_meas = traj.y + noise_level*randn(1, N);

% Plot planned vs measured trajectory
figure('Name','Trajectory Planning and Simulation');
subplot(2,2,1);
plot(traj.x, traj.y, 'b-', 'LineWidth', 2); hold on;
plot(traj.x_meas, traj.y_meas, 'rx');
legend('Planned','Measured');
xlabel('X (m)'); ylabel('Y (m)'); grid on;
title('Trajectory'); axis equal;

% Plot velocity profile
subplot(2,2,2);
plot(traj.t, traj.vx, 'b-', traj.t, traj.vy, 'r-');
legend('Vx','Vy'); xlabel('Time (s)'); ylabel('Velocity (m/s)'); grid on;
title('Velocity Profile');

% Plot acceleration profile
subplot(2,2,3);
plot(traj.t, traj.ax, 'b-', traj.t, traj.ay, 'r-');
legend('Ax','Ay'); xlabel('Time (s)'); ylabel('Acceleration (m/s^2)'); grid on;
title('Acceleration Profile');

% Plot position error
subplot(2,2,4);
err = sqrt((traj.x - traj.x_meas).^2 + (traj.y - traj.y_meas).^2);
plot(traj.t, err, 'k-'); xlabel('Time (s)'); ylabel('Position Error (m)'); grid on;
title('Position Error (Planned vs Measured)');

% Print summary statistics
fprintf('Trajectory Planning Simulation Summary:\n');
fprintf('  Path length: %.2f m\n', L);
fprintf('  Duration: %.2f s\n', T);
fprintf('  Max position error: %.4f m\n', max(err));
fprintf('  Mean position error: %.4f m\n', mean(err));
