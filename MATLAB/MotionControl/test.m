clear; close all; clc;

sim = remApi('remoteApi');      % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1);             % just in case, close all opened connections
clientID = sim.simxStart('127.0.0.1', 19999, true, true, 5000, 5);
sim.simxSynchronous(clientID, true);

robot_length = struct('base_length', 0.50, ...
                      'base_width',  0.20, ...
                      'L1',          0.05, ...
                      'L2',          0.15, ...
                      'L3',          0.1);
robot_config = struct('leg_type',    "", ...
                      'joint_angle', "");
robot_config.robot_length = robot_length;
sensor_data  = struct('gps_x', nan, 'gps_y', nan, 'gps_z', nan, ...
                      'ax',    nan, 'ay',    nan, 'az',    nan, ...
                      'vx',    nan, 'vy',    nan, 'vz',    nan, ...
                      'theta_scan', [], 'rho',    []);
state        = struct('first_pose', [], ...  
                      'gps_x',      [], 'gps_y',  [], ...
                      'akf_x',      [0], 'akf_y',  [0], ...       
                      'slam_x',     [], 'slam_y', []);

maxRange = 8; 
slamObj = lidarSLAM(20, maxRange);

figMap = figure('Name','Bản đồ SLAM và Vị trí');
axMap = axes(figMap);
hold(axMap,'on');
axis(axMap,'equal');
grid(axMap,'on');
xlabel(axMap,'x (m)'); ylabel(axMap,'y (m)');

robot_length = struct('base_length', 0.5,...
                'base_width', 0.2,...
                'L1', 0.05,...
                'L2', 0.15,...
                'L3', 0.1);

robot_config = struct('leg_type', "",...
                      'joint_angle', "");
robot_config.robot_length = robot_length;
robot_motion.gait = "ZERO";
control_gait(robot_config, robot_motion, sim, clientID,slamObj, axMap);

sensor_data = struct(...
    'gps_x', nan, 'gps_y', nan, 'gps_z', nan, ...
    'ax', nan, 'ay', nan, 'az', nan, ...
    'vx', nan, 'vy', nan, 'vz', nan, ...
    'theta_scan', [], 'rho', [] ...
);

state = struct(...
    'first_pose', [], ...  
    'gps_x', [], 'gps_y', [], ...       
    'slam_x', [], 'slam_y', [], ...
    'ekf_x', [], 'ekf_y', [] ...
);

initial_pose.x = 0; 
initial_pose.y = 0;
initial_pose.theta = 0;
akfObj = AKF(initial_pose, 300); %N_max = 300

[res, measuredData] = sim.simxGetStringSignal(clientID, 'measuredDataAtThisTime', sim.simx_opmode_streaming);
sim.simxAddStatusbarMessage(clientID,'Begin Simulation',sim.simx_opmode_oneshot);

signals = {'GPS1',     'GPS2',     'GPS3', ...
           'Accel1',   'Accel2',   'Accel3', ...
           'Velocity1','Velocity2','Velocity3'};
for i = 1:length(signals)
    sim.simxGetFloatSignal(clientID, signals{i}, sim.simx_opmode_streaming);
end

robot_motion.gait = "ZERO";
control_gait(robot_config, robot_motion, sim, clientID);
if (clientID>-1)
    disp('Connected to remote API server');  
    while true
        % pause(3);
        % robot_motion.gait = "WALK";
        % robot_motion.step = 10;
        % control_gait(robot_config, robot_motion, sim, clientID, sensor_data, slamObj, axMap, state,akfObj);  
        % robot_motion.gait = "FORWARD";
        % robot_motion.step = 10;
        % control_gait(robot_config, robot_motion, sim, clientID, sensor_data, slamObj, axMap, state,akfObj); 
        % robot_motion.gait = "TURN_RIGHT";
        % robot_motion.step = 90;
        % control_gait(robot_config, robot_motion, sim, clientID, sensor_data, slamObj, axMap, state,akfObj);  
        % robot_motion.gait = "FORWARD";
        % robot_motion.step = 10;
        % control_gait(robot_config, robot_motion, sim, clientID, sensor_data, slamObj, axMap, state,akfObj);  
        % robot_motion.gait = "TURN_RIGHT";
        % robot_motion.step = 90;
        % control_gait(robot_config, robot_motion, sim, clientID, sensor_data, slamObj, axMap, state,akfObj);
        % robot_motion.gait = "FORWARD";
        % robot_motion.step = 10;
        % control_gait(robot_config, robot_motion, sim, clientID, sensor_data, slamObj, axMap, state,akfObj);  
        % pause(3);
        pause(3);
        robot_motion.gait = "FORWARD";
        target_steps = 10; 
        state = control(robot_config, robot_motion,sim, clientID, target_steps, sensor_data, slamObj, axMap, state, akfObj);
        robot_motion.gait = "TURN_RIGHT";
        target_steps = 2;
        control(robot_config, robot_motion, sim, clientID, target_steps, sensor_data, slamObj, axMap, state, akfObj);
        robot_motion.gait = "FORWARD";
        target_steps = 10;
        state = control(robot_config, robot_motion, sim, clientID, target_steps, sensor_data, slamObj, axMap, state, akfObj);
        robot_motion.gait = "TURN_RIGHT";
        target_steps = 2;
        control(robot_config, robot_motion, sim, clientID, target_steps, sensor_data, slamObj, axMap, state, akfObj);
        pause(3);
    end 
else
    disp('Failed connecting to remote API server');
end
    sim.delete(); % call the destructor!
    
    disp('Program ended');