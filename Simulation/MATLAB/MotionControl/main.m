clear; close all; clc;

% Initialize project paths
setup();

sim = remApi('remoteApi');      % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1);             % just in case, close all opened connections
clientID = sim.simxStart('127.0.0.1', 19999, true, true, 5000, 5);
sim.simxSynchronous(clientID, true);
simClient    = struct('sim', sim, ...
                      'clientID', clientID);

% Motion Control Parameters
robot_length = struct('base_length',    0.50,  ...
                      'base_width',     0.20,  ...
                      'L1',             0.05,  ...
                      'L2',             0.15,  ...
                      'L3',             0.10);
robot_config = struct('leg_type',       "  ",  ...
                      'joint_angle',    "  ",  ...
                      'length', robot_length);
robot_motion = struct('gait',           "  ",  ...
                      'step',           0.00);
robot        = struct('config', robot_config,  ...
                      'motion', robot_motion);

% Navigation and SLAM Parameters
sensor_data  = struct('gps_x', nan, 'gps_y', nan, 'gps_z', nan, ...
                      'ax',    nan, 'ay',    nan, 'az',    nan, ...
                      'vx',    nan, 'vy',    nan, 'vz',    nan, ...
                      'theta_scan', [], 'rho',    []);
state        = struct('first_pose', [], ...  
                      'gps_x',      [], 'gps_y',  [], ...
                      'akf_x',       0, 'akf_y',   0, ...       
                      'slam_x',     [], 'slam_y', [], ...
                      'last_scan',  []);
signals      =       {'GPS1',       'GPS2',       'GPS3',   ...
                      'Accel1',     'Accel2',     'Accel3', ...
                      'Velocity1',  'Velocity2',  'Velocity3'};
for i = 1:length(signals)
    sim.simxGetFloatSignal(clientID, signals{i}, sim.simx_opmode_streaming);
end

maxRange = 8; 
slamObj  = lidarSLAM(20, maxRange);

figMap = figure('Name','SLAM Map and Position');
axMap  = axes(figMap);
hold  (axMap,'on');
axis  (axMap,'equal');
grid  (axMap,'on');
xlabel(axMap,'x (m)'); ylabel(axMap,'y (m)');

initial_pose.x     = 0; 
initial_pose.y     = 0;
initial_pose.theta = 0;
akfObj = AKF(initial_pose, 300); %N_max = 300

[res, measuredData] = sim.simxGetStringSignal(clientID, 'measuredDataAtThisTime', sim.simx_opmode_streaming);
sim.simxAddStatusbarMessage(clientID,'Begin Simulation',sim.simx_opmode_oneshot);

if (clientID>-1)
    disp('Connected to remote API server');  
    while true
        controlPath(robot, simClient, sensor_data, slamObj, axMap, state, akfObj);        
    end 
else
    disp('Failed connecting to remote API server');
end
    sim.delete(); % call the destructor!
    
    disp('Program ended');