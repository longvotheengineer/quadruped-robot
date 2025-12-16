% clear; clc; close all;

robot_length = struct('base_length', 0.5,...
                'base_width', 0.2,...
                'L1', 0.05,...
                'L2', 0.15,...
                'L3', 0.1);

robot_config = struct('leg_type', "",...
                      'joint_angle', "");
robot_config.robot_length = robot_length;

robot_config.leg_type = "left-front"; 
% robot_config.leg_type = "left-behind";
% robot_config.leg_type = "right-front";
% robot_config.leg_type = "right-behind";

% WorkSpace(robot_config);  // left-front leg workspace

leg = InitModel(robot_config);
leg.plot([0, 0, 0]);

TrajectoryPlanning(robot_config);