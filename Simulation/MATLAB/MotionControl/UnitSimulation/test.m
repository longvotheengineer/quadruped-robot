% clear; clc; close all;

robot_length = struct('base_length', 1.20, ...
                      'base_width',  0.90, ...
                      'L1',          0.20, ...
                      'L2',          0.80, ...
                      'L3',          0.80);

robot_config = struct('leg_type',    "", ...
                      'joint_angle', "");
robot_config.robot_length = robot_length;

robot_config.leg_type = "left-front"; 
% robot_config.leg_type = "left-behind";
% robot_config.leg_type = "right-front";
% robot_config.leg_type = "right-behind";

% WorkSpace(robot_config);  % left-front leg workspace

leg = initModel(robot_config);
leg.plot([0, 0, 0]);

testControlGait(robot_config);