function controlPath(robot, simClient, sensor_data, slamObj, axMap, state, akfObj)
    robot.motion.gait = "ZERO";
    controlGait(robot, simClient, sensor_data);  
    % pause(3);
    % robot.motion.gait = "WALK";
    % robot.motion.step = 100;
    % controlGait(robot, simClient, sensor_data);   
    % robot.motion.gait = "FORWARD";
    % robot.motion.step = 100;
    % controlGait(robot, simClient, sensor_data);        
    robot.motion.gait = "BACKWARD";
    robot.motion.step = 10;
    controlGait(robot, simClient, sensor_data); 
    robot.motion.gait = "TURN_RIGHT";
    robot.motion.step = 2;
    controlGait(robot, simClient, sensor_data);  
    robot.motion.gait = "BACKWARD";
    robot.motion.step = 10;
    controlGait(robot, simClient, sensor_data);  
    robot.motion.gait = "TURN_RIGHT";
    robot.motion.step = 5;
    controlGait(robot, simClient, sensor_data);
    robot.motion.gait = "BACKWARD";
    robot.motion.step = 20;
    controlGait(robot, simClient, sensor_data);  
    robot.motion.gait = "TURN_LEFT";
    robot.motion.step = 3;
    controlGait(robot, simClient, sensor_data);
    % robot.motion.gait = "BACKWARD";
    % robot.motion.step = 100;
    % controlGait(robot, simClient, sensor_data); 

    % Navigation - Currently Disabled and wrong implementation because of merge issues    
    % robot.motion.gait = "FORWARD";
    % target_steps = 10; 
    % state = control(robot, simClient, target_steps, sensor_data, slamObj, axMap, state, akfObj);
    % robot.motion.gait = "TURN_RIGHT";
    % target_steps = 2;
    % state = control(robot, simClient, target_steps, sensor_data, slamObj, axMap, state, akfObj);
end