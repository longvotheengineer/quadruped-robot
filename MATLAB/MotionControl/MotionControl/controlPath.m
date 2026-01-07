function controlPath(robot, simClient)
    % pause(3);
    % robot.motion.gait = "WALK";
    % robot.motion.step = 10;
    % controlGait(robot, simClient);          
    robot.motion.gait = "FORWARD";
    robot.motion.step = 10;
    controlGait(robot, simClient); 
    robot.motion.gait = "TURN_RIGHT";
    robot.motion.step = 2;
    controlGait(robot, simClient);  
    robot.motion.gait = "FORWARD";
    robot.motion.step = 10;
    controlGait(robot, simClient);  
    robot.motion.gait = "TURN_RIGHT";
    robot.motion.step = 5;
    controlGait(robot, simClient);
    robot.motion.gait = "FORWARD";
    robot.motion.step = 20;
    controlGait(robot, simClient);  
    robot.motion.gait = "TURN_LEFT";
    robot.motion.step = 3;
    controlGait(robot, simClient);
    robot.motion.gait = "BACKWARD";
    robot.motion.step = 10;
    controlGait(robot, simClient); 
end