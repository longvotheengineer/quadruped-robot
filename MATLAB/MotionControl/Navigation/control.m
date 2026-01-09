function state = control(robot, simClient, total_steps, sensor_data, slamObj, axMap, state, akfObj)
    num_loops = total_steps; 
    for i = 1 : num_loops
        robot.motion.step = 1; 
        sensor_data = controlGait(robot, simClient, sensor_data, slamObj, axMap, state, akfObj); 
        [state, sensor_data] = correction_AKF(simClient, sensor_data, slamObj, axMap, state, akfObj);
        drawnow limitrate;
    end
end