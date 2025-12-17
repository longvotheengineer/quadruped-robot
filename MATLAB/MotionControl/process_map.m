function [state, sensor_data] = process_map(clientID, sim, sensor_data, slamObj, axMap, update_cnt, state)

    if mod(update_cnt, 20) == 0 
        sensor_data = read_sensor_data(clientID, sim, sensor_data);
        
        if ~isempty(sensor_data.rho)
            try addScan(slamObj, lidarScan(double(sensor_data.rho)/1000, sensor_data.theta_scan)); catch; end
        end
        
        if isempty(state.first_pose)
            state.first_pose.x = sensor_data.gps_x;
            state.first_pose.y = sensor_data.gps_y;
            state.first_pose.theta = pi; 
        end 
        
        if ~isempty(state.first_pose)        
            dx = sensor_data.gps_x - state.first_pose.x;
            dy = sensor_data.gps_y - state.first_pose.y;
            alpha = -state.first_pose.theta; 
            
            gps_x = dx * cos(alpha) - dy * sin(alpha);
            gps_y = dx * sin(alpha) + dy * cos(alpha);
            
            state.path_x(end+1) = gps_x;  
            state.path_y(end+1) = gps_y;
        end
    end 

    if mod(update_cnt, 50) == 0
        cla(axMap);
        try show(slamObj, 'Parent', axMap, 'Poses', 'on'); catch; end
        hold(axMap, 'on');
        
        if ~isempty(state.path_x)
            plot(axMap, state.path_x, state.path_y, 'r.-', 'LineWidth', 1, 'MarkerSize', 5);
        end
        drawnow limitrate;
    end 
end