function [state, sensor_data] = process_map(clientID, sim, sensor_data, slamObj, axMap, update_cnt, state)

    if mod(update_cnt, 20) == 0 
        sensor_data = read_sensor_data(clientID, sim, sensor_data);
        
        if ~isempty(sensor_data.rho)
            try addScan(slamObj, lidarScan(double(sensor_data.rho)/1000, sensor_data.theta_scan));
                all_poses = slamObj.PoseGraph.nodes; 
                current_slam_pose = all_poses(end, :); 
                state.slam_x(end+1) = current_slam_pose(1);
                state.slam_y(end+1) = current_slam_pose(2);
            catch; end
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
            
            state.gps_x(end+1) = gps_x;  
            state.gps_y(end+1) = gps_y;
        end
    end 

    if mod(update_cnt, 50) == 0
        cla(axMap);
        try show(slamObj, 'Parent', axMap, 'Poses', 'off'); catch; end
        hold(axMap, 'on');
        
        if ~isempty(state.gps_x)
            plot(axMap, state.gps_x, state.gps_y, 'r.-', 'LineWidth', 1, 'MarkerSize', 5);
        end
        if ~isempty(state.slam_x)
            plot(axMap, state.slam_x, state.slam_y, 'b.-', 'LineWidth', 1, 'MarkerSize', 5);
        end
        drawnow limitrate;
    end 
end