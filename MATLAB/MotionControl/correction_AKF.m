function [state, sensor_data] = correction_AKF(clientID, sim, sensor_data, slamObj, axMap, state, akfObj)
    [~, sensor_data.gps_x] = sim.simxGetFloatSignal(clientID,'GPS1',sim.simx_opmode_buffer);
    [~, sensor_data.gps_y] = sim.simxGetFloatSignal(clientID,'GPS2',sim.simx_opmode_buffer);
    [sensor_data.theta_scan, sensor_data.rho] = get_laser(clientID, sim); 
    gps_pos_update = [];
    if isempty(state.first_pose) && ~isnan(sensor_data.gps_x)
        state.first_pose.x = sensor_data.gps_x;
        state.first_pose.y = sensor_data.gps_y;
        state.first_pose.theta = pi; 
    end
    if ~isempty(state.first_pose) && ~isnan(sensor_data.gps_x)
        dx = sensor_data.gps_x - state.first_pose.x;
        dy = sensor_data.gps_y - state.first_pose.y;
        dist_check = sqrt((dx - state.akf_x(end))^2 + (dy - state.akf_y(end))^2);
        if dist_check < 2.0 
            theta_rot = state.first_pose.theta;
            gps_x = dx * cos(theta_rot) + dy * sin(theta_rot);
            gps_y = -dx * sin(theta_rot) + dy * cos(theta_rot);
            
            gps_pos_update.x = gps_x;
            gps_pos_update.y = gps_y;
            state.gps_x(end+1) = gps_x;
            state.gps_y(end+1) = gps_y;
        end
    end

    slam_pos_update = [];
    current_icp_error = 0.5; 
    if ~isempty(sensor_data.rho)
            poseGuess = akfObj.get_pose_guess();
            scan = lidarScan(double(sensor_data.rho)/1000, sensor_data.theta_scan);
            
            if isfield(state, 'last_scan') && ~isempty(state.last_scan)
                current_icp_error = ICPerror(scan, state.last_scan);
            end
            state.last_scan = scan;
            
            [isAdded, ~] = addScan(slamObj, scan, poseGuess);
            
            if isAdded
                nodes = nodeEstimates(slamObj.PoseGraph); 
                current_slam_pose = nodes(end, :); 
                
                state.slam_x(end+1) = current_slam_pose(1);
                state.slam_y(end+1) = current_slam_pose(2);
                
                slam_pos_update.x = current_slam_pose(1);
                slam_pos_update.y = current_slam_pose(2);
            end
    end
    params.hdop = 1.0;          
    params.icp_error = current_icp_error;    
    params.N_k = length(sensor_data.rho); 
    akfObj.correction_adaptive(gps_pos_update, slam_pos_update, params);

    optimal_pose = akfObj.get_pose_guess();
    state.akf_x(end+1) = optimal_pose(1);
    state.akf_y(end+1) = optimal_pose(2);
    cla(axMap);
    try show(slamObj, 'Parent', axMap, 'Poses', 'off'); catch; end
    hold(axMap, 'on');
    if ~isempty(state.akf_x), plot(axMap, state.akf_x, state.akf_y, 'g.-', 'MarkerSize', 5); end
    if ~isempty(state.slam_x), plot(axMap, state.slam_x, state.slam_y, 'b.-', 'MarkerSize', 5); end
    if ~isempty(state.gps_x), plot(axMap, state.gps_x, state.gps_y, 'r.', 'MarkerSize', 5); end
    drawnow limitrate;
end