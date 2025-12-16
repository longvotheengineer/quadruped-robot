function sensor_data= read_sensor_data(clientID, sim, sensor_data)
    [~, sensor_data.gps_x] = sim.simxGetFloatSignal(clientID,'GPS1',sim.simx_opmode_buffer);
    [~, sensor_data.gps_y] = sim.simxGetFloatSignal(clientID,'GPS2',sim.simx_opmode_buffer);
    [~, sensor_data.gps_z] = sim.simxGetFloatSignal(clientID,'GPS3',sim.simx_opmode_buffer);
    
    [~, sensor_data.ax] = sim.simxGetFloatSignal(clientID,'Accel1',sim.simx_opmode_buffer);
    [~, sensor_data.ay] = sim.simxGetFloatSignal(clientID,'Accel2',sim.simx_opmode_buffer);
    [~, sensor_data.az] = sim.simxGetFloatSignal(clientID,'Accel3',sim.simx_opmode_buffer);
    
    [~, sensor_data.vx] = sim.simxGetFloatSignal(clientID,'Velocity1',sim.simx_opmode_buffer);
    [~, sensor_data.vy] = sim.simxGetFloatSignal(clientID,'Velocity2',sim.simx_opmode_buffer);
    [~, sensor_data.vz] = sim.simxGetFloatSignal(clientID,'Velocity3',sim.simx_opmode_buffer);
    [sensor_data.theta_scan, sensor_data.rho, ~] = get_laser(); 
end
