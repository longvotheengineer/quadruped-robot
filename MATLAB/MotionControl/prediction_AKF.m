function sensor_data = prediction_AKF(clientID, sim, sensor_data, akfObj)
    dt = 0.05; 
    [res1, ax] = sim.simxGetFloatSignal(clientID, 'Accel1', sim.simx_opmode_buffer);
    [res2, ay] = sim.simxGetFloatSignal(clientID, 'Accel2', sim.simx_opmode_buffer);
    [res3, gz] = sim.simxGetFloatSignal(clientID, 'Velocity3', sim.simx_opmode_buffer);
    if (res1 == sim.simx_return_ok), sensor_data.ax = ax; end
    if (res2 == sim.simx_return_ok), sensor_data.ay = ay; end
    if (res3 == sim.simx_return_ok), sensor_data.gz = gz; end
    if ~isnan(sensor_data.ax) && ~isnan(sensor_data.gz)
        akfObj.prediction(sensor_data.ax, sensor_data.ay, sensor_data.gz, dt);
    end
end