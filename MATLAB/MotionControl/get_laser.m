function [theta,rho,h_laser] = get_laser(clientID, sim)
    scan_reduction = 5;
    
    [result,data] = sim.simxGetStringSignal(clientID,'measuredDataAtThisTime',sim.simx_opmode_buffer);
  
    if (result ~= sim.simx_return_ok)
        disp('Error in reading laser scan');
    end
    
        laserData = sim.simxUnpackFloats(data);
        laserDataX = laserData(1:2:end-1);
        laserDataY = laserData(2:2:end);
        theta = atan2(laserDataY, laserDataX);
        rho = laserDataX./cos(theta);
        inRangeIdx = find(rho < 100);
        theta  = theta(inRangeIdx);
        rho  = rho(inRangeIdx); 
    
        theta  = theta(1:scan_reduction:end);
        rho = rho(1:scan_reduction:end)*1000;
    
        [X,Y] = pol2cart(theta',rho');
        h_laser = scatter(X,Y,10,'filled','blue'); 
           
end

