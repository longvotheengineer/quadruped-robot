function [theta,rho] = get_laser()
%Function to get laser scanner readings
%Returns theta and rho values of laser scan.
%Unit in millimeters and radians.

    global vrep; 
    global clientID;
    
    scan_reduction = 5;
    
    [result,data] = vrep.simxGetStringSignal(clientID,'measuredDataAtThisTime',vrep.simx_opmode_buffer);
  
    if (result ~= vrep.simx_return_ok)
        disp('Error in reading laser scan');
    end
    
        laserData = vrep.simxUnpackFloats(data);
        laserDataX = laserData(1:2:end-1);
        laserDataY = laserData(2:2:end);
        theta = atan2(laserDataY, laserDataX);
        rho = laserDataX./cos(theta);
        inRangeIdx = find(rho < 4.9);
        theta  = theta(inRangeIdx);
        rho  = rho(inRangeIdx); 
    
        theta  = theta(1:scan_reduction:end);
        rho = rho(1:scan_reduction:end)*1000;
    
        [X,Y] = pol2cart(theta',rho');
        %h_laser = scatter(X,Y,10,'filled','blue'); 
           
end

