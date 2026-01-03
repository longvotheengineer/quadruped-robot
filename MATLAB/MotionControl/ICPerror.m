function rmse = ICPerror(currScan, prevScan)
    try
        [tform, ~] = matchScans(currScan, prevScan, 'MaxIterations', 25);
        transScan = transformScan(currScan, tform);
        ptCloudCurr = transScan.Cartesian;
        ptCloudPrev = prevScan.Cartesian;
        if exist('knnsearch', 'file')
            [~, dists] = knnsearch(ptCloudPrev, ptCloudCurr);
        else
            [~, dists] = dsearchn(ptCloudPrev, ptCloudCurr);
        end
        rmse = sqrt(mean(dists.^2));
        if rmse > 2.0
            rmse = 2.0; 
        end
        
    catch
        rmse = 2; 
    end
end