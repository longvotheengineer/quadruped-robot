function [ICPerror, Nk] = estimateLidarNoise(prevScan, currScan, Nmax)
    ICPerror = 100; Nk = 0;
    if isempty(prevScan) || isempty(currScan)
        return;
    end
    try
        [x1,y1] = pol2cart(prevScan.Angles, prevScan.Ranges);
        [x2,y2] = pol2cart(currScan.Angles, currScan.Ranges);
        pts1 = [x1(:), y1(:), zeros(numel(x1),1)];
        pts2 = [x2(:), y2(:), zeros(numel(x2),1)];
        Nk = min([size(pts2,1), Nmax]);
        pc1 = pointCloud(pts1);
        pc2 = pointCloud(pts2);2
        gridSize = 0.05;
        pc1 = pcdownsample(pc1,'gridAverage',gridSize);
        pc2 = pcdownsample(pc2,'gridAverage',gridSize);
        if pc1.Count < 10 || pc2.Count < 10
            ICPerror = 50;
            return;
        end
        tform = pcregistericp(pc2, pc1, 'MaxIterations',50, 'Tolerance',[0.001, 0.005]);
        pc2_reg = pctransform(pc2, tform);
        M = pc1.Location; R = pc2_reg.Location;
        idx = knnsearch(M(:,1:2), R(:,1:2));
        d = sqrt(sum((R(:,1:2) - M(idx,1:2)).^2,2));
        ICPerror = sqrt(mean(d.^2));
        if ~isfinite(ICPerror), ICPerror = 50; end
    catch
        ICPerror = 50;
        Nk = 0;
    end
end