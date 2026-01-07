
if ~exist('slamObj', 'var')
    error('  Run file test.m !');
end

if ~exist('maxRange', 'var')
    maxRange = 8; 
end

if ~exist('state', 'var') || isempty(state.akf_x)
    current_robot_pos = [0, 0];
else
    current_robot_pos = [state.akf_x(end), state.akf_y(end)];
end

try
    [scansSLAM, poses] = scansAndPoses(slamObj);
    resolution = 20; 
    occMap = buildMap(scansSLAM, poses, resolution, maxRange);
    figure('Name', 'Occupancy Map');
    show(occMap);
    title('Occupancy Map Created from Lidar SLAM');

    save('occupancy.mat', 'occMap', '-v7.3'); 

catch ME
    error(['Error map: ', ME.message]);
end

%%Path Planning A*
map = occMap; 
planner = plannerAStarGrid(map);
%startLocation = current_robot_pos; 
startLocation = [0.5 -1.5]
endLocation   = [-1, 2]; 
if checkOccupancy(map, startLocation) == 1
    warning('startLocation error!');
end
if checkOccupancy(map, endLocation) == 1
    warning('endLocation error! ');
end

try
    path = plan(planner, startLocation, endLocation, "world");
catch ME
    disp('Error plan:');
    disp(ME.message);
    path = [];
end
figure('Name', 'A* Path Planning Result');
show(map);
hold on;

if ~isempty(path)
    plot(path(:,1), path(:,2), 'r-', 'LineWidth', 2);
    plot(startLocation(1), startLocation(2), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8);
    plot(endLocation(1), endLocation(2), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
    
    title(['A* Result: Path Found (Length: ', num2str(size(path,1)), ' steps)']);
    legend('Path', 'Start', 'Goal');
    disp('succeed');
else
    plot(startLocation(1), startLocation(2), 'go', 'MarkerFaceColor', 'g');
    plot(endLocation(1), endLocation(2), 'ro', 'MarkerFaceColor', 'r');
    
    title('A* Result: No Path Found!');
    disp('Fail ');
end
hold off;