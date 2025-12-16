load('occupancy.mat','occMap');
map = occMap;   % occMap là đối tượng occupancyMap

% --- Tạo planner A* ---
planner = plannerAStarGrid(map);

% --- Thiết lập điểm bắt đầu và kết thúc (theo tọa độ thế giới) ---
startLocation = [0 0];
endLocation   = [2 3];

% --- Tìm đường bằng A* ---
path = plan(planner,startLocation,endLocation,"world");

% --- Hiển thị kết quả ---
show(map);
hold on;
if ~isempty(path)
    plot(path(:,1),path(:,2),'r-','LineWidth',2);
    plot(startLocation(1),startLocation(2),'go','MarkerFaceColor','g');
    plot(endLocation(1),endLocation(2),'ro','MarkerFaceColor','r');
    title('Đường đi tìm được bằng thuật toán A*');
else
    title('Không tìm thấy đường đi hợp lệ!');
end
