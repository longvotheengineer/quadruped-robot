function SLAM
clc; clear;
global vrep clientID slamObj keys speed rotSpeed; ;
persistent originPos originYaw prevAccel prevTime lastGPS prevScanForNoise;

%% --- Kết nối với CoppeliaSim ---
vrep = remApi('remoteApi');
vrep.simxFinish(-1);
clientID = vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

if clientID < 0
    error('Không kết nối được với CoppeliaSim.');
else
    disp('Đã kết nối CoppeliaSim');
end

%% --- Lấy handle Dummy ---
[ret, dummyHandle] = vrep.simxGetObjectHandle(clientID, 'dummy', vrep.simx_opmode_blocking);


%% --- Initiate LIDAR / streaming ---
[~,~] = vrep.simxGetStringSignal(clientID,'measuredDataAtThisTime',vrep.simx_opmode_streaming);
vrep.simxAddStatusbarMessage(clientID,'Begin Simulation',vrep.simx_opmode_oneshot);

signals = {'GPS1','GPS2','GPS3','Accel1','Accel2','Accel3','Velocity1','Velocity2','Velocity3'};
for i = 1:length(signals)
    vrep.simxGetFloatSignal(clientID, signals{i}, vrep.simx_opmode_streaming);
end

%% --- GUI 
fig = figure('Name','Điều khiển Dummy',...
    'Position',[700 400 300 300],...
    'KeyPressFcn',@onKeyPress,...
    'KeyReleaseFcn',@onKeyRelease,...
    'CloseRequestFcn',@onClose,...
    'MenuBar','none','NumberTitle','off');

uicontrol('Style','pushbutton','String','Forward (W)',...
    'Position',[110 200 80 40], 'FontSize',10,...
    'Callback', @(src,event) pressKey('W'));
uicontrol('Style','pushbutton','String','Left (A)',...
    'Position',[30 150 80 40], 'FontSize',10,...
    'Callback', @(src,event) pressKey('A'));
uicontrol('Style','pushbutton','String','Stop (S)',...
    'Position',[110 150 80 40], 'FontSize',10,...
    'Callback', @(src,event) pressKey('S'));
uicontrol('Style','pushbutton','String','Right (D)',...
    'Position',[190 150 80 40], 'FontSize',10,...
    'Callback', @(src,event) pressKey('D'));
uicontrol('Style','pushbutton','String','Backward (X)',...
    'Position',[110 100 80 40], 'FontSize',10,...
    'Callback', @(src,event) pressKey('X'));

%% --- Biến trạng thái ---
keys = struct('W',false,'X',false,'A',false,'D',false);
speed = 0.1;       % m/lặp
rotSpeed = pi/15;   % rad/lặp

%% --- Thiết lập SLAM & figure/axes ---
maxRange = 5; % m
slamObj = lidarSLAM(20, maxRange); % 20 cells/m

figMap = figure('Name','Bản đồ SLAM và Quỹ đạo');
axMap = axes(figMap);
hold(axMap,'on');
axis(axMap,'equal');
grid(axMap,'on');
xlabel(axMap,'x (m)'); ylabel(axMap,'y (m)');
legendEntries = {}; % for legend later

gps_traj = [];
ekf_traj = [];
gt_traj  =[];

%% --- Khởi tạo EKF ---
dt = 0.1;
% trạng thái: [x; y; vx; vy; theta]
x_ekf = zeros(5,1);
P = diag([0.5, 0.5, 0.5, 0.5, 0.1]);   % 5x5 - Phương sai khởi tạo
Q = diag([1e-1, 1e-1, 1e-2, 1e-2, 1e-4]); % Nhiễu quá trình (Process Noise)
% Low-pass accel
alpha = 0.85;
if isempty(prevAccel), prevAccel = [0;0]; end
if isempty(prevTime), prevTime = tic; end
% scan storage
scan = []; % will hold last lidarScan
if isempty(prevScanForNoise), prevScanForNoise = []; end
plotInterval = 3;
loopCount = 0;
sigma_r = 0.1; % Độ lệch chuẩn GPS cơ bản (VDOP/HDOP = 1.0)

%% --- MAIN LOOP (an toàn khi stop) ---
while ishandle(fig) && vrep.simxGetConnectionId(clientID) > -1
    loopStart = tic;
    loopCount = loopCount + 1;
    try
        % --- Read robot pose from simulator (Ground Truth) ---
        [retPos, pos_gt] = vrep.simxGetObjectPosition(clientID, dummyHandle, -1, vrep.simx_opmode_blocking);
        [retOri, ori] = vrep.simxGetObjectOrientation(clientID, dummyHandle, -1, vrep.simx_opmode_blocking);
        
        if retPos ~= 0 || retOri ~= 0
            pause(0.01); continue; % Kiểm tra lỗi ngay sau khi đọc
        end
        yaw = ori(3);
        
        % 1. Khởi tạo Origin (Chỉ chạy một lần)
        if isempty(originPos)
            originPos = pos_gt(1:2); % Vị trí ban đầu (x, y)
            originYaw = yaw;         % Góc quay ban đầu
            x_ekf(1:2) = 0;          % EKF state position = 0
            x_ekf(5) = 0;            % EKF state yaw = 0
        end

        % 2. Lưu Ground Truth vào hệ SLAM map
        % Ma trận xoay ngược lại góc khởi tạo (originYaw)
        R_toSlam = [cos(-originYaw) -sin(-originYaw);
                    sin(-originYaw)  cos(-originYaw)];
        
        % Chuyển đổi: World Frame -> SLAM Frame (bằng cách dịch và xoay)
        pos_slam_gt = R_toSlam * (pos_gt(1:2)' - originPos(:));
        gt_traj = [gt_traj; pos_slam_gt']; % Thêm vào mảng theo dõi
        % --- Update dummy by key ---
        % Sử dụng pos_gt làm vị trí hiện tại để điều khiển
        pos = pos_gt;
        if keys.W
            pos = pos + [cos(yaw)*speed, sin(yaw)*speed, 0];
        elseif keys.X
            pos = pos - [cos(yaw)*speed, sin(yaw)*speed, 0];
        end
        if keys.A
            yaw = yaw + rotSpeed;
        elseif keys.D
            yaw = yaw - rotSpeed;
        end
        vrep.simxSetObjectPosition(clientID, dummyHandle, -1, pos, vrep.simx_opmode_oneshot);
        vrep.simxSetObjectOrientation(clientID, dummyHandle, -1, [0 0 yaw], vrep.simx_opmode_oneshot);

        % --- Read sensors (buffer) ---
        [ret_xgps, x_gps] = vrep.simxGetFloatSignal(clientID,'GPS1',vrep.simx_opmode_buffer);
        [ret_ygps, y_gps] = vrep.simxGetFloatSignal(clientID,'GPS2',vrep.simx_opmode_buffer);
        [ret_zgps, z_gps] = vrep.simxGetFloatSignal(clientID,'GPS3',vrep.simx_opmode_buffer);

        [ret_ax, ax] = vrep.simxGetFloatSignal(clientID,'Accel1',vrep.simx_opmode_buffer);
        [ret_ay, ay] = vrep.simxGetFloatSignal(clientID,'Accel2',vrep.simx_opmode_buffer);
        [ret_az, az] = vrep.simxGetFloatSignal(clientID,'Accel3',vrep.simx_opmode_buffer);

        [ret_vx, gx] = vrep.simxGetFloatSignal(clientID,'Velocity1',vrep.simx_opmode_buffer);
        [ret_vy, gy] = vrep.simxGetFloatSignal(clientID,'Velocity2',vrep.simx_opmode_buffer);
        [ret_vz, gz] = vrep.simxGetFloatSignal(clientID,'Velocity3',vrep.simx_opmode_buffer);

        % guard invalid signals -> set NaN
        if ret_xgps ~= 0, x_gps = NaN; end
        if ret_ygps ~= 0, y_gps = NaN; end
        if ret_ax ~= 0, ax = NaN; end
        if ret_ay ~= 0, ay = NaN; end
        if ret_vz ~= 0, gz = NaN; end

        % --- Get LIDAR scan ---
        [theta_scan, rho] = get_laser(); % rho in mm
        if ~isempty(rho)
            try
                addScan(slamObj, lidarScan((rho / 1000), theta_scan));
            catch
                % nếu addScan fail thì bỏ qua
            end
        end

       % --- Time dt ---
        nowT = toc(prevTime);
        if nowT <= 0, nowT = dt; end
        prevTime = tic;
        
        % --- Low-pass accel ---
        % Giả sử accel_raw = [ax; ay] đã được xoay về World/SLAM frame
        accel_raw = [ax; ay];
        if any(isnan(accel_raw))
            accel_f = prevAccel;
        else
            % Xoay gia tốc từ Body frame sang World/SLAM frame
            theta_prev = x_ekf(5);
            R_body_to_world = [cos(theta_prev) -sin(theta_prev);
                               sin(theta_prev)  cos(theta_prev)];
            accel_world = R_body_to_world * accel_raw(1:2); % Chỉ dùng ax, ay
            
            accel_f = alpha * prevAccel + (1-alpha) * accel_world;
        end
        prevAccel = accel_f;
        
        % --- EKF Prediction: State Extrapolation ---
        theta_prev = x_ekf(5);
        
        % Vận tốc góc gz từ Gyro/CoppeliaSim
        gz_valid = ~isnan(gz);
        omega_z = x_ekf(5); % Giả sử omega_z là theta_dot
        if gz_valid
            omega_z = gz; % Sử dụng phép đo trực tiếp
        end
        
        % 1. Vị trí
        x_pred(1) = x_ekf(1) + x_ekf(3)*nowT + 0.5 * accel_f(1) * nowT^2;
        x_pred(2) = x_ekf(2) + x_ekf(4)*nowT + 0.5 * accel_f(2) * nowT^2;
        
        % 2. Vận tốc
        x_pred(3) = x_ekf(3) + accel_f(1) * nowT;
        x_pred(4) = x_ekf(4) + accel_f(2) * nowT;
        
        % 3. Góc quay (Sử dụng vận tốc góc gz)
        if gz_valid
            x_pred(5) = wrapToPi(theta_prev + omega_z * nowT);
        else
            % Nếu không có gz, giữ nguyên hoặc dùng một mô hình quay khác
            x_pred(5) = theta_prev; 
        end
        x_pred = x_pred(:);
        
       % --- EKF Prediction: State Extrapolation ---
% ... (Giữ nguyên phần tính x_pred) ...

% --- Jacobian F (Ma trận chuyển trạng thái tuyến tính) ---
theta_prev = x_ekf(5);
ax_raw = accel_raw(1); % Lấy ax thô (Body Frame)
ay_raw = accel_raw(2); % Lấy ay thô (Body Frame)

% 1. Tính các đạo hàm riêng của gia tốc theo góc Theta
% d(ax)/d(theta) = -ax_raw * sin(theta) - ay_raw * cos(theta)
dax_dtheta = -ax_raw * sin(theta_prev) - ay_raw * cos(theta_prev);
% d(ay)/d(theta) = ax_raw * cos(theta) - ay_raw * sin(theta)
day_dtheta = ax_raw * cos(theta_prev) - ay_raw * sin(theta_prev);

F = eye(5);

% Cập nhật các thành phần Vị trí theo Vận tốc (x, y theo vx, vy)
F(1,3) = nowT;
F(2,4) = nowT;

% Cập nhật các thành phần Vị trí theo Góc (x, y theo theta)
F(1,5) = 0.5 * dax_dtheta * nowT^2;
F(2,5) = 0.5 * day_dtheta * nowT^2;

% Cập nhật các thành phần Vận tốc theo Góc (vx, vy theo theta)
F(3,5) = dax_dtheta * nowT;
F(4,5) = day_dtheta * nowT;

% F(5,5) = 1 (vì d(theta_pred)/d(theta) = 1)
% F(5,1-4) = 0 (vì theta không phụ thuộc x, y, vx, vy)

P_pred = F * P * F' + Q;
       %% --- Adaptive EKF update ---
        Nmax = 100; % Giữ nguyên Nmax
        [ICPerror, Nk] = estimateLidarNoise(prevScanForNoise, scan, Nmax);
        prevScanForNoise = scan;
        
        [ret_hdop, hdop] = vrep.simxGetFloatSignal(clientID,'HDOP',vrep.simx_opmode_buffer);
        [ret_vdop, vdop] = vrep.simxGetFloatSignal(clientID,'VDOP',vrep.simx_opmode_buffer);
        if ret_hdop ~= 0 || isnan(hdop), hdop = 2.0; end
        if ret_vdop ~= 0 || isnan(vdop), vdop = 2.0; end
        
        % Tính toán Rk (Theo lý thuyết B: Adaptive R)
        sigma_xyk = hdop * sigma_r; % GPS Horizontal Variance
        sigma_zk = vdop * sigma_r;  % GPS Vertical Variance (Không dùng)
        
        % LRk = ICPerror + (Nmax - Nk)
        LRk_index = ICPerror + (Nmax - Nk); 
        
        % Chuẩn hóa/Giới hạn LRk_index để không quá lớn
        % Giới hạn tối đa (max_noise_LIDAR) để EKF không bỏ qua LIDAR hoàn toàn
        max_noise_LIDAR = 0.5; 
        LRk = min(max_noise_LIDAR, max(0.01, LRk_index)); 
        
        measAvailGPS = all(~isnan([x_gps, y_gps]));
        measAvailLIDAR = ~isempty(scan) && Nk > 5; % Cần ít nhất 5 features tốt
        
        % Lấy tọa độ GPS (World -> SLAM Frame)
        gps_slam = [NaN; NaN];
        if measAvailGPS
            R_toSlam = [cos(-originYaw) -sin(-originYaw);
                        sin(-originYaw)  cos(-originYaw)];
            gps_world = [x_gps; y_gps];
            gps_slam = R_toSlam * (gps_world - originPos(:));
            gps_traj = [gps_traj; gps_slam'];
            lastGPS.pos = gps_world;
            lastGPS.time = tic;
        end
        
        % Lấy tọa độ SLAM (đã có trong SLAM Frame)
        plidar = [NaN; NaN];
        try
            slamPoses = poses(slamObj); % Nx3 [x y theta]
            if ~isempty(slamPoses)
                plidar = slamPoses(end,1:2)';
            end
        catch
            % ignore
        end
        
% Vị trí INS dự đoán (trích xuất từ trạng thái EKF dự đoán)
P_INS_pred = x_pred(1:2); 

z = []; 
H = []; 
Rk_diag = [];

if measAvailGPS
    % Tính Sai số GPS (Theo lý thuyết: P_INS - P_GPS)
    delta_P_GPS = P_INS_pred - gps_slam; 
    z = [z; delta_P_GPS(1); delta_P_GPS(2)];
    
    % H là ma trận Jacobian ánh xạ trạng thái tuyệt đối đến sai số
    % Vì z = P_INS - P_GPS, d(z)/d(x) = 1, d(z)/d(y) = 1.
    H = [H; 
         1 0 0 0 0;  
         0 1 0 0 0]; 
    
    Rk_diag = [Rk_diag; sigma_xyk^2; sigma_xyk^2]; % Phương sai GPS
end

if measAvailLIDAR && all(isfinite(plidar))
    % Tính Sai số LIDAR (Theo lý thuyết: P_INS - P_LIDAR)
    delta_P_LIDAR = P_INS_pred - plidar;
    z = [z; delta_P_LIDAR(1); delta_P_LIDAR(2)];
    
    % H tương tự: ma trận Jacobian ánh xạ trạng thái tuyệt đối đến sai số
    H = [H; 
         1 0 0 0 0;  
         0 1 0 0 0];
         
    Rk_diag = [Rk_diag; LRk; LRk]; % Phương sai LIDAR đã điều chỉnh
end

Rk = diag(Rk_diag);
        
% Thực hiện Cập nhật EKF
if ~isempty(z)
    % 1. Innovation Covariance
    S = H * P_pred * H' + Rk;
    
    % 2. Kalman Gain
    K = P_pred * H' / S; 
    
    % 3. State Update
    
    % Sai số Đổi mới (Innovation): Theo lý thuyết là z.
    Innovation = z;
    
    % Cập nhật Trạng thái
    % LỖI TÍCH LŨY: x_ekf = x_pred + K * Innovation;
    % KHẮC PHỤC: Cần ĐẢO DẤU sai số để kéo P_INS về phía P_Sensor.
    % Sai số Đổi mới KHẮC PHỤC: P_Sensor - P_INS_pred = -z
    x_ekf = x_pred - K * Innovation; % 
    
    % 4. Covariance Update (Dạng Joseph ổn định hơn)
    I = eye(size(P_pred));
    P = (I - K * H) * P_pred * (I - K * H)' + K * Rk * K';
    
    x_ekf(5) = wrapToPi(x_ekf(5));
else
    % Nếu không có phép đo nào, giữ nguyên giá trị dự đoán
    x_ekf = x_pred;
    P = P_pred;
end
ekf_traj = [ekf_traj; x_ekf(1:2)'];

        %% --- Plot SLAM map & trajectories (nhanh) ---
        cla(axMap);
        try
            show(slamObj,'Parent',axMap,'Poses','on');
        catch
            % ignore if show fails
        end

        hold(axMap,'on');
        if ~isempty(gps_traj)
            plot(axMap, gps_traj(:,1), gps_traj(:,2), 'r', 'LineWidth',1.2);
        end
        if ~isempty(ekf_traj)
            plot(axMap, ekf_traj(:,1), ekf_traj(:,2), 'g', 'LineWidth',1.5);
        end
       
        legend(axMap, {'SLAM map','GPS','AKF','SLAM poses'}, 'Location','best');
        hold(axMap,'off');
        drawnow limitrate;

        % timing control
        elapsed = toc(loopStart);
        pause(max(0, dt - elapsed));

    catch ME
        % Nếu có lỗi (ví dụ mất kết nối), hiển thị cảnh báo rồi thoát vòng lặp an toàn
        warning('Lỗi trong vòng lặp: %s. Đang thoát vòng lặp...', ME.message);
        break;
    end
   

end

%% --- Sau khi vòng while kết thúc: dựng occupancy map 1 lần ---
try
    disp('--- Vòng lặp kết thúc, đang dựng occupancy map cuối cùng ---');
    [scansSLAM, poses] = scansAndPoses(slamObj);
    % Nếu không có scans/poses, buildMap sẽ báo lỗi -> bắt trong try
    occMap = buildMap(scansSLAM, poses, 10, maxRange); % resolution = 10 cells/m
    figure('Name','Occupancy Map (Final)');
    show(occMap);
    save('occupancy.mat','occMap','-v7.3'); 
    title('Occupancy Map');
catch ME
    warning('Không thể tạo bản đồ occupancy sau khi dừng: %s', ME.message);
end
[scans, sp] = scansAndPoses(slamObj);
slam_traj = sp(:,1:2);
save('traj_data.mat','gps_traj','ekf_traj','slam_traj','gt_traj','-v7.3');
analyzeResults(gps_traj, ekf_traj, slam_traj, gt_traj);
%% --- Cleanup ---
try
    if vrep.simxGetConnectionId(clientID) > -1
        vrep.simxFinish(clientID);
    end
catch
    % ignore
end
try
    vrep.delete();
catch
    % ignore
end
disp('Đã ngắt kết nối.');

%% ====== Callbacks ======
    function pressKey(k, ~, ~)
        switch k
            case 'W', keys = resetKeys(); keys.W = true;
            case 'X', keys = resetKeys(); keys.X = true;
            case 'A', keys = resetKeys(); keys.A = true;
            case 'D', keys = resetKeys(); keys.D = true;
            case 'S', keys = resetKeys();
        end
    end

    function ks = resetKeys()
        ks = struct('W',false,'X',false,'A',false,'D',false);
    end

    function onKeyPress(~, event)
        switch upper(event.Key)
            case 'W', keys.W = true;
            case 'X', keys.X = true;
            case 'A', keys.A = true;
            case 'D', keys.D = true;
            case 'S', keys = resetKeys();
        end
    end

    function onKeyRelease(~, event)
        switch upper(event.Key)
            case 'W', keys.W = false;
            case 'X', keys.X = false;
            case 'A', keys.A = false;
            case 'D', keys.D = false;
        end
    end

    function onClose(~,~)
        try delete(gcf); catch; end
    end

end

