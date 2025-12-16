function analyzeResults(gps_traj, ekf_traj, slam_traj, gt_traj)
    
    if isempty(gt_traj) || isempty(ekf_traj) || isempty(slam_traj)
        disp('Không đủ dữ liệu để phân tích (cần GPS, EKF, SLAM, GT).');
        return;
    end
    
    % Căn chỉnh độ dài quỹ đạo (chọn quỹ đạo ngắn nhất)
    min_len = min([size(gps_traj,1), size(ekf_traj,1), size(slam_traj,1), size(gt_traj,1)]);
    
    if min_len < 2
        disp('Quá ít điểm dữ liệu để phân tích.');
        return;
    end
    
    gps_traj = gps_traj(1:min_len, :);
    ekf_traj = ekf_traj(1:min_len, :);
    slam_traj = slam_traj(1:min_len, :);
    gt_traj = gt_traj(1:min_len, :);
    
    % --- 1. Tính Tổng Quãng Đường Di Chuyển (Total Distance Traveled - TDT) từ GT ---
    
    delta_gt = gt_traj(2:end, :) - gt_traj(1:end-1, :);
    distance_segments = sqrt(sum(delta_gt.^2, 2));
    TDT = sum(distance_segments);
    
    if TDT == 0
        disp('Tổng quãng đường di chuyển (GT) bằng 0. Không thể tính phần trăm sai số.');
        TDT = 1; % Gán 1 để tránh chia cho 0
    end
    
    % --- 2. Tính APE (Absolute Pose Error) cho RMSE (Error) ---
    calculate_error = @(P_est, P_gt) sqrt(sum((P_est - P_gt).^2, 2));
    
    ape_gps = calculate_error(gps_traj, gt_traj);
    ape_ekf = calculate_error(ekf_traj, gt_traj);
    ape_slam = calculate_error(slam_traj, gt_traj);
    
    % Tính RMSE của APE (Error)
    rmse_ape_gps = sqrt(mean(ape_gps.^2));
    rmse_ape_ekf = sqrt(mean(ape_ekf.^2));
    rmse_ape_slam = sqrt(mean(ape_slam.^2));
    
    % --- 3. Tính RPE (Relative Pose Error) cho RMSE (Drift) ---
    % Công thức: RPE_k = || (P_est(k+1) - P_est(k)) - (P_gt(k+1) - P_gt(k)) ||
    calculate_drift = @(P_est, P_gt) sqrt(sum( ((P_est(2:end,:) - P_est(1:end-1,:)) - (P_gt(2:end,:) - P_gt(1:end-1,:))).^2, 2));
    
    rpe_gps = calculate_drift(gps_traj, gt_traj);
    rpe_ekf = calculate_drift(ekf_traj, gt_traj);
    rpe_slam = calculate_drift(slam_traj, gt_traj);
    
    % Tính RMSE của RPE (Đây là giá trị DRIFT/Độ trôi)
    drift_gps = sqrt(mean(rpe_gps.^2));
    drift_ekf = sqrt(mean(rpe_ekf.^2));
    drift_slam = sqrt(mean(rpe_slam.^2));
    
    % --- 4. Tính Drift Rate (% TDT) ---
    % Drift Rate = (Drift / TDT) * 100
    drift_rate_gps = (drift_gps / TDT) * 100;
    drift_rate_ekf = (drift_ekf / TDT) * 100;
    drift_rate_slam = (drift_slam / TDT) * 100;

    % --- 5. Tổng hợp và Hiển thị Kết quả ---
    disp(' ');
    disp('========================================================================');
    disp('           TỔNG HỢP PHÂN TÍCH SAI SỐ (ERROR & DRIFT RATE)');
    disp('========================================================================');
    
    fprintf('Tổng quãng đường di chuyển (Ground Truth): %.2f m\n', TDT);
    disp('------------------------------------------------------------------------');
    
    % Tiêu đề cột
    header = {'Phương pháp', 'Error (RMSE APE)', 'Drift Rate (% TDT)'};
    fprintf('%-15s | %-16s | %-16s\n', header{:});
    disp('------------------------------------------------------------------------');
    
    % GPS
    fprintf('%-15s | %-16.4f | %-16.2f%%\n', 'GPS', rmse_ape_gps, drift_rate_gps);
    
    % SLAM (LiDAR)
    fprintf('%-15s | %-16.4f | %-16.2f%%\n', 'LiDAR (SLAM)', rmse_ape_slam, drift_rate_slam);
    
    % AKF
    fprintf('%-15s | %-16.4f | %-16.2f%%\n', 'AKF (Fusion)', rmse_ape_ekf, drift_rate_ekf);
    
    disp('========================================================================');
end