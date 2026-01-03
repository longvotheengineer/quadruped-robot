classdef AKF < handle
    properties
        x;          % State vector: [x; y; theta; vx; vy] 
        P;          % Covariance matrix (5x5)
        Q;          % Process noise covariance (5x5)
        
        % Parameter Adaptive R 
        sigma_r_gps; 
        N_max;      
        scale_N;     % Hệ số tỉ lệ để chuyển đổi (N_max - Nk) sang phương sai
    end
    
    methods
        function obj = AKF(initial_pose, N_max_val)
            if nargin < 1
                obj.x = [0; 0; 0; 0; 0]; % [x, y, theta, vx, vy]
            else
                % Khởi tạo vận tốc ban đầu là 0
                obj.x = [initial_pose.x; initial_pose.y; initial_pose.theta; 0; 0];
            end
            
            obj.P = eye(5) * 0.1; 
            obj.Q = diag([0.05, 0.05, 0.05, 0.2, 0.2].^2); 
            
            obj.sigma_r_gps = 0.5; 
            if nargin < 2
                obj.N_max = 300;   
            else
                obj.N_max = N_max_val;
            end
            obj.scale_N = 0.001;  
        end

        function prediction(obj, ax, ay, omega, dt)
            if dt <= 0; return; end
            
            theta = obj.x(3);
            vx = obj.x(4);
            vy = obj.x(5); 
            c = cos(theta);
            s = sin(theta);
            
            obj.x(1) = obj.x(1) + (vx * c - vy * s) * dt;
            obj.x(2) = obj.x(2) + (vx * s + vy * c) * dt;
            obj.x(3) = obj.x(3) + omega * dt;
            obj.x(3) = atan2(sin(obj.x(3)), cos(obj.x(3))); 
            obj.x(4) = vx + (ax + vy * omega) * dt; 
            obj.x(5) = vy + (ay - vx * omega) * dt;
            
            %Jacobian F 
            F = eye(5);
            F(1, 3) = (-vx * s - vy * c) * dt;
            F(2, 3) = (vx * c - vy * s) * dt;
            F(1, 4) = c * dt;
            F(2, 4) = s * dt;
            F(1, 5) = -s * dt;
            F(2, 5) = c * dt;
            F(4, 5) = omega * dt; 
            F(5, 4) = -omega * dt;
            obj.P = F * obj.P * F' + obj.Q;
        end
        
        % Adaptive Correction Step
        function correction_adaptive(obj, gps_pos, slam_pos, params)
            z = [];
            H = [];
            R_diag = [];
            if ~isempty(gps_pos) && ~isnan(gps_pos.x)
                z = [z; gps_pos.x; gps_pos.y];
                H_gps = zeros(2, 5); 
                H_gps(1,1) = 1; 
                H_gps(2,2) = 1;
                
                H = [H; H_gps];
                sigma_xy = params.hdop * obj.sigma_r_gps;
                R_diag = [R_diag; sigma_xy^2; sigma_xy^2];
            end
            if ~isempty(slam_pos) && ~isnan(slam_pos.x)
                z = [z; slam_pos.x; slam_pos.y];
                H_slam = zeros(2, 5);
                H_slam(1,1) = 1; 
                H_slam(2,2) = 1;
                
                H = [H; H_slam];
                term_features = max(0, (obj.N_max - params.N_k)); 
                LR = params.icp_error + term_features * obj.scale_N;
                R_diag = [R_diag; LR; LR];
            end
            
            if isempty(z); return; end
            
            % Standard Kalman Update
            R = diag(R_diag);
            S = H * obj.P * H' + R;
            K = obj.P * H' / S;
            
            y_res = z - H * obj.x; 
            obj.x = obj.x + K * y_res;
            
            I = eye(5); % Identity matrix 5x5
            obj.P = (I - K * H) * obj.P;
        end
        
        function pose = get_pose_guess(obj)
            pose = [obj.x(1), obj.x(2), obj.x(3)];
        end
    end
end