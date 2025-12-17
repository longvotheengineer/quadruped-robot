function theta_i = Gait(robot_motion, robot_config)
    waypoint_n = 50;
    waypoint_n_zero = 500;
    
    switch robot_motion.gait
        case "ZERO"
            [x, y, z] = ForwardKinematics(0, 0, 0, robot_config);
            switch robot_config.leg_type
                case "left-front"
                    pos_A = [x, y, z];
                    pos_B = [0.25, 0.15, -0.24];
                case "left-behind"
                    pos_A = [x, y, z];
                    pos_B = [-0.25, 0.15, -0.24];
                case "right-front"
                    pos_A = [x, y, z];
                    pos_B = [0.25, -0.15, -0.24];
                case "right-behind"
                    pos_A = [x, y, z];
                    pos_B = [-0.25, -0.15, -0.24];
                otherwise
            end
        case "FORWARD"
            switch robot_config.leg_type
                case "left-front"
                    pos_A = [0.25, 0.15, -0.24];
                    pos_B = [0.25, 0.15, -0.20];
                    pos_C = [0.29, 0.15, -0.20];
                    pos_D = [0.29, 0.15, -0.24];
                    % pos_A = [0.24, 0.15, -0.24];
                    % pos_B = [0.24, 0.15, -0.20];
                    % pos_C = [0.30, 0.15, -0.20];
                    % pos_D = [0.30, 0.15, -0.24];
                case "left-behind"
                    pos_A = [-0.25, 0.15, -0.24];
                    pos_B = [-0.25, 0.15, -0.20];
                    pos_C = [-0.21, 0.15, -0.20];
                    pos_D = [-0.21, 0.15, -0.24];
                    % pos_A = [-0.24, 0.15, -0.24];
                    % pos_B = [-0.24, 0.15, -0.20];
                    % pos_C = [-0.18, 0.15, -0.20];
                    % pos_D = [-0.18, 0.15, -0.24];
                case "right-front"
                    pos_A = [0.25, -0.15, -0.24];
                    pos_B = [0.25, -0.15, -0.20];
                    pos_C = [0.29, -0.15, -0.20];
                    pos_D = [0.29, -0.15, -0.24];
                    % pos_A = [0.24, -0.15, -0.24];
                    % pos_B = [0.24, -0.15, -0.20];
                    % pos_C = [0.30, -0.15, -0.20];
                    % pos_D = [0.30, -0.15, -0.24];
                case "right-behind"
                    pos_A = [-0.25, -0.15, -0.24];
                    pos_B = [-0.25, -0.15, -0.20];
                    pos_C = [-0.21, -0.15, -0.20];
                    pos_D = [-0.21, -0.15, -0.24];
                    % pos_A = [-0.24, -0.15, -0.24];
                    % pos_B = [-0.24, -0.15, -0.20];
                    % pos_C = [-0.18, -0.15, -0.20];
                    % pos_D = [-0.18, -0.15, -0.24];
                otherwise
            end
        case "BACKWARD"
            switch robot_config.leg_type
                case "left-front"                                       
                    pos_A = [0.29, 0.15, -0.24];
                    pos_B = [0.29, 0.15, -0.20];
                    pos_C = [0.25, 0.15, -0.20];
                    pos_D = [0.25, 0.15, -0.24];
                case "left-behind"
                    pos_D = [-0.25, 0.15, -0.24];
                    pos_C = [-0.25, 0.15, -0.20];
                    pos_B = [-0.21, 0.15, -0.20];
                    pos_A = [-0.21, 0.15, -0.24];
                case "right-front"
                    pos_D = [0.25, -0.15, -0.24];
                    pos_C = [0.25, -0.15, -0.20];
                    pos_B = [0.29, -0.15, -0.20];
                    pos_A = [0.29, -0.15, -0.24];
                case "right-behind"
                    pos_D = [-0.25, -0.15, -0.24];
                    pos_C = [-0.25, -0.15, -0.20];
                    pos_B = [-0.21, -0.15, -0.20];
                    pos_A = [-0.21, -0.15, -0.24];
                otherwise
            end
        case "TURN_LEFT"
            step_angle  =  0.15; 
            z_ground    = -0.22;  
            z_lift      = -0.18;

            switch robot_config.leg_type
                case "left-front"
                    neutral_pos = [0.25, 0.15]; 
                case "left-behind"
                    neutral_pos = [-0.25, 0.15];
                case "right-front"
                    neutral_pos = [0.25, -0.15];
                case "right-behind"
                    neutral_pos = [-0.25, -0.15];
                otherwise
                    neutral_pos = [0, 0];
            end
            
            cos_theta_start = cos(-step_angle); 
            sin_theta_start = sin(-step_angle);            
            cos_theta_end   = cos(step_angle); 
            sin_theta_end   = sin(step_angle);
            
            x0 = neutral_pos(1);
            y0 = neutral_pos(2);
            xA = x0 * cos_theta_start - y0 * sin_theta_start;
            yA = x0 * sin_theta_start + y0 * cos_theta_start;           
            xD = x0 * cos_theta_end - y0 * sin_theta_end;
            yD = x0 * sin_theta_end + y0 * cos_theta_end;
            
            pos_A = [xA, yA, z_ground];
            pos_B = [xA, yA, z_lift];
            pos_C = [xD, yD, z_lift];
            pos_D = [xD, yD, z_ground];
        case "TURN_RIGHT"
            step_angle  =  0.15; 
            z_ground    = -0.22;
            z_lift      = -0.18;

            switch robot_config.leg_type
                case "left-front"
                    neutral_pos = [0.25, 0.15]; 
                case "left-behind"
                    neutral_pos = [-0.25, 0.15];
                case "right-front"
                    neutral_pos = [0.25, -0.15];
                case "right-behind"
                    neutral_pos = [-0.25, -0.15];
                otherwise
                    neutral_pos = [0, 0];
            end
            
            cos_theta_start = cos(step_angle);
            sin_theta_start = sin(step_angle);            
            cos_theta_end   = cos(-step_angle);
            sin_theta_end   = sin(-step_angle);
            
            x0 = neutral_pos(1);
            y0 = neutral_pos(2);           
            xA = x0 * cos_theta_start - y0 * sin_theta_start;
            yA = x0 * sin_theta_start + y0 * cos_theta_start;            
            xD = x0 * cos_theta_end - y0 * sin_theta_end;
            yD = x0 * sin_theta_end + y0 * cos_theta_end;
            
            pos_A = [xA, yA, z_ground];
            pos_B = [xA, yA, z_lift];
            pos_C = [xD, yD, z_lift];
            pos_D = [xD, yD, z_ground];
        case "WALK"
            % 1. Stability Parameters
            z_neutral = -0.18; % Safe height
            
            % Dimensions
            L_half = 0.25; 
            W_half = 0.15; 
            
            % 2. Determine Leg Identity & Phase Offset
            % We order them: LF -> RB -> LB -> RF (Standard Creep)
            switch robot_config.leg_type
                case "left-front"
                    leg_sign_x =  1; leg_sign_y =  1;
                    phase_offset = 0.00; 
                case "right-behind"
                    leg_sign_x = -1; leg_sign_y = -1;
                    phase_offset = 0.25;
                case "left-behind"
                    leg_sign_x = -1; leg_sign_y =  1;
                    phase_offset = 0.50;
                case "right-front"
                    leg_sign_x =  1; leg_sign_y = -1;
                    phase_offset = 0.75;
                otherwise
                    leg_sign_x = 0; leg_sign_y = 0;
                    phase_offset = 0;
            end
            
            neutral_pos = [leg_sign_x * L_half, ...
                           leg_sign_y * W_half, ...
                           z_neutral];

        otherwise
    end
        
    if robot_motion.gait == "ZERO"
        theta_iA{1} = [0, 0, 0];
        theta_iA{2} = [0, 0, 0];
        theta_iA{3} = [0, 0, 0];
        theta_iA{4} = [0, 0, 0];
        theta_iB{1} = [0,  pi, -pi/4];
        theta_iB{2} = [0,  pi, -pi/4];
        theta_iB{3} = [0, -pi,  pi/4];
        theta_iB{4} = [0, -pi,  pi/4];
        waypoint_jointspace_AB1{1} = linspace(theta_iA{1}(1), theta_iB{1}(1), waypoint_n_zero);
        waypoint_jointspace_AB1{2} = linspace(theta_iA{2}(1), theta_iB{2}(1), waypoint_n_zero);
        waypoint_jointspace_AB1{3} = linspace(theta_iA{3}(1), theta_iB{3}(1), waypoint_n_zero);
        waypoint_jointspace_AB1{4} = linspace(theta_iA{4}(1), theta_iB{4}(1), waypoint_n_zero);
        waypoint_jointspace_AB2{1} = linspace(theta_iA{1}(2), theta_iB{1}(2), waypoint_n_zero);
        waypoint_jointspace_AB2{2} = linspace(theta_iA{2}(2), theta_iB{2}(2), waypoint_n_zero);
        waypoint_jointspace_AB2{3} = linspace(theta_iA{3}(2), theta_iB{3}(2), waypoint_n_zero);
        waypoint_jointspace_AB2{4} = linspace(theta_iA{4}(2), theta_iB{4}(2), waypoint_n_zero);
        waypoint_jointspace_AB3{1} = linspace(theta_iA{1}(3), theta_iB{1}(3), waypoint_n_zero);
        waypoint_jointspace_AB3{2} = linspace(theta_iA{2}(3), theta_iB{2}(3), waypoint_n_zero);
        waypoint_jointspace_AB3{3} = linspace(theta_iA{3}(3), theta_iB{3}(3), waypoint_n_zero);
        waypoint_jointspace_AB3{4} = linspace(theta_iA{4}(3), theta_iB{4}(3), waypoint_n_zero);
        
        theta_i = cell(1, 4);
        for i = 1 : waypoint_n_zero
            theta_i{i} = zeros(waypoint_n_zero, 3);
        end
        for i = 1 : waypoint_n_zero
            theta_i{1}(i, :) = [waypoint_jointspace_AB1{1}(i), waypoint_jointspace_AB2{1}(i), waypoint_jointspace_AB3{1}(i)];
            theta_i{2}(i, :) = [waypoint_jointspace_AB1{2}(i), waypoint_jointspace_AB2{2}(i), waypoint_jointspace_AB3{2}(i)];
            theta_i{3}(i, :) = [waypoint_jointspace_AB1{3}(i), waypoint_jointspace_AB2{3}(i), waypoint_jointspace_AB3{3}(i)];
            theta_i{4}(i, :) = [waypoint_jointspace_AB1{4}(i), waypoint_jointspace_AB2{4}(i), waypoint_jointspace_AB3{4}(i)];
        end    
    
    
    elseif robot_motion.gait == "WALK"
        % === STABLE GAIT: 4-BEAT CRAWL ===
        
        walk_points = 200; % Resolution
        num_cycles  = 2;   % How many steps to take
        
        % Motion Settings
        step_len    = 0.08; % 8cm step (Safe)
        step_height = 0.04; % 4cm lift
        duty_factor = 0.75; % 75% Stance, 25% Swing
        
        % Generate Time Vector (Normalized 0 -> num_cycles)
        t_total = linspace(0, num_cycles, walk_points)';
        
        theta_i = zeros(walk_points, 3);
        
        for i = 1 : walk_points
            % 1. Calculate Local Phase for this specific leg
            % t_total(i) runs 0 -> 2. We add offset.
            % Modulo 1 keeps it in the 0.0 -> 1.0 range (The Loop)
            current_t = t_total(i) + phase_offset;
            cycle_phase = mod(current_t, 1);
            
            dx = 0;
            dy = 0;
            dz = 0;
            
            % 2. Determine State: Swing or Stance?
            if cycle_phase < (1 - duty_factor)
                % === SWING PHASE (Air) ===
                % Phase goes from 0 to 0.25
                % Leg moves from Back (-step/2) to Front (+step/2)
                
                swing_p = cycle_phase / (1 - duty_factor); % Normalize 0->1
                
                % X Motion: Interpolate Back to Front
                % Start: -step_len/2, End: +step_len/2
                dx = -step_len/2 + (step_len * swing_p);
                
                % Z Motion: Parabola (Lift)
                % sin(0)=0, sin(pi)=1, sin(2pi)=0
                dz = step_height * sin(swing_p * pi);
                
            else
                % === STANCE PHASE (Ground) ===
                % Phase goes from 0.25 to 1.0
                % Leg moves from Front (+step/2) to Back (-step/2)
                % This pushes the body forward!
                
                stance_p = (cycle_phase - (1 - duty_factor)) / duty_factor; % Normalize 0->1
                
                % X Motion: Interpolate Front to Back
                dx = step_len/2 - (step_len * stance_p);
                
                % Z Motion: 0 (Stay on ground)
                dz = 0; 
            end
            
            % 3. Apply Offsets to Neutral Position
            px = neutral_pos(1) + dx;
            py = neutral_pos(2) + dy;
            pz = neutral_pos(3) + dz;
            
            % 4. Solve IK
            [theta1, theta2, theta3] = InverseKinematics(px, py, pz, robot_config);
            theta_i(i, :) = [theta1 theta2 theta3];
        end



    else         
        waypoint_AB1 = linspace(pos_A(1), pos_B(1), waypoint_n);
        waypoint_AB2 = linspace(pos_A(2), pos_B(2), waypoint_n);
        waypoint_AB3 = linspace(pos_A(3), pos_B(3), waypoint_n);
        waypoint_BC1 = linspace(pos_B(1), pos_C(1), waypoint_n);
        waypoint_BC2 = linspace(pos_B(2), pos_C(2), waypoint_n);
        waypoint_BC3 = linspace(pos_B(3), pos_C(3), waypoint_n);
        waypoint_CD1 = linspace(pos_C(1), pos_D(1), waypoint_n);
        waypoint_CD2 = linspace(pos_C(2), pos_D(2), waypoint_n);
        waypoint_CD3 = linspace(pos_C(3), pos_D(3), waypoint_n);
        waypoint_DA1 = linspace(pos_D(1), pos_A(1), waypoint_n);
        waypoint_DA2 = linspace(pos_D(2), pos_A(2), waypoint_n);
        waypoint_DA3 = linspace(pos_D(3), pos_A(3), waypoint_n);
        waypoint_AB = [waypoint_AB1' waypoint_AB2' waypoint_AB3'];
        waypoint_BC = [waypoint_BC1' waypoint_BC2' waypoint_BC3'];
        waypoint_CD = [waypoint_CD1' waypoint_CD2' waypoint_CD3'];
        waypoint_DA = [waypoint_DA1' waypoint_DA2' waypoint_DA3'];
        waypoint = [waypoint_AB; waypoint_BC; waypoint_CD; waypoint_DA];

        theta_i = zeros(size(waypoint, 1), 3);
        for i = 1 : size(waypoint, 1)
            px = waypoint(i, 1);
            py = waypoint(i, 2);
            pz = waypoint(i, 3);
            [theta1, theta2, theta3] = InverseKinematics(px, py, pz, robot_config);
            theta_i(i, :) = [theta1 theta2 theta3]; 
        end
    end         
end