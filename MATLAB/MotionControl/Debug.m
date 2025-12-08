% chọn một cấu hình test (ví dụ first sample)
qtest = q_row(1,:);   % 1x3

% 1) lấy từng A matrix (RTB không áp base/tool)
Aseq = leg.A([1 2 3], qtest);   % 4x4x3

% 2) compose theo RTB
T_links = eye(4);
for i=1:3
    T_links = T_links * Aseq(:,:,i);
end
T_rtb_full = leg.base * T_links * leg.tool;
pos_rtb = transl(T_rtb_full)   % [x; y; z] theo RTB

% 3) tính FK tự viết (bạn có hàm ForwardKinematics)
[x_fk,y_fk,z_fk] = ForwardKinematics(qtest(1), qtest(2), qtest(3));
pos_fk = [x_fk; y_fk; z_fk]

% 4) in ra hiệu
disp('RTB pos vs FK pos:')
disp([pos_rtb, pos_fk, pos_rtb - pos_fk])
