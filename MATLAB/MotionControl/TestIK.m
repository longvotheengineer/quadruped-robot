theta1 = pi/2;
theta2 = pi/2;
theta3 = pi/2;

L1 = 0.05;
L2 = 0.15;
L3 = 0.1;
L = 0.5;
W = 0.2;

L/2 + L3*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)) + L2*sin(theta2);
W/2 + L3*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)) + L1*cos(theta1) - L2*cos(theta2)*sin(theta1);
L1*sin(theta1) - L3*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + L2*cos(theta1)*cos(theta2);

% L1*sin(theta1) - L3*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + L2*cos(theta1)*cos(theta2)
% L2*cos(theta2)*sin(theta1) - L1*cos(theta1) - L3*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1))
% L3*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)) + L2*sin(theta2)

