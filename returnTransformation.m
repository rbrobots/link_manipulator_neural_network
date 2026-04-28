function position = returnTransformation(target_joint)
% Returns the [x, y, z] position of the specified joint frame for a 5-DOF
% robot arm using DH-parameter forward kinematics.
%
% target_joint: 1–5 returns the origin of that joint frame;
%               6 returns the end-effector position.

% Link lengths / offsets (cm)
d1 = 6.6;   % base height
d2 = 12;    % link 1 length
d3 = 12.7;  % link 2 length
d4 = 3.2;   % link 3 length
d5 = 6.6;   % end-effector offset

position = -1;  % default: invalid joint index

syms theta1 theta2 theta3 theta4 theta5;

% DH transformation matrices for each consecutive joint pair
T01 = [cos(theta1),  0,  sin(theta1), 0;
       sin(theta1),  0, -cos(theta1), 0;
       0,            1,  0,           d1;
       0,            0,  0,           1];

T12 = [cos(theta2), -sin(theta2), 0, d2*cos(theta2);
       sin(theta2),  cos(theta2), 0, d2*sin(theta2);
       0,            0,           1, 0;
       0,            0,           0, 1];

T23 = [cos(theta3), -sin(theta3), 0, d3*cos(theta3);
       sin(theta3),  cos(theta3), 0, d3*sin(theta3);
       0,            0,           1, 0;
       0,            0,           0, 1];

T34 = [-sin(theta4), 0,  cos(theta4), 0;
        cos(theta4), 0,  sin(theta4), 0;
        0,           1,  0,           0;
        0,           0,  0,           1];

T45 = [cos(theta5), -sin(theta5), 0, 0;
       sin(theta5),  cos(theta5), 0, 0;
       0,            0,           1, d4;
       0,            0,           0, 1];

T5E = [1, 0, 0, 0;
       0, 1, 0, 0;
       0, 0, 1, d5;
       0, 0, 0, 1];

% Cumulative transforms from base (frame 0) to each joint frame
T02 = T01 * T12;
T03 = T02 * T23;
T04 = T03 * T34;
T05 = T04 * T45;
T0E = T05 * T5E;

% Extract the translation (position) column from the requested transform
switch target_joint
    case 1; position = [T01(1,4), T01(2,4), T01(3,4)];
    case 2; position = [T02(1,4), T02(2,4), T02(3,4)];
    case 3; position = [T03(1,4), T03(2,4), T03(3,4)];
    case 4; position = [T04(1,4), T04(2,4), T04(3,4)];
    case 5; position = [T05(1,4), T05(2,4), T05(3,4)];
    case 6; position = [T0E(1,4), T0E(2,4), T0E(3,4)];
end
