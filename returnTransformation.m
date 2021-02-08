function output = returnTransformation(n)
%calculateFK
%This file derives the forward kinematics
d1=6.6;
d2=12;
d3=12.7;
d4=3.2;
d5=6.6;   
output=-1;

syms theta1 theta2 theta3 theta4 theta5;

T01=[cos(theta1),0,sin(theta1),0; sin(theta1), 0,-cos(theta1),0;0,1,0,d1;0,0,0,1];%YES
T12=[cos(theta2),-sin(theta2),0,d2*cos(theta2);sin(theta2),cos(theta2),0,d2*sin(theta2);0,0,1,0;0,0,0,1;];%YES
T23=[cos(theta3),-sin(theta3),0,d3*cos(theta3);sin(theta3),cos(theta3),0,d3*sin(theta3);0,0,1,0;0,0,0,1;];%YES
%T34=[cos(theta4),0,sin(theta4),0;sin(theta4),0,-cos(theta4),0;0,1,0,0;0,0,0,1;];%
T34=[-sin(theta4),0,cos(theta4),0;cos(theta4),0,sin(theta4),0;0,1,0,0;0,0,0,1;];%owen switch sin and cos
T45=[cos(theta5),-sin(theta5),0,0;sin(theta5),cos(theta5),0,0;0,0,1,d4;0,0,0,1;];%LOOK AT THIS-- APPARENTLY theta5 SHOULD BE 0 BECAUSE MAKES NO DIFF TO POSITION
T5E=[1,0,0,0;0,1,0,0;0,0,1,d5;0,0,0,1];

%Find final transformation matrix

T02=T01*T12;
T03=T01*T12*T23;
T04=T01*T12*T23*T34;
T05=T01*T12*T23*T34*T45;
T0E=T01*T12*T23*T34*T45*T5E;

switch n
    case 1
        output=[T01(1,4),T01(2,4),T01(3,4)];%T01
    case 2
        output=[T02(1,4),T02(2,4),T02(3,4)];%T02
    case 3
        output=[T03(1,4),T03(2,4),T03(3,4)];%T03
    case 4
        output=[T04(1,4),T04(2,4),T04(3,4)];%T04
    case 5
        output=[T05(1,4),T05(2,4),T05(3,4)];%T05
    case 6
        output=[T0E(1,4),T0E(2,4),T0E(3,4)];%T0E
end
