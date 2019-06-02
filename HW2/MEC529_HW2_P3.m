% MEC529 Matlab Homework 2 Problem 3 Codes Created by Yongxin Guo
addpath('/Users/guoyongxin/Desktop/Assignment_Academics/Senior_Second semester/MEC529/Myfunctions');

close all
clear
clc

%-----test for question (a)-------
axis = [0;1;0];
angle = pi/4;
Rot = AxisAngle_to_Rot(axis,angle);
disp(Rot);
%-----test ends-------------------

%-----test for question (b)-------
%take the rotation matrix obtained from test for (a) to see if we can get
%the same axis-angle representation that were the inputs for test (a)
[axis_b,angle_b] = Rot_to_AxisAngle(Rot);
disp("axis of rotation： ");
disp(axis_b);
disp("angle of rotation: ");
disp(angle_b);
%-----test ends-------------------

%-----question (c)----------------
% assign the axis of rotation and angle of rotation in a reversed order as
% stated in the problem statement.
% rotation v (to frame v)
theta1 = pi/6;
axis1 = [0;1;0];
% rotation iv (to frame iv)
theta2 = -pi/7;
axis2 = [1;0;0];
% rotation iii (to frame iii)
theta3 = pi/4;
axis3 = [0;0;1];
% rotation ii (to frame ii)
theta4 = -5*pi/8;
axis4 = [0;1;0];
% rotation i (to frame i)
theta5 = 3*pi/8;
axis5 = [0;0;1];

% multiply the rotation matrix obtained from function in question (a) in a
% reversed order of the sequence of rotations specified in the problem
% statement or the order specified above. (This sequence of rotations
% are so-called Euler angles around the moving axis)
R_v_to_iv = AxisAngle_to_Rot(axis1,theta1);
R_iv_to_iii = AxisAngle_to_Rot(axis2,theta2);
R_iii_to_ii = AxisAngle_to_Rot(axis3,theta3);
R_ii_to_i = AxisAngle_to_Rot(axis4,theta4);
R_i_to_w = AxisAngle_to_Rot(axis5,theta5);
% w means the world frame.

% multiply the above elementary rotation matrices together 
R_total = R_i_to_w*R_ii_to_i*R_iii_to_ii*R_iv_to_iii*R_v_to_iv;
disp(R_total);
% input the total rotation matrix into function in question (b) to get the
% axis-angle representation
[axis_c,angle_c] = Rot_to_AxisAngle(R_total);
disp("In question c, the axis of rotation is: ");
disp(axis_c);
disp("the angle of rotation is: ");
disp(angle_c);





