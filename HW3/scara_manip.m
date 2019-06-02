% MEC529 Matlab Homework 3 Problem 2 part(i) Codes Created by Yongxin Guo
addpath('/Users/guoyongxin/Desktop/Assignment_Academics/Senior_Second semester/MEC529/Myfunctions');

close all
clear
clc

% assign configuration constants.
l0 = 0.2;
l1 = 0.6;
l2 = 0.3;
theta1 = pi/4;
theta2 = pi/4;
theta3 = pi/4;
theta4 = 0.1;
theta = [theta1;theta2;theta3;theta4];
% create gst0 matrix.
R0 = eye(3);
P0 = [0;l1+l2;l0];
gst0 = [R0,P0;[0 0 0],1];
% create axis of motion.
axis1 = [0;0;1];
axis2 = [0;0;1];
axis3 = [0;0;1];
axis4 = [0;0;1];
axis_joints = [axis1,axis2,axis3,axis4];
% create q_matrix.
q1 = [0;0;0];
q2 = [0;l1;0];
q3 = [0;l1+l2;0];
q4 = [0;0;0];
q_joints = [q1,q2,q3,q4];
% create matrix for the type of joints
type_joints = ["R";"R";"R";"P"];
% compute gst(theta)
gst_theta = manipdkin(gst0, axis_joints, q_joints, type_joints, theta);
disp("The gst(theta) transformation matrix is shown below: ");
disp(gst_theta);


