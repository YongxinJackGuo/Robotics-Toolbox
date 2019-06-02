% MEC529 Matlab Homework 3 Problem 2 part(ii) Codes Created by Yongxin Guo
addpath('/Users/guoyongxin/Desktop/Assignment_Academics/Senior_Second semester/MEC529/Myfunctions');

close all
clear
clc


% assign configuration constants.
l0 = 0.8;
l1 = 0.8;
l2 = 0.5;
theta = (pi/3)*ones(6,1);
% create gst0 matrix.
R0 = eye(3);
P0 = [0;l1+l2;l0];
gst0 = [R0,P0;[0 0 0],1];
% create axis of motion.
axis1 = [0;0;1];
axis2 = [-1;0;0];
axis3 = [-1;0;0];
axis4 = [0;0;1];
axis5 = [-1;0;0];
axis6 = [0;1;0];
axis_joints = [axis1,axis2,axis3,axis4,axis5,axis6];
% create q_matrix.
q1 = [0;0;l0];
q2 = [0;0;l0];
q3 = [0;l1;l0];
q4 = [0;l1+l2;l0];
q5 = [0;l1+l2;l0];
q6 = [0;l1+l2;l0];
q_joints = [q1,q2,q3,q4,q5,q6];
% create matrix for the type of joints
type_joints = ["R";"R";"R";"R";"R";"R"];
% compute gst(theta)
gst_theta = manipdkin(gst0, axis_joints, q_joints, type_joints, theta);
disp("The gst(theta) transformation matrix is shown below: ");
disp(gst_theta);


