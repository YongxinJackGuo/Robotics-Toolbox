% MEC529 Matlab Homework 4 Problem 2.4.b Codes Created by Yongxin Guo
addpath('/Users/guoyongxin/Desktop/Assignment_Academics/Senior_Second semester/MEC529/Myfunctions');

close all
clear
clc

l0 = 1;
l1 = 1;
l2 = 0.5;
TtoP = 0.06;
thetai = pi/3;
thetaidot = 1;

% each axis of joints.
w1 = [0;0;1];
w2 = [-1;0;0];
w3 = [-1;0;0];
w4 = [0;0;1];
w5 = [-1;0;0];
axis_joints = [w1,w2,w3,w4,w5];
% each q vecotr of joints.
q1 = [0;0;l0];
q2 = q1;
q3 = [0;l1;l0];
q4 = [0;l1+l2;l0];
q5 = q4;
q_joints = [q1,q2,q3,q4,q5];
% eacht type of joints
type_joints = ["R";"R";"R";"R";"R"];
% each displacement and rates of joints
theta = [thetai; thetai; thetai; thetai; thetai];
thetadot = [thetaidot;thetaidot;thetaidot;thetaidot;thetaidot];
% base transformation matrix
gst0 = [eye(3),[0;l1+l2+TtoP;l0];[0 0 0],1];
% Compute spatial jacobian
Js = SpatialmanipJac(axis_joints, q_joints, type_joints,theta);
% Compute spatial velocity twist
Vs = SpatialVelTwist(Js,thetadot);

% display results
disp("Spatial quantities (expressed in the base frame): ");
disp("Linear velocity of origin of the tool frame: ");
disp(Vs(1:3));
disp("Angular velocity of origin of the tool frame: ");
disp(Vs(4:6));
