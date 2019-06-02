% MEC529 Matlab Midterm Problem 3 Question (a) and (b), IK for elbow manipulator Codes Created by Yongxin Guo
addpath('/Users/guoyongxin/Desktop/Assignment_Academics/Senior_Second semester/MEC529/Myfunctions');

close all
clear
clc
% assign configuration constants.
l0 = 0.4;
l1 = 0.4;
l2 = 0.3;
l3 = 0.05;
R0 = eye(3);
P0 = [0;l1+l2+l3;l0];
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
q2 = q1;
q3 = [0;l1;l0]; % q3 here is q2 in the homework figure.
q4 = [0;l1+l2;l0];
q5 = q4;
q6 = q4; % q4 here is q3 in the homework figure.
q_joints = [q1,q2,q3,q4,q5,q6];
% create matrix for the type of joints
type_joints = ["R";"R";"R";"R";"R";"R"];

%----------------------------Question(a)----------------------------------
disp("***************************Question (a)*********************************");
disp("****************************Starts here*********************************" + newline);
% given transformation matrix.
T_peg = [eye(3),[0.3;0.3;0];[0 0 0],1];
T_hole = [1 0 0 0.4;0 0 1 0.5;0 -1 0 0.1;0 0 0 1];
% feed T_peg and T_Hole into the elbow IK function and get the messages
% displayed and angles.
T_peg_IK_angles = getElbowManipIK(T_peg,gst0,axis_joints,q_joints,type_joints,"peg");
disp(T_peg_IK_angles);
disp("************************************************************************");
disp("************************************************************************");
disp(newline);
T_hole_IK_angles = getElbowManipIK(T_hole,gst0,axis_joints,q_joints,type_joints,"hole");
disp("In radians: ")
disp(T_hole_IK_angles);
disp("In degrees: ");
disp(T_hole_IK_angles*180/pi);

%----------------------------Question(b)----------------------------------
disp("***************************Question (b)*********************************");
disp("****************************Starts here*********************************" + newline);
% select one soln at hole position from question (a) for the target
% configuration as the input of the computation of the joint rates.
% Note that the desired joint rates are [0;0;-0.01;0;0;0];
targetSolnNum = 7; % choose No.8 soln as the target config.
theta = T_hole_IK_angles(:,targetSolnNum); % assign to theta vector. 
Vs = [0;0;-0.01;0;0;0]; % given joint rates.
Js = SpatialmanipJac(axis_joints,q_joints,type_joints,theta); %compute Spatial Jacobian.
thetadot = Js\Vs; % compute joint rate. Note that Js\Vs means inv(Js)*Vs. 
disp("The elbow manipulator is in the following configuration (No." + num2str(targetSolnNum) + " config) when inserting the peg into the hole: ");
disp(theta);
disp("The desired joint angle rates for achieving the target end-effector speed of only -0.01 m/s along z-axis is: ");
disp(thetadot);


