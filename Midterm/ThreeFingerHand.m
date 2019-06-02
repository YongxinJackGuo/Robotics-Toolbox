% MEC529 Matlab Midterm Problem 2 Question (d): grasp an ellipsoid. Codes Created by Yongxin Guo
addpath('/Users/guoyongxin/Desktop/Assignment_Academics/Senior_Second semester/MEC529/Myfunctions');

close all
clear
clc

%--------------------------part 1: Question(b)----------------------------
% assign variable

% ellipsoid
a = 0.01;
b = 0.03;
c = 0.02;
ellipsoid_para = [a;b;c];

% axis of rotation
w_1 = [1;0;0];
w_2 = [1/sqrt(2);-1/sqrt(2);0];
w_3 = [1/sqrt(2);1/sqrt(2);0];
joint_axis = [w_1,w_1,w_2,w_2,w_2,w_3,w_3,w_3];

% base points
base_1 = [0;0;0];
base_2 = [0.02;0.03;0.01];
base_3 = [0;0.03;0];
base_pts = [base_1,base_2,base_3];

% joint types
joint_types = ["R";"R";"R";"R";"R";"R";"R";"R"];

% body constants.
L1_1 = 0.05;
L2_1 = 0.03;
L1_2 = 0.03;
L2_2 = 0.03;
L3_2 = 0.02;
L1_3 = 0.03;
L2_3 = 0.03;
L3_3 = 0.02;
body_consts = [L1_1;L2_1;L1_2;L2_2;L3_2;L1_3;L2_3;L3_3];

% q points.
q1_1 = [0;0;0];
q2_1 = [0;0;L1_1];
q1_2 = [0.02;0.03;0.01];
q2_2 = [0.02;0.03;0.01+L1_2];
q3_2 = [0.02;0.03;0.01+L1_2+L2_2];
q1_3 = [0;0.03;0];
q2_3 = [0;0.03;L1_3];
q3_3 = [0;0.03;L1_3+L2_3];
q_joint = [q1_1,q2_1,q1_2,q2_2,q3_2,q1_3,q2_3,q3_3];


% body configurations
I = eye(3);
P = [0.01;0.015;0.06];
T = [I,P;[0 0 0],1];

% joint_angles
theta1_1 = pi/3;
theta2_1 = pi/4;
theta1_2 = pi/5;
theta2_2 = pi/4;
theta3_2 = pi/8;
theta1_3 = pi/4.5;
theta2_3 = pi/6.4;
theta3_3 = pi/5.7;
theta = [theta1_1;theta2_1;theta1_2;theta2_2;theta3_2;theta1_3;theta2_3;theta3_3];

% get contact points expressed in {O} and {P} along with information about
% contacting.
[contact_pts_O, contact_pts_P,msg] = getFingerContactPt(theta,joint_axis,q_joint,joint_types,T,body_consts,base_pts,ellipsoid_para);
disp("--------------------------part 1: Question(b)----------------------------");
disp(contact_pts_P);
disp(contact_pts_O);
disp(msg);


%--------------------------part 2: Question(c)----------------------------

% rpy for three finger frame {Fi}
rpy_1 = [0;0;0];
rpy_2 = [0;0;-45*pi/180];
rpy_3 = [0;0;45*pi/180];
rpy = [rpy_1,rpy_2,rpy_3];

phi_2 = pi;%theta1_2 + theta2_2 + theta3_2 + pi/2; % third input parameter of RRR planar manipulator in addition to y and z.
phi_3 = pi;%theta1_3 + theta2_3 + theta3_3 + pi/2; % third input parameter of RRR planar manipulator in addition to y and z.
RRR_phi = [phi_2;phi_3];


[joint_angles_1,joint_angles_2,joint_angles_3] = getFingerAngles(contact_pts_O,T,base_pts,rpy,body_consts,RRR_phi);
disp("--------------------------part 2: Question(c)----------------------------");
disp("The required joint angles for thumb (Finger 1)");
disp(joint_angles_1);
disp("The required joint angles for F1 (Finger 2)");
disp(joint_angles_2);
disp("The required joint angles for F2 (Finger 3)");
disp(joint_angles_3);



