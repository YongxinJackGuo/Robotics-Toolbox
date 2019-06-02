% MEC529 Matlab Homework 4 Problem 2.4.a Codes Created by Yongxin Guo
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
w6 = [0;1;0];
axis_joints = [w1,w2,w3,w4,w5,w6];
% each q vecotr of joints.
q1 = [0;0;l0];
q2 = q1;
q3 = [0;l1;l0];
q4 = [0;l1+l2;l0];
q5 = q4;
q6 = q4;
q_joints = [q1,q2,q3,q4,q5,q6];
% eacht type of joints
type_joints = ["R";"R";"R";"R";"R";"R"];
% each displacement and rates of joints
theta = thetai*ones(6,1);
thetadot = [thetaidot;thetaidot;thetaidot;thetaidot;thetaidot;thetaidot];
% base transformation matrix
gst0 = [eye(3),[0;l1+l2+TtoP;l0];[0 0 0],1];
% Compute gst(theta).
gst_theta = manipdkin(gst0, axis_joints, q_joints, type_joints, theta);
% Compute spatial jacobian
Js = SpatialmanipJac(axis_joints, q_joints, type_joints,theta);
% Compute spatial velocity twist
Vs = SpatialVelTwist(Js,thetadot);
ws = Vs(4:6); % extract spatial angular velocity of the end-effector
vs = Vs(1:3); % extract spatial linear velocity of the end-effector
% Compute body jacobian
Jb = BodymanipJac(Js, gst0, axis_joints, q_joints, type_joints, theta);
% Compute body velocity twist
Vb = BodyVelTwist(Jb,thetadot);
vb = Vb(1:3);
wb = Vb(4:6);
% Note that the vs component inside Vs vector is the velocity of an
% instantanous point attached to the tool frame passing through the origin
% of the world frame. In order to calculate the velocity of the origin of
% the tool frame expressed in spatial frame, which is question (b), we have
% to use equation (5.14).
ws_hat = [0 -ws(3) ws(2);ws(3) 0 -ws(1);-ws(2) ws(1) 0]; % ws_hat matrix
Vs_hat = [ws_hat vs; 0 0 0 0]; % Construct Vs_hat twist.
p = gst_theta(1:3,4); % extract the position vector at "theta moment".
% Compute the spatial linear velocity of origin of the tool frame
Vts = Vs_hat*[p;1];
vts = Vts(1:3);
wts = ws; % The spatial angular velocity of any point attached to the tool frame is the same.


% display results
disp("Question (a). Spatial quantities: ");
disp("Linear velocity of end-effector: ");
disp(vs);
disp("Angular velocity of end-effector: ");
disp(ws);
disp("--------------------------------------------------");
disp("Question (a). Body quantities");
disp("Linear velocity of end-effector: ");
disp(vb);
disp("Angular velocity of end-effector: ");
disp(wb);
disp("--------------------------------------------------");
disp("Question (b). Spatial quantities: ");
disp("Linear velocity of the origin of the tool frame: ");
disp(vts);
disp("Angular velocity of the origin of the tool frame: ");
disp(wts);

