% MEC529 Midterm Problem 1 Question (c), Created by Yongxin Guo
close all
clear
clc

%-------------------------part 1------------------------------------
disp("**************************Part 1*********************************")
% Given contact points and body configs, and output the joint angles. which
% is for verification of question (b).
% Contact pts expressed in homogenous representation.
ct_pt1 = [2;     1;    0;    1];
ct_pt2 = [1.4;   1;    0;    1];
ct_pt3 = [1.4;   2;    0;    1];
ct_pt4 = [2;     1.8;  0.4;  1]; % lift in the air
contact_pts = [ct_pt1,ct_pt2,ct_pt3,ct_pt4];
legNum = 4;
% body constants
l1 = 0.2;
l2 = 0.25;
w = 0.3;
l = 0.5;
body_consts = [l1;l2;w;l];

% body rotate around x-axis, roll angle = 10 degrees
roll = 10*(pi/180); % convert it to radian
pitch = 0;
yaw = 0;
rpy = [roll;pitch;yaw];
Rot = RPY_to_Rot(rpy);
% body position
P = [1.7;1.5;0.35];
% body configuration;
config = [Rot,P;[0 0 0],1];

% Get joint angles
[jointAngles,msg] = getJointAngles(config,body_consts,contact_pts);
disp(newline + "The required joint angles for four legs are (row->2 joint angles, column->4 legs): ");
disp(newline + "1st solution set are:")
disp(jointAngles(1:2,:));
disp("2nd solution set are: ");
disp(jointAngles(3:4,:));
disp(msg + newline);

%-------------------------part 2------------------------------------
disp("**************************Part 2*********************************");
% given joint angles and body configurations, and output the contact
% points. Which is for verification of question (a), and we can directly
% use the joint angles output from part 1 as the given variables.
% Then we can compare out contact pts output with the given contact pts in
% part 1 to see if they are identical.

% choose the first set of soln from previous joint angle output as the
% given.
joint_given = jointAngles(1:2,:);
contact_pts_output = getContactPt(joint_given,config,body_consts);
for i = 1:legNum
    disp(newline + "Leg " + num2str(i) + " contact points: ");
    disp(contact_pts_output(1:3,i)); % drop out the homogenous representation.
end

diff = abs(contact_pts_output - contact_pts); % compute the difference with the contact points from part 1.
criterion = 1e-10; % set a criterion for comparison.
if norm(diff) <= criterion
    disp(newline+ "Conclusion: the result is consistent! The contact points are valid!")
else
    disp(newline + "Large difference exists in the verification process. The contact points may not be valid!")
end


