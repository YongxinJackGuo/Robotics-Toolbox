% MEC529 Matlab Homework 1 Problem 2 Codes Created by Yongxin Guo
addpath('/Users/guoyongxin/Desktop/Assignment_Academics/Senior_Second semester/MEC529/Myfunctions');
close all
clear
clc

% The test section that validates two functions.
% ------------test starts------------
Links = [0.07; 0.03; 0.02]; % assign links
configuration = [pi/6; pi/8; pi/7]; % assign the input configuration
pose = RRR_direct_2D(Links, configuration); % calculate pose
% display the pose calculated in function RRR_direct_2D
disp("test for direct kinematics(phi in angles, not radians):")
disp(pose);
% input the pose calculated above into function RRR_inverse_2D to check if it matches the input configuration
config = RRR_inverse_2D(Links, pose);
% display the output configuration and check if it matches the input
% config.
disp("test for inverse kinematics, with 2 sets of solutions(elbow up and elbow down)");
disp(config);
% ------------test ends--------------


%-------------problem (d)------------
numberOfConfigs = 5; % assign #.of valid configs. we need 5 as stated in problem d.
xForce = 0.5;  % assign x-component force
yForce = 0.05; % assign y-component force
forceVector = [xForce; yForce]; % assign to a vector
% assign link lengths
l1 = 0.04;
l2 = 0.03;
l3 = 0.02;
links = [l1;l2;l3];
phi = atan2(yForce,xForce); % compute the angle for the end effector
% compute 5 random configurations for three joints and display
configs_d = RRR_random_config(numberOfConfigs,phi);
disp("5 random configurations:");
disp(configs_d);
% compute 5 set of torques corresponding to 5 random configurations and display
torques = RRR_torque_2D(forceVector,numberOfConfigs,configs_d,links);
disp("5 sets of torque corresponding to random configs:")
disp(torques);
%-------------end of problem (d)-----