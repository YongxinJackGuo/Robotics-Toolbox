% MEC529 Matlab Homework 2 Problem 5 Codes Created by Yongxin Guo
addpath('/Users/guoyongxin/Desktop/Assignment_Academics/Senior_Second semester/MEC529/Myfunctions');

close all
clear
clc

%----question (c)--------------
%----test codes for validating functi ons in (a) and (b)
% declare a RPY matrix
RPY_angles_init = [pi/8;pi/6;pi/2.8];
% compute the Rotation Matrix
R = RPY_to_Rot(RPY_angles_init);
disp("Rotation Matrix obtained from Function (a): ");
disp(R);
% input the Rotation Matrix obtained above to Rot_to_RPY function to check
% if we can get the same values as declared for RPY_angles
RPY_angles_end = Rot_to_RPY(R);
disp("RPY angles obtained from Function (b): ");
disp(RPY_angles_end);
disp("Initial RPY angles for comparison: ");
disp(RPY_angles_init);
% check equality
equaltiy = false;
criteria = 1E-10;
for i=1:3
    % check if each angle difference in two RPY matrices are within the criteria (a very small number!)
    if (RPY_angles_end(i)-RPY_angles_init(i))<criteria 
        % assign true to equality if current angle difference is within
        % criteria!
        equality = true;
    else
        % if there exists inequality in a certain pair of angles, assign
        % false to equaltiy and then break the for loop to quit
        equality = false;
        break;
    end
end

% check equality and display corresponding messages.
if equality == true
    disp("The codes are validated to be correct because the input RPY angles are equal to the RPY angles that went through two functions!!");
else
    disp("The codes has error!!");
end
%----question (c) ends---------














