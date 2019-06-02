% MEC529 Matlab Homework 2 Problem 6 Codes Created by Yongxin Guo
addpath('/Users/guoyongxin/Desktop/Assignment_Academics/Senior_Second semester/MEC529/Myfunctions');

close all
clear
clc

%---test codes---------------
disp("Test codes for question (B): ");
Q=[sqrt(0.2),sqrt(0.3),sqrt(0.4),sqrt(0.1)];
Rotation=Quat_to_Rot(Q);
disp("Rotation Matrix R: ");
disp(Rotation);
Quat=Rot_to_Quat(Rotation);
disp("Input Quaternion for Comparison: ");
disp(Q);
disp("Quaternion: ");
disp(Quat);
%---test ends----------------

%--------question (c)---------
disp("***************************************************");
disp("Question (C) section: ")
% first of all, generate 5 sets of random rpy angles and convert them into
% SO(3) Rotation Group
numOfRandom = 5;
% declare the equality.
equality=false;
for i=1:numOfRandom
    % generate random set of rpy vector
    rpy=2*pi*rand(1,3);
    % convert it into Rotation Matrix
    R=RPY_to_Rot(rpy);
    % Convert random Rotation Matrix into Quaternions using functions below
    Quaternion=Rot_to_Quat(R);
    % Convert the Quaternions back to Rotation Matrix using functions below
    R_c=Quat_to_Rot(Quaternion);
    % Compute the difference between the input and output Rotation Matrix
    diff=R_c-R;
    % display the results
    disp("-----------------------------------------------");
    disp("The random set " + i + ": ");
    disp("    The input Rotation Matrix: ");
    disp(R);
    disp("    The output Rotation Matrix: ");
    disp(R_c);
    disp("    The difference is: ");
    disp(diff);
    if max(diff)>1E-10
        disp("The codes fail and the random generation of matrix stops!");
        equality=false;
        break;
    else
        disp("Equality for set " + i + " holds!" + newline);
        equality=true;
    end
end

% check the overall equality and make conclusion
disp("------------------------------------------------");
disp(newline+"Conclusion:");
switch equality
    case false
        disp("    The codes fail!");
    otherwise
        disp("    The codes are verified to be correct!");
end
%--------question (c) ends---------











