% This function creates a 7-DOF Baxter robotic amr with two kinds of data type (struct, SerialLink) that contains information such as axis_joints, q_joints, type_joints, gst0 and etc.

function [baxter,baxterSL] = createBaxter(theta_init)
addpath('~/Desktop/Assignment_Academics/Senior_Second semester/MEC529/Myfunctions');
% The function takes no input since all the parameters of the elbow are
% already known. Any modification to the elbow manipulator should be
% carried out inside this function.

%-----------------------Create struct type of elbow-----------------------------
%   Coordinate --------------> (Y)
%             | (Z out of screen)
%             |
%             |
%             |
%             |
%             V (X)
% assign configuration constants.
S0S1_z = 0.27035;  
S0S1_x = 0.069;
S1E1_z = 0.36435;
S1E1_x = 0.069;
E1W1_z = 0.37429;
E1W1_x = 0.010;
W1EE_z = 0.229525;
% create gst0 matrix.
R0 = eye(3);
straightLength = S0S1_x + S1E1_z + E1W1_z + W1EE_z; 
x0 = straightLength * sin(pi/4);
y0 = straightLength * cos(pi/4);
z0 = S0S1_z - S1E1_x - E1W1_x;
P0 = [x0;y0;z0];
% create axis of motion.
w1r = [0;0;1];
w2r = [-1/sqrt(2);1/sqrt(2);0];
w3r = [1/sqrt(2);1/sqrt(2);0];
w4r = w2r;
w5r = w3r;
w6r = w2r;
w7r = w3r;
% create q_matrix.
q1 = [0; 0; 0];
q2 = [S0S1_x/sqrt(2); S0S1_x/sqrt(2); S0S1_z];
q3 = q2;
q4 = [(S0S1_x+S1E1_z)/sqrt(2); (S0S1_x+S1E1_z)/sqrt(2); S0S1_z-S1E1_x];
q5 = q4;
q6 = [(S0S1_x+S1E1_z+E1W1_z)/sqrt(2); (S0S1_x+S1E1_z+E1W1_z)/sqrt(2); S0S1_z-S1E1_x-E1W1_x];
q7 = q6;
% Create elbow struct
baxter.name = "7-DOF Baxter Robotic Arm";
baxter.q_joints = [q1,q2,q3,q4,q5,q6,q7]; % assign q_joints
baxter.type_joints = ["R";"R";"R";"R";"R";"R";"R"]; % assign type_joints
baxter.axis_joints = [w1r,w2r,w3r,w4r,w5r,w6r,w7r]; % assign axis_joints
baxter.theta_init = theta_init*ones(7,1)*pi/180; % create random initial theta to avoid Jacobian singularity.
baxter.gst0 = [R0,P0;[0 0 0],1]; % assign gst0
baxter.link_lengths = [S0S1_z,S0S1_x,S1E1_z,S1E1_x,E1W1_z,E1W1_x,W1EE_z];
% get initial rotation matrix.
baxter.g_init = manipdkin(baxter.gst0,baxter.axis_joints,baxter.q_joints,...
    baxter.type_joints,baxter.theta_init);

%-------------Create SerialLink Baxter object using Robotics Toolbox by Peter Corke---------------
% DH parameters for each link of elbow manipulator.
% with input form DH = [theta d a alpha].
L(1) = Link([0 S0S1_z S0S1_x -pi/2]);
L(2) = Link([0 0 0 pi/2]);
L(3) = Link([0 S1E1_z S1E1_x -pi/2]);
L(4) = Link([0 0 0 pi/2]);
L(5) = Link([0 E1W1_z E1W1_x -pi/2]);
L(6) = Link([0 0 0 pi/2]);
L(7) = Link([0 W1EE_z 0 0]);

baxterSL = SerialLink(L,'name','Baxter robotic arm','base',transl(0,0,0));
end

