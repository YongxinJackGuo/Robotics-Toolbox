% This function creates a 6-DOF elbow manipulator with two kinds of data type (struct, SerialLink) that contains information such as axis_joints, q_joints, type_joints, gst0 and etc.

function [elbow,elbowSL] = createElbow(theta_init)
addpath('~/Desktop/Assignment_Academics/Senior_Second semester/MEC529/Myfunctions');
% The function takes no input since all the parameters of the elbow are
% already known. Any modification to the elbow manipulator should be
% carried out inside this function.

%-----------------------Create struct type of elbow-----------------------------
% assign configuration constants.
l0 = 0.8;
l1 = 0.8;
l2 = 0.5;
% create gst0 matrix.
R0 = eye(3);
endEffector_length = 0.2;
P0 = [0;l1+l2+endEffector_length;l0];
% create axis of motion.
axis1 = [0;0;1];
axis2 = [-1;0;0];
axis3 = [-1;0;0];
axis4 = [0;0;1];
axis5 = [-1;0;0];
axis6 = [0;1;0];
% create q_matrix.
q1 = [0;0;l0];
q2 = [0;0;l0];
q3 = [0;l1;l0];
q4 = [0;l1+l2;l0];
q5 = [0;l1+l2;l0];
q6 = [0;l1+l2;l0];
% Create elbow struct
elbow.name = "6-DOF Elbow manipulator";
elbow.q_joints = [q1,q2,q3,q4,q5,q6]; % assign q_joints
elbow.type_joints = ["R";"R";"R";"R";"R";"R"]; % assign type_joints
elbow.axis_joints = [axis1,axis2,axis3,axis4,axis5,axis6]; % assign axis_joints
elbow.theta_init = theta_init*ones(6,1)*pi/180; % create random initial theta to avoid Jacobian singularity.
elbow.gst0 = [R0,P0;[0 0 0],1]; % assign gst0
elbow.link_lengths = [l0,l1,l2,endEffector_length];
% get initial rotation matrix.
elbow.g_init = manipdkin(elbow.gst0,elbow.axis_joints,elbow.q_joints,...
    elbow.type_joints,elbow.theta_init);

%-------------Create SerialLink elbow object using Robotics Toolbox by Peter Corke---------------
% DH parameters for each link of elbow manipulator.
% with input form DH = [theta d a alpha].
L(1) = Link([0 l0 0 -pi/2]);
L(2) = Link([0 0 l1 0]);
L(3) = Link([0 0 l2 pi/2]);
L(4) = Link([0 0 0 -pi/2]);
L(5) = Link([0 0 0 pi/2]);
L(6) = Link([0 endEffector_length 0 0]);

elbowSL = SerialLink(L,'name','Elbow Manipulator');


end

