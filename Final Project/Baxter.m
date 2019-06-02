% This file is the main application file for 7-DOF Baxter manipulator. The tasks include pick and place problem, drawing a letter or any patterns and other general use involved robotic arm motion planning.

close all
clear
clc

tic
addpath('~/Desktop/Assignment_Academics/Senior_Second semester/MEC529/Myfunctions');
addpath('~/Desktop/Assignment_Academics/Senior_Second semester/MEC529/Final Project/baxter_common-master/baxter_description/urdf');

%   Coordinate --------------> (Y)
%             | (Z out of screen)
%             |
%             |
%             |
%             |
%             V (X)

% Create 7-DOF baxter robotic arm.
theta_init = 5; % assign 5 degrees to each baxter joint.
[baxter,baxterSL] = createBaxter(theta_init);
% get initial rotation matrix and transformation matrix.
R_init = baxter.g_init(1:3,1:3);
P_init = baxter.g_init(1:3,4);
% generate some random rotation matrix.
R_1 = RPY_to_Rot([0;0;45]*pi/180);
R_2 = RPY_to_Rot([0;45;0]*pi/180) * R_1;
% construct some configuration in gst form.
config_1 = [R_init,[0.6;0.7;0.2];[0 0 0],1];
config_2 = [R_1,[0;0.8;0.8];[0 0 0],1]; 
config_3 = [R_1,[-0.6;0.7;0.5];[0 0 0],1]; 
config_4 = [R_2,[-0.6;0.7;0.2];[0 0 0],1];
config_5 = [R_2,[-0.3;-0.3;0.7];[0 0 0],1];
CONFIGS = [config_1,config_2,config_3,config_4];
% perform a distance check for all the intermediate configuration.
[row_notUsed,configNum] = size(CONFIGS);
configNum = configNum / 4;
for k = 1:configNum
    if manip_distCheck(baxter.link_lengths,CONFIGS(1:3,4*k)) == false
        disp("The position of configuration " + k + ...
            " is too far to reach!" + newline());
        break;
    end
end
% Create traj_params struct.
traj_params.tau = 0.01;
traj_params.beta = 1;
traj_params.threshold = [0.01,0.01];
% get trajectory
[TRAJECTORY,STEPNUM,msg] = getAggregatedTraj(CONFIGS,baxter,traj_params);
disp(msg);
[row_notUsed,col,dataPoints] = size(TRAJECTORY.trans);
% 3D Tranjectory plot
figure;
% squeeze is to reduce 3D matrix to 1D vector for plotting.
X = squeeze(TRAJECTORY.trans(1,4,:));
Y = squeeze(TRAJECTORY.trans(2,4,:));
Z = squeeze(TRAJECTORY.trans(3,4,:));
plot3(X,Y,Z,'k-','LineWidth',5);
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
hold on;
xlim([-1,1]);
ylim([-1,1]);
zlim([-1,1]);

% get number of sub-trajectory.
[row1,subTrajNumber] = size(TRAJECTORY.subTrajPoints);

%{
%-----------------------------Coordinates Plot and Animation-----------------
% initialize current trajectory points.
currentTrajPoints = 0;
currentStep = 1;
for i = 1:subTrajNumber
    % get the end point for each sub-trajetory.
    currentTrajPoints = currentTrajPoints + TRAJECTORY.subTrajPoints(i);
    % get corresponding step size.
    stepSize = floor(TRAJECTORY.subTrajPoints(i)/10);
    while currentStep <= currentTrajPoints
        currentPoint = [X(currentStep);Y(currentStep);Z(currentStep)];
        factor = 15;
        drawUnitVectorFrame(currentPoint,TRAJECTORY.unitVectorFrame(:,currentStep),factor)
        currentStep = currentStep + stepSize; % update currentStep.
        stepSize = floor(stepSize * 2); % increase stepSize by 2 everytime.
    end
    currentStep = currentTrajPoints; % move the step back to the start point of each sub-trajectory.
end


% create animation of the tool frame trajectory using robotics toolbox by Peter Corke.
%tranimate(TRAJECTORY.trans);
%-----------------------------------Section End------------------------------
%}


% create the ground
fill3([1 -1 -1 1 ], [1 1 -1 -1], [0 0 0 0], [1 -1 1 -1]);
% create three boxes of 5 by 5 by 2
plotcube([0,0.7,0.15],0.35,0.35,0.35,[0.4660, 0.6740, 0.1880]);
plotcube([0,0.7,0.35],0.3,0.3,0.1,[0, 0.75, 0.7]);
plotcube([0,0.7,0.45],0.2,0.2,0.1,[0, 0.4470, 0.7410]);





%*****************************Elbow in SerialLink Demonstration*****************************
%-----------------------------Robotic Arm Animation--------------------------
% Elbow manipulator animation.
ballScale = 1;
camzoom(2);
view(90,26);
lightangle(90,35);
ballSurf = 0;
% create a unit radius ball;
[X_ball,Y_ball,Z_ball] = sphere;
for j = 1:dataPoints
    theta1 = TRAJECTORY.theta(1,j);
    theta2 = TRAJECTORY.theta(2,j);
    theta3 = TRAJECTORY.theta(3,j);
    theta4 = TRAJECTORY.theta(4,j);
    theta5 = TRAJECTORY.theta(5,j);
    theta6 = TRAJECTORY.theta(6,j);
    theta7 = TRAJECTORY.theta(7,j);
    plot(baxterSL,[pi/4+theta1 pi/2+theta2 theta3 theta4...
        theta5 theta6 theta7]);   % pi/4 of first theta is to rotate the baxter arm base 
                                  % to match with the world frame coordinate.
                                       
    % plot a ball object at the tool frame.
    %ballSurf = surf(ballScale*X_ball+X(j),...
     %   ballScale*Y_ball+Y(j),ballScale*Z_ball+Z(j));
    
    %set(ballSurf,'FaceColor',[0 0 1],'FaceAlpha',0.65,'FaceLighting',...
     %   'gouraud','EdgeColor','none');
     
end
hold off;
disp("Status Update: Animation Done!");

%-----------------------------Section End-------------------------------------



