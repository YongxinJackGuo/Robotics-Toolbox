% A function that returns a aggregated trajectory that connects multiple configurations.

function [TRAJECTORY,STEPNUM,msg] = getAggregatedTraj(CONFIGS,Robot,Traj_params)
addpath('~/Desktop/Assignment_Academics/Senior_Second semester/MEC529/Myfunctions');
% ----------------------------------------------------------------------------
% CONFIGS is the aggregated matrix for multiple configurations. It consists  |
% of multiple 4 by 4 config unit.                                            |
% ----------------------------------------------------------------------------
% Robot is structure variable that contains axis joints, q joints, type      |
% joints, theta_init, gst0 (start configs), and robot name                   |
% ----------------------------------------------------------------------------
% Traj_params contains the parameters for the trajectory generation, which   |
% are tau, beta, and threshold                                               |
% ----------------------------------------------------------------------------
% /////////////////////////////////////////////////////////////////////////////
% ----------------------------------------------------------------------------
% Output1 TRAJECTORY.theta is the aggregated trajectory in joint space that  |
% connects multiple configs.                                                 |
% ----------------------------------------------------------------------------
% Output2 TRAJECTORY.trans is the aggregated transformation matrix in SE(3)  |
% that connects multiple configs.                                            |
%-----------------------------------------------------------------------------
% Output3 TRAJECTORY.unitVectorFrame is aggregated unit vector               |
% frames for rotation                                                        |
% ----------------------------------------------------------------------------
% Output4 TRAJECTORY.subTrajPoints stores trajPoints                         |
% for each sub-trajectory                                                    |
% ----------------------------------------------------------------------------
% Output5 STEPNUM is the aggregated step number for the entire trajectory    |
% ----------------------------------------------------------------------------
% Aggregated output TRAJECTORY stores output 1, 2 and 3 and 4.

tic;
[row,column] = size(CONFIGS);
configNum = column / 4; % configNum is the number of sub=trajectories.

% get info. from robot
robot = Robot.name;
q_joints = Robot.q_joints;
axis_joints = Robot.axis_joints;
type_joints = Robot.type_joints;
theta_init = Robot.theta_init;
gst0 = Robot.gst0;
link_lengths = Robot.link_lengths;

% get dof.
[dof,col] = size(theta_init);

% get infor. from Traj_params;
tau = Traj_params.tau;
beta = Traj_params.beta;
threshold = Traj_params.threshold;

% pre-assign THETA_traj, TRANS_traj and UNITVECTORS_traj;
dummy = 100000000;
TRAJECTORY.theta = zeros(dof,dummy);
TRAJECTORY.trans = zeros(4,4,dummy);
TRAJECTORY.unitVectorFrame = zeros(9,dummy);
% pre-assign STEPNUM, aggregated stepNum and TRAJPoints
TRAJPoints = 0;
STEPNUM = 0;

% get new initial configuration.
config_init = manipdkin(gst0,axis_joints,q_joints,type_joints,theta_init);

% start looping
for i = 1:configNum
    % obtain next config. we call it final config.
    config_final = CONFIGS(:, (4*i-3):(4*i) );
    % get the trajectory between two configs.
    [Trajectory,stepNum,msg_subTraj] = getTrajectory(gst0,theta_init,axis_joints,...
        q_joints,type_joints,link_lengths,config_init,...
        config_final,tau,beta,threshold);
    
    % obtain last theta.
    trajPoints = stepNum + 1;
    theta_last = Trajectory.theta(:,trajPoints); % stepNum = trajPoints - 1.
    % store sub-trajectory points.
    TRAJECTORY.subTrajPoints(i) = trajPoints;
    
    % determine if the total trajectory points exceeds the maximum.
    if (TRAJPoints + trajPoints) > dummy
        disp('Trajectory points exceed the limite. Matrix size is not enough!');
        break;
    end
    
    % store trajectory.
    TRAJECTORY.theta(:,(TRAJPoints + 1):(TRAJPoints + trajPoints)) = Trajectory.theta;
    TRAJECTORY.trans(:,:,(TRAJPoints + 1):(TRAJPoints + trajPoints)) = Trajectory.trans;
    TRAJECTORY.unitVectorFrame(:,(TRAJPoints + 1):(TRAJPoints + trajPoints)) = Trajectory.unitVectorFrame;
    % Update STEPNUM and TRAJPoints.
    STEPNUM = STEPNUM + stepNum;
    TRAJPoints = TRAJPoints + trajPoints;
    
    % update theta_init, config_init.
    theta_init = theta_last;
    config_init = config_final;
    
    % check the message sent from sub-trajectory.
    switch strcmp(msg_subTraj,"Point cannot be reached.") 
        case true % The path fails.
            msg = "Trajectory Status:" + newline() + msg_subTraj + " Fail between"+...
                " configuration " + (i-1) + " and configuration " + i;
            % Trim the matrix and return.
            TRAJECTORY.theta = TRAJECTORY.theta(:,1:TRAJPoints);
            TRAJECTORY.trans = TRAJECTORY.trans(:,:,1:TRAJPoints);
            TRAJECTORY.unitVectorFrame = TRAJECTORY.unitVectorFrame(:,1:TRAJPoints);
            return;
    end
    
end

% Trim the matrix and return.
TRAJECTORY.theta = TRAJECTORY.theta(:,1:TRAJPoints);
TRAJECTORY.trans = TRAJECTORY.trans(:,:,1:TRAJPoints);
TRAJECTORY.unitVectorFrame = TRAJECTORY.unitVectorFrame(:,1:TRAJPoints);

% generate simulation message.
msg = "Trajectory Status:" + newline() + "The total trajectory steps for " +...
    robot + ": " + num2str(STEPNUM) + newline() + "The total computational time: "...
    + num2str(toc);

end

