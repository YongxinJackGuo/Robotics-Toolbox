% A function that returns the time-parametrized trajectory <between two configurations> in joint-space based on the inital and final configuration.

function [Trajectory,stepNum,msg] = getTrajectory(gst0,theta_init,axis_joints,q_joints,type_joints,link_lengths,config_init,config_final,tau_init,beta_init,threshold)
addpath('~/Desktop/Assignment_Academics/Senior_Second semester/MEC529/Myfunctions');
% -----------------------------------------------------------------------------
% tau is the step-size parameter, which has range of [0,1]                    |
% -----------------------------------------------------------------------------
% beta(<=1) is step length parameter for how far one is moving from theta(t). |
% -----------------------------------------------------------------------------
% config_init and config_final are the initial and final configuration of the |
% manipulator expressed in transformation matrix g form.                      |
% -----------------------------------------------------------------------------
% threshold is a 1 by 2 matrix, [d_threshold,phi_threshold]                   |
%------------------------------------------------------------------------------
% /////////////////////////////////////////////////////////////////////////////
% -----------------------------------------------------------------------------
% output 1 theta_traj will be a sequence of n by 1 vector, or in other words, |
% theta_traj has size of n by m. n denotes d.o.f and (m-1) denotes number of  |
% steps.                                                                      |
% -----------------------------------------------------------------------------
% output 2 trans_traj is a sequence of transformation matrix for the          |
% trajectory in Cartesian coordinates.                                        |
% -----------------------------------------------------------------------------
% output 3 unitVec_traj is a sequence of unit vectors for the trajectory      |
% representing the rotation. The size is 9 by trajPoints.                     |
% -----------------------------------------------------------------------------
% Trajectory stores output 1, 2 and 3.


% assign threshold.
d_threshold = threshold(1);
phi_threshold = threshold(2);

% convert config_init and config_final from g matrix to unit dual quaternion.
A0 = Trans_to_UnitDualQuat(config_init);
Ad = Trans_to_UnitDualQuat(config_final);
% extract final position vector Pd and final rotation quat. Qd from config_final and Ad.
Pd = config_final(1:3,4);
Qd = Ad(:,1);
% extract initial position and rotation.
P0 = config_init(1:3,4);
R0 = config_init(1:3,1:3);
% get d.o.f
[dof,col] = size(theta_init);
% pre-assign theta_traj, trans_traj, and unitVec_traj.
dummy = 1000000;
Trajectory.theta = zeros(dof,dummy);
Trajectory.trans = zeros(4,4,dummy);
Trajectory.unitVectorFrame = zeros(9,dummy);
Trajectory.unitVectorFrame(:,1) = Rot_to_UnitVectors(R0); % store the first unit vector frame.
Trajectory.trans(:,:,1) = [R0,P0;[0 0 0], 1]; % store first transformation matrix.
Trajectory.theta(:,1) = theta_init; % store first set of joint angles.


% initialize the tolerance threshold.
d = 1;
phi = 1; % Euclidean distance between two unit quaternions.

% initialize a counter and stepNum
counter = 1;
stepNum = 0;
% assign inital tau and beta
tau = tau_init;
beta = beta_init;
while (d>d_threshold) || (phi>phi_threshold)  % if above the threshold, keep iterating.
    % update counter and stepNum
    counter = counter + 1;
    stepNum = counter - 1; % substract the first initial point.
    % get next interpolated configuration in unit quat. representation.
    A_next = SE3_Interpolation(A0,Ad,tau);
    % get next step angle using Jacobian-based Motion planning algorithm.
    theta_next = getNextStepAngle_JacBased(theta_init,axis_joints,q_joints,type_joints,A0,A_next,beta);
    % forward kinematics from theta_next to gst_next.
    g_next = manipdkin(gst0,axis_joints,q_joints,type_joints,theta_next);
    % extract P_next and R_next
    P_next = g_next(1:3,4);
    R_next = g_next(1:3,1:3);
    
    % check if P_next is reachable.
    switch manip_distCheck(link_lengths,P_next)
        case false
            msg = "Point cannot be reached.";
            % trim the theta_traj, trans_traj and unitVec_traj matrix.
            Trajectory.theta = Trajectory.theta(:,1:counter);
            Trajectory.unitVectorFrame = Trajectory.unitVectorFrame(:,1:counter);
            Trajectory.trans = Trajectory.trans(:,:,1:counter);
            return;
    end
    
    % convert g_next to A_next_real, which is the real next configuration obtained.
    A_next_real = Trans_to_UnitDualQuat(g_next);
    % extract the rotation quaternion.
    Q_next = A_next_real(:,1);
    
    % compute the threshold.
    d = sqrt((Pd(3)-P_next(3))^2 + (Pd(2)-P_next(2))^2 + (Pd(1)-P_next(1))^2);
    phi = min(norm(Q_next - Qd),norm(Q_next + Qd));
    
    % if it is approaching the target, then increase tau.
    if d < 0.1
        tau = 5 * tau_init;
        beta = 0.9 * beta_init;
    else
    end
    
    % store joint angles, position and unit vector frame of the trajectory.
    Trajectory.theta(:,counter) = theta_next;
    Trajectory.trans(:,:,counter) = [R_next,P_next;[0 0 0],1];
    Trajectory.unitVectorFrame(:,counter) = Rot_to_UnitVectors(R_next);
    
    % update initial configuration A0 and theta_init
    A0 = A_next_real;
    theta_init = theta_next;
end

% trim the theta_traj, trans_traj and unitVec_traj matrix.
Trajectory.theta = Trajectory.theta(:,1:counter);
Trajectory.unitVectorFrame = Trajectory.unitVectorFrame(:,1:counter);
Trajectory.trans = Trajectory.trans(:,:,1:counter);
% return message.
msg = "Trajectory generated successfully!";


end