% A function that returns a set of angles for next step based on Jacobian-based Motion Planning Algorithm.

function theta_next = getNextStepAngle_JacBased(theta_cur,axis_joints,q_joints,type_joints,unitDualQuat_cur,unitDualQuat_next,beta)
% theta_cur means current thetas, which is a n by 1 column vector, with n
% being the d.o.f
% unitDualQuat_cur means current unit dual quaternion of the end effector.
% unitDualQuat_next means next unit dual quaternion of the end effector
% obtained from interpolation in SE(3).
% beta(<=1) is step length parameter for how far one is moving from theta(t)

% get d.o.f
[dof,col] = size(theta_cur); 
% get spatial jacobian of the manipulator.
Js = SpatialmanipJac(axis_joints, q_joints, type_joints,theta_cur);
% convert current unit dual quternion to gamma
gamma = UnitDualQuat_to_Gamma(unitDualQuat_cur);
% extract rotation quaternion Q and position vector p from gamma.
p = gamma(1:3);
Q = gamma(4:7);
% compute J1, the representative jacobian of quaternion representation.
q0 = Q(1);
q1 = Q(2);
q2 = Q(3);
q3 = Q(4);
J1 = [-q1 q0 q3 -q2;-q2 -q3 q0 q1;-q3 q2 -q1 q0]; % 3 by 4 size
% compute J2.
p_hat = Vec_to_SkewAsy(p); % 3 by 3 size.
I = eye(3);
J2 = [I 2*p_hat*J1;zeros(3,3) 2*J1]; % 6 by 7 size

% compute B(theta_current);
if dof > 6 % more than 6 dof, e.g 7 dof baxter.
    B_cur = (transpose(Js)/(Js*transpose(Js)))*J2; % use pseudo-inverse.
elseif dof == 6 % 6-dof elbow manipulator
    B_cur = Js\J2; 
else % under-actuated.
    B_cur = (transpose(Js)/(Js*transpose(Js)))*J2; 
end

% convert the next unit dual quternion to gamma notation
gamma_next = UnitDualQuat_to_Gamma(unitDualQuat_next);
% compute next set of theta based on Euler discretization.
theta_next = theta_cur + beta*B_cur*(gamma_next-gamma);

end

