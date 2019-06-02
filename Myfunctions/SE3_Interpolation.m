% This function is the interpolation in SE(3) and it takes input of tau, initial and final configuration in dual quaternion representation, and then return the interpolated configuration in dual quaternion along the time-parameterized trajectory.

function C = SE3_Interpolation(A,B,tau)
% input A and B are initial and final configuration in SE(3) respectively
% with the dual quaternion represention whose sizes are both 4 by 2. 
% and tau is the step-size within the range of [0,1].

C = Dual_quat_multp(A,Dual_quat_pow(Dual_quat_multp(Dual_quat_conj(A),B),tau));


end

