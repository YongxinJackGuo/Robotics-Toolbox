% A function that computes the conjugate of a dual quaternion

function A_conj = Dual_quat_conj(A)
% Note that A is a dual quaternion with 4 by 2 sizes
% Extract the quaternion component
P = A(:,1);
Q = A(:,2);
% compute conjugates of the components
P_conj = Quat_conj(P);
Q_conj = Quat_conj(Q);
A_conj = [P_conj,Q_conj];
end

