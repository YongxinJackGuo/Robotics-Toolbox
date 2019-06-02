% A function that computes the multiplication of two dual quaternions.

function C = Dual_quat_multp(A,B)
% A and B are both dual quaternions with 4 by 2 size.
% resulted C will also be a dual quaternion.

% extract the quaternion component from A and B
P = A(:,1);
Q = A(:,2);
U = B(:,1);
V = B(:,2);
% compute the quaternion component of the product C.
C1 = Quat_multp(P,U);
C2 = Quat_multp(Q,U) + Quat_multp(P,V);
% concatenate
C = [C1,C2];
end

