% A function that computes the product of the multiplication of two quaternions.

function C = Quat_multp(Q,P)
% extract the components from Quaternions
q0 = Q(1);
q_v = Q(2:4);
p0 = P(1);
p_v = P(2:4);
% compute the components of the product.
c0 = q0*p0 - transpose(q_v)*p_v;
c_v = q0*p_v + p0*q_v + cross(q_v,p_v);
% concatenate the components for the result.
C = [c0;c_v];
end
