% A function that computes the power of a dual quaternion.

function A_tau = Dual_quat_pow(A,tau)
% A is the input dual quaternion and tau is the power.

% extract Ar and Ad from A.
Ar = A(:,1);
Ad = A(:,2);

% get position vector.
p_quat = 2*Quat_multp(Ad,Quat_conj(Ar));
p_vec = p_quat(2:4);
% convert Ar to axis-angle representation.
R = Quat_to_Rot(Ar);
[l_vec,theta] = Rot_to_AxisAngle(R); % l is the axis of rotation, and theta is the angle of rotation.
% turn l into a quaternion notation.
l_quat = [0;l_vec]; % l in quaternion notation
% calculate d, a scalar
d = transpose(p_quat)*l_quat;

if theta == 0  % when it is pure translation.
    l_vec = p_vec./sqrt(transpose(p_vec)*p_vec); % unit axis of translation.
    l_quat = [0;l_vec];
    d = transpose(p_quat)*l_quat;
    m = [0;0;0];
else % has rotation.
    % calculate m. m is a 3 by 1 column vector
    m = (1/2)*(cross(p_vec,l_vec)+(p_vec-d*l_vec)*cot(theta/2));
end

% calculate l_bar, a dual vector with 3 by 2 size
l_bar = [l_vec,m];
% calculate theta_bar, a dual number with 1 by 2 size
theta_bar = [theta/2, d/2]; % Not used in this function.
%----------calculate the component of the A_tau-------------------
% compute cos(tau*theta_bar), a dual number with 1 by 2 size, which is also the
% scalar part of the dual quaternion A_tau.
A_tau_scalar = [cos(tau*theta/2), -(tau*d/2)*sin(tau*theta/2)];
% compute l_bar*sin(tau*theta_bar), which is a dual vector with 3 by 2
% sizes. It is the vector part of the dual quaternion A_tau.
A_tau_vec = Dual_vecAndnum_pro([sin(tau*theta/2), (tau*d/2)*cos(tau*theta/2)],l_bar);
A_tau = [A_tau_scalar;A_tau_vec];

end

