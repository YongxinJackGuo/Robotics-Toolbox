% A function that computes the inverse of a quaternion.

function Q_inv = Quat_inv(Q)
Q_conj = Quat_conj(Q);
Q_magn_sq = Quat_magn(Q)^2;
% Compute inverse
Q_inv = Q_conj/Q_magn_sq;
end

