% A function that computes the magnitude of a quaternion.

function Q_magn = Quat_magn(Q)
% compute conjugate.
Q_conj = Quat_conj(Q);
% compute the magnitude.
Q_magn = sqrt(Quat_multp(Q,Q_conj)); % Note it is still in quat. notation.
% extract the first element.
Q_magn = Q_magn(1);
end

