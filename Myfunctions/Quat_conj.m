% A function that computes the conjugate of a quaternion.
function Q_conj = Quat_conj(Q)
q0_conj = Q(1);
qv_conj = -1*Q(2:4);
Q_conj = [q0_conj;qv_conj];
end

