% A function that computes the conjugate of a dual number.

function D_conj = Dual_number_conj(D)
% Note D is 1 by 2 row vector or matrix.
D_conj = [D(1), -1*D(2)];
end

