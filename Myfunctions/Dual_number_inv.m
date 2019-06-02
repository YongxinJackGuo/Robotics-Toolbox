
function D_inv = Dual_number_inv(D)
% D is a 1 by 2 row vector or matrix
a = D(1);
b = D(2);
D_inv = (1/a) * [1, -b/a];
end

