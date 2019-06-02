% A function that computes dual number multiplication

function D3 = Dual_number_multp(D1,D2)
% D1 and D2 are 1 by 2 row vector or matrix.
D3(1) = D1(1)*D2(1);
D3(2) = D1(1)*D2(2) + D2(1)*D1(2);
end

