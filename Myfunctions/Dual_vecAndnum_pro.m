% A function that computes the product of a dual vector and a dual number.

function DE = Dual_vecAndnum_pro(D,E)
% D is a dual number with 1 by 2 size.
% E is a dual vector with 3 by 2 size.
% DE is the product, which is also a dual vector with 3 by 2 size.
DE = zeros(3,2);

for i = 1:3
    % E(i,:) is a component of dual vector E, and it is a dual number.
    DE(i,:) = Dual_number_multp(D,E(i,:));
end

end

