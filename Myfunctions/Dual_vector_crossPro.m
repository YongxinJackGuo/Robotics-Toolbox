% A function that computes the cross product of two dual vectors

function EcrossF = Dual_vector_crossPro(E,F)
% E and F are both dual vectors with 3 by 2 size
% The result EcrossF is also a dual vector.

% Extract the components from E and F, which are all dual numbers.
E1 = E(1,:);
E2 = E(2,:);
E3 = E(3,:);
F1 = F(1,:);
F2 = F(2,:);
F3 = F(3,:);
% compute the result EcrossF in a component-wise fashion.
EcrossF(1,:) = Dual_number_multp(E2,F3) - Dual_number_multp(E3,F2);
EcrossF(2,:) = Dual_number_multp(E3,F1) - Dual_number_multp(E1,F3);
EcrossF(3,:) = Dual_number_multp(E1,F2) - Dual_number_multp(E2,F1);
end

