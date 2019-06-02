% A function that computes Dual vector addition.

function G = Dual_vector_add(E,F)
% Note that dual vector E and F are both 3 by 2 matrix. 3 represents a
% vector, and 2 represents the duality.

G = zeros(3,2); % result G is also a dual vector with 3 by 2 size.

for i = 1:3
    % E(i,:) and F(i,:) are both dual numbers, components of dual vector E and F.
    G(i,:) = Dual_number_add(E(i,:),F(i,:));
end

end

