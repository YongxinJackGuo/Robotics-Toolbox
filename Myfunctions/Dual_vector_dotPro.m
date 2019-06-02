% A function that computes the dot product of two dual vectors

function EdotF = Dual_vector_dotPro(E,F)
% Note the E and F are both dual vector with 3 by 2 size.
% the result EdotF will be a dual number (dual scalar).
EdotF = zeros(1,2);

for i = 1:3
    % E(i,:) and F(i,:) are both dual number.
    EdotF = Dual_number_add(Dual_number_multp(E(i,:),F(i,:)),EdotF);
end
end

