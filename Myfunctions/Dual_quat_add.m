% A function that computes the addition of two dual quaternions

function C = Dual_quat_add(A,B)
% Note that A and B are dual quaternion with 4 by 2 size. 4 represents the
% quaternion and 2 represents the duality.
% The result C is also a dual quaternion with 4 by 2 size.
C = zeros(4,2);
% compute the result component-wisely
for i = 1:4
    % A(i,:) and B(i,:) are both dual numbers.
    C(i,:) = Dual_number_add(A(i,:),B(i,:));
end
end

