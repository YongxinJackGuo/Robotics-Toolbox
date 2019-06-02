% A function that convert unit dual quaternion representation to gamma 'y' representation.

function gamma = UnitDualQuat_to_Gamma(A)
% A is a dual quaternion with 4 by 2 size. 
% output gamma is a 7 by 1 column vector, which is [p,Q]. Where Q is the
% quaternion for rotation with 4 by 1 size, and p is the position vector
% with 3 by 1 size.

% Extract rotation quaternion.
Q = A(:,1);
% Convert dual quaternion A to transformation matrix g
g = UnitDualQuat_to_Trans(A);
% Extract position vector from g.
p = g(1:3,4);
% concatenate the resultant gamma y.
gamma = [p;Q];

end

