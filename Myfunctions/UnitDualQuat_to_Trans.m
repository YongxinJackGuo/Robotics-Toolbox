% A function that converts the unit dual quaternion to the transformation matrix.

function g = UnitDualQuat_to_Trans(A)
% Input A is a unit dual quaternion with 4 by 2 size

% extract Ar and Ad component from A.
Ar = A(:,1);
Ad = A(:,2);

% convert Ar to Rotation Matrix R
R = Quat_to_Rot(Ar);
% convert Ad to P.
Ar_conj = Quat_conj(Ar);
P = 2*Quat_multp(Ad,Ar_conj);
% exclude the first entry of 0 since P was in quaternion representation.
P = P(2:4);
% return output
g = [R P;[0 0 0] 1];

end

