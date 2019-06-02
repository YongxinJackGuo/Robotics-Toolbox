% A function that converts transformation matrix g to unit dual quaternion.

function A = Trans_to_UnitDualQuat(T)
% T is a 4 by 4 transformation matrix.
% The resultant unit quaternion A is a 4 by 2 matrix.
% extract the components of T.
R = T(1:3,1:3);
P = T(1:3,4);
P = [0;P]; % convert into quaternion notation with the scalar part being 0.
% convert R to Ar, the real part of dual quaternion A.
Ar = transpose(Rot_to_Quat(R));
% get Ad, the dual part of dual quaternion A.
Ad = (1/2)*Quat_multp(P,Ar);
A = [Ar,Ad];
end

