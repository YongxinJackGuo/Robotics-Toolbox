% This function converts a tranformation matrix into its inverse, created by Yongxin Guo.
function inverse_T_matrix = getTransInv(T)
    % Input T is a transformation matrix with [R,P;[0 0 0],1];
    Rot = T(1:3,1:3); % get rotation matrix from T.
    P = T(1:3,4); % get Position vector P from T.
    inverse_T_matrix = [transpose(Rot),-1*transpose(Rot)*P;[0 0 0],1];
end

