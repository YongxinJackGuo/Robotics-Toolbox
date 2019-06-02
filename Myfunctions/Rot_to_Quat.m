% A function that converts Rotation matrix to Quaternions
function Q = Rot_to_Quat(R)
% Assign the elements in Rotation matrix for convenience
r11 = R(1,1);
r12 = R(1,2);
r13 = R(1,3);
r21 = R(2,1);
r22 = R(2,2);
r23 = R(2,3);
r31 = R(3,1);
r32 = R(3,2);
r33 = R(3,3);
% declare column vector b expressed in Equation (2.23) in lecture notes
b = [r11;r22;r33;1];
% declare the inverse of a matrix that represents the coefficients
% expressed in equation (2.21) in lecture notes. The inverse of that matrix
% is just 0.25 times the transpose of that matrix itself!
invA = 0.25*[1 1 1 1;1 -1 -1 1;-1 1 -1 1;-1 -1 1 1];
% compute the Qsquare term
Qsquare = invA*b;
% find the index of the maximum value in the vector, if there are multiple
% maximum values (less likely), then return the first one.
maxIndex=find(Qsquare==max(Qsquare),1,'first');
qknown=sqrt(max(Qsquare));
% determine which solution will be used.
switch maxIndex
    % solution 1 will be used
    case 1
        q0=qknown;
        q1=(r32-r23)/(4*q0);
        q2=(r13-r31)/(4*q0);
        q3=(r21-r12)/(4*q0);
    % solution 2 will be used
    case 2
        q1=qknown;
        q0=(r32-r23)/(4*q1);
        q2=(r12+r21)/(4*q1);
        q3=(r13+r31)/(4*q1);
    % solution 3 will be used
    case 3
        q2=qknown;
        q0=(r13-r31)/(4*q2);
        q1=(r12+r21)/(4*q2);
        q3=(r23+r32)/(4*q2);
    otherwise
        q3=qknown;
        q0=(r21-r12)/(4*q3);
        q1=(r13+r31)/(4*q3);
        q2=(r23+r32)/(4*q3);
end

Q=[q0,q1,q2,q3];

end