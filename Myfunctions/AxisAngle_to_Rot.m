% Convert Axis-Angle Representation to Rotation Matrix
function R = AxisAngle_to_Rot(axis,angle)
% assign the variables for convenience
w1 = axis(1);
w2 = axis(2);
w3 = axis(3);
% create a template for the skew-symmetric matrix.
skew = zeros(3,3);
% create the skew-symmetric matrix
skew(2,1) = w3;
skew(1,2) = -1*skew(2,1);
skew(1,3) = w2;
skew(3,1) = -1*skew(1,3);
skew(3,2) = w1;
skew(2,3) = -1*skew(3,2);
% assign cosine and sine to variables
C = cos(angle);
S = sin(angle);
v = 1-C;
% compute the Rotation matrix using Equation (2.4) in lecture notes.
R = eye(3)+skew*S+skew*skew*v;
end