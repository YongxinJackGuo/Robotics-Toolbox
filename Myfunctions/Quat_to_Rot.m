% A function that converts Quaternions to Rotation matrix
function R = Quat_to_Rot(Q)
% Assign the elements in Quaternions for convenience
q0 = Q(1);
q1 = Q(2);
q2 = Q(3);
q3 = Q(4);
% Compute the Rotation Matrix Elements using Elements of Quaternions above
r11 = q0^2+q1^2-q2^2-q3^2;
r12 = 2*(q1*q2-q0*q3);
r13 = 2*(q1*q3+q0*q2);
r21 = 2*(q1*q2+q0*q3);
r22 = q0^2+q2^2-q1^2-q3^2;
r23 = 2*(q2*q3-q0*q1);
r31 = 2*(q1*q3-q0*q2);
r32 = 2*(q2*q3+q0*q1);
r33 = q0^2+q3^2-q1^2-q2^2;
% Assign the elements into the Rotation Matrix
R = [r11,r12,r13;r21,r22,r23;r31,r32,r33];
end