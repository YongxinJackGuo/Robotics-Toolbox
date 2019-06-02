% A function that converts Rotation matrix to RPY angles
function RPY_angles = Rot_to_RPY(R)
% assign the Rotation Matrix variables for convenience
r11 = R(1,1);
r12 = R(1,2);
r13 = R(1,3);
r21 = R(2,1);
r22 = R(2,2);
r23 = R(2,3);
r31 = R(3,1);
r32 = R(3,2);
r33 = R(3,3);
% Compute the cos(beta), sin(beta) and pitch angle beta
cbeta = sqrt(r11^2+r21^2);
sbeta = -r31;
beta = atan2(sbeta,cbeta);
% Compute the cos(alpha), sin(alpha) and yaw angle alpha
calpha = r11/cbeta;
salpha = r21/cbeta;
alpha = atan2(salpha,calpha);
% Compute the cos(gamma), sin(gamma) and roll angle gamma
cgamma = r33/cbeta;
sgamma = r32/cbeta;
gamma = atan2(sgamma,cgamma);
% assign the rpy angles to return function
RPY_angles = [gamma; beta; alpha];
end