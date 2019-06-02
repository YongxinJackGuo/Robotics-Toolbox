% A function that converts RPY angles to Rotation matrix
function R = RPY_to_Rot(RPY_angles)
% assign the RPY angles for convenience
% Roll angle
gamma = RPY_angles(1); 
% Pitch angle
beta = RPY_angles(2);
% Yaw angle
alpha = RPY_angles(3);
% Compute each elementary rotation matrices for RPY.
% Roll Matrix
R1 = [1,0,0;0,cos(gamma),-sin(gamma);0,sin(gamma),cos(gamma)];
% Pitch Matrix
R2 = [cos(beta),0,sin(beta);0,1,0;-sin(beta),0,cos(beta)];
% Yaw Matrix
R3 = [cos(alpha),-sin(alpha),0;sin(alpha),cos(alpha),0;0,0,1];
% Total Rotation Matrix
R = R3*R2*R1;
end