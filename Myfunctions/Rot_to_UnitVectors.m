% A function takes rotation matrix and return three unit vector (9 by 1) corresponding to that rotation.

function u = Rot_to_UnitVectors(Rot)
% Rot is rotations matrix.
% output ux uy and uz are three unit vectors.

% From equation R*(u)=v, where u is the rotated vector, and v is the base
% frame unit vector. We can get --> u=Rt*(v).

% transpose of Rot.
Rt = transpose(Rot);

% define three base frame unit vectors.
vx = [1;0;0];
vy = [0;1;0];
vz = [0;0;1];

% get three rotated unit vectors.
ux = Rt * vx;
uy = Rt * vy;
uz = Rt * vz;

% return u
u = [ux;uy;uz]; % 9 by 1 size.
end