function Adg = AdjointOfg(g)
% Assign Rotation matrix, the first element in the first row, to Adjoint
% Matrix.
R = g(1:3,1:3);
% Assign the position vector
p = g(1:3,4);
% Convert it into skew-symmetric matrix.
phat = [0,-p(3),p(2);p(3),0,-p(1);-p(2),p(1),0];
% Assign the elements to Adjoint Matrix
Adg = [R, phat*R; zeros(3,3), R];
end

