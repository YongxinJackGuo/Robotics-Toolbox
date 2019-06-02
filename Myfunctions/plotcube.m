% This function plots a cube

function plotcube(origin,X,Y,Z,color)
% input X Y Z are the lengths of the cube.
% input origin locates at the center of the cube.

% offset the origin by bringing it to the corner.
origin(1) = origin(1) - X*0.5;
origin(2) = origin(2) - Y*0.5;
origin(3) = origin(3) - Z*0.5;
ver = [1 1 0;
    0 1 0;
    0 1 1;
    1 1 1;
    0 0 1;
    1 0 1;
    1 0 0;
    0 0 0];

%  Define the faces of the unit cubic
fac = [1 2 3 4;
    4 3 5 6;
    6 7 8 5;
    1 2 8 7;
    6 7 1 4;
    2 3 5 8];

cube = [ver(:,1)*X+origin(1),ver(:,2)*Y+origin(2),ver(:,3)*Z+origin(3)];
patch('Faces',fac,'Vertices',cube,'FaceColor',color);
end