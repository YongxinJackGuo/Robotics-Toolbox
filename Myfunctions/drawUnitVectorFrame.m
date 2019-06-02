% A function that draw the unit vector frame at specified point.

function drawUnitVectorFrame(point,unitVectorFrame,f)
% unitVectorFrame is a 9 by 1 column vector that contains three unit
% vectors vertically.
% point is a 3 by 1 column vector.
% f is the factor that enlarge or shrink the unit vectors.

% extract three unit vectors Ux Uy and Uz.
Ux = unitVectorFrame(1:3)/f;
Uy = unitVectorFrame(4:6)/f;
Uz = unitVectorFrame(7:9)/f;

% add unit vectors to trajectory point to get position vector to that
% unit vector point.
Ux = point + Ux;
Uy = point + Uy;
Uz = point + Uz;
% get three components X Y and Z from point
X = point(1);
Y = point(2);
Z = point(3);
% plot unit vector Ux
plot3([Ux(1) X],[Ux(2) Y],[Ux(3) Z],'r-','LineWidth',1,'Marker','<');
% plot unit vector Uy
plot3([Uy(1) X],[Uy(2) Y],[Uy(3) Z],'b-','LineWidth',1,'Marker','<');
% plot unit vector Uz
plot3([Uz(1) X],[Uz(2) Y],[Uz(3) Z],'k-','LineWidth',1,'Marker','<');
%====================================================================
end

