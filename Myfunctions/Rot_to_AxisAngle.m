% A function that converts Rotation matrix to Axis-Angle Representation
function [axis,angle] = Rot_to_AxisAngle(R)
% compute the trace of R
Trace = R(1,1)+R(2,2)+R(3,3);
% compute the angle of rotation
angle = acos((Trace-1)/2);
%compute the axis of rotation
axis = (1/(2*sin(angle)))*[R(3,2)-R(2,3); R(1,3)-R(3,1); R(2,1)-R(1,2)];
end
