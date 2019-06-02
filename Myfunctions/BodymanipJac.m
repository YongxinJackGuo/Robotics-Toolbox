% Compute the Body Jacobian of a given manipulator.
function Jb = BodymanipJac(Js, gst0, axis_joints, q_joints, type_joints, theta)
% compute the gst(theta), the transformation matrix between the tool frame
% and the base frame.
gst = manipdkin(gst0, axis_joints, q_joints, type_joints, theta);
% assign each element inside the gst(theta) matrix for doing the inverse
% calculation later
R = gst(1:3,1:3); % rotation matrix
p = gst(1:3,4); % position vector
Rt = transpose(R); % Transpose of Rot.
% compute the inverse of gst(theta).
gst_inv = [Rt, -1*Rt*p; [0 0 0], 1];
% compute the Adjoint matrix of inverse of gst(theta), which is just the
% inverse of Adjoint matrix of gst(theta).
Adgst_inv = AdjointOfg(gst_inv);
% compute Body Jacobian.
Jb = Adgst_inv*Js;
end

