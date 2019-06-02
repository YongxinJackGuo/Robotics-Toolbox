% Compute the spatial velocity twist V of the tool frame or an end-effector for a given manipulator
function Vs = SpatialVelTwist(Js,thetadot)
% compute V. V=J(theta)*thetadot.
Vs = Js*thetadot;
end

