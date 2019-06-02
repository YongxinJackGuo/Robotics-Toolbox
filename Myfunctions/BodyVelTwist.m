% Compute the body velocity twist V of the tool frame or an end-effector for a given manipulator
function Vb = BodyVelTwist(Jb,thetadot)
% compute body velocity twist.
Vb = Jb*thetadot;
end

