% This function checks the first criterion that if the manipulator can reach the point or not by simply compare the distance of the point to the origin and the maximum distance of the tool frame to the origin.

function bolean = manip_distCheck(Link_lengths,P)
% output returns 1 if reachable and 0 if not.

% extract the position vector and calculate the maximum distance from tool
% frame to the origin.
dist_max = sum(Link_lengths);

% compute distance of the point.
dist = sqrt(transpose(P)*P);

if dist <= dist_max
    bolean = 1;
else
    bolean = 0;
end

end