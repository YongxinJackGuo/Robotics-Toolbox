function pose_end = RRR_direct_2D(Link_Lengths, config) % question (a)
angleConverter = 180/pi;  % convert radian to angles
% calculate x using D.P.K equations
pose_end(1) = Link_Lengths(1)*cos(config(1))+Link_Lengths(2)*cos(config(1)+config(2))+Link_Lengths(3)*cos(config(1)+config(2)+config(3));
% calculate y using D.P.K equations
pose_end(2) = Link_Lengths(1)*sin(config(1))+Link_Lengths(2)*sin(config(1)+config(2))+Link_Lengths(3)*sin(config(1)+config(2)+config(3));
% calculate phi using D.P.K equations
pose_end(3) = (config(1) + config(2) + config(3))*angleConverter;
end