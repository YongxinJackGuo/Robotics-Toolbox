function pos_end = RR_direct_2D(Link_Lengths,config)   %output position of end-effector (column vector) by inputting angles and link lengths
pos_end(1) = Link_Lengths(1)*cos(config(1))+Link_Lengths(2)*cos(config(1)+config(2));  % x-component
pos_end(2) = Link_Lengths(1)*sin(config(1))+Link_Lengths(2)*sin(config(1)+config(2));  % y-component
end

