% a function that computes the torque for random configruations generated
% in another function.
function torque = RRR_torque_2D(force,numberOfConfigs,config,Link_Lengths)
torque = zeros(3,numberOfConfigs); % declare the torque matrix
l1 = Link_Lengths(1); % assign link 1
l2 = Link_Lengths(2); % assign link 2
l3 = Link_Lengths(3); % assign link 3
for i = 1:numberOfConfigs
    % compute sine and cosine for the J_transport matrix
    s1 = sin(config(1,i));
    s12 = sin(config(1,i)+config(2,i));
    s123 = sin(config(1,i)+config(2,i)+config(3,i));
    c1 = cos(config(1,i));
    c12 = cos(config(1,i)+config(2,i));
    c123 = cos(config(1,i)+config(2,i)+config(3,i));
    J_transport = [-l1*s1-l2*s12-l3*s123,   l1*c1+l2*c12+l3*c123;
                    -l3*s123-l2*s12,        l3*c123+l2*c12;
                    -l3*s123,               l3*c123];
    %compute the torque using equation of tau = J_t*force
    torque(:,i) = J_transport*force;
end
end
