% P1 Q(b). A function that takes joint angles and body configs as the input and output the contact points for four legs
function contact_pts = getContactPt(joint_angles,body_configs,body_consts)
    % joint angles is a 2 by 4 matrix that consists of two angles from each
    % of four legs.
    % body configurations is a transformation matrix
    % body consts stores constants associated with the robot, such as
    % link length, body length and width. body_consts=[l1;l2;w;l];
    w = body_consts(3); % x-component distance from the hip to the body center.
    l = body_consts(4); % y-component distance from the hip to the body center.
    l1 = body_consts(1); % link length 1
    l2 = body_consts(2); % link length 2
    I = eye(3);
    contact_pts = zeros(4,4); % assign contact points beforehand. it is 4 by 4 matrix becaue of homo. rep. for points
    legNum = 4; % assign leg number.
    T_BToW = body_configs; % assign the transformation matrix from body frame to world frame first
    % assign the position vector between the body frame to the hip frames
    % expressed in body frame.
    P_bToh1 = [w;-l;0]; 
    P_bToh2 = [-w;-l;0];
    P_bToh3 = [-w;l;0];
    P_bToh4 = [w;l;0];
    % put them in a aggregated matrix for being easier to assign them in
    % the transformation matrix from hip frame to body frame.
    P_bTohi = [P_bToh1,P_bToh2,P_bToh3,P_bToh4]; 
    for i = 1:legNum
        T_HiToB = [I,P_bTohi(:,i);[0 0 0],1]; % transformation matrix from hip to body
        % assign theta 1 and 2 for each of four legs.
        theta1i = joint_angles(1,i);
        theta2i = joint_angles(2,i);
        % create the position vector from contact point to the hip joint
        % expressed in hip frame {H}.
        P_tiToHi = [0;l1*cos(theta1i)+l2*cos(theta1i+theta2i);l1*sin(theta1i)+l2*sin(theta1i+theta2i);1];
        % Convert the vector P_tiToHi expressed in hip frame {H} to P_tiToB
        % expressed in body frame {B}.
        P_tiToB = T_HiToB * P_tiToHi;
        % Convert P_tiToB expressed in {B} to P_tiToW expressed in world
        % frame {W}, which are just the contact points we're looking for.
        P_tiToW = T_BToW * P_tiToB;
        % assign the contact point for each of four legs into contact_pt
        % aggregated matrix.
        contact_pts(:,i) = P_tiToW; 
    end
end

