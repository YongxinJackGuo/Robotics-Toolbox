% This function takes link length and end-effector position as the input and output the joint angles for 2D RR planar case.
function config = RR_inverse_2D(Link_Lengths,pos_end)  %output angles (2 by 2 matrix, each column vector for each of the two solns) by inputting posistion of end-effector and link lengths
cosTheta2 = (pos_end(1)^2+pos_end(2)^2-Link_Lengths(1)^2-Link_Lengths(2)^2)/(2*Link_Lengths(1)*Link_Lengths(2)); %first equation for theta2
if (cosTheta2 >= -1)&&(cosTheta2 <= 1)  %check if the point is accessible.
    sinTheta2 = sqrt(1-cosTheta2^2); %second equation for theta2
    
    config(2,1) = atan2(sinTheta2,cosTheta2);  %calculate theta2 for the first soln.
    config(1,1) = atan2(pos_end(2),pos_end(1)) - atan2(Link_Lengths(2)*sinTheta2,Link_Lengths(1)+Link_Lengths(2)*cosTheta2); %calculate theta1 for the first soln.
    config(2,2) = atan2(-1*sinTheta2,cosTheta2); %theta2 for second soln.
    config(1,2) = atan2(pos_end(2),pos_end(1)) - atan2(Link_Lengths(2)*-1*sinTheta2,Link_Lengths(1)+Link_Lengths(2)*cosTheta2); %calculate theta1 for the first soln.
else
    config = "The point is not accessible"; %retunr the error message.
end
end