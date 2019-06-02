% a function that takes #. config. and end eff. angle as the input, and output the configs.
function config_for_d = RRR_random_config(numberOfConfig,phi)
% angleConverter = 180/pi;  % convert radian to angles
% assign the angle limits.
lowerTheta1Limit = 0;
upperTheta1Limit = 3*pi/4;
lowerTheta2Limit = 3*pi/2;
upperTheta2Limit = 2*pi;
lowerTheta3Limit = 3*pi/2;
upperTheta3Limit = 2*pi;
config_for_d = zeros(3,numberOfConfig); % delcare a matrix for storing future random configs.
for i = 1:numberOfConfig
    theta3 = 0; % assign a dummy value to theta3 in order to get into the while loop
    while ~((lowerTheta3Limit<=theta3)&&(theta3<=upperTheta3Limit)) %check if theta3 is valid
        randNumber = rand(1,1); % assign a random number between 0 and 1.
        %generate the random angles using equation: lower+(upper-lower)*rand
        theta1 = lowerTheta1Limit+(upperTheta1Limit-lowerTheta1Limit)*randNumber;
        theta2 = lowerTheta2Limit+(upperTheta2Limit-lowerTheta2Limit)*randNumber;
        theta3 = (phi-theta1-(theta2-2*pi))+2*pi; % compute the theta3
    end
    config_for_d(:,i) = [theta1;theta2;theta3];% assign valid angles to matrix as the return
end
end