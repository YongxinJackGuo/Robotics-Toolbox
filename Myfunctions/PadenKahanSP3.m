% HW5 Problem 3: Paden Kahan Sub-problem 3. Single zero-pitch twist with point p, q and distance delta
function angle = PadenKahanSP3(axis_rotation, pt_p, pt_q, pt_r, delta)
alpha = pt_p - pt_r; % compute vector rp;
beta = pt_q - pt_r;  % compute vector rq;
w = axis_rotation; % assign axis of rotation for convenience.
alpha_prime = alpha - w*transpose(w)*alpha;
beta_prime = beta - w*transpose(w)*beta;
deltaPrimeSq = delta^2 - (abs(transpose(w)*(pt_p - pt_q)))^2;
alphaPrimeMagSq = transpose(alpha_prime)*alpha_prime;
betaPrimeMagSq = transpose(beta_prime)*beta_prime;
alphaPrimeMag = sqrt(alphaPrimeMagSq);
betaPrimeMag = sqrt(betaPrimeMagSq);
theta0 = atan2(transpose(w)*cross(alpha_prime,beta_prime),transpose(alpha_prime)*beta_prime);
phi = acos((alphaPrimeMagSq + betaPrimeMagSq - deltaPrimeSq)/(2*alphaPrimeMag*betaPrimeMag)); % angle phi. in mls page 103.
theta1 = theta0 + phi; % first solution
theta2 = theta0 - phi; % second solution
angle = [theta1; theta2];
end

