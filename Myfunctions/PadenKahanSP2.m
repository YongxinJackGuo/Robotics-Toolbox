% HW5 Problem 2: Paden Kahan Sub-problem 2. Double zero-pitch twist with point p, q and c.
function angles = PadenKahanSP2(axis_rotation1,axis_rotation2,pt_p,pt_q,pt_r)
% compute alpha and beta, which are the vector rp and rq.
alpha = pt_p - pt_r;
beta = pt_q - pt_r;
% assign axes of rotation for convenience.
w1 = axis_rotation1;
w2 = axis_rotation2;
% compute a1, a2, and a3.
den = (transpose(w1)*w2)^2-1;
a1 = ((transpose(w1)*w2)*transpose(w2)*alpha-transpose(w1)*beta)/den;
a2 = ((transpose(w1)*w2)*transpose(w1)*beta-transpose(w2)*alpha)/den;
alpha_mag_sq = transpose(alpha)*alpha; % compute the magnitude of alpha
w1Xw2_mag_sq = transpose(cross(w1,w2))*cross(w1,w2); % compute the magnitude of the vector of w1 cross w2.
a3_sq = (alpha_mag_sq-a1^2-a2^2-2*a1*a2*transpose(w1)*w2)/w1Xw2_mag_sq;
if a3_sq == 0
    a3 = a3_sq; % only one soln, when two circles intersect
    % compute vector gamma, which is vector rc.
    gamma = a1*w1+a2*w2+a3*(cross(w1,w2));
    pt_c = gamma + pt_r;
     % use SP1 function to get angles.
    theta2 = PadenKahanSP1(w2,pt_p,pt_c,pt_r);
    theta1 = PadenKahanSP1(w1,pt_c,pt_q,pt_r);
    angles(1,1:2) = [theta1; theta2];
elseif a3_sq < 0
    angles = "doesn't exit"; % no soln, two circles fail to intersect
else
    % two solns.
    a3_1 = sqrt(a3_sq);
    a3_2 = -1*a3_1; 
    % compute vector gamma, which is vector rc.
    gamma_1 = a1*w1+a2*w2+a3_1*(cross(w1,w2));
    gamma_2 = a1*w1+a2*w2+a3_2*(cross(w1,w2));
    % compute point c.
    pt_c_1 = gamma_1 + pt_r;
    pt_c_2 = gamma_2 + pt_r;
    % use SP1 function to get angles.
    % first soln.
    theta2_1 = PadenKahanSP1(w2,pt_p,pt_c_1,pt_r);
    theta1_1 = PadenKahanSP1(w1,pt_c_1,pt_q,pt_r);
    angles(1:2,1) = [theta1_1; theta2_1];
    % second soln
    theta2_2 = PadenKahanSP1(w2,pt_p,pt_c_2,pt_r);
    theta1_2 = PadenKahanSP1(w1,pt_c_2,pt_q,pt_r);
    angles(1:2,2) = [theta1_2; theta2_2];
end

    


end

