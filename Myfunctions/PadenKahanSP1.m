% HW5 Problem 1: Paden Kahan Sub-problem 1. Single zero-pitch twist with point p and q.
function angle = PadenKahanSP1(axis_rotation,pt_p,pt_q,pt_r)
alpha = pt_p - pt_r; % compute vector rp;
beta = pt_q - pt_r;  % compute vector rq;
w = axis_rotation; % assign axis of rotation for convenience.
alpha_prime = alpha - w*transpose(w)*alpha;
beta_prime = beta - w*transpose(w)*beta;
angle = atan2(transpose(w)*cross(alpha_prime,beta_prime),transpose(alpha_prime)*beta_prime);
end

