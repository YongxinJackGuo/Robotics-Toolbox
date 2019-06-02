% A function that converts a vector into skew-asymmetric representation.

function V_hat = Vec_to_SkewAsy(V)
% V is a 3 by 1 column vector
v1 = V(1);
v2 = V(2);
v3 = V(3);
V_hat = [0 -v3 v2; v3 0 -v1; -v2 v1 0];
end

