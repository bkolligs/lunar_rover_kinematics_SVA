function V = cartesian2SpatialVelocity(q, tList)
%cartesian2SpatialVelocity transforms q_dot into q_dot prime
T_w2b = tList(:, :, 1);
R_w2b = T_w2b(1:3, 1:3);
I = eye(4);
omega = computeOmega(q(1:3));
V = blkdiag(omega, R_w2b, I);
end

