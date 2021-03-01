function omega = computeOmega(ori)
%calculates transform between spatial velocity and time derivative of pose.
phi = ori(1);
theta = ori(2);
psi = ori(3);
omega = [1  sin(phi)*tan(theta)  cos(phi)*tan(theta);
         0       cos(phi)             -sin(phi)     ;
         0  sin(phi)/cos(theta)  cos(phi)/cos(theta)];
end

