function [psi_dot,x_dot] = driveArcActuation(r, v, time)
%driveArcActuation returns the corresponding x_dot and y_dot for a
%particular drive arc
% returns:
%       psi_dot - yaw velocity in radians/s
%       x_dot - desired x velocity
psi_dot = v/(r*time);
x_dot = v/time;
end

