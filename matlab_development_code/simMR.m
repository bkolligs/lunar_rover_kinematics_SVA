% testing out kinematics for MR

% initialize the rover object
w = 0.1;
l = 0.2;
h = 0.1;
r = 0.07;
q = zeros(10, 1);
% adjust height above ground
q(6) = 0.17;

dt = 0.05;
q_last = q;
for q_t = 0:dt:4*pi
    % adjust wheel position at each timestep
    q(7:10) = [q_t;2*q_t;q_t;2*q_t];
%     % adjust vehicle orientation alpha (x), beta (y), gamma (z)
%     q(1:3) = [q_t/20; 0; q_t/10];
    

    rover = kinematicMR(w, l, h, r, q);
    tList = rover.constructTreeFromTable;
    J = rover.jacobianGenerate(tList);
    rover.q_dot_ = (rover.q_ - q_last) / dt;
    q_last = rover.q_;
    % testing jacobian function
    % transform to spatial velocity
    V = cartesian2SpatialVelocity(rover.q_, tList);
    vc = J*V*rover.q_dot_;
%     rover.q_dot_
    vc
    q_reverse = rover.q_ + rover.q_dot_ * dt;
    
%     drawVelocity(vc, w, l)
    modelMR(tList);
    pause(0.001)
end
