% testing out kinematics for MR

% initialize the rover object
syms w l h r
q = sym(zeros(10, 1));
q_dot = sym(zeros(10, 1));
% adjust height above ground
q(6) = r;

% rover object
rover = symbolicMR(w, l, h, r, q);

% simulation time value
sim_start = 0;
sim_end = 10;
dt = 0.01;
times = sim_start:dt:sim_end;

state = sym(zeros(10, length(times)));
state_dot = sym(zeros(10, length(times)));

for q_t = 1:length(times)
%     % navigation kinematics
%     % desired joint velocities rad/s
%     q_dot(7:10) = simulationWaypoints(q_t, 'nav', length(times), rover.q_);
%     % forward kinematics loop
%     state(:, q_t) = rover.navigationLoop(q_dot, dt);
    
    % actuation kinematics
    q_dot(1:6) = simulationWaypoints(q_t, 'act', length(times), rover.q_);
    % actuation loop
    state(:, q_t) = rover.actuationLoop(q_dot, dt);
    state_dot(:, q_t) = rover.q_dot_;
end