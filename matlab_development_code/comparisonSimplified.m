% testing out kinematics for MR

% initialize the rover object
w = 0.1;
l = 0.2;
h = 0.1;
r = 0.07;
q = zeros(10, 1);
q_dot = zeros(10, 1);
% adjust height above ground
q(6) = 0.17;

% rover object
rover = kinematicMR(w, l, h, r, q);

% simulation time value
sim_start = 0;
sim_end = 10;
dt = 0.01;
times = sim_start:dt:sim_end;

state = zeros(10, length(times));
stateSTATIC = zeros(10, length(times));
state_dot = zeros(10, length(times));
state_dotSTATIC = zeros(10, length(times));

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
    stateSTATIC(:, q_t) = rover.actuationLoopSTATIC(q_dot, dt);
    state_dot(:, q_t) = rover.q_dot_;
    state_dotSTATIC(:, q_t) = rover.q_dot_STATIC_;

end

% plot the state variables for the online calculation
figure()
fontsize = 25;
subplot(2, 2, 1)
hold on
plot(state(4, :), state(5, :), 'r')
% plot(test_body(1, :), test_body(2, :), 'b')
hold off
% legend('Integration', 'Transform')
xlabel('$x$', 'Interpreter', 'Latex', 'FontSize', fontsize)
ylabel('$y$', 'Interpreter', 'Latex', 'FontSize', fontsize)
title('Rover Position $x,y$','Interpreter', 'Latex', 'FontSize', fontsize)

subplot(2, 2, 2)
plot(times, state(3, :))
xlabel('$t$', 'Interpreter', 'Latex', 'FontSize', fontsize)
ylabel('$\psi$', 'Interpreter', 'Latex', 'FontSize', fontsize)
title('Rover Yaw $\psi$','Interpreter', 'Latex', 'FontSize', fontsize) 

subplot(2, 2, 3)
hold on
plot(times, sin(state(7, :)), 'r')
plot(times, sin(state(9, :)), 'b--')
xlabel('$t$', 'Interpreter', 'Latex', 'FontSize', fontsize)
ylabel('$\sin(\theta_i)$', 'Interpreter', 'Latex', 'FontSize', fontsize)
hold off
legend('Front', 'Back')
title('Rover Left Wheels Joint Value','Interpreter', 'Latex', 'FontSize', fontsize)


subplot(2, 2, 4)
hold on
plot(times, sin(state(8, :)), 'r')
plot(times, sin(state(10, :)), 'b--')
xlabel('$t$', 'Interpreter', 'Latex', 'FontSize', fontsize)
ylabel('$\sin(\theta_i)$', 'Interpreter', 'Latex', 'FontSize', fontsize)
legend('Front', 'Back')
hold off
title('Rover Right Wheels Joint Value','Interpreter', 'Latex', 'FontSize', fontsize)

% plot the state variables for the offline calculation
figure()
fontsize = 25;
subplot(2, 2, 1)
hold on
plot(state(4, :), stateSTATIC(5, :), 'r')
% plot(test_body(1, :), test_body(2, :), 'b')
hold off
% legend('Integration', 'Transform')
xlabel('$x$', 'Interpreter', 'Latex', 'FontSize', fontsize)
ylabel('$y$', 'Interpreter', 'Latex', 'FontSize', fontsize)
title('Static Jacobian Rover Position $x,y$','Interpreter', 'Latex', 'FontSize', fontsize)

subplot(2, 2, 2)
plot(times, stateSTATIC(3, :))
xlabel('$t$', 'Interpreter', 'Latex', 'FontSize', fontsize)
ylabel('$\psi$', 'Interpreter', 'Latex', 'FontSize', fontsize)
title('Static Jacobian Rover Yaw $\psi$','Interpreter', 'Latex', 'FontSize', fontsize) 

subplot(2, 2, 3)
hold on
plot(times, sin(stateSTATIC(7, :)), 'r')
plot(times, sin(stateSTATIC(9, :)), 'b--')
xlabel('$t$', 'Interpreter', 'Latex', 'FontSize', fontsize)
ylabel('$\sin(\theta_i)$', 'Interpreter', 'Latex', 'FontSize', fontsize)
hold off
legend('Front', 'Back')
title('Static Jacobian Rover Left Wheels Joint Value','Interpreter', 'Latex', 'FontSize', fontsize)


subplot(2, 2, 4)
hold on
plot(times, sin(stateSTATIC(8, :)), 'r')
plot(times, sin(stateSTATIC(10, :)), 'b--')
xlabel('$t$', 'Interpreter', 'Latex', 'FontSize', fontsize)
ylabel('$\sin(\theta_i)$', 'Interpreter', 'Latex', 'FontSize', fontsize)
legend('Front', 'Back')
hold off
title('Static Jacobian Rover Right Wheels Joint Value','Interpreter', 'Latex', 'FontSize', fontsize)

