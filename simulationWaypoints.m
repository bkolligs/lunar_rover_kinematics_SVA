function output = simulationWaypoints(time, direction, total_time, cur_state)
%simulationWaypoints passes different joint rates depending on the time
if strcmp('nav', direction)
    if time > 0 && time <= 0.25*total_time
        % drive forward
        output = [pi;
                  pi;
                  pi;
                  pi];
    elseif time > 0.25*total_time && time <= 0.5*total_time
        % turn right in place
        output = [2*pi;
                 -2*pi;
                  2*pi;
                 -2*pi];

    elseif time > 0.5*total_time && time <= 0.75*total_time
        % drive forward
        output = [pi;
                  pi;
                  pi;
                  pi];

    elseif time > 0.75*total_time && time <= 0.9*total_time
        % turn left
        output = [ 0;
                   4*pi;
                   0;
                   6*pi];
    else
        % drive forward
        output = [pi;
                  pi;
                  pi;
                  pi];
    end

elseif strcmp('act', direction)
    % desired body velocities...really only makes sense if x and y are
    % together
    % [phi_dot; theta_dot; psi_dot; x_dot; y_dot; z_dot]
    if time > 0 && time <= 0.1*total_time
        output = [0;
                  0;
                  0;
                  1;
                  0;
                  0];
    elseif time > 0.1*total_time && time <= 0.5*total_time
        [psi_dot, x_dot] = driveArcActuation(0.5, pi/4, 4);
        output = [0;
                  0;
                  psi_dot;
                  x_dot;
                  0;
                  0];
    elseif time > 0.5*total_time && time <= 0.7*total_time
        output = [0;
                  0;
                  0;
                  1;
                  0;
                  0];
    elseif time > 0.7*total_time && time <= 0.8*total_time
        [psi_dot, x_dot] = driveArcActuation(0.5, pi/4, 1);
        output = [0;
                  0;
                  psi_dot;
                  x_dot;
                  0;
                  0];
    elseif time > 0.8*total_time && time < total_time*0.9
        output = [0;
                  0;
                  0;
                  3;
                  0;
                  0];
    else
        output = [0;
                  0;
                  0;
                  0;
                  0;
                  0];
    end
end
end

