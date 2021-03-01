function [] = plotWheel(rigid, rigid_frame, body_origin, wheel_index)
%handles plotting of the particular wheel and its contact frame
% assumes that the contact frame is located 4 entries after wheel
plot3([body_origin(1);rigid(1,wheel_index)], ...
      [body_origin(2);rigid(2,wheel_index)], ...
      [body_origin(3);rigid(3,wheel_index)], 'm')
drawAxes(rigid(:, wheel_index), rigid_frame(:, :, wheel_index))
% wheel contact frame
drawAxes(rigid(:, wheel_index+4), rigid_frame(:, :, wheel_index+4))
end

