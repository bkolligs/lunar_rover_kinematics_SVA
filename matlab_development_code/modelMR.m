function [] = modelMR(t_list)
% graph the frame locations NOTE as of 01/17 there is an error somewhere
% TODO: find the error that is flipping x, y graphing
scale = 0.08;
basis = eye(3)*scale;
% obtain world to body origin point
origin = [0;0;0;1];

rigid = zeros(4, length(t_list));
rigid_frame = zeros(4, 3, length(t_list));
for i = 1:length(t_list)
    rigid(:, i) = t_list(:,:,i) * origin;
    rigid_frame(:, :, i) = t_list(:, :, i) * [basis; 1 1 1];
end
% rigid_frame
% rigid
clf
hold on
body_origin = rigid(1:3, 1);
% plot wireframe model body axes
drawAxes(body_origin, rigid_frame(:, :, 1));
% wheel fl
plotWheel(rigid, rigid_frame, body_origin, 2)

% wheel fr
plotWheel(rigid, rigid_frame, body_origin, 3)


% wheel bl
plotWheel(rigid, rigid_frame, body_origin, 4)

% wheel br
plotWheel(rigid, rigid_frame, body_origin, 5)

hold off

lims = [-4, 4];
xlim(lims)
ylim([-4, 4])
zlim([-0.1, 0.6])
% adjust camera view
view(0, 90)
xlabel('$x$', 'Interpreter', 'Latex', 'FontSize', 30)
ylabel('$y$', 'Interpreter', 'Latex', 'FontSize', 30)
zlabel('$z$', 'Interpreter', 'Latex', 'FontSize', 30)
end

