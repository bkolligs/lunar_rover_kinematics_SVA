function [] = drawAxes(root,axes)
% draw axes around a base frame for visual clarity
plot3([root(1);axes(1, 1, 1)], [root(2);axes(2, 1, 1)], [root(3);axes(3, 1, 1)], 'r')
plot3([root(1);axes(1, 2, 1)], [root(2);axes(2, 2, 1)], [root(3);axes(3, 2, 1)], 'g')
plot3([root(1);axes(1, 3, 1)], [root(2);axes(2, 3, 1)], [root(3);axes(3, 3, 1)], 'b')
end

