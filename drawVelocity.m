function [outputArg1,outputArg2] = drawVelocity(vc, w, l)
%drawVelocity visualizes velocity vectors in the base frame
% display magnitudes
mags = [norm(vc(1:3)); norm(vc(4:6)); norm(vc(7:9));norm(vc(10:12))]

clf
hold on
plot([w vc(1)], [l vc(2)])
plot([-w vc(4)], [l vc(5)])
plot([w vc(7)], [-l vc(8)])
plot([-w vc(10)], [-l vc(11)])

xlim([-0.4, 0.4])
ylim([-0.4, 0.4])
end

