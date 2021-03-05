function T = homogTransform(ori, pos)
% now to apply any rotations
% TODO: check the order of the euler angles
% x axis rotation
alpha = ori(1);
Rx = [1     0           0       0;
      0 cos(alpha) -sin(alpha)  0;
      0 sin(alpha)  cos(alpha)  0;
      0     0           0       1];
% y axis rotation
beta = ori(2);
Ry = [cos(beta)  0 sin(beta)  0;
         0       1     0      0;
     -sin(beta)  0 cos(beta)  0;
         0       0     0      1];
% z axis rotation
gamma = ori(3);
Rz = [cos(gamma) -sin(gamma) 0 0;
      sin(gamma)  cos(gamma) 0 0;
         0           0       1 0;
         0           0       0 1];
T = Rz*Ry*Rx;
T(1:3, 4) = pos;
end