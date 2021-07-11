function [J, J_v, J_r, J_v_inv, J_r_inv] = jacobians(w, l, h, r)

J = [                0, - conj(h) - conj(r), -conj(w), 1, 0, 0, -r,  0,  0,  0;
conj(h) + conj(r),                   0,  conj(l), 0, 1, 0,  0,  0,  0,  0;
          conj(w),            -conj(l),        0, 0, 0, 1,  0,  0,  0,  0;
                0, - conj(h) - conj(r),  conj(w), 1, 0, 0,  0, -r,  0,  0;
conj(h) + conj(r),                   0,  conj(l), 0, 1, 0,  0,  0,  0,  0;
         -conj(w),            -conj(l),        0, 0, 0, 1,  0,  0,  0,  0;
                0, - conj(h) - conj(r), -conj(w), 1, 0, 0,  0,  0, -r,  0;
conj(h) + conj(r),                   0, -conj(l), 0, 1, 0,  0,  0,  0,  0;
          conj(w),             conj(l),        0, 0, 0, 1,  0,  0,  0,  0;
                0, - conj(h) - conj(r),  conj(w), 1, 0, 0,  0,  0,  0, -r;
conj(h) + conj(r),                   0, -conj(l), 0, 1, 0,  0,  0,  0,  0;
         -conj(w),             conj(l),        0, 0, 0, 1,  0,  0,  0,  0];
     
J_v = J(:, 1:6);
J_r = J(:, 7:10);

J_v_inv = [                           0,                           0,                                   1/(4*conj(w)),                           0,                           0,                                  -1/(4*conj(w)),                            0,                            0,                                   1/(4*conj(w)),                           0,                            0,                                 -1/(4*conj(w));
                           0,                           0,                                  -1/(4*conj(l)),                           0,                           0,                                  -1/(4*conj(l)),                            0,                            0,                                   1/(4*conj(l)),                           0,                            0,                                  1/(4*conj(l));
-w/(4*(abs(l)^2 + abs(w)^2)), l/(4*(abs(l)^2 + abs(w)^2)),                                               0, w/(4*(abs(l)^2 + abs(w)^2)), l/(4*(abs(l)^2 + abs(w)^2)),                                               0, -w/(4*(abs(l)^2 + abs(w)^2)), -l/(4*(abs(l)^2 + abs(w)^2)),                                               0, w/(4*(abs(l)^2 + abs(w)^2)), -l/(4*(abs(l)^2 + abs(w)^2)),                                              0;
                         1/4,                           0, -(l*(r*abs(h)^2 + h*abs(r)^2))/(4*h*r*abs(l)^2),                         1/4,                           0, -(l*(r*abs(h)^2 + h*abs(r)^2))/(4*h*r*abs(l)^2),                          1/4,                            0,  (l*(r*abs(h)^2 + h*abs(r)^2))/(4*h*r*abs(l)^2),                         1/4,                            0, (l*(r*abs(h)^2 + h*abs(r)^2))/(4*h*r*abs(l)^2);
                           0,                         1/4, -(w*(r*abs(h)^2 + h*abs(r)^2))/(4*h*r*abs(w)^2),                           0,                         1/4,  (w*(r*abs(h)^2 + h*abs(r)^2))/(4*h*r*abs(w)^2),                            0,                          1/4, -(w*(r*abs(h)^2 + h*abs(r)^2))/(4*h*r*abs(w)^2),                           0,                          1/4, (w*(r*abs(h)^2 + h*abs(r)^2))/(4*h*r*abs(w)^2);
                           0,                           0,                                             1/4,                           0,                           0,                                             1/4,                            0,                            0,                                             1/4,                           0,                            0,                                            1/4];
J_r_inv = [ -1/r, 0, 0,    0, 0, 0,    0, 0, 0,    0, 0, 0;
           0, 0, 0, -1/r, 0, 0,    0, 0, 0,    0, 0, 0;
           0, 0, 0,    0, 0, 0, -1/r, 0, 0,    0, 0, 0;
           0, 0, 0,    0, 0, 0,    0, 0, 0, -1/r, 0, 0];
end