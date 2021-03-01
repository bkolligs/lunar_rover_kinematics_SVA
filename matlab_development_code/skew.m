function skewed = skew(vector)
%skew forms a skew symmetric matrix from a 3x3 vector
a = vector(1);
b = vector(2);
c = vector(3);
skewed = [ 0 -c  b;
           c  0 -a;
          -b  a  0];
end

