function [S] = skew(u)
% Returns a skew-symetric of matrix

ux = u(1);
uy = u(2);
uz = u(3);

S = [0     -uz     uy;
     uz     0     -ux 
    -uy     ux     0 ];


end

