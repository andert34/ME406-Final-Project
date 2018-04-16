function [ Hrotx ] = Hrotx( theta )
%HROTX Homogenous rotation about the x axis
%   Gives a homogeneous transformation matrix

z = [0;0;0];
Hrotx = [rotx(theta) z;...
         z'          1];

end

