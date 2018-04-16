function [ Hrotz ] = Hrotz( theta )
%HROTZ Homogenous rotation about the z axis
%   Gives a homogeneous transformation matrix

z = [0;0;0];
Hrotz = [rotz(theta) z;...
         z'          1];

end