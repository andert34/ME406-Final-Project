function [ Hroty ] = Hroty( theta )
%HROTY Homogenous rotation about the y axis
%   Gives a homogeneous transformation matrix

z = [0;0;0];
Hroty = [roty(theta) z;...
         z'          1];

end