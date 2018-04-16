function [ T ] = rotz(theta)
% This function rotates a coordinate system by theta radians.
% around the z-axis
    T = [cos(theta) -sin(theta) 0            ;
         sin(theta) cos(theta)  0            ;
         0           0           1          ];
end
