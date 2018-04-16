function [ T ] = roty(theta)
% This function rotates a coordinate system by theta radians.
% around the y-axis
    T = [cos(theta) 0           sin(theta);
         0          1          0          ;
        -sin(theta) 0          cos(theta);];
end
