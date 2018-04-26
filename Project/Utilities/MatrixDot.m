function [ Tdot ] = MatrixDot( T, gamma, gammadot )
%DCMDot finds the time derivative of a DCM 
%   Finds Tdot given T, gamma, and gammadot symbolic variables

dimensions = size(T);
width = dimensions(2);

for i = 1:width
    Tdot(:,i) = jacobian(T(:,i),gamma)*gammadot;
end
end