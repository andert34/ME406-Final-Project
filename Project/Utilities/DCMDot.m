function [ Tdot ] = DCMDot( T, gamma, gammadot )
%DCMDot finds the time derivative of a DCM 
%   Finds Tdot given T, gamma, and gammadot symbolic variables

Tdot = [jacobian(T(:,1),gamma)*gammadot, jacobian(T(:,2),gamma)*gammadot, ...
        jacobian(T(:,3),gamma)*gammadot];

end

