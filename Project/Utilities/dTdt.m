function [Tdot] = dTdt(T, gamma, gammadot)
% This function returns the time derivative with respect to gamma of a
% direction cosine matrix. 
%   [Tdot] = dTdt(T, gamma, gammadot)

Tdot =     [jacobian(T(:,1), gamma)*gammadot, ...
            jacobian(T(:,2), gamma)*gammadot, ...
            jacobian(T(:,3), gamma)*gammadot]; 

end

