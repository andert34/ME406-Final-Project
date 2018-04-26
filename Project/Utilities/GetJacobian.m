function [J, Jdot] = GetJacobian(gamma, gammadot)
% This function accepts a vector of joint coordinates and generates the
% geometric Jacobian for the Rethink Sawyer robot for each link, computed
% recursively.
%
%   J = GetJacobian(gamma) returns the Jacobian for each link as cells in a
%   cell array, after computing the forward kinematics.
%
%   [J, Jdot] = GetJacobian(gamma, gammadot) returns the Jacobian for each
%   link as cells in a cell array, after computing the forward kinematics.
%   Optional argument gammadot will generate the time derivative of the
%   Geometric Jacobian, returned as variable Jdot.

[~, ~, IT, NT, Nr] = SawyerFK(gamma);

for k = 1:8
   IT{k} = IT{k}(1:3, 1:3);
end

J = cell(8,1);
Ihat = cell(8,1);


rotInd = [3 2 1 2 1 2 1];
for h = 1:7
    Ihat{h} = zeros(3,7);
    Ihat{h}(rotInd(h), h) = 1;
end
Ihat{8} = zeros(3,7);

J{1} = [Ihat{1};zeros(3, 7)];
for k = 2:8
    J{k} = [NT{k}.'                             zeros(3);
        -IT{k-1}*skew(Nr{k})     eye(3) ]*J{k-1}...
        + [Ihat{k};zeros(3,7)];
end
if(nargin > 1)
    Jdot = cell(8,1);
    Iw = cell(8,1);             % Inertial Frame Angular Velocities
    Irdot = cell(8,1);          % Inertial Frame Translational Velocities
    Nw = cell(8,1);             % Body Frame Angular Velocities
    wr = J{2}*gammadot;
    Iw{1} = wr(1:3);
    Irdot{1} = wr(4:6);
    
    % Generate Body Frame Ang. Velocities
    for k = 1:8
        Nw{k} = Ihat{k}*gammadot;
        wr = J{k}*gammadot;
        Iw{k} = wr(1:3);
        Irdot{k} = wr(4:6);
    end
    Jdot{1} = zeros(6,7);
    
    % Generate Jacobian Derivative
    for k = 2:8
        Jdot{k} = [-skew(Nw{k})*NT{k}.'                     zeros(3);
            -IT{k-1}*(skew(Iw{k-1})*skew(Nr{k}))     zeros(3)]*J{k-1}...
            +[NT{k}.'              zeros(3);
            -IT{k-1}*skew(Nr{k})     eye(3)]*Jdot{k-1};
    end
end
end

