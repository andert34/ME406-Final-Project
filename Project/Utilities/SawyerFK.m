function [IIrE, ITE, IH] = SawyerFK(gamma)
% This function returns the Forward Kinematics in the Task-Space, given the
% vector gamma of the joint coordinates.
    IH = cell(9,1);
    
    IH{1} = Htrans(0,0,0.0825)*Hrotz(gamma(1));                 % Link 1
    IH{2} = IH{1}*Htrans(0.081,0.062,0.236)*Hroty(gamma(2));    % Link 2
    IH{3} = IH{2}*Htrans(0.14,0.1315,0)*Hrotx(gamma(3));        % Link 3
    IH{4} = IH{3}*Htrans(0.263,-0.045,0)*Hroty(-gamma(4));      % Link 4
    IH{5} = IH{4}*Htrans(0.1265,-0.1265,0)*Hrotx(gamma(5));     % Link 5
    IH{6} = IH{5}*Htrans(0.277,0.03975,0)*Hroty(gamma(6));      % Link 6
    IH{7} = IH{6}*Htrans(0.07637,0.0958,0)*Hrotx(gamma(7));     % Link 7
    IH{8} = IH{7}*Htrans(0.06325,0,0);                          % End. Eff.
    IH{9} = IH{1}*Htrans(0,0,0.31);                             % Screen 
    
    IIrE = IH{8}(1:3,4);
    ITE = IH{8}(1:3,1:3);
end

