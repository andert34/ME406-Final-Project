function [IIrE, ITE, FK, NT, Nr] = SawyerFK(gamma)
% This function returns the Forward Kinematics of the Rethink Sawyer robot
% in the Task-Space, given the vector gamma of the joint coordinates.
% 
%   [IIrE, ITE, FK] = SawyerFK(gamma) returns the End-Effector position
%   (IIrE) and orientation (ITE), as well as a cell array representing the
%   position and orientation of all joints as measured in the Inertial
%   Frame as homogeneous matrices (FK). 
%   
% 
%   [IIrE, ITE, FK, NT, Nr] = SawyerFK(gamma) returns the End-Effector position
%   (IIrE) and orientation (ITE), as well as a cell array representing the
%   position and orientation of all joints as measured in the Inertial
%   Frame as homogeneous matrices (FK). In addition, this also returns a
%   cell array of rotation matrics (NT) and joint vectors (Nr) of each link
%   measured with respect to the previous link. 


    FK = cell(9,1);
    NT = cell(8, 1);
    Nr = cell(8, 1);
    
%   Body Frame
    NT{1} = Htrans(0,0,0.0825)*Hrotz(gamma(1));                 % Link 1 (1)
    NT{2} = Htrans(0.081,0.062,0.236)*Hroty(gamma(2));          % Link 2 (2)    
    NT{3} = Htrans(0.14,0.1315,0)*Hrotx(gamma(3));              % Link 3 (3)   
    NT{4} = Htrans(0.263,-0.045,0)*Hroty(-gamma(4));            % Link 4 (4)    
    NT{5} = Htrans(0.1265,-0.1265,0)*Hrotx(gamma(5));           % Link 5 (5)
    NT{6} = Htrans(0.277,0.03975,0)*Hroty(gamma(6));            % Link 6 (6)
    NT{7} = Htrans(0.07637,0.0958,0)*Hrotx(gamma(7));           % Link 7 (7)
    NT{8} = Htrans(0.06325,0,0);                                % End Eff. (E)
    
%   Inertial Frame
    FK{1} = NT{1};                                              % Link 1 (I)
    FK{2} = FK{1}*NT{2};                                        % Link 2 (I)
    FK{3} = FK{2}*NT{3};                                        % Link 3 (I)
    FK{4} = FK{3}*NT{4};                                        % Link 4 (I)
    FK{5} = FK{4}*NT{5};                                        % Link 5 (I)
    FK{6} = FK{5}*NT{6};                                        % Link 6 (I)
    FK{7} = FK{6}*NT{7};                                        % Link 7 (I)
    FK{8} = FK{7}*NT{8};                                        % End Eff. (I)
    FK{9} = FK{1}*Htrans(0,0,0.31);                             % Screen (I)
    
    
    IIrE = FK{8}(1:3,4);
    ITE = FK{8}(1:3,1:3);
    
%   Optional Body Measurements
    if(nargout > 3)
       for k = 1:8
          Nr{k} = NT{k}(1:3, 4);
          NT{k} = NT{k}(1:3, 1:3);
       end
    end
end


