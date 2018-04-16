function [] = SawyerAnimation(bx,by,bz,theta,phi,psi,theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta8, fignum)
%   Animate the 14-DOF Spherical Robot designed by Dr. Isenberg.
%   This function accpets three joint angles, plots the Spherical 3-DOF 
%   robot, and optionally returns the graphics handles corresponding to  
%   each link of the robot.
%
%   [hB hL1 hL2 hL3] = SphericalAnimation(theta1,theta2,d)
%       returns the figure handles corresponding to each link.
%
%
%   SpaceAnimation(theta1,theta2,d, fignum)  
%       animates the Spherical 3DOF robot on the figure corresponding to 
%       the optional integer parameter fignum.
%
%   Example:
%   SphericalAnimation(0, 0, 0, 3)
%       This animates the robot in the zeroed configuration on figure 3.

persistent FH BH L1H L2H L3H L4H L5H L6H L7H ...
    Base Base_f Link1 Link1_f Link2 ...
    Link2_f Link3 Link3_f Link4 Link4_f Link5 Link5_f Link6 Link6_f...
    Link7 Link7_f Link8 Link8_f CouplerColor LinkColor TipColor

if(~sum(ishghandle(FH)))
    % If the figure has been closed (or was never open),
    % import stl data, plot base, and format figure
    [Base, Base_f]   = stlread('./SpaceSTL/SpaceRobot2018Spacecraft.STL');
    [Link1, Link1_f] = stlread('./SpaceSTL/SpaceRobot2018Coupler.STL');
    [Link2, Link2_f] = stlread('./SpaceSTL/SpaceRobot2018Link.STL');
    [Link3, Link3_f] = stlread('./SpaceSTL/SpaceRobot2018Coupler.STL');
    [Link4, Link4_f] = stlread('./SpaceSTL/SpaceRobot2018Link.STL');
    [Link5, Link5_f] = stlread('./SpaceSTL/SpaceRobot2018Coupler.STL');
    [Link6, Link6_f] = stlread('./SpaceSTL/SpaceRobot2018Link.STL');
    [Link7, Link7_f] = stlread('./SpaceSTL/SpaceRobot2018Coupler.STL');
    [Link8, Link8_f] = stlread('./SpaceSTL/SpaceRobot2018Tip.STL');
    
    % The 'fignum' parameter is optional, and defaults to the next
    % available figure.
    if(exist('fignum', 'var'))
        FH = figure(fignum);
    else
        FH = figure;
    end
    
    % Format Figure Lighting and Axes
    camlight left
    axis equal
    set(gca, 'projection', 'perspective')
    view([1;1.;.5])
    axis([-0.7 0.7 -0.7 0.7 -0.2 0.7])
    hold on;
    xlabel('x (m)')
    ylabel('y (m)')
    zlabel('z (m)')
    title('Spherical 3DOF Robot')
    grid minor
    
else
    
    % If the figure is open, remove links (because they're probably moving)
    % in preparation for a new plot
    delete(BH);
    delete(L1H);
    delete(L2H);
    delete(L3H);
    delete(L4H);
    delete(L5H);
    delete(L6H);
    delete(L7H);
    delete(L8H);
end

% Forward Kinematics of the Robot using Homogeneous Matrices
HB = Htrans(bx,0,0)*Htrans(0,by,0)*Htrans(0,0,bz)*Hrotx(theta+pi/2)*Hroty(phi)*Hrotz(psi);
H1 = HB * Htrans(0,0,0.16)*Hrotz(theta1);
H2 = H1 * Htrans(0.02,0,0.02)*Hrotx(theta2);
H3 = H2 * Htrans(0,0,0.08)*Hrotz(theta3);
H4 = H3 * Htrans(0.02,0,0.02)*Hrotx(theta4);
H5 = H4 * Htrans(0,0,0.08)*Hrotz(theta5);
H6 = H5 * Htrans(0.02,0,0.02)*Hrotx(theta6);
H7 = H6 * Htrans(0,0,0.08)*Hrotz(theta7);
H8 = H7 * Htrans(0.02,0,0.02)*Hrotx(theta8);


% Link Model Vertices
Base_v =  HB*[Base, ones([length(Base) 1])]';
Link1_v = H1*[Link1, ones([length(Link1) 1])]';
Link2_v = H2*[Link2, ones([length(Link2) 1])]';
Link3_v = H3*[Link3, ones([length(Link3) 1])]';
Link4_v = H4*[Link4, ones([length(Link4) 1])]';
Link5_v = H5*[Link5, ones([length(Link5) 1])]';
Link6_v = H6*[Link6, ones([length(Link6) 1])]';
Link7_v = H7*[Link7, ones([length(Link7) 1])]';
Link8_v = H8*[Link8, ones([length(Link8) 1])]';

CouplerColor = [ 0.768627, 0.109804, 0.0745098];
TipColor = [0.878431, 0.670588, 0.227451];
LinkColor = [0.8000 0.3098 0.2863];

% Plot Each Link
BH  = patch('Faces',Base_f, 'Vertices',Base_v(1:3,:)', 'EdgeColor','None','FaceColor', CouplerColor);
L1H = patch('Faces',Link1_f,'Vertices',Link1_v(1:3,:)','EdgeColor','None','FaceColor', CouplerColor);
L2H = patch('Faces',Link2_f,'Vertices',Link2_v(1:3,:)','EdgeColor','None','FaceColor', TipColor);
L3H = patch('Faces',Link3_f,'Vertices',Link3_v(1:3,:)','EdgeColor','None','FaceColor', CouplerColor);
L4H = patch('Faces',Link4_f,'Vertices',Link4_v(1:3,:)','EdgeColor','None','FaceColor', TipColor);
L5H = patch('Faces',Link5_f,'Vertices',Link5_v(1:3,:)','EdgeColor','None','FaceColor', CouplerColor);
L6H = patch('Faces',Link6_f,'Vertices',Link6_v(1:3,:)','EdgeColor','None','FaceColor', TipColor);
L7H = patch('Faces',Link7_f,'Vertices',Link7_v(1:3,:)','EdgeColor','None','FaceColor', CouplerColor);
L8H = patch('Faces',Link8_f,'Vertices',Link8_v(1:3,:)','EdgeColor','None','FaceColor', TipColor);


% This small delay allows the figure to be refreshed WITHOUT
% using the MatLab "drawnow" command, which is less efficient
pause(0.000001)
end

