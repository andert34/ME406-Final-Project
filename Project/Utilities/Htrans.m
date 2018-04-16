function [ Htrans ] = Htrans( r1, r2, r3 )
%HTRANS Homogenous translation along the vector r
%   Gives a homogeneous translation matrix

sym Htrans;
r = [r1;r2;r3]; 

z = [0;0;0];
Htrans = [eye(3) r;...
         z'      1];

end