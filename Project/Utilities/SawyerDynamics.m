function [ bdot Hm Dm Gm Bm Cm ] = SawyerDynamics( b )
%SawyerDynamics Solves for the current state rate (bdot) of the Sawyer
%robot given the current state (b) and optionally returns Hm, Dm, Gm, Bm,
%and Cm. 
%
%   The input state b should be a 14 element vector of the form:
%
%   theta1
%   theta2
%   theta3
%   theta4
%   theta5
%   theta6
%   theta7
%   theta1dot
%   theta2dot
%   theta3dot
%   theta4dot
%   theta5dot
%   theta6dot
%   theta7dot
%
%   Hm, Dm, Gm, Bm, and Cm are 7x7 matrices. 

% Retrieve vectors of generalized coordinates from system state
gamma = b(1:7);
gammadot = b(8:14);

% Find recursively calculated Jacobians and Jacobian dots
[J, Jdot] = getJacobian(gamma,gammadot);

%% Define System Parameters
persistent BBJ
persistent BBGam
persistent m
persistent BBrcm

% Link Masses
m = [8.19264294;    % Link 1
	 5.50781768;    % Link 2
	 3.81978662;    % Link 3
	 3.37865995;    % Link 4
	 3.04430043;    % Link 5
     1.93085347;    % Link 6
	 0.66211106;    % Link 7
	 2.52094779];   % Screen
 
% Center of mass locations (meters, body-frame of link)
BBrcm = {[0.02734797; -0.00317970; 0.15563460];     % Link 1
         [0.02479837; 0.09522711; 0.00001324];      % Link 2
         [0.14336135; -0.00104665; 0.00000231];     % Link 3
         [0.02658902; -0.09611792; -0.00006966];    % Link 4
         [0.14486497; 0.00099740; 0.00027129];      % Link 5
         [0.01219767; 0.07446539; -0.00128796];     % Link 6
         [0.03037622; -0.01061201; -0.00000331];    % Link 7
         [0.00366851; 0.00049050; 0.11638732]};     % Screen

% Inertia Matrices (kg m^2)
BBJ  = { [0.26607023 -0.00038981 -0.04801856; ...   % Link 1
         -0.00038981 0.28078511 0.00174486; ...
         -0.04801856 0.00174486 0.03266273];
    
         [0.06807013 -0.01811791 0.00000238; ...  % Link 2
         -0.01811791 0.02398210 -0.00001133; ...
          0.00000238 -0.00001133 0.08122087];
     
         [0.00458681 0.00169482 0.00000052; ...  % Link 3
          0.00169482 0.11052135 -0.00000006; ...
          0.00000052 -0.00000006 0.10995021];
     
         [0.03973192 0.01142099 -0.00000198; ...  % Link 4
          0.01142099 0.01197658 -0.00003404; ...
         -0.00000198 -0.00003404 0.04758280];

         [0.00284307 -0.00117636 -0.00008063; ...  % Link 5
         -0.00117636 0.08916094 -0.00000018; ...
         -0.00008063 -0.00000018 0.08895676];
    
         [0.01427892 -0.00224277 0.00003962; ...  % Link 6
         -0.00224277 0.00311424 0.00019771; ...
          0.00003962 0.00019771 0.01544417]; 
     
         [0.00082641 0.00022344 0.00000013; ...  % Link 7
          0.00022344 0.00103886 0.00000004; ...
          0.00000013 0.00000004 0.00134811]; 

         [0.04954300 -0.00001992 -0.00086409; ...  % Screen
         -0.00001992 0.04540562 -0.00012555; ...
         -0.00086409 -0.00012555 0.00621396]};
    
% Vectors of First Mass Moments
BBGam = {  [0.2241   -0.0261    1.2751];  % Link 1
           [0.1366    0.5245    0.0001];  % Link 2
           [0.5476   -0.0040    0.0000];  % Link 3
           [0.0898   -0.3247   -0.0002];  % Link 4
           [0.4410    0.0030    0.0008];  % Link 5
           [0.0236    0.1438   -0.0025];  % Link 6
           [0.0201   -0.0070   -0.0000];  % Link 7
           [0.0092    0.0012    0.2934]}; % Screen
           
% Define Pittman DC054B-1 Motor Parameters
persistent Ra Ja Kt Kemf ba Ca N eta
Ra = 2.79; % Armature resistance in ohms
Ja = 1.1e-5; % Armature inertia in kg m^2
Kt = 0.0525; % Torque constant in Nm/A
Kemf = 0.0525; % Back-EMF constant in Nm/A
ba = 1.1e-5; % Motor viscous friction in Nm s/rad
Ca = 0.0085; % Motor static friction torque in Nm

% Define Pittman 19.7:1 G51A Spur Gearbox Parameters
N = 19.7;
eta = 0.73;

Jm = diag([Ra*Ja*N/Kt, Ra*Ja*N/Kt, Ra*Ja*N/Kt, Ra*Ja*N/Kt, Ra*Ja*N/Kt,...
     Ra*Ja*N/Kt, Ra*Ja*N/Kt],0);
 
Bm = diag([Ra*ba*N/Kt + Kemf*N, Ra*ba*N/Kt + Kemf*N, Ra*ba*N/Kt + Kemf*N,...
     Ra*ba*N/Kt + Kemf*N, Ra*ba*N/Kt + Kemf*N, Ra*ba*N/Kt + Kemf*N, ...
     Ra*ba*N/Kt + Kemf*N],0);

Cm = diag([Ra*Ca/Kt*sign(N), Ra*Ca/Kt*sign(N), Ra*Ca/Kt*sign(N), ...
     Ra*Ca/Kt*sign(N), Ra*Ca/Kt*sign(N), Ra*Ca/Kt*sign(N), Ra*Ca/Kt*sign(N)],0);

Rm = diag([Ra/(Kt*eta*N), Ra/(Kt*eta*N), Ra/(Kt*eta*N), Ra/(Kt*eta*N), ...
     Ra/(Kt*eta*N), Ra/(Kt*eta*N), Ra/(Kt*eta*N)],0);

%% Calculate H, D, and G

% Get DCMs in inertial frame
[~, ~, IT] = SawyerFK(gamma);
for k = 1:8
   IT{k} = IT{k}(1:3, 1:3);
end

% H and D
H = zeros(7);
D = zeros(7,1);
G = zeros(7,1);
Iw = cell(7);

for h = 1:7
    % goober is common to H and D
    goober = J{h}.'*[BBJ{h}          skew(BBGam{h})*IT{h}.'; ...
             IT{h}*skew(BBGam{h}).'            m(h)*eye(3)]; 

    H = H + goober*J{h};
    
    wr = J{h}*gammadot;
    Iw{h} = wr(1:3);    % Angular velocity of joint wrt inertial frame
     
    D = D + goober*Jdot{h}*gammadot + J{h}.'*[cross(Iw{h},BBJ{h}*Iw{h}); ...
        IT{h}*cross(Iw{h},cross(Iw{h},BBGam{h}))];
    
    G = G + J{h}.'*[cross(BBrcm{h},IT{h}.'*[0;0;-m{h}*g]); [0;0;m{h}*g]];
end

% Optional Outputs
Hm = (Jm + Rm*H); % System mass matrix including motor parameters
Dm = Rm*D;        % Vector of coriolis and centripetal forces
Gm = Rm*G;        % Vector of gravitational forces

bdot(1:7) = b(8:14);             % New joint velocities
bdot(8:14) = Hm \ (Va - Dm - (Bm)*b(8:14) ...
    - (Cm)*sign(bdot(1:7)) - Gm); % Joint accelerations

end