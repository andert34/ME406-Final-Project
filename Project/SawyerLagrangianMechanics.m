%% Tymothy Anderson, Andy Worthington ME406 Final Project
% Find H, d and g for Sawyer using Lagrangian mechanics. 
clc;
clear;
tic
%% Symbolic Variables
syms theta1 theta2 theta3 theta4 theta5 theta6 theta7 
syms theta1dot theta2dot theta3dot thetadot theta5dot theta6dot theta7dot
syms theta1ddot theta2ddot theta3ddot theta4ddot theta5ddot ...
    theta6ddot theta7ddot

% Generalized coordinates (joint angles)
gamma = [bx by bz phi theta psi th1 th2 th3 th4 th5 th6 th7 th8].';
gammadot = [bxdot bydot bzdot phidot thetadot psidot th1dot th2dot th3dot...
    th4dot th5dot th6dot th7dot th8dot].';
gammaddot = [bxddot byddot bzddot phiddot thetaddot psiddot th1ddot...
    th2ddot th3ddot th4ddot th5ddot th6ddot th7ddot th8ddot].';

%% Mass Properties
% Link Masses (kilograms)
m1 = 8.19264294;
m2 = 5.50781768;
m3 = 3.81978662;
m4 = 3.37865995;
m5 = 3.04430043;
m6 = 1.93085347;
m7 = 0.66211106;
mscreen = 2.52094779;

% Center of mass locations (meters, body-frame of link)
r_11rcm = [0.02734797; -0.00317970; 0.15563460];
r_22rcm = [0.02479837; 0.09522711; 0.00001324];
r_33rcm = [0.14336135; -0.00104665; 0.00000231];
r_44rcm = [0.02658902; -0.09611792; -0.00006966];
r_55rcm = [0.14486497; 0.00099740; 0.00027129];
r_66rcm = [0.01219767; 0.07446539; -0.00128796];
r_77rcm = [0.03037622; -0.01061201; -0.00000331];
r_screenrcm = [0.00366851; 0.00049050; 0.11638732];

% Vectors of first-mass moments (kg m)
Gam1 = m1*r_11rcm;
Gam2 = m2*r_22rcm;
Gam3 = m3*r_33rcm;
Gam4 = m4*r_44rcm;
Gam5 = m5*r_55rcm;
Gam6 = m6*r_66rcm;
Gam7 = m7*r_77rcm;
Gamscreen = mscreen*r_screenrcm;

% Inertia Matrices (kg m^2)
J1    = [0.26607023 -0.00038981 -0.04801856; ...
        -0.00038981 0.28078511 0.00174486; ...
        -0.04801856 0.00174486 0.03266273];
    
J2    = [0.06807013 -0.01811791 0.00000238; ...
        -0.01811791 0.02398210 -0.00001133; ...
         0.00000238 -0.00001133 0.08122087];
     
J3    = [0.00458681 0.00169482 0.00000052; ...
         0.00169482 0.11052135 -0.00000006; ...
         0.00000052 -0.00000006 0.10995021];
     
J4    = [0.03973192 0.01142099 -0.00000198; ...
         0.01142099 0.01197658 -0.00003404; ...
        -0.00000198 -0.00003404 0.04758280];

J5    = [0.00284307 -0.00117636 -0.00008063; ...
        -0.00117636 0.08916094 -0.00000018; ...
        -0.00008063 -0.00000018 0.08895676];
    
J6    = [0.01427892 -0.00224277 0.00003962; ...
        -0.00224277 0.00311424 0.00019771; ...
         0.00003962 0.00019771 0.01544417]; 
     
J7    = [0.00082641 0.00022344 0.00000013; ...
         0.00022344 0.00103886 0.00000004; ...
         0.00000013 0.00000004 0.00134811]; 

Jscreen = [0.04954300 -0.00001992 -0.00086409; ...
          -0.00001992 0.04540562 -0.00012555; ...
          -0.00086409 -0.00012555 0.00621396];
      
%% Robot kinematics
% Inertial Frame Positions (meters)
InertialPositions = SawyerFK(gamma);
IIrE = InertialPositions(1);
ITE = InertialPositions(2);
IH = InertialPositions(3);

% Velocity and Acceleration
T_IITbdot = simplify(DCMDot(T_IITb, gamma, gammadot));
S = T_IITb.'*T_IITbdot;
wb = [S(3,2); S(1,3); S(2,1)];

disp('b');
toc
    
T_IIT1dot = simplify(DCMDot(T_IIT1, gamma, gammadot));
S = T_IIT1.'*T_IIT1dot;
w1 = [S(3,2); S(1,3); S(2,1)];
r_IIr1dot = simplify(jacobian(r_IIr1, gamma)*gammadot);
 
disp(1);
toc
    
T_11T2dot = simplify(DCMDot(T_11T2, gamma, gammadot));
r_11r2dot = [0;0;0];

T_IIT2dot = simplify(DCMDot(T_IIT2, gamma, gammadot));
S = T_IIT2.'*T_IIT2dot;
w2 = [S(3,2); S(1,3); S(2,1)];
r_IIr2dot = simplify(jacobian(r_IIr2, gamma)*gammadot);

disp(2);
toc

T_22T3dot = simplify(DCMDot(T_22T3, gamma, gammadot));
r_22r3dot = [0;0;0];

T_IIT3dot = simplify(DCMDot(T_IIT3, gamma, gammadot));
S = T_IIT3.'*T_IIT3dot;
w3 = [S(3,2); S(1,3); S(2,1)];
r_IIr3dot = simplify(jacobian(r_IIr3, gamma)*gammadot);

disp(3);
toc

T_33T4dot = simplify(DCMDot(T_33T4, gamma, gammadot));
r_33r4dot = [0;0;0];

T_IIT4dot = simplify(DCMDot(T_IIT4, gamma, gammadot));
S = T_IIT4.'*T_IIT4dot;
w4 = [S(3,2); S(1,3); S(2,1)];
r_IIr4dot = simplify(jacobian(r_IIr4, gamma)*gammadot);

disp(4);
toc

T_44T5dot = simplify(DCMDot(T_44T5, gamma, gammadot));
r_44r5dot = [0;0;0];

T_IIT5dot = simplify(DCMDot(T_IIT5, gamma, gammadot));
S = T_IIT5.'*T_IIT5dot;
w5 = [S(3,2); S(1,3); S(2,1)];
r_IIr5dot = simplify(jacobian(r_IIr5, gamma)*gammadot);

disp(5);
toc

T_55T6dot = simplify(DCMDot(T_55T6, gamma, gammadot));
r_55r6dot = [0;0;0];

T_IIT6dot = simplify(DCMDot(T_IIT6, gamma, gammadot));
S = T_IIT6.'*T_IIT6dot;
w6 = [S(3,2); S(1,3); S(2,1)];
r_IIr6dot = simplify(jacobian(r_IIr6, gamma)*gammadot);

disp(6);
toc

T_66T7dot = simplify(DCMDot(T_66T7, gamma, gammadot));
r_66r7dot = [0;0;0];

T_IIT7dot = simplify(DCMDot(T_IIT7, gamma, gammadot));
S = T_IIT7.'*T_IIT7dot;
w7 = [S(3,2); S(1,3); S(2,1)];
r_IIr7dot = simplify(jacobian(r_IIr7, gamma)*gammadot);

disp(7);
toc

T_77T8dot = simplify(DCMDot(T_77T8, gamma, gammadot));
r_77r8dot = [0;0;0];

T_IIT8dot = simplify(DCMDot(T_IIT8, gamma, gammadot));
S = T_IIT8.'*T_IIT8dot;
w8 = [S(3,2); S(1,3); S(2,1)];
r_IIr8dot = simplify(jacobian(r_IIr8, gamma)*gammadot); 

disp(8);
toc

 %% Kinetic Energy
 
% Link 1
K1 = simplify((1/2)*m1*(r_IIr1dot.'*r_IIr1dot) + r_IIr1dot.'* ...
    r_IIr1dot*cross(w1,Gam1) + (1/2)*w1.'*J1*w1);

% Link 2
K2 = simplify((1/2)*mL*(r_IIr2dot.'*r_IIr2dot) + r_IIr2dot.'* ...
    T_IIT2*cross(w2,GamL) + (1/2)*w2.'*JL*w2);

% Link 3
K3 = simplify((1/2)*mc*(r_IIr3dot.'*r_IIr3dot) + r_IIr3dot.'* ...
    T_IIT3*cross(w3,Gamc) + (1/2)*w3.'*Jc*w3);

% Link 4
K4 = simplify((1/2)*mL*(r_IIr4dot.'*r_IIr4dot) + r_IIr4dot.'* ...
    T_IIT4*cross(w4,GamL) + (1/2)*w4.'*JL*w4);

% Link 5
K5 = simplify((1/2)*mc*(r_IIr5dot.'*r_IIr5dot) + r_IIr5dot.'* ...
    T_IIT5*cross(w5,Gamc) + (1/2)*w5.'*Jc*w5);

% Link 6
K6 = simplify((1/2)*mL*(r_IIr6dot.'*r_IIr6dot) + r_IIr6dot.'* ...
    T_IIT6*cross(w6,GamL) + (1/2)*w6.'*JL*w6);

% Link 7
K7 = simplify((1/2)*mc*(r_IIr7dot.'*r_IIr7dot) + r_IIr7dot.'* ...
    T_IIT7*cross(w7,Gamc) + (1/2)*w7.'*Jc*w7);

% Link 8
K8 = simplify((1/2)*mt*(r_IIr8dot.'*r_IIr8dot) + r_IIr8dot.'* ...
    T_IIT8*cross(w8,Gamt) + (1/2)*w8.'*Jt*w8);

% Robot
K  = simplify(K1+K2+K3+K4+K5+K6+K7+K8);

%% Lagrangian of the Robot
L = K;

%% Equations of Motion (standard form)
% H matrix
H = simplify(jacobian(jacobian(K, gammadot).',gammadot));
% d vector
D = simplify(jacobian(jacobian(K, gammadot).',gamma) * gammadot ...
    - jacobian(K, gamma).');
%g vector
g = simplify(jacobian(U, gamma).');
    
% % EOMs
% Fstd = simplify(H*gammaddot + D + g);
% 
% %% Print results
% fprintf('\n\n\n\n\n\n\n\n\n\n\n')
% disp(H)
% disp(D)
% disp(g)