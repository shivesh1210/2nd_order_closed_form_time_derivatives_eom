clear all
close all
clc
%% Equation of Motion for 2R serial chain robot

%% Declaration of global and symbolic variables
global Param; % Structure with all geoemtric and dynamic robot parameters
global Chain; % Structure with all temporal data
global n; % DOF, number of joints
global g_vector; % gravity vector
global ee; % end effector configuration

syms q1 q2 real % Generalised position vector components
syms dq1 dq2 real % Generalised velocity vector components
syms ddq1 ddq2 real % Generalised acceleration vector components
syms dddq1 dddq2 real % Generalised jerk vector components
syms ddddq1 ddddq2 real % Generalised jounce vector components
syms m1 m2 I1 I2 real % mass and Moment of Inertia
syms cg1 cg2 g real % center of gravity of links
syms L1 L2 real % Link lengths
syms pi % symbolic treatment of pi (saves you from numerical issues)

g_vector = [0,g,0];

%% Joint screw coordinates in spatial representation

e1 = [0;0;1];
y1 = [0;0;0];
Param(1).Y = [e1;cross(y1,e1)];
e2 = [0;0;1];
y2 = [L1;0;0];
Param(2).Y = [e2;cross(y2,e2)];

%% Reference configurations of bodies (i.e. of body-fixed reference frames)

r1 = [0;0;0];
r2 = [L1;0;0];

Param(1).A = [eye(3),r1;[0,0,0],[1]];
Param(2).A = [eye(3),r2;[0,0,0],[1]];

% Param(1).B = [eye(3),r1;[0,0,0],[1]];
% Param(2).B = [eye(3),r2;[0,0,0],[1]];

%% End-effector configuration wrt last link body fixed frame in the chain
re = [L2;0;0];
ee = [eye(3),re;[0,0,0],[1]];

%% Joint screw coordinates in body-fixed representation computed from screw coordinates in IFR
Param(1).X = SE3AdjInvMatrix(Param(1).A)*Param(1).Y;
Param(2).X = SE3AdjInvMatrix(Param(2).A)*Param(2).Y;

%% Joint screw coordinates in body-fixed representation
% Param(1).X = [0, 0, 1, 0., 0., 0]';
% Param(2).X = [0, 0, 1, 0., 0., 0]';

%% Mass-Inertia paramaters
cg1 = [L1 0 0].';
cg2 = [L2 0 0].';
I1 = m1*L1*L1;
I2 = m2*L2*L2;
Param(1).Mb = MassMatrixMixedData(m1 , I1*eye(3) , cg1);
Param(2).Mb = MassMatrixMixedData(m2 , I2*eye(3) , cg2);

%% 
for i=1:n 
    Chain(i).V = zeros(n,1);
    Chain(i).Vd = zeros(n,1);
    Chain(i).f = zeros(4,4);
    Chain(i).C = zeros(4,4);
    Chain(i).Crel = zeros(4,4); % C_i,i-1
    Chain(i).AdCrel = zeros(6,6);
    Chain(i).adX = zeros(6,6);
    Chain(i).W = zeros(6,1);   % interbody wrench
    Chain(i).Q = 0;
end
%% Declaring generalised vectors
 q = [q1 q2];
 qd = [dq1 dq2];
 q2d = [ddq1 ddq2];
 q3d = [dddq1 dddq2];
 q4d = [ddddq1 ddddq2];
 n = length(q);
 WEE = zeros(6,1); 

%% Final EOM
[Q,Qd,Q2d] = ClosedFormInvDyn_BodyFixed(q',qd',q2d',q3d',q4d');

%% Symbolic verification 
% First order derivative
tau1 = Q(1);
tau1dot = jacobian(tau1,q)*qd' + jacobian(tau1,qd)*q2d' + jacobian(tau1, q2d)*q3d';
tau1dot = simplify(tau1dot);
if isequal(tau1dot, Qd(1))
    disp('1st Order Derivatives from the closed form forumulation matches the symbolic differentiation of Tau1');
else
    disp('Attention: 1st Order Derivatives from the closed form forumulation DO NOT match the symbolic differentiation of Tau1');
end

tau2 = Q(2);
tau2dot = jacobian(tau2,q)*qd' + jacobian(tau2,qd)*q2d' + jacobian(tau2, q2d)*q3d';
tau2dot = simplify(tau2dot);
if isequal(tau2dot, Qd(2))
    disp('1st Order Derivatives from the closed form forumulation matches the symbolic differentiation of Tau2');
else
    disp('Attention: 1st Order Derivatives from the closed form forumulation DO NOT match the symbolic differentiation of Tau2');
end

% Second order derivative
tau1d = Qd(1);
tau1ddot = jacobian(tau1d,q)*qd' + jacobian(tau1d,qd)*q2d' + jacobian(tau1d, q2d)*q3d' + jacobian(tau1d, q3d)*q4d';
tau1ddot = simplify(tau1ddot);
if isequal(tau1ddot, Q2d(1))
    disp('2nd Order Derivatives from the closed form forumulation matches the symbolic differentiation of Tau1dot');
else
    disp('Attention: 2nd Order Derivatives from the closed form forumulation DO NOT match the symbolic differentiation of Tau1dot');
end

tau2d = Qd(2);
tau2ddot = jacobian(tau2d,q)*qd' + jacobian(tau2d,qd)*q2d' + jacobian(tau2d, q2d)*q3d' + jacobian(tau2d, q3d)*q4d';
tau2ddot = simplify(tau2ddot);
if isequal(tau2ddot, Q2d(2))
    disp('2nd Order Derivatives from the closed form forumulation matches the symbolic differentiation of Tau2dot');
else
    disp('Attention: 2nd Order Derivatives from the closed form forumulation DO NOT match the symbolic differentiation of Tau2dot');
end

