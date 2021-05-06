% author: Shivesh Kumar
% last modified: April 27, 2021

clear all
close all
clc

global Param; % Structure with all geoemtric and dynamic robot parameters
global Chain; % Structure with all temporal data

global n; % DOF, number of joints
n = 7;

global g_vector; % gravity vector
g_vector = [0; 0; 9.80665];

% Geometric parameters of the robot [Y.R. Sturz, 2017]
r3 = 0.42;
r5 = 0.4;

%% Reference configurations of bodies (i.e. of body-fixed reference frames) w.r.t their previous bodies
Param(1).B = [eye(3),[0,0,0.]';[0,0,0],[1]];
Param(2).B = [SO3Exp([1,0,0],pi/2),[0,0,0]';[0,0,0],[1]];
Param(3).B = [SO3Exp([1,0,0],-pi/2),[0,0,r3]';[0,0,0],[1]];
Param(4).B = [SO3Exp([1,0,0],-pi/2),[0,0,0]';[0,0,0],[1]];
Param(5).B = [SO3Exp([1,0,0],pi/2),[0,0,r5]';[0,0,0],[1]];
Param(6).B = [SO3Exp([1,0,0],pi/2),[0,0,0]';[0,0,0],[1]];
Param(7).B = [SO3Exp([1,0,0],-pi/2),[0,0,0]';[0,0,0],[1]];

%% Reference configurations of bodies (i.e. of body-fixed reference frames) w.r.t previous body
% Param(1).A = Param(1).B;
% Param(2).A = Param(1).B*Param(2).B;
% Param(3).A = Param(2).B*Param(3).B;
% Param(4).A = Param(3).B*Param(4).B;
% Param(5).A = Param(4).B*Param(5).B;
% Param(6).A = Param(5).B*Param(6).B;
% Param(7).A = Param(6).B*Param(7).B;

%% Joint screw coordinates in body-fixed representation
Param(1).X = [0, 0, 1, 0., 0., 0]';
Param(2).X = [0, 0, 1, 0., 0., 0]';
Param(3).X = [0, 0, 1, 0., 0., 0]';
Param(4).X = [0, 0, 1, 0., 0., 0]';
Param(5).X = [0, 0, 1, 0., 0., 0]';
Param(6).X = [0, 0, 1, 0., 0., 0]';
Param(7).X = [0, 0, 1, 0., 0., 0]';

%% Joint screw coordinates in body-fixed representation
% Param(1).Y = SE3AdjMatrix(Param(1).A)*Param(1).X;
% Param(2).Y = SE3AdjMatrix(Param(2).A)*Param(2).X;
% Param(3).Y = SE3AdjMatrix(Param(3).A)*Param(3).X;
% Param(4).Y = SE3AdjMatrix(Param(4).A)*Param(4).X;
% Param(5).Y = SE3AdjMatrix(Param(5).A)*Param(5).X;
% Param(6).Y = SE3AdjMatrix(Param(6).A)*Param(6).X;
% Param(7).Y = SE3AdjMatrix(Param(7).A)*Param(7).X;

%% Mass-Inertia paramater as reported in [Y.R. Sturz, 2017]
m1   = 3.94781;
c1x  = -0.00351;
c1y  = 0.00160;
c1z  = -0.03139;
I1xx = 0.00455;
I1xy = 0.00000;
I1xz = -0.00000;
I1yy = 0.00454;
I1yz = 0.00001;
I1zz = 0.00029;

m2   = 4.50275;
c2x  = -0.00767;
c2y  = 0.16669;
c2z  = -0.00355;
I2xx = 0.00032;
I2xy = 0.00000;
I2xz = 0.00000;
I2yy = 0.00010;
I2yz = -0.00000;
I2zz = 0.00042;

m3   = 2.45520;
c3x  = -0.00225;
c3y  = -0.03492;
c3z  = -0.02652;
I3xx = 0.00223;
I3xy = -0.00005;
I3xz = 0.00007;
I3yy = 0.00219;
I3yz = 0.00007;
I3zz = 0.00073;
 
m4   = 2.61155;
c4x  = 0.00020;
c4y  = -0.05268;
c4z  = 0.03818;
I4xx = 0.03844;
I4xy = 0.00088;
I4xz = -0.00112;
I4yy = 0.01144;
I4yz = -0.00111;
I4zz = 0.04988;

m5   = 3.41000;
c5x  = 0.00005;
c5y  = -0.00237;
c5z  = -0.21134;
I5xx = 0.00277;
I5xy = -0.00001;
I5xz = 0.00001;
I5yy = 0.00284;
I5yz = -0.00000;
I5zz = 0.00012;

m6   = 3.38795;
c6x  = 0.00049;
c6y  = 0.02019;
c6z  = -0.02750;
I6xx = 0.00050;
I6xy = -0.00005;
I6xz = -0.00003;
I6yy = 0.00281;
I6yz = -0.00004;
I6zz = 0.00232;
 
m7   = 0.35432;
c7x  = -0.03466;
c7y  = -0.02324;
c7z  = 0.07138;
I7xx = 0.00795;
I7xy = 0.00022;
I7xz = -0.00029;
I7yy = 0.01089;
I7yz = -0.00029;
I7zz = 0.00294;

Param(1).Mb = MassMatrixMixedData(m1, ...
    InertiaMatrix(I1xx, I1xy, I1xz, I1yy, I1yz, I1zz), ...
    [c1x, c1y, c1z]);
Param(2).Mb = MassMatrixMixedData(m2, ...
    InertiaMatrix(I2xx, I2xy, I2xz, I2yy, I2yz, I2zz), ...
    [c2x, c2y, c2z]);
Param(3).Mb = MassMatrixMixedData(m3, ...
    InertiaMatrix(I3xx, I3xy, I3xz, I3yy, I3yz, I3zz), ...
    [c3x, c3y, c3z]);
Param(4).Mb = MassMatrixMixedData(m4, ...
    InertiaMatrix(I4xx, I4xy, I4xz, I4yy, I4yz, I4zz), ...
    [c4x, c4y, c4z]);
Param(5).Mb = MassMatrixMixedData(m5, ...
    InertiaMatrix(I5xx, I5xy, I5xz, I5yy, I5yz, I5zz), ...
    [c5x, c5y, c5z]);
Param(6).Mb = MassMatrixMixedData(m6, ...
    InertiaMatrix(I6xx, I6xy, I6xz, I6yy, I6yz, I6zz), ...
    [c6x, c6y, c6z]);
Param(7).Mb = MassMatrixMixedData(m7, ...
    InertiaMatrix(I7xx, I7xy, I7xz, I7yy, I7yz, I7zz), ...
    [c7x, c7y, c7z]);

for i=1:n 
    Chain(i).V = zeros(n,1);
    Chain(i).Vd = zeros(n,1);
    Chain(i).V2d = zeros(n,1);
    Chain(i).V3d = zeros(n,1);
    Chain(i).f = zeros(4,4);
    Chain(i).C = zeros(4,4);
    Chain(i).Crel = zeros(4,4); % C_i,i-1
    Chain(i).AdCrel = zeros(6,6);
    Chain(i).adX = zeros(6,6);
    Chain(i).W = zeros(6,1);   % interbody wrench
    Chain(i).Wd = zeros(6,1);
    Chain(i).W2d = zeros(6,1);
    Chain(i).Q = 0;
    Chain(i).Qd = 0;
    Chain(i).Q2d = 0;
 end;

N=10000;  % number of samples
T=5;    % simulation time
dt=T/N; % time step size

q = zeros(N,n);
qd = zeros(N,n);
q2d = zeros(N,n);
q3d = zeros(N,n);
q4d = zeros(N,n);
Q = zeros(N,n);

%% 2nd-order inverse dynamics run
% Test trajectory according to equation (31) and table I in [C. Gaz et al., RAL, Vol. 4, No. 4, 2019]
tic
for i=1:N+1
    t = (i-1)*dt;
    q(i,:) = [-1.2943753211777664*cos(1.7073873117335832*t), 0.7175341454355011* cos(3.079992797637052*t), -0.5691380764966176* cos(2.1084514453622774*t),   0.5848944158627155*cos(3.5903916041026207*t), 1.6216297151633214* cos(1.4183262544423447*t), -0.9187855709752027*cos(2.285625793808507*t), 0.4217605991935227*cos(5.927533308659986*t)];
    qd(i,:) =[2.21*sin(1.7073873117335832*t),-2.21*sin(3.079992797637052*t),1.2*sin(2.1084514453622774*t),-2.1*sin(3.5903916041026207*t),-2.3*sin(1.4183262544423447*t),2.1*sin(2.285625793808507*t),-2.5*sin(5.927533308659986*t)];
    q2d(i,:) = [3.7733259589312187*cos(1.7073873117335832*t), -6.8067840827778845* cos(3.079992797637052*t), 2.5301417344347326* cos(2.1084514453622774*t),   -7.5398223686155035*cos(3.5903916041026207*t), -3.2621503852173928* cos(1.4183262544423447*t), 4.799814166997865*cos(2.285625793808507*t),   -14.818833271649964*cos(5.927533308659986*t)];
    q3d(i,:) = [-6.442528865314118*sin(1.7073873117335832*t), 20.96484595002641* sin(3.079992797637052*t), -5.334680996940331* sin(2.1084514453622774*t), 27.070914928702237* sin(3.5903916041026207*t),   4.626793537293037*sin(1.4183262544423447*t), -10.970579065577812*sin(2.285625793808507*t), 87.83912781318399*sin(5.927533308659986*t)];
    q4d(i,:) = [-10.999892040114684*cos(1.7073873117335832*t), 64.57157452965167* cos(3.079992797637052*t), -11.247915858545515* cos(2.1084514453622774*t), 97.19518567538881* cos(3.5903916041026207*t),   6.562302747826879*cos(1.4183262544423447*t), -25.074638485300277*cos(2.285625793808507*t), 520.6693559162899*cos(5.927533308659986*t)];
    WEE = 10*sin(pi*t)*ones(6,1);
    WDEE = 10*pi*cos(pi*t)*ones(6,1);
    W2DEE = -10*pi*pi*sin(pi*t)*ones(6,1);
    [Q_closedform(i,:), Qd_closedform(i,:), Q2d_closedform(i,:)] = ClosedFormInvDyn_BodyFixed(q(i,:)',qd(i,:)',q2d(i,:)',q3d(i,:)',q4d(i,:)', WEE, WDEE, W2DEE);
%     [Q(i,:), Qd(i,:), Q2d(i,:)] = InvDyn_BodyFixed(q(i,:)',qd(i,:)',q2d(i,:)',q3d(i,:)',q4d(i,:)');    
%     [Q_closedform(i,:), Qd_closedform(i,:), Q2d_closedform(i,:)] = ClosedFormInvDyn_BodyFixed(q(i,:)',qd(i,:)',q2d(i,:)',q3d(i,:)',q4d(i,:)');
end
toc

% Build the time vector
time=0:dt:T;

figure;
hold on
plot(time, q);
xlabel('Time, $t$ (s)','Interpreter','latex');
ylabel('Joint Position, $\mathbf{q}(t)$ (rad)','Interpreter','latex');
legend('$q_1$', '$q_2$','$q_3$', '$q_4$', '$q_5$','$q_6$','$q_7$','Interpreter','latex');
% title('Input Joint Trajectories $\mathbf{q}(t)$','Interpreter','latex');

figure;
hold on
plot(time, Q_closedform);
xlabel('Time, $t$ (s)','Interpreter','latex');
ylabel('Generalized Forces, $\mathbf{Q}(t)$ (Nm)','Interpreter','latex');
legend('$Q_1$', '$Q_2$','$Q_3$', '$Q_4$', '$Q_5$','$Q_6$','$Q_7$','Interpreter','latex');
% title('Closed Form Generalized Forces $\mathbf{Q}$','Interpreter','latex');

Qdc_num = diff(Q_closedform)/dt;

figure;
hold on
plot(time, Qd_closedform);
plot(time(1:end-1), Qdc_num,'--');
xlabel('Time, $t$ (s)','Interpreter','latex');
ylabel('1st Order Generalized Forces, $\mathbf{\dot{Q}(t)}$ (Nm/s)','Interpreter','latex');
legend('$\dot{Q}_1$', '$\dot{Q}_2$','$\dot{Q}_3$', '$\dot{Q}_4$', '$\dot{Q}_5$','$\dot{Q}_6$','$\dot{Q}_7$','$\dot{Q}_1^{num}$', '$\dot{Q}_2^{num}$', '$\dot{Q}_3^{num}$', '$\dot{Q}_4^{num}$', '$\dot{Q}_5^{num}$', '$\dot{Q}_6^{num}$','$\dot{Q}_7^{num}$', 'Interpreter','latex');
% title('Closed Form vs Numerical: 1st order Generalized Forces $\mathbf{\dot{Q}}$','Interpreter','latex');

Qddc_num = diff(Qd_closedform)/dt;

figure;
hold on
plot(time, Q2d_closedform);
plot(time(1:end-1), Qddc_num,'--');
xlabel('Time, $t$ (s)','Interpreter','latex');
ylabel('2nd Order Generalized Forces, $\mathbf{\ddot{Q}(t)}$ $(Nm/s^2)$','Interpreter','latex');
legend('$\ddot{Q}_1$', '$\ddot{Q}_2$','$\ddot{Q}_3$', '$\ddot{Q}_4$', '$\ddot{Q}_5$','$\ddot{Q}_6$','$\ddot{Q}_7$','$\ddot{Q}_1^{num}$', '$\ddot{Q}_2^{num}$', '$\ddot{Q}_3^{num}$', '$\ddot{Q}_4^{num}$', '$\ddot{Q}_5^{num}$', '$\ddot{Q}_6^{num}$','$\ddot{Q}_7^{num}$', 'Interpreter','latex');
% title('Closed Form vs Numerical: 2nd order Generalized Forces $\mathbf{\ddot{Q}}$','Interpreter','latex');

