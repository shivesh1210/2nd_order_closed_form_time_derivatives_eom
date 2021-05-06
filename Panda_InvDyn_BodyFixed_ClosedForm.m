% author: Shivesh Kumar
% last modified: April 27, 2021

clear all
close all
clc

global Param; % Structure with all geoemtric and dynamic robot parameters
global Chain; % Structure with all temporal data

global n; % DOF, number of joints
n = 7;

global g; % gravity vector
g = [0; 0; 9.80665];

%% Joint screw coordinates in spatial representation
% Geometric parameters of the robot
d1 = 0.333;
d3 = 0.316;
a4 = 0.0825;
d5 = 0.384;
a7 = 0.088;
Param(1).Y = ScrewCoordinatesIFR([0,0,d1],[0,0,1]);
Param(2).Y = ScrewCoordinatesIFR([0,0,d1],[0,1,0]);
Param(3).Y = ScrewCoordinatesIFR([0,0,d1+d3],[0,0,1]);
Param(4).Y = ScrewCoordinatesIFR([a4,0,d1+d3],[0,-1,0]);
Param(5).Y = ScrewCoordinatesIFR([0,0,d1+d3+d5],[0,0,1]);
Param(6).Y = ScrewCoordinatesIFR([0,0,d1+d3+d5],[0,-1,0]);
Param(7).Y = ScrewCoordinatesIFR([a7,0,d1+d3+d5],[0,0,-1]);

%% Reference configurations of bodies (i.e. of body-fixed reference frames) in IFR
Param(1).A = [eye(3),[0,0,d1]';[0,0,0],[1]];
Param(2).A = [SO3Exp([1,0,0],-pi/2),[0,0,d1]';[0,0,0],[1]];
Param(3).A = [eye(3),[0,0,d1+d3]';[0,0,0],[1]];
Param(4).A = [SO3Exp([1,0,0],pi/2),[a4,0,d1+d3]';[0,0,0],[1]];
Param(5).A = [eye(3),[0,0,d1+d3+d5]';[0,0,0],[1]];
Param(6).A = [SO3Exp([1,0,0],pi/2),[0.0,0,d1+d3+d5]';[0,0,0],[1]];
Param(7).A = [SO3Exp([1,0,0],pi),[a7,0,d1+d3+d5]';[0,0,0],[1]];

%% Reference configurations of bodies (i.e. of body-fixed reference frames) w.r.t previous body
Param(1).B = Param(1).A;
Param(2).B = SE3Inv(Param(1).A)*Param(2).A;
Param(3).B = SE3Inv(Param(2).A)*Param(3).A;
Param(4).B = SE3Inv(Param(3).A)*Param(4).A;
Param(5).B = SE3Inv(Param(4).A)*Param(5).A;
Param(6).B = SE3Inv(Param(5).A)*Param(6).A;
Param(7).B = SE3Inv(Param(6).A)*Param(7).A;

%% Joint screw coordinates in body-fixed representation
Param(1).X = SE3AdjInvMatrix(Param(1).A)*Param(1).Y;
Param(2).X = SE3AdjInvMatrix(Param(2).A)*Param(2).Y;
Param(3).X = SE3AdjInvMatrix(Param(3).A)*Param(3).Y;
Param(4).X = SE3AdjInvMatrix(Param(4).A)*Param(4).Y;
Param(5).X = SE3AdjInvMatrix(Param(5).A)*Param(5).Y;
Param(6).X = SE3AdjInvMatrix(Param(6).A)*Param(6).Y;
Param(7).X = SE3AdjInvMatrix(Param(7).A)*Param(7).Y;

%% Intertia paramater as reported in [C. Gaz, 2019]
Param(1).Mb = MassMatrixMixedData(4.970684, ...
    [0.70337,-1.39e-04,6.772e-03;
    -1.39e-04,0.70661,1.9169e-02;
    6.772e-03,1.9169e-02,9.117e-03], ...
    [3.875e-03, 2.081e-03, -0.1750]);
Param(2).Mb = MassMatrixMixedData(0.646926, ...
    [7.962e-03, -3.925e-03, 1.0254e-02;
     -3.925e-03, 2.811e-02, 7.04e-04;
     1.0254e-02, 7.04e-04, 2.5995e-02], ...
    [-3.141e-03, -2.872e-02, 3.495e-03]);
Param(3).Mb = MassMatrixMixedData(3.228604, ...
    [3.7242e-02, -4.761e-03, -1.1396e-02;
     -4.761e-03, 3.6155e-02, -1.2805e-02;
     -1.1396e-02, -1.2805e-02, 1.083e-02], ...
    [2.7518e-02, 3.9252e-02, -6.6502e-02]);
Param(4).Mb = MassMatrixMixedData(3.587895, ...
    [2.5853e-02, 7.796e-03, -1.332e-03;
     7.796e-03, 1.9552e-02, 8.641e-03;
     -1.332e-03, 8.641e-03, 2.8323e-02], ...
    [-5.317e-02, 0.104419, 2.7454e-02]);
Param(5).Mb = MassMatrixMixedData(1.225946, ...
    [3.5549e-02, -2.117e-03, -4.037e-03;
     -2.117e-03, 2.9474e-02, 2.29e-04;
     -4.037e-03, 2.29e-04, 8.627e-03], ...
    [-1.1953e-02, 4.1065e-02, -3.8437e-02]);
Param(6).Mb = MassMatrixMixedData(1.666555, ...
    [1.964e-03, 1.09e-04, -1.158e-03;
     1.09e-04, 4.354e-03, 3.41e-04;
     -1.158e-03, 3.41e-04, 5.433e-03], ...
    [6.0149e-02, -1.4117e-02, -1.0517e-02]);
Param(7).Mb = MassMatrixMixedData(0.735522, ...
    [1.2516e-02, -4.28e-04, -1.196e-03;
     -4.28e-04, 1.0027e-02, -7.41e-04;
     -1.196e-03, 7.41e-04, 4.815e-03], ...
    [1.0517e-02, -4.252e-03, 6.1597e-02]);


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
for k=1:6
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
% [Q_closedform(i,:), Qd_closedform(i,:), Q2d_closedform(i,:)] = ClosedFormInvDyn_BodyFixed(q(i,:)',qd(i,:)',q2d(i,:)',q3d(i,:)',q4d(i,:)');
end
end

time=0:dt:T;

figure;
hold on
plot(time, q);
xlabel('Time (s)','Interpreter','latex');
ylabel('Joint Position (rad)','Interpreter','latex');
legend('$q_1$', '$q_2$','$q_3$', '$q_4$', '$q_5$','$q_6$','$q_7$','Interpreter','latex');
title('Input Joint Trajectories $\mathbf{q}(t)$','Interpreter','latex');

figure;
hold on
plot(time, Q_closedform);
xlabel('Time (s)','Interpreter','latex');
ylabel('Generalized Forces (Nm)','Interpreter','latex');
legend('$Q_1$', '$Q_2$','$Q_3$', '$Q_4$', '$Q_5$','$Q_6$','$Q_7$','Interpreter','latex');
title('Closed Form Generalized Forces $\mathbf{Q}$','Interpreter','latex');

Qdc_num = diff(Q_closedform)/dt;

figure;
hold on
plot(time, Qd_closedform);
plot(time(1:end-1), Qdc_num,'--');
xlabel('Time (s)','Interpreter','latex');
ylabel('Generalized Forces (Nm/s)','Interpreter','latex');
legend('$\dot{Q}_1$', '$\dot{Q}_2$','$\dot{Q}_3$', '$\dot{Q}_4$', '$\dot{Q}_5$','$\dot{Q}_6$','$\dot{Q}_7$','$\dot{Q}_1^{num}$', '$\dot{Q}_2^{num}$', '$\dot{Q}_3^{num}$', '$\dot{Q}_4^{num}$', '$\dot{Q}_5^{num}$', '$\dot{Q}_6^{num}$','$\dot{Q}_7^{num}$', 'Interpreter','latex');
title('Closed Form vs Numerical: 1st order Generalized Forces $\mathbf{\dot{Q}}$','Interpreter','latex');

Qddc_num = diff(Qd_closedform)/dt;

figure;
hold on
plot(time, Q2d_closedform);
plot(time(1:end-1), Qddc_num,'--');
xlabel('Time (s)','Interpreter','latex');
ylabel('Generalized Forces $(Nm/s^2)$','Interpreter','latex');
legend('$\ddot{Q}_1$', '$\ddot{Q}_2$','$\ddot{Q}_3$', '$\ddot{Q}_4$', '$\ddot{Q}_5$','$\ddot{Q}_6$','$\ddot{Q}_7$','$\ddot{Q}_1^{num}$', '$\ddot{Q}_2^{num}$', '$\ddot{Q}_3^{num}$', '$\ddot{Q}_4^{num}$', '$\ddot{Q}_5^{num}$', '$\ddot{Q}_6^{num}$','$\ddot{Q}_7^{num}$', 'Interpreter','latex');
title('Closed Form vs Numerical: 2nd order Generalized Forces $\mathbf{\ddot{Q}}$','Interpreter','latex');

