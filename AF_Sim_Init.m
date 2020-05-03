%% AerE 531 Final Project Initialization Script
% AF_Sim_Init.m defines the trim and initial conditions for the 

clear,clc, 
close all

%% Trim Conditions
u0 = [ 1.29  -0.1174  0  0];%[dt de dr da]
% u0 = [1.29, 0, 0,  0];

% setup initial conditions for state
IC = [64.828,     0,  -0.0153, 0,  0,  0,     0, 0,   0,     0,    0, 1119];

uin = [0, 0, 0, 0];

% Time array
T1 = 0.1*[0:899]';

% Trim conditions arrays
SS = zeros(900,1);
thr = 1.29*ones(900,1);
throttle = [T1, thr];
aileron = [T1, SS];
rudder = [T1, SS];
E1 = -0.1174*ones(900,1);
elevator = [T1, E1];

%% State Space

% Initial conditions for state
V0 = IC(1);
alpha0 = IC(3);
q0 = IC(4);
theta0 = 0;

uin = [0, 0, 0, 0];


% create elevator doublet input
% elevator(1000:1050, 2) = 0;
% elevator(1051:1100, 2) = -0.22;

roll = zeros(900,1);
roll(100:900) = 5;
pitch = zeros(900,1);
pitch(100:900) = 2;

roll_cmd = [T1, roll];%degrees
pitch_cmd = [T1, pitch];

%% LQR Longitudonal
x0 = [ theta0 V0  alpha0  q0]';

A_long = [...
   0         0         0    1.0000      0 % theta
 -32.2000   -48   13.1983         3     0 % V
  0.0001   -0.0253   -0.2370    1.0000   0 % alpha
    0   -0.0549   -5.5837   -3.7947     0 % q
   -1        0         0         0      0 ]; % Int(theta_cmd - theta)dt

B_long = [...
     0
     0
     0.9
   2.9035
     0 ];   

%% LQR setup
Q = diag([1 0.1 0.1 0.1 1000]);

R = 1e-1;


Klqr = lqr(A_long, B_long, Q, R);
%% LQR Lateral
max_dr = 30;
max_da = 10;

A_lat = [ 0        0         0    1.0000         0  0
         0         0         0         0    1.0000  0
    0.4967         0   -0.0002   -0.0130   -0.9593  0
         0         0   -0.0076    0.0040    0.0030  0
         0         0    0.0010   -0.6571   -0.0002  0
        -1         0         0         0         0  0];
     
     
B_lat = [0         0
         0         0
         0    0.3726
    1.0256    4.6310
   -0.0549  -28.1998
         0         0];

wc = 0.025;
Q = diag([0.5,0.0001,0.5,0.5,1,20]);
R = diag([1/(max_da^2), 1/(max_dr^2)]);
k_lqr = lqr(A_lat, B_lat, Q, R);  

%% Simulation
tsim = 90;
out = sim('ControllersComparison.slx',tsim);
t = out.tout;


%% Parsing
% States
PID_Long_States = out.yout{1}.Values.Data;
PiD_Lat_States = out.yout{5}.Values.Data;
LQR_Long_States = out.yout{9}.Values.Data;
LQR_Lat_States = out.yout{13}.Values.Data;

% Commands
% pitch_cmd = out.yout{17}.Values.Data;
% roll_cmd = out.yout{18}.Values.Data;

% Responses
pitch_PID = out.yout{2}.Values.Data;
pitch_LQR = out.yout{10}.Values.Data;
roll_PID = out.yout{6}.Values.Data;
roll_LQR = out.yout{14}.Values.Data;

% Controller
% Time History Input
u_PIDlong = out.yout{3}.Values.Data;
u_PIDlat = out.yout{7}.Values.Data;
u_LQRlong = out.yout{11}.Values.Data;
u_LQRdr = out.yout{16}.Values.Data(:,1);
u_LQRda = out.yout{15}.Values.Data(:,1);

% Time History Rate
du_PIDlong = out.yout{4}.Values;
du_PIDlat = out.yout{8}.Values;
du_LQRlong = out.yout{12}.Values.Data;
du_LQRdr = out.yout{16}.Values.Data(:,2);
du_LQRda = out.yout{15}.Values.Data(:,2);


% Open Loop Response
Long_response = out.yout{17}.Values.Data;
Lat_response = out.yout{19}.Values.Data;
oploop_pitch = out.yout{18}.Values.Data;
oploop_roll = out.yout{20}.Values.Data;

%% Comparisons
control_PIDlong = stepinfo(pitch_PID,t)
control_PIDlat = stepinfo(roll_PID,t)
control_PIDlong = stepinfo(pitch_LQR,t)
control_PIDlong = stepinfo(pitch_PID,t)

%% Controller Results
figure('position',[50 50 900 600])
subplot(2,1,1); hold on ; grid on
title('Longitudinal Mode Controller Response')
plot(T1,pitch_cmd(:,2),':K','LineWidth',2)
% plot(t,oploop_pitch,'LineWidth',2)
plot(t,pitch_PID,'b','LineWidth',2)
plot(t,pitch_LQR,'r','LineWidth',2)
ylabel('Pitch angle (deg)'); xlabel('Time (sec)'); xlim([0 60])
legend('Command', 'PID','LQR','location','east')
set(gca,'fontsize',12)

subplot(2,1,2);hold on ;grid on
title('Lateral Mode Controller Response')
plot(T1,roll_cmd(:,2),':K','LineWidth',2)
% plot(t,oploop_roll,'LineWidth',2)
plot(t,roll_PID,'b','LineWidth',2)
plot(t,roll_LQR,'r','LineWidth',2)
ylabel('Roll angle (deg)'); xlabel('Time (sec)'); xlim([0 60])
legend('Command', 'PID','LQR','location','east')
set(gca,'fontsize',12)

