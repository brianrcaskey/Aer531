
clear all,clc, close all

% Trim Conditions
u0 = [1.29, 0, 0,  0];

% setup initial conditions for state
IC = [64.828,     0,  -0.0153, 0,  0,  0,     0, 0,   0,     0,    0, 1119];

uin = [0, 0, 0, 0];
throttle = 0;


% setup time matrix
T1 = 0.1*[0:2999]';

% setup trim conditions
SS = zeros(3000,1);
thr = zeros(3000,1);
throttle = [T1, thr];
aileron = [T1, SS];
rudder = [T1, SS];
E1 = -0.1174*ones(3000,1);
elevator = [T1, E1];


%% Create LQR Controller
A_lat = [0         0         0    1.0000         0      0
         0         0         0         0    1.0000      0
    0.4967         0   -0.0002   -0.0130   -0.9593      0
         0         0   -0.0076    0.0035    0.0030      0
         0         0    0.0010   -0.6571   -0.0002      0
        -1         0         0         0         0      0];
     
     
B_lat = [0         0
         0         0
         0    0.3726
    1.0256    4.6083
   -0.0541  -28.1956
         0         0];



Q = diag([1,0.001,1,3,1,10]);



max_da = 10;
max_dr = 30;
R = diag([1/(max_da^2), 1/(max_dr^2)]);



k_lqr = lqr(A_lat, B_lat, Q, R)

