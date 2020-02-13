
clear all,clc

u0 = [1.29, 0, 0,  0];
IC = [64.828,     0,  -0.0153, 0,  0,  0,     0, 0,   0,     0,    0, 1119];

uin = [0, 0, 0, 0];
throttle = 0;

T1 = 0.1*[0:2999]';
SS = zeros(3000,1);

thr = zeros(3000,1);

throttle = [T1, thr];

E1 = -0.1174*ones(3000,1);
elevator = [T1, E1];

elevator(1000:1050, 2) = 0;
elevator(1051:1100, 2) = -0.22;

aileron = [T1, SS];

rudder = [T1, SS];


%[V_dot, gamma_dot, alpha_dot, q_dot, p_dot, mu_dot, beta_dot, r_dot, chi_dot, n_dot, e_dot, h_dot] = EOM(u0);