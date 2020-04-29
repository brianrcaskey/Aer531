

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

% create elevator doublet inout
elevator(1000:1050, 2) = 0;
elevator(1051:1100, 2) = -0.22;

%% run Simulink model for elevator doublet response
sim('AF_SimLQR_V2',300);
figure
yyaxis left
time = linspace(1,300,length(ans.Altitude));
plot(time,ans.Altitude)
ylabel('Altitude (ft)')

yyaxis right
plot(T1,(180/pi).*elevator(:,2))
ylabel('Elevator Deflection (deg)')
grid on
title('Longitudinal Response to Elevator Doublet')
legend on
legend ('Altitude (ft)', 'Elevator Deflection (deg)')
hold off

% run Simulink model for rudder input response
elevator = [T1, E1]; % reset elevator input matrix
rudder(1000:1020, 2) = 10*pi/179;
%rudder(1051:1100, 2) = -0.05;


% sim('AF_Sim2',300);

figure
yyaxis left
time = linspace(1,300,length(ans.HeadingAngle));
plot(time,(180/pi)*ans.HeadingAngle)
ylabel('Heading Angle (deg)')

yyaxis right
plot(time,(180/pi)*ans.BankAngle)
hold on
plot(T1,(180/pi)*rudder(:,2))
ylabel('Bank Angle & Rudder Deflection (deg)')
grid on
title('Heading and Bank Angle Response to Rudder Input')
legend on
legend ('Heading Angle (deg)', 'Bank Angle (deg)', 'Rudder Deflection (deg)')
hold off

%[V_dot, gamma_dot, alpha_dot, q_dot, p_dot, mu_dot, beta_dot, r_dot, chi_dot, n_dot, e_dot, h_dot] = EOM(u0);

