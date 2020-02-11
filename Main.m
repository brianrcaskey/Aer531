clear;clc;close all

% initila condition 
p0 = 0; %Roll Rate
q0 = 0; %Pitch Rate
r0 = 0; %Yaw Rate
V0 = 64.8280; %Velocity (ft/s)
chi0 = 0; %Heading angle (deg to rads)
gamma0 = 0; %flight path angle (deg to rads)
mu0 = 0; %Bank Angle (About Velocity Vector)
alpha0 = 0; %Angle of Attack
beta0 = 0; %angle of sideslip from deg to rads
xi0 = 0;  % North 
eta0 = 0;  % East
h0 = 100; %Altitude (feet)

% control
% Control
T0 = 0; % Thrust
de0 = 0; % Elevator Deflection (down is +) {deg}
drt0 = 0; % Rudder Deflection {deg}
da0 = 0; % Aileron Deflection (deg)

% time
time = 0:60;

% ODE45
X0 = [p0 q0 r0 V0 chi0 gamma0 mu0 alpha0 beta0 xi0 eta0 h0]';
control = [T0 de0 drt0 da0]';

opts = odeset('RelTol',1e-3,'AbsTol',1e-6);
[t,X] = ode45(@(t,X) EOM(t,X, control), time  ,  X0 , opts);

%% graph
% Post processing
r2d = 180/pi;
p = X(:, 1)*r2d; %Roll Rate
q = X(:, 2)*r2d; %Pitch Rate
r = X(:, 3)*r2d; %Yaw Rate
V = X(:, 4); %Velocity (ft/s)
chi = X(:, 5)*r2d; %Heading angle (deg to rads)
gamma = X(:, 6)*r2d; %flight path angle (deg to rads)
mu = X(:, 7)*r2d; %Bank Angle (About Velocity Vector)
alpha = X(:, 8)*r2d; %Angle of Attack
beta = X(:, 9)*r2d; %angle of sideslip from deg to rads
xi = X(:, 10);  % North 
eta = X(:, 11);  % East
h = X(:, 12); %Altitude (feet)

figure('position',[50 50 1200 600], 'color','w')
subplot(4,3,1)
plot(t, p, 'linewidth',1.5); ylabel('Roll rate (deg/sec)')
subplot(4,3,2)
plot(t, q, 'linewidth',1.5); ylabel('Pitch rate (deg/sec)')
subplot(4,3,3)
plot(t, r, 'linewidth',1.5); ylabel('Yaw rate (deg/sec)')

subplot(4,3,4)
plot(t, V, 'linewidth',1.5); ylabel('Velocity (ft/sec)')
subplot(4,3,5)
plot(t, chi, 'linewidth',1.5); ylabel('Heading angle (deg)')
subplot(4,3,6)
plot(t, gamma, 'linewidth',1.5); ylabel('Flight path angle (deg)')

subplot(4,3,7)
plot(t, mu, 'linewidth',1.5); ylabel('Bank Angle (deg)')
subplot(4,3,8)
plot(t, alpha, 'linewidth',1.5); ylabel('Angle of Attacke (deg)')
subplot(4,3,9)
plot(t, beta, 'linewidth',1.5); ylabel('Sideslipe angle (deg)')

subplot(4,3,10)
plot(t, xi, 'linewidth',1.5); ylabel('North (ft)')
subplot(4,3,11)
plot(t, eta, 'linewidth',1.5); ylabel('Eath (ft)')
subplot(4,3,12)
plot(t, h, 'linewidth',1.5); ylabel('Altitude (ft)')
