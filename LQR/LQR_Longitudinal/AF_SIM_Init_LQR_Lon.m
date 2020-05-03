clear;clc;close all
% AERE 531 final project
% Youngro Lee / youngro@iastate.edu

% State Vector
% -----------------------------
% V       Velocity {ft/s}
% gamma   Flight path angle {rad}
% alpha   Angle of attack {rad}
% q       Pitch Rate {rad/s}
% p       Roll Rate {rad/s}
% mu      Bank Angle (About Velocity Vector) {rad}
% beta    Sideslip Angle {rad}
% r       Yaw Rate {rad/s}
% chi     Heading angle {rads}
% north   North Position {ft}
% east    East Position {ft}
% h       Altitude {ft}
% -----------------------------

% Control Vector
% -----------------------------
% Throttle (ft slug/sec^2)
% Elevator (rad)
% Rudder (rad)
% Aileron (rad)
% -----------------------------

%% dynamics setup
% initial conditions
ic = [64.828  0  -0.0153  0  0  0  0  0  0  0  0  1119];

V0 = ic(1);
alpha0 = ic(3);
q0 = ic(4);
theta0 = 0;

% trim conditions
u0 = [ 1.29  -0.1174  0  0];
x0 = [ theta0 V0  alpha0  q0]';

% reference input - pitch angle
pitch_cmd = 2*pi/180;

% Create LQR Controller
A_long = [...
   0         0         0    1.0000      0 % theta
 -32.2000   -7   13.1983         3     0 % V
  0.0001   -0.253   -0.2370    1.0000   0 % alpha
    0   -0.0549   -5.5837   -3.7947     0 % q
   -1        0         0         0      0 ]; % Int(theta_cmd - theta)dt

B_long = [...
     0
     0
     7
   2.9035
     0 ];   

%% LQR setup
Q = diag([0.1 0.1 1 10 100]);

R = 1e-2;

Klqr = lqr(A_long, B_long, Q, R);

%% simulation
tf = 20;
r2d = 180/pi;

out = sim('AF_SimLQR_Lon');

control = out.control_input;
t = out.tout; % time {s}
state = out.state_output;
theta = out.pitch*r2d; % Pitch angle {deg}
dudt = out.dudt*r2d;

V       = state(:,1); % Velocity {ft/s}
gamma   = state(:,2)*r2d; %Flight path angle {deg}
alpha   = state(:,3)*r2d; %Angle of attack {deg}
q       = state(:,4)*r2d; %Pitch Rate {deg/s}
p       = state(:,5)*r2d; %Roll Rate {deg/s}
mu      = state(:,6)*r2d; %Bank Angle (About Velocity Vector) {deg}
beta    = state(:,7)*r2d; %Sideslip Angle {deg}
r       = state(:,8)*r2d; %Yaw Rate {deg/s}
chi     = state(:,9)*r2d; %Heading angle {deg}
north   = state(:,10); %North Position {ft}
east    = state(:,11); %East Position {ft}
h       = state(:,12); %Altitude {ft}

figure
title('Elevator')
subplot 211; hold on ; grid on 
plot(t, (control(:,2)-u0(2))*r2d)

subplot 212; hold on ; grid on 
plot(t, dudt*r2d)

figure('position',[50 50 1200 1200])
subplot(4,3,1); hold on; grid on;
plot(t,V,'linewidth',2)
xlabel('t (s)'); ylabel('V (ft/s)')
set(gca,'fontsize',12','fontname','times')

subplot(4,3,2); hold on; grid on;
plot(t,h,'linewidth',2)
xlabel('t (s)'); ylabel('h (ft)')
set(gca,'fontsize',12','fontname','times')

subplot(4,3,3); hold on; grid on;
plot(east,north,'linewidth',2)
xlabel('E (ft)'); ylabel('N (ft)')
set(gca,'fontsize',12','fontname','times')

subplot(4,3,4); hold on; grid on;
plot(t,theta,'linewidth',2)
xlabel('t (s)'); ylabel('\theta (deg)')
set(gca,'fontsize',12','fontname','times')

subplot(4,3,5); hold on; grid on;
plot(t,gamma,'linewidth',2)
xlabel('t (s)'); ylabel('\gamma (deg)')
set(gca,'fontsize',12','fontname','times')

subplot(4,3,6); hold on; grid on;
plot(t,chi,'linewidth',2)
xlabel('t (s)'); ylabel('\chi (deg)')
set(gca,'fontsize',12','fontname','times')

subplot(4,3,7); hold on; grid on;
plot(t,alpha,'linewidth',2)
xlabel('t (s)'); ylabel('\alpha (deg)')
set(gca,'fontsize',12','fontname','times')

subplot(4,3,8); hold on; grid on;
plot(t,beta,'linewidth',2)
xlabel('t (s)'); ylabel('\beta (ft/s)')
set(gca,'fontsize',12','fontname','times')

subplot(4,3,9); hold on; grid on;
plot(t,mu,'linewidth',2)
xlabel('t (s)'); ylabel('\mu (deg)')
set(gca,'fontsize',12','fontname','times')

subplot(4,3,10); hold on; grid on;
plot(t,p,'linewidth',2)
xlabel('t (s)'); ylabel('p (deg/s)')
set(gca,'fontsize',12','fontname','times')

subplot(4,3,11); hold on; grid on;
plot(t,q,'linewidth',2)
xlabel('t (s)'); ylabel('q (deg/s)')
set(gca,'fontsize',12','fontname','times')

subplot(4,3,12); hold on; grid on;
plot(t,r,'linewidth',2)
xlabel('t (s)'); ylabel('r (deg/s)')
set(gca,'fontsize',12','fontname','times')

