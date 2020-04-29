
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

roll = zeros(3000,1);
roll(101:3000) = 5*pi/180;

roll_cmd = [T1, roll];

% Create LQR Controller
A_lat = [ 0         0         0    1.0000         0 0
         0         0         0         0    1.0000  0
    0.4967         0   -0.0002   -0.0130   -0.9593  0
         0         0   -0.0076    0.0040    0.0030  0
         0         0    0.0010   -0.6571   -0.0002  0
        -1         0         0         0         0      0];
     
     
B_lat = [0         0
         0         0
         0    0.3726
    1.0256    4.6310
   -0.0549  -28.1998
         0         0];



Q = diag([1,0.001,1,2,1,8]);



max_da =[1:1:10];
LQR_ail = struct([]);
LQR_ail_dt = struct([]);
LQR_rud = struct([]);
LQR_rud_dt = struct([]);
LQR_roll_angle = struct([]);

max_dr = 20;






% for i = 1:length(max_da)
%     
%     R = diag([1/(max_da(i)^2), 1/(max_dr^2)]);
%     k_lqr = lqr(A_lat, B_lat, Q, R)  
%     sim('AF_SimLQR_V2.slx', 10)
%     LQR_ail{i} = ans.ail
%     LQR_ail_dt{i} = ans.ail_dt
%     LQR_rud{i} = ans.rud
%     LQR_rud_dt{i} = ans.rud_dt
%     LQR_roll_angle{i} = ans.roll_angle
%     
% end
% 
% figure
% hold on
% legendCell = strcat('max da =', string(num2cell(max_da)))
% for i = 1:length(max_da)
%     
%    plot(LQR_ail_dt{i}.time(1:13),LQR_ail_dt{i}.data(1:13))
%     
% end
% hold off
% grid on
% legend(legendCell)
% 
% figure 
% hold on
% legendCell = strcat('max da =', string(num2cell(max_da)))
% 
% for i = 1:length(max_da)
%     
%    plot(LQR_roll_angle{i}) 
%     
% end
% grid on
% hold off
% legend(legendCell)
wc = 0.025;
max_dr = 30;
max_da = 10;

Q = diag([1,0.0001,1,1,1,30]);
R = diag([1/(max_da^2), 1/(max_dr^2)]);
k_lqr = lqr(A_lat, B_lat, Q, R) 

%%

 

runtime = 50;
runtime_t = runtime*10;
sim('AF_SimLQR_V2.slx',runtime)
figure
subplot(3,1,1)
hold on
plot(T1(1:runtime_t),roll(1:runtime_t)*180/pi)
plot(ans.roll_angle)
hold off
grid on
legend('Command', 'Roll Angle')

subplot(3,1,2)
hold on
grid on
plot(ans.rud)
plot(ans.ail)
hold off
ylabel('Deflection (deg)')
xlabel('Time (sec)')
legend('Aileron','Rudder')

subplot(3,1,3)
hold on
grid on
plot(ans.ail_dt.*pi/180)
plot(ans.rud_dt.*pi/180)
legend('Aileron','Rudder')
ylabel('Actuator speed (rad/sec)')
xlabel('Time (sec)')

hold off
