%AerE 531 EOM Function
%Written on 02/05/2020 by students at Iowa State University
clear,clc

%function x_dot = EOM(in)

%global maneuver alpha_dot
%global alpha_dot;
alpha_dot = 0;
x_dot = [];
%temporary input vector

u0 = [1.29, -0.1174, 0,  0, 64.828,     0,  -0.0153, 0,  0,  0,     0, 0,   0,     0,    0, 1119, 100];
   % [T,     de,    dr, da, V,      gamma,  AoA,     q,  p, mu,  beta, r, chi, north, east,    h, clock-input]
   

   
in = u0;


% PARSING INPUTS
%=============================================================
T   = in(1); %Thrust
de  = in(2); %Elevator deflection (down is +) {deg}
drt = in(3); %Rudder deflection {deg}
da  = in(4); %Aileron deflection {deg}

V       = in(5);  %Velocity {ft/s}
gamma   = in(6);  %Flight path angle {rad}
alpha   = in(7);  %Angle of attack {rad}
q       = in(8);  %Pitch Rate {rad/s}
p       = in(9);  %Roll Rate {rad/s}
mu      = in(10); %Bank Angle (About Velocity Vector) {rad}
beta    = in(11); %Sideslip Angle {rad}
r       = in(12); %Yaw Rate {rad/s}
chi     = in(13); %Heading angle {rads}
north   = in(14); %North Position {ft}
east    = in(15); %East Position {ft}
h       = in(16); %Altitude {ft}

tm = in(17);

% ATMOSPHERIC PROPERTIES
%=============================================================
rho = 0.0023081;    %Air Density (slugs per ft^3) -> do we want to make this a function of height?
g = 32.17;          %ft/s^2


% AIRCRAFT PROPERTIES
%==============================================================


m = 0.487669;       %Slugs - empty mass w/o fuel
Iyy = 1.5523;       %Inertia found with empty mass
Ixx = 1.948;        %Inertia
Izz = 1.9166;       %Inertia
Ixz = 0;            %Assumed zero due to symmetric aircraft

S = 10.56;          %Wing area - square feet
b = 9.16;           % Wing span - feet


CLow = 0.421;       %Wing coefficient of lift at 0deg AoA
CLaw = 4.59;        %Wing coefficient of lift per AoA
CDminw = 0.011;     %wing minimum coefficient of drag
ARw = (b^2)/S;      %Wing aspect ratio
e = 0.75;           %Span efficiency - ESTIMATION
Kw = 1/(pi*ARw*e);  %
Cmw = -0.005;       %Wing moment coefficient
cgw = -(5/12);      %Distance aero center is back from cg - 5 inches
c = (16/12);        %Root chord of wing (16") - feet
lambda = 0.72955;   %Taper ratio from S = (Cr*(1+lambda)*b)/2
CLat = 0.76;        %Vert tail coefficient of lift per AoA
CDmint = 0.002;     %vert tail minimum coeffcient of drag
Kt = 0.446;         %TO DO - MORE CALC?
it = deg2rad(2);    %Tail incidence 2 degrees to radians
Te = 0.422;         %Tail control surface effectiveness (??)
nt = 1;             %??
St = S;             %Horizontal tail area square feet = ref area (??)
cgt = 3.5;          %Distance tail aero center back from a/c cg

CLavt = 0.0969;     %Vertical tail coefficient of lift per AoA(??)
CDminvt = 0.001;    %vert tail min coefficient of drag (??)
Svt = S;            %Ref area of vertical tail = ref area (??)
Tr = 0.434;         %Name??
nvt = nt;           %same as hori tail (??)
cgvt = cgt;         %same as hori tail (??)

Cmaf = 0.114;       %fuselage moment coefficient
CDf = 0.005;        %Fuselage coefficient of drag

Cnda = -0.0128;     %per rad (??)
Clda = 0.244;       %per rad (??)


%Eqns 17-25
CLw=CLow+CLaw*alpha;
CDw=CDminw+Kw*CLw^2;
E=2*(CLow+CLaw*(alpha-alpha_dot*(cgt+cgw)/V))/(pi*ARw);
alphat=alpha+it+Te*de+q*cgt/V-E;
CLt=CLat*alphat;
CDt=CDmint+Kt*CLt^2;
Cmf=Cmaf*alpha;
CDvt=CDminvt;
Clp=-1/12*CLaw*(1+3*lambda)/(1+lambda);

Clb=-.1;
Clr=.01;

%Eqns 26 - 33
qb=.5*rho*V^2;
Lw=qb*S*CLw;
Dw=qb*S*CDw;
Mw=qb*S*c*Cmw;
Lt=nt*qb*St*CLt;
Dt=nt*qb*St*CDt;
Df=qb*S*CDf;
Mf=qb*S*c*Cmf;
Dvt=nt*qb*Svt*CDvt;

% Calculate Forces and Moments
% Lift
L=Lw+Lt*cos(E-q*cgt/V)-(Dt+Dvt)*sin(E-q*cgt/V);
% Drag
D=Dw+(Dt+Dvt)*cos(E-q*cgt/V)+Lt*sin(E-q*cgt/V)+Df;
% Side Force
Y=nvt*qb*Svt*CLavt*(-beta+Tr*drt+r*cgvt/V);
% Pitch Moment
Mc=Lw*cgw*cos(alpha)+Dw*cgw*sin(alpha)+Mw-Lt*cgt*cos(alpha-E+q*cgt/V)-(Dt+Dvt)*cgt*sin(alpha-E+q*cgt/V)+Mf;
% Yaw Moment
Nc=-qb*nvt*Svt*CLavt*(-beta+Tr*drt+r*cgvt/V)*cgvt+(-qb*S*b*Cnda*da);
% Roll Moment
Lc=qb*S*b^2/(2*V)*(Clp*p+2*V/b*Clb*beta+Clr*drt+Clda*da*2*V/b);




% -=-=-=-=-=- NONLINEAR 6-DOF EQUATION OF MOTION (EOMs) -=-=-=-=-=-=-=-=-
%
% These are the state derivative equations; the comment names the state,
% but the equation is for its derivative (rate)
%
% The equations are arranged by aircraft mode
%
% NOTE: These assume that Ixz=0, if not, then eq's need to be modified
% Longitudinal (phugoid and short period): V, gamma, q, alpha
% Phugoid: V, gamma
 % Velocity
 V_dot=1/m*(-D*cos(beta)+Y*sin(beta)+T*cos(beta)*cos(alpha))-...
 g*sin(gamma);
 % Flight Path Angle
 gamma_dot=1/(m*V)*(-D*sin(beta)*sin(mu)-Y*sin(mu)*cos(beta)...
 +L*cos(mu)+T*(cos(mu)*sin(alpha)+sin(mu)*sin(beta)*cos(alpha)))...
 -g/V*cos(gamma);
% Short Period: alpha, q
 % Angle of Attach
 alpha_dot=q-tan(beta)*(p*cos(alpha)+r*sin(alpha))-1/(m*V*cos(beta))...
 *(L+T*sin(alpha))+g*cos(gamma)*cos(mu)/(V*cos(beta));
 % Pitch Rate
 q_dot=Mc/Iyy+1/Iyy*(Izz*p*r-Ixx*r*p);
% Lateral (roll) - Directional (yaw): p, mu, beta, r
% Roll: p, mu
 % Roll Rate
 p_dot=Lc/Ixx+1/Ixx*(Iyy*r*q-Izz*q*r);

 % Bank Angle (about velocity vector)
 mu_dot=1/cos(beta)*(p*cos(alpha)+r*sin(alpha))+1/(m*V)*(D*sin(beta)...
 *cos(mu)*tan(gamma)+Y*tan(gamma)*cos(mu)*cos(beta)+L*(tan(beta)+...
 tan(gamma)*sin(mu))+T*(sin(alpha)*tan(gamma)*sin(mu)+sin(alpha)*...
 tan(beta)-cos(alpha)*tan(gamma)*cos(mu)*sin(beta)))-...
 g/V*cos(gamma)*cos(mu)*tan(beta);

% Dutch Roll: beta, r
 % Side Slip Angle
 beta_dot=-r*cos(alpha)+p*sin(alpha)+1/(m*V)*(D*sin(beta)+Y*cos...
 (beta)-T*sin(beta)*cos(alpha))+g/V*cos(gamma)*sin(mu);

 % Yaw Rate

 r_dot=Nc/Izz+1/Izz*(Ixx*p*q-Iyy*p*q);

% Heading Angle (from North)
 chi_dot=1/(m*V*cos(gamma))*(D*sin(beta)*cos(mu)+Y*cos(mu)*cos(beta)...
 +L*sin(mu)+T*(sin(mu)*sin(alpha)-cos(mu)*sin(beta)*cos(alpha)));

% Kinematic Equations
 % North Position
 n_dot=V*cos(gamma)*cos(chi);

 % East Position
 e_dot=V*cos(gamma)*sin(chi);

 % Altitude
 h_dot=V*sin(gamma);

% Pack derivatives into output vector x_dot
 x_dot(1) = V_dot;
 x_dot(2) = gamma_dot;
 x_dot(3) = alpha_dot;
 x_dot(4) = q_dot;
 x_dot(5) = p_dot;
 x_dot(6) = mu_dot;
 x_dot(7) = beta_dot;
 x_dot(8) = r_dot;
 x_dot(9) = chi_dot;
 x_dot(10) = n_dot;
 x_dot(11) = e_dot;
 x_dot(12) = h_dot;

%end