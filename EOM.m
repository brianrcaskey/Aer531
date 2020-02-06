%AerE 531 EOM Function
%Written on 02/05/2020 by students at Iowa State University

clear,clc,close all

%temporary input vector
in = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16];

% PARSING INPUTS
%=============================================================
T   = in(1); %Thrust
de  = in(2); %Elevator deflection (down is +) {deg}
drt = in(3); %Rudder deflection {deg}
dq  = in(4); %Aileron deflection {deg}

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
it = deg2rad(2);    %Tail incidence 2 degrees
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