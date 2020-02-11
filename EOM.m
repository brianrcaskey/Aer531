function Xdot=EOM(t, X0, control)
% Forces and Moments, and Equations of Motion
% for the SIG Rascal Model Airplane
% by Youngro Lee
% t
% *** QUANTITY ****** UNITS *********************************************
% *** mass -> {slugs}
% *** length -> {ft}
% *** area -> {ft^2}
% *** velocity -> {ft/s}
% *** acceleration-> {ft/s^2}
% *** density -> {slugs/ft^3}
% *** force -> {lbf}
% *** moments -> {lbf-ft}
% *** angles -> {radians} (calculations)
% *** velocity -> {ft/s}
% *** ang. vel. -> {rad/s}
% *** ang. accel. -> {rad/s^2}
% ***********************************************************************
alphadot = 0;

% Control
T = control(1); % Thrust
de = control(2); % Elevator Deflection (down is +) {deg}
drt = control(3); % Rudder Deflection {deg}
da = control(4); % Aileron Deflection (deg)

% States
q = X0(1); % Pitch Rate {rad/s}
p = X0(2); % Roll Rate {rad/s}
r = X0(3); % Yaw Rate {rad/s}
V = X0(4); % Velocity {ft/s}
chi = X0(5); % Heading angle {rads}
gamma = X0(6); % Flight path angle {rad}
mu = X0(7); % Bank Angle (About Velocity Vector) {rad}
alpha = X0(8); % Angle of attack {rad}
beta = X0(9); % Sideslip Angle {rad}
xi = X0(10); % North Position {ft}
eta = X0(11); % East Position {ft}
h = X0(12); % Altitude {ft}

% Environment value
rho=.0023081; %Air Density (Slugs per ft^3) - Checked
g=32.17; %Gravity - Checked
m = 0.19830199; %Slugs - Empty Mass of A/C (w/o fuel)(7.117 Kg empty)
Iyy = 0.0134383734; %Inertias Experimentally determined using Empty Mass
Ixx = 0.0217331154; %Inertia Units are (slugs*ft^2) - Checked
Izz = 0.0128081512; % - Checked


S=6.45835; %Square Feet - Wing Area Converted from
%Manuf 1520 sq in area - Checked (also Matches DigDat)
CLow=.421; %from Look up table for Eppler 193 at AoA=0,
%DigDat = .421, Line # 277 - Checked CL column
CLaw=4.59; %Coef of Lift for finite wing = Cla/(1+ (Cla/(pi*ARw*e)
%DigDat = 4.59, Line #276-277, CLA column- Checked
CDminw=.011; %DigDat Min Drag of Wing at AoA = -6 deg,
%Dig Dat Line# 274- Checked
ARw=7.9456; %Aspect Ratio AR = (b^2)/S- Checked
e = .75; %Span efficiency factor -estimation- Checked
Kw = 1/(pi*ARw*e); % = 1/(pi*AR*e)- Checked
Cmw=-0.005; %DigDat = AoA =0, Line #277- Checked
cgw=-.416; %Distance Aero Center is back from CG, 5 Inches
c=1.3333; %Feet - Root Chord of Wing (16")- Checked
b=9.16; %Feet - Span (110")- Checked
lambda=.72955; %Taper Ratio from S=(Cr*(1+Lambda)*b)/2- Checked
CLat=.76; %Dig Dat, Line #346, CLA Column- Checked
CDmint=.002; %Dig Dat, Line #345, at AoA = -2 deg
Kt=.446; %=1/(pi*e*AR) =SET SAME AS WING OR DIGDAT From BLAKE
it=2*pi/180; %Tail incidence 2 degrees
Te=.422; %Blake from DigDat
nt=1; %Blake from DigDat
St=S; %Horiz Tail Area Square Feet=Reference Area = Wing Area
cgt=3.5; %Distance tail Aero Center back from A/C CG,
%42 inches - Measured
CLavt=.0969; %Dig Dat Line #190
CDminvt=.001; %Dig Dat Line #409, AOA = -10 deg, Column CD
Svt=S; %Vert Tail Area Square Feet = Reference Area = Wing Area
Tr=.434; %Blake from DigDat
nvt=nt; %Same as Hori Tail
cgvt=cgt; %Same as Hori tail
Cmaf=.114; %Dig Dat, Line#209, at AoA = 0
CDf=.005; %Dig Dat, Line#209, at AoA = 0
Cnda = -0.0128; %per rad DigDat
Clda = 0.244; %per rad DigDat
% Define/calculate any needed coefficients, forces, etc.
CLw=CLow+CLaw*alpha;
CDw=CDminw+Kw*CLw^2;
E=2*(CLow+CLaw*(alpha-alphadot*(cgt+cgw)/V))/(pi*ARw);
alphat=alpha+it+Te*de+q*cgt/V-E;
CLt=CLat*alphat;
CDt=CDmint+Kt*CLt^2;
Cmf=Cmaf*alpha;
CDvt=CDminvt;
Clp=-1/12*CLaw*(1+3*lambda)/(1+lambda);
Clb=-.1;
Clr=.01;
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
Mc=Lw*cgw*cos(alpha)+Dw*cgw*sin(alpha)+Mw-Lt*cgt*...
cos(alpha-E+q*cgt/V)-(Dt+Dvt)*cgt*sin(alpha-E+q*cgt/V)+Mf;
% Yaw Moment
Nc=-qb*nvt*Svt*CLavt*(-beta+Tr*drt+r*cgvt/V)*cgvt+(-qb*S*b*Cnda*da);
% Roll Moment
Lc=qb*S*b^2/(2*V)*(Clp*p+2*V/b*Clb*beta+Clr*drt+Clda*da*2*V/b);

% -=-=-=-=-=- NONLINEAR 6-DOF EQUATION OF MOTION (EOMs) -=-=-=-=-=-=-=-=-
% Roll Rate
pdot=Lc/Ixx+1/Ixx*(Iyy*r*q-Izz*q*r);
% Pitch Rate
qdot=Mc/Iyy+1/Iyy*(Izz*p*r-Ixx*r*p);
% Yaw Rate
rdot=Nc/Izz+1/Izz*(Ixx*p*q-Iyy*p*q);

% Velocity
Vdot=1/m*(-D*cos(beta)+Y*sin(beta)+T*cos(beta)*cos(alpha))-g*sin(gamma);

% Heading Angle (from North)
chidot=1/(m*V*cos(gamma))*(D*sin(beta)*cos(mu)+Y*cos(mu)*cos(beta)+L*sin(mu)+T*(sin(mu)*sin(alpha)-cos(mu)*sin(beta)*cos(alpha)));
% Flight Path Angle
gammadot=1/(m*V)*(-D*sin(beta)*sin(mu)-Y*sin(mu)*cos(beta)+L*cos(mu)+T*(cos(mu)*sin(alpha)+sin(mu)*sin(beta)*cos(alpha)))...
        -g/V*cos(gamma);
% Bank Angle (about velocity vector)
mudot=1/cos(beta)*(p*cos(alpha)+r*sin(alpha))...
    +1/(m*V)*(D*sin(beta)*cos(mu)*tan(gamma)+Y*tan(gamma)*cos(mu)*cos(beta)+L*(tan(beta)+tan(gamma)*sin(mu))...
    +T*(sin(alpha)*tan(gamma)*sin(mu)+sin(alpha)*tan(beta)-cos(alpha)*tan(gamma)*cos(mu)*sin(beta)))...
-g/V*cos(gamma)*cos(mu)*tan(beta);

% Angle of Attach
alphadot=q-tan(beta)*(p*cos(alpha)+r*sin(alpha))-1/(m*V*cos(beta))*(L+T*sin(alpha))+g*cos(gamma)*cos(mu)/(V*cos(beta));
% Side Slip Angle
betadot=-r*cos(alpha)+p*sin(alpha)+1/(m*V)*(D*sin(beta)+Y*cos(beta)-T*sin(beta)*cos(alpha))+g/V*cos(gamma)*sin(mu);

% Kinematic Equations
% North Position
xidot=V*cos(gamma)*cos(chi);
% East Position
etadot=V*cos(gamma)*sin(chi);
% Altitude
hdot=V*sin(gamma);


Xdot = [pdot  qdot rdot Vdot chidot gammadot mudot alphadot betadot xidot etadot hdot]';
end