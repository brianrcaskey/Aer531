clear,clc
I_xx_B = 1.948; %[slug*ft^2]
I_yy_B = 1.5523;
I_zz_B = 1.9166;
I_xz_B = 0;

S =10.56;
m = 0.487669;
cbar=16/2;
b=9.16;

u1=64.8280; %ft/sec
V = u1;
alpha = 2*pi/180;
alpha_dot = 0;

de = -0.1174; %rads

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

q = 0.5*rho*(V^2);


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
%Calculating necessary coefficients for use in Lift, Drag, and Moment
%equations
CLw=CLow+CLaw*alpha; %Coefficient of lift due to wing
CDw=CDminw+Kw*CLw^2; %Coefficient of drag due to wing
E=2*(CLow+CLaw*(alpha-alpha_dot*(cgt+cgw)/V))/(pi*ARw); %Not sure what to call this variable
alphat=alpha+it+Te*de+q*cgt/V-E; %angle of attack at horizontal tail
CLt=CLat*alphat; %Coefficient of lift due to horizonatl tail
CDt=CDmint+Kt*CLt^2; %Coefficient of drag due to horizontal tail
Cmf=Cmaf*alpha; %Moment coefficient due to fueselage 
CDvt=CDminvt; %Coefficient of drag due to vertical tail
Clp=-1/12*CLaw*(1+3*lambda)/(1+lambda); 

Clb=-.1; %rolling moment coefficient due to sideslip beta
Clr=.01;

%Eqns 26 - 33
%Forces and moments due to wing, horizontal and vertical tail, and
%fuselage:
qbar=.5*rho*V^2; %q bar (dynamic pressure)
Lw=qbar*S*CLw;   %lift due to wing
Dw=qbar*S*CDw;   %drag due to wing
Mw=qbar*S*c*Cmw; %pitching moment due to wing
Lt=nt*qbar*St*CLt; %lift due to horizontal tail
Dt=nt*qbar*St*CDt; %drag due to horizontal tai
Df=qbar*S*CDf;    %drag due to fuselage
Mf=qbar*S*c*Cmf;  %moment due to fuselage
Dvt=nt*qbar*Svt*CDvt; %drag due to vertical tail


%====================================================================


% NOTE EVERYTHING IS CURRENTLY PER RADIAN
C_D_u = 0; %??
C_D_1 = CDw+CDt+CDvt; %Total drag coefficient

C_T_x_1 = C_D_1;
C_D_alpha=0.362; %??
C_D_delta_e =0;

C_L_1 = CLw+CLt; %Total lift coefficient
C_T_x_u = -0.162; %??
C_L_u =0;
C_L_alpha = deg2rad(0.11); %Rascal pg72
C_L_alpha_dot=4.5; %??
C_L_q = 4.885;  %??
C_L_delta_e = 0.9;  %??

C_m_u =0; 
C_m_1 =0;
C_m_T_u =0;
C_m_T_1 =0;
C_m_alpha = deg2rad(-0.006); %Rascal pg72
C_m_T_alpha =0;
C_m_alpha_dot = -14.8; %??
C_m_q = deg2rad(-0.233); %Rascal pg72
C_m_delta_e= deg2rad(0.011); %Rascal pg72

C_y_beta = deg2rad(-0.0056); %Rascal pg72
C_y_p = -0.1138; %Extra paper pg24
C_y_r =0.356;
C_y_delta_a =0;
C_y_delta_r = 0.23;
C_l_beta = deg2rad(-0.0018); %Rascal pg72
C_l_p = deg2rad(0.013); %Rascal pg72
C_l_r = deg2rad(0.01); %Rascal pg72
C_l_delta_a = deg2rad(0.244); %Rascal pg72
C_l_delta_r = 0.0192;
C_n_beta = deg2rad(0.00023); %Rascal pg72
C_n_T_beta=0;
C_n_p =-0.0380; %% Find in 'useful paper'
C_n_r = deg2rad(-0.0006); %Rascal pg72
C_n_delta_a= deg2rad(-0.0128); %Rascal pg72
C_n_delta_r =-0.1152;


save('Aircraft_531.mat')