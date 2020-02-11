clear,clc


% 
% %function LDM = LiftDragMoment(q_bar, alpha)
% 
% alpha = 2*pi/180; %temporary input - AoA in rads
% V = 176;           %temporary input - velocity in ft/s
% delta_e = 2*pi/180;
% 
% q = .5*(V^2)*0.0765;
% 
% %Define import options
% opts = detectImportOptions('LiftDragMoment_Coeff.xlsx'); %detects import options
% opts.SelectedVariableNames = [3];   %imports only 3rd column of data
% 
% T = readmatrix('LiftDragMoment_Coeff.xlsx', opts);
% 
% %Define coefficients for buildup
% CL_0_w      = T(1);
% CL_alpha_w  = T(2);
% CD_min_w    = T(3);
% CM_w        = T(4);
% CL_alpha_t  = T(5);
% CD_min_t    = T(6);
% CL_alpha_h  = T(7);
% CD_min_h    = T(8);
% CM_alpha_f  = T(9);
% CD_f        = T(10);
% S_w         = T(11); %reference wing area
% c_w         = T(12); %reference chord
% b_w         = T(13); %reference span
% x_cg        = T(14);
% z_cg        = T(15);
% i_w         = T(16);
% i_t         = T(17);
% e_w         = T(18); %Oswald efficiency factor
% cg_w        = T(19);
% cg_t        = T(20);
% T_e         = T(21);
% e_t         = T(22);
% b_t         = T(23);
% c_t         = T(24);
% 
% %Factors derived from aircraft dimensions
% A
% R_w    = b_w/c_w;
% AR_t    = b_t/c_t;
% K_w     = 1/(pi*e_w*AR_w);
% K_t     = 1/(pi*e_t*AR_t);
% 
% 
% %Eqns 17-24
% CL_w = CL_0_w + CL_alpha_w*alpha;
% CD_w = CD_min_w + K_w*(CL_w^2);     
% E = 2*(CL_0_w + CL_alpha_w * (alpha - alpha*(cg_t + cg_w)/V))/(pi*AR_w);
% alpha_t = alpha + i_t + T_e*delta_e + q*cg_t / V - E;
% CL_t = CL_alpha_t*alpha_t;
% CD_t = CD_min_t + K_t*(CL_t^2);
% Cm_f = 

% in1 = ?Constants
% in2 = ?Forces
% in3 = ?Moments
% in4 = ?Atmospheric conditions


d_th= 1.29;
d_e = - 0.1174;
d_r =  0;
d_a = 0; 
uin = [d_th d_e d_r d_a];

P = 0;
Q = 0; 
R = 0; 
V = 64.8280; 
beta = 0;
alpha = -0.0153;
mu = 0;
gamma = 0; 
chi = 0; 
zeta = 0; 
eta = 0; 
h = 1119;
u0 = [P Q R V beta alpha mu gamma chi zeta eta h];
% sim('simulationjake',in1,value,in2,value,in3,value,in4,value)
% %end

sim('simulationjake')