% Cessna 310 - sample

% Geometric data
S = 174;
cbar = 4.9;
b=36;

% Climb case! -------------------
u1 = 133.5;
qbar = 21.2;
alpha_1 = 5.4*pi/180;
theta_1 = 0;
I_xx_B = 2650;
I_yy_B = 1346;
I_zz_B = 1967;
I_xz_B = 0;

% Longitudinal
C_D_u         = 0;
C_D_1         = 0.057;
C_T_x_u       = -0.171;
C_T_x_1       = 0.057;
C_D_alpha     = 0.380;
C_L_1         = 0.719;
C_D_delta_e   = 0;
C_L_u         = 0;
C_L_alpha     = 4.41;
C_L_alpha_dot = 1.7;
C_L_q         = 3.9;
C_L_delta_e   = 0.43;
C_m_u         = 0;
C_m_1         = 0;
C_m_T_u       = 0;
C_m_T_1       = 0;
C_m_alpha     = -0.650;
C_m_T_alpha   = 0;
C_m_alpha_dot = -5.57;
C_m_q         = -15.2;
C_m_delta_e   = -1.369;


C_y_beta      = -0.404;
C_y_p         = -0.145;
C_y_r         = 0.267;
C_y_delta_a   = 0;
C_y_delta_r   = 0.187;
C_l_beta      = -0.0895;
C_l_p         = -0.487;
C_l_r         = 0.1869;
C_l_delta_a   = 0.229;
C_l_delta_r   = 0.0147;
C_n_beta      = 0.0907;
C_n_p         = -0.0649;
C_n_r         = -0.1199;
C_n_delta_a   = -0.0504;
C_n_delta_r   = -0.0805;

save('Cessna_310_climb.mat')