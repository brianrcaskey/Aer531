function [A_long, B_long, A_lat, B_lat]...
    = State_space_model(...
    alpha_1, U1, I_xz_s, I_xx_s, I_yy_s, I_zz_s,...
    X_u, X_T_u, X_alpha, X_delta_e, Z_u, Z_alpha, Z_alpha_dot,...
    Z_q, Z_delta_e, M_u, M_T_u, M_alpha, M_T_alpha, M_alpha_dot,...
    M_q, M_delta_e,...
    Y_beta, Y_p, Y_r, Y_delta_a, Y_delta_r,...
    L_beta, L_p, L_r, L_delta_a, L_delta_r,...
    N_beta, N_T_beta, N_p, N_r, N_delta_a, N_delta_r)

% Do not change the code above this line!
%------------------------------------------------------------------------

theta_1 = alpha_1;
g = 32.2;
% U1 = U1*1.6878; %conversion from kts to ft/sec
%Longitudinal
k_long = [1, 0, 0, 0;
0, 1, 0, 0;
0, 0, U1-Z_alpha_dot, 0;
0, 0, -M_alpha_dot , 1];

A_long = (k_long)\[0,0,0,1;...
-g*cosd(theta_1), X_u+X_T_u, X_alpha, 0;...
-g*sind(theta_1), Z_u, Z_alpha, U1+Z_q;...
0, M_u+M_T_u, M_alpha+M_T_alpha, M_q];

B_long = (k_long)\[0; X_delta_e; Z_delta_e; M_delta_e];

%Lateral-directional
A_1 = I_xz_s/I_xx_s;
B_1 = I_xz_s/I_zz_s;
k_lat = [1,0,0,0,0;...
0,1,0,0,0;...
0,0,U1,0,0;...
0,0,0,1,-A_1;...
0,0,0,-B_1,1];


A_lat = k_lat\[0,0,0,1,0;...
0,0,0,0,1;...
g*cosd(theta_1), 0, Y_beta, Y_p, Y_r-U1; ...
0,0,L_beta,L_p,L_r;...
0,0,N_beta+N_T_beta,N_p,N_r];
B_lat = (k_lat)\[0,0; 0,0; Y_delta_a, Y_delta_r; L_delta_a, L_delta_r; 
    N_delta_a, N_delta_r];




%------------------------------------------------------------------------
end