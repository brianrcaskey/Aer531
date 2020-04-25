function [Y_beta, Y_p, Y_r, Y_delta_a, Y_delta_r,...
    L_beta, L_p, L_r, L_delta_a, L_delta_r,...
    N_beta, N_T_beta, N_p, N_r, N_delta_a, N_delta_r]...
    = Lateral_derivatives(qbar, S, U1, m, b, I_xx_s, I_zz_s,...
    C_y_beta, C_y_p, C_y_r, C_y_delta_a, C_y_delta_r,...
    C_l_beta, C_l_p, C_l_r, C_l_delta_a, C_l_delta_r,...
    C_n_beta, C_n_p, C_n_r, C_n_delta_a, C_n_delta_r, C_n_T_beta)

% Do not change the code above this line!
%------------------------------------------------------------------------


%Lateral-directional
Y_beta = qbar*S*C_y_beta/m;
Y_p = qbar*S*b*C_y_p/(2*m*U1);
Y_r = qbar*S*b*C_y_r/(2*m*U1);
Y_delta_a = qbar*S*C_y_delta_a/m;
Y_delta_r = qbar*S*C_y_delta_r/m;
L_beta = qbar*S*b*C_l_beta/I_xx_s;
L_p = qbar*S*b^2*C_l_p/(2*I_xx_s*U1);
L_r = qbar*S*b^2*C_l_r/(2*I_xx_s*U1);
L_delta_a = qbar*S*b*C_l_delta_a/I_xx_s;
L_delta_r = qbar*S*b*C_l_delta_r/I_xx_s;
N_beta = qbar*S*b*C_n_beta/I_zz_s;
N_T_beta = qbar*S*b*C_n_T_beta/I_zz_s;
N_p = qbar*S*b^2*C_n_p/(2*I_zz_s*U1);
N_r = qbar*S*b^2*C_n_r/(2*I_zz_s*U1);
N_delta_a = qbar*S*b*C_n_delta_a/I_zz_s;
N_delta_r = qbar*S*b*C_n_delta_r/I_zz_s;

% --------------------------------------------------------------------
end