function [X_u, X_T_u, X_alpha, X_delta_e, Z_u, Z_alpha, Z_alpha_dot,...
    Z_q, Z_delta_e, M_u, M_T_u, M_alpha, M_T_alpha, M_alpha_dot,...
    M_q, M_delta_e] ...
    = Longitudinal_derivatives(qbar, S, m, U1, I_yy_s, cbar,...
    C_D_u, C_D_1, C_T_x_u, C_T_x_1, C_D_alpha, C_L_1,...
    C_D_delta_e, C_L_u, C_L_alpha, C_L_alpha_dot,...
    C_L_q, C_L_delta_e, C_m_u, C_m_1, C_m_T_u, C_m_T_1,...
    C_m_alpha, C_m_T_alpha, C_m_alpha_dot, C_m_q, C_m_delta_e)

% Do not change input or output code! 
%---------------------------------------------------------------------

% Longitudinal
X_u = -qbar*S*(C_D_u + 2*C_D_1)/(m*U1);
X_T_u = qbar*S*(C_T_x_u + 2*C_T_x_1)/(m*U1);
X_alpha = -qbar*S*(C_D_alpha - C_L_1)/m;
X_delta_e = -qbar*S*C_D_delta_e/m;
Z_u = -qbar*S*(C_L_u + 2*C_L_1)/(m*U1);
Z_alpha = -qbar*S*(C_L_alpha + C_D_1)/m;
Z_alpha_dot = -qbar*S*cbar*C_L_alpha_dot/(2*m*U1);
Z_q = -qbar*S*cbar*C_L_q/(2*m*U1);
Z_delta_e = -qbar*S*C_L_delta_e/m;
M_u = qbar*S*cbar*(C_m_u+2*C_m_1)/(I_yy_s*U1);
M_T_u = qbar*S*cbar*(C_m_T_u + 2*C_m_T_1)/(I_yy_s*U1);
M_alpha = qbar*S*cbar*C_m_alpha/I_yy_s;
M_T_alpha = qbar*S*cbar*C_m_T_alpha/I_yy_s;
M_alpha_dot = qbar*S*cbar^2*C_m_alpha_dot/(2*I_yy_s*U1);
M_q = qbar*S*cbar^2*C_m_q/(2*I_yy_s*U1);
M_delta_e = qbar*S*cbar*C_m_delta_e/I_yy_s;







% --------------------------------------------------------------------
end