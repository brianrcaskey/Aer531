
clear,clc

u0 = [1.29, -0.1174, 0,  0, 64.828,     0,  -0.0153, 0,  0,  0,     0, 0,   0,     0,    0, 1119];
uin = zeros(1,16);
In1 = 0;

%[V_dot, gamma_dot, alpha_dot, q_dot, p_dot, mu_dot, beta_dot, r_dot, chi_dot, n_dot, e_dot, h_dot] = EOM(u0);