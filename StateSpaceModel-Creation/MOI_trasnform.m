function [I_xx_s, I_yy_s, I_zz_s, I_xz_s]...
    = MOI_trasnform(...
    I_xx_B, I_yy_B, I_zz_B, I_xz_B,...
    alpha_1)

I_yy_s = I_yy_B;
% Do not change above this line.
% Reference Roskam p.346
% --------------------------------------------------------------------

Result = [cos(alpha_1)^2, sin(alpha_1)^2, -sin(2*alpha_1);
    sin(alpha_1)^2, cos(alpha_1)^2, sin(2*alpha_1);
    0.5*sin(2*alpha_1), -0.5*sin(2*alpha_1), cos(2*alpha_1)]*[I_xx_B; I_zz_B; I_xz_B];

I_xx_s=Result(1);
I_zz_s=Result(2);
I_xz_s=Result(3);


% --------------------------------------------------------------------
end