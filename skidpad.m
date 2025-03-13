mu = 1.5;
d_mu = -0.0005;
m = 250;
g = 9.81;
r = 7.625;
l = 1.5;
rcg = 0.2;
rca = 0.5;
a1 = l*(1-rcg);
a2 = l*rcg;
f_nom = 1125;
c_a = 3;
rho = 1.204;

v = 10;
tolerance = 0.00001;
max_iter = 1000;
iter = 0;

while true
    f_z = m*g;
    f_aero = 1/2*c_a*rho*v.^2;
    f_af = f_aero * rca;
    f_ar = f_aero * (1-rca);
    f_zf = f_z * (a2/l) + f_af;
    f_zr = f_z * (a1/l) + f_ar;
    f_y = f_z*mu;
    mu_f = mu + d_mu*(f_zf-f_nom);
    mu_r = mu + d_mu*(f_zr-f_nom);
    f_yfmax = f_zf*mu_f;
    f_yrmax = f_zr*mu_r;

    m_f = f_yfmax * a1;
    m_r = f_yrmax * a2;

    m_min = min(m_f,m_r);

    f_yf = m_min/a1;
    f_yr = m_min/a2;
    f_yt = f_yf + f_yr;

    a_y = f_y / m;
    a_yt = f_yt / m;
    g_yt = a_yt / g;
    v_y = sqrt(a_yt * r);
    grip = f_y / f_z;

    if abs(v - v_y) < tolerance
        break;
    end

    v = (v + v_y)/2;

    iter = iter + 1;

    if iter > max_iter
        break;
    end
end