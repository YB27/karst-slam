rho_s_plus, rho_s_minus, rho_psi_plus, rho_psi_minus = symbols('rho_s__+ rho_s__- rho_psi__+ rho_psi__-')
a_pp = MatrixSymbol('a__++',3,1)
a_pm = MatrixSymbol('a__+-',3,1)
a_mp = MatrixSymbol('a__-+',3,1)
a_mm = MatrixSymbol('a__--',3,1)
b_p_t = MatrixSymbol('b_t__+',3,1)
b_m_t = MatrixSymbol('b_t__-',3,1)
dt_p  = MatrixSymbol('dt_p',3,1)
dt_m = MatrixSymbol('dt_m',3,1)
dtt = MatrixSymbol('dtt',3,1)

n = rho_s_plus * rho_psi_plus * a_pp + rho_s_minus * rho_psi_minus * a_mm - \
    rho_s_minus * rho_psi_plus * a_mp - rho_s_plus * rho_psi_minus * a_pm + \
    rho_s_plus * b_p_t - rho_s_minus * b_m_t + \
    rho_psi_plus * dt_p - rho_psi_minus * dt_m + dtt

res = n*n.T  

    
