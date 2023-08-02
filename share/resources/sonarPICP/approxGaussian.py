import numpy as np
import scipy as scp
import scipy.special
import scipy.stats

""" 
    Here we compute the gaussian approximation for the elevation angles distributions 
    as explained in [1], section 5.4

    References
    ----------
    .. [1] Yohan Breux and André Mas and Lionel Lapierre, "On-manifold Probabilistic ICP : Application to Underwater Karst
    Exploration"
"""

def expectationPsi_closeForm(params):
    """ 
        Compute the expectation of the sonar rotation angle (psi) 

        Parameters
        ----------
        params : dict 
                 dict containing at least the parameter ["psi"], which itself contains ["mean"] and ["std"].
                 The corresponding values can be arrays.

        Returns 
        -------
        e_cos : float / array
                expectation of cos(phi). If the input contains arrays, it is an array.
        e_sin : float / array
                expectation of sin(phi). If the input contains arrays, it is an array.  
        e_cs : float / array
                expectation of cos(phi)*sin(phi). If the input contains arrays, it is an array. 
        e_csqr : float / array
                expectation of cos^2(phi). If the input contains arrays, it is an array. 
        e_sqr : float / array
                expectation of sin^2(phi). If the input contains arrays, it is an array. 
        
        Notes
        -----
        Computation is explained in [1], section 5.4, Proposition 5 
    """

    psi_mean = params["psi"]["mean"]
    psi_var = params["psi"]["std"]**2

    exp_half_var_minus = np.exp(-0.5*psi_var)
    exp_double_var_minus = np.exp(-2.*psi_var)
    cos_2_psi = np.cos(2.*psi_mean)

    e_cos = np.cos(psi_mean)*exp_half_var_minus
    e_sin = np.sin(psi_mean)*exp_half_var_minus
    e_csqr = 0.5*(1. + exp_double_var_minus*cos_2_psi)
    e_ssqr = 0.5*(1. - exp_double_var_minus*cos_2_psi)
    e_cs = 0.5*exp_double_var_minus*np.sin(2.*psi_mean)

    return e_cos, e_sin, e_cs, e_csqr, e_ssqr

def expectationTheta_closeForm(params, n_approx):
    """ 
        Compute expectation relative to theta in closed-form

        Parameters
        ----------
        params : dict 
                 dict containing the parameter ["theta"], which itself contains ["alpha"],["beta"] and ["b"].
                 The corresponding values can be arrays.
        
        Returns
        -------
        e_cos : float / array
                expectation of cos(theta). If the input contains arrays, it is an array.
        e_sin : float / array
                expectation of sin(theta). If the input contains arrays, it is an array.  
        e_cs : float / array
                expectation of cos(pthetahi)*sin(theta). If the input contains arrays, it is an array. 
        e_csqr : float / array
                expectation of cos^2(theta). If the input contains arrays, it is an array. 
        e_sqr : float / array
                expectation of sin^2(theta). If the input contains arrays, it is an array. 

        Notes
        -----
        Computation is explained in [1], section 5.4 Proposition 3     
    """

    alpha = params['theta']['alpha']
    beta = params['theta']['beta']
    b = params['theta']['b']
    half_b = 0.5 * b

    # Compute the hypergeometric function terms F_12(-2n,alpha, alpha+beta;2) 
    F_12_cos = scp.special.hyp2f1(-2. * np.arange(0, n_approx, step=1), alpha, alpha + beta, 2)

    # Compute the hypergeometric function terms F_12(-2n-1,alpha, alpha+beta;2) 
    F_12_sin = scp.special.hyp2f1(-2. * np.arange(0, n_approx, step=1) - np.ones((n_approx,)), alpha, alpha + beta, 2)

    # Compute expectations relative to theta
    e_cos = 0
    e_sin = 0
    e_cs = 0
    e_csqr = 1.
    for i in range(0, n_approx):
        if (i % 2 == 0):
            sign = 1
        else:
            sign = -1
        a_pair = sign / np.math.factorial(2. * i)
        a_impair = sign / np.math.factorial(2. * i + 1.)
        c = half_b ** (2. * i)
        d = b ** (2 * i)
        e_cos += a_pair * c * F_12_cos[i]
        e_sin += a_impair * c * F_12_sin[i]
        e_cs += a_impair * d * F_12_sin[i]
        if(i > 0):
            e_csqr += 0.5 * a_pair * d * F_12_cos[i]

    e_sin *= -half_b
    e_cs *= -half_b
    e_sqr = 1. - e_csqr

    return e_cos, e_sin, \
           e_cs, e_csqr,\
           e_sqr

def approximateToGaussian_closedForm(params):
    """ 
        Approximate the local cartesian coords by a Gaussian using closed-form expressions 

        Parameters
        ----------
        params : dict 
                 dict containing :  - the parameter ["rho"], which itself contains ["mean"] and ["std"].
                                    - the parameter ["psi"], which itself contains ["mean"] and ["std"].
                                    - the parameter ["theta"], which itself contains ["alpha"],["beta"] and ["b"].
                 The corresponding values can be arrays.

        Returns
        -------
        dict 
            3D point gaussian pdf ie mean and cov 

        Notes
        -----
        Computation is explained in [1], section 5.4
    """
    # Number of terms in the series to take 
    n_approx = 5 

    # Expectations relative to theta (sonar elevation angle)
    e_c_t, e_s_t, e_cs_t, e_csqr_t, e_ssqr_t = expectationTheta_closeForm(params, n_approx)

    # Expectations relative to psi (sonar rotation angle)
    e_c_p, e_s_p, e_cs_p, e_csqr_p, e_ssqr_p = expectationPsi_closeForm(params)

    # Sonar range
    rho_mean = params["rho"]["mean"]
    rho_var = params["rho"]["std"]**2
    mean = rho_mean*np.array([e_c_t * e_c_p,
                              e_c_t * e_s_p,
                              e_s_t])

    # [1], equations (81-89)
    rho_mean_sqr = rho_mean**2
    e_rho_sqr = rho_mean_sqr + rho_var
    e_c_t_sqr = e_c_t**2
    sigma_xx = e_rho_sqr*e_csqr_p*e_csqr_t - rho_mean_sqr*(e_c_p**2)*e_c_t_sqr
    sigma_yy = e_rho_sqr*e_ssqr_p*e_csqr_t - rho_mean_sqr*(e_s_p**2)*e_c_t_sqr
    sigma_zz = e_rho_sqr*e_ssqr_t - rho_mean_sqr*(e_s_t**2)
    sigma_xy = e_rho_sqr*e_cs_p*e_csqr_t - rho_mean_sqr*e_c_p*e_s_p*e_c_t_sqr
    sigma_xz = e_rho_sqr*e_c_p*e_cs_t - rho_mean_sqr*e_c_p*e_c_t*e_s_t
    sigma_yz = e_rho_sqr*e_s_p*e_cs_t - rho_mean_sqr*e_s_p*e_c_t*e_s_t
    cov = np.array([[sigma_xx, sigma_xy, sigma_xz],
                    [sigma_xy, sigma_yy, sigma_yz],
                    [sigma_xz, sigma_yz, sigma_zz]
                   ])

    return {'mean': mean, 'cov': cov}
