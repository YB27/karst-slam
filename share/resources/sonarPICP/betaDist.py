import numpy as np

"""
    Fonctions related to the beta distribution

    References
    ----------
    .. [1] https://en.wikipedia.org/wiki/Beta_distribution
    .. [2] Breux, Yohan, and Lionel Lapierre. 
           "Elevation Angle Estimations of Wide-Beam Acoustic Sonar Measurements 
            for Autonomous Underwater Karst Exploration." Sensors 20.14 (2020): 4028.
"""

def beta_mode(alpha, beta, b):
    """ 
        Mode of a scaled Beta distribution (translation -b/2, scale b) with parameters alpha,beta 

        Parameters
        ----------
        alpha : float
                Parameter of the beta distribution. Must be > 0.
        beta : float
               Parameter of the beta distribution. Must be > 0.
        b : float
            scale (corresponds to the angular overture of the sonar scan)

        Returns
        -------
        float
              Mode of the scaled beta distribution

        Notes
        -----
        See [1] for details on the beta distribution 
    """

    res = 0.
    if(alpha > 1 and beta > 1):
        res = 0.5*b*(alpha - beta)/(alpha + beta - 2.)
    elif(alpha <= 1 and beta > 1):
        res = -0.5*b + 1e-3
    elif(beta <= 1 and alpha > 1):
        res = 0.5*b - 1e-3
    elif(alpha < 1 and beta < 1):
        print("Error : theta distribution is bimodal and should not happen !")
    return res

def alphaBetaFromMeanVar(theta_mean, theta_var, b):
    """ 
        Compute the parameters alpha,beta of a scaled Beta distribution (translation -b/2, scale b) 
        from its mean and variance.

        Parameters
        ----------
        theta_mean : float
                     Mean value
        theta_var : float
                    Variance. Must be > 0
        b : float
            scale (corresponds to the angular overture of the sonar scan)

        Returns
        -------
        theta_alpha : float
                      Parameter alpha of a beta distribution (Must be > 0)
        theta_beta : float
                     Parameter beta of a beta distribution (Must be >0)
        
        Notes
        -----
        See [2], equations (40)(41)
    """
    
    theta_k = (b ** 2 - 4. * (theta_mean ** 2 + theta_var)) / (8. * b * theta_var)
    theta_alpha = (b + 2. * theta_mean) * theta_k
    theta_beta = (b - 2. * theta_mean) * theta_k
    return theta_alpha, theta_beta

def alphaBetaFromModeVar(theta_mode, theta_var, b):
    """ 
        Compute the parameters alpha,beta of a scaled Beta distribution (translation -b/2, scale b) 
        from its mode and variance.  
    
        Parameters
        ----------
        theta_mode : float
                     Mode value
        theta_var : float
                    Variance. Must be > 0
        b : float
            scale (corresponds to the angular overture of the sonar scan)

        Returns
        -------
        alpha : float
                      Parameter alpha of a beta distribution (Must be > 0)
        beta : float
                     Parameter beta of a beta distribution (Must be >0)
    """

    b_sqr = b**2
    k = b - 2.*theta_mode
    a = k**2
    coeffs = [8.*theta_var*b**3,
              b_sqr*( 4.*theta_var*(b - 14.*theta_mode) - (b + 2.*theta_mode)*a ),
              4.*theta_mode*b*( 4.*theta_var*(8.*theta_mode - b) + b*a ),
              16.*theta_var*(b - 6.*theta_mode)*theta_mode**2]

    # Beta is the real root 
    beta  = -1
    alpha = -1
    roots = np.roots(coeffs)
    for r in roots:
        # Do not use isComplex to check for real value 
        # (numerical approximation can lead to complex with near 0 imaginary part) 
        if(np.abs(r.imag) < 1e-6 and r.real > 1):
            # To keep the right solution, check that it gives back the same mean and variance
            beta_ = r.real
            alpha_ = ((b + 2. * theta_mode) * beta_ - 4. * theta_mode) / k
            recomputed_mode = 0.5 * b * (alpha_ - beta_) / (alpha_ + beta_ - 2)
            recomputed_var = (alpha_ * beta_ * b ** 2) / ((alpha_ + beta_ + 1) * (alpha_ + beta_) ** 2)
            if(np.abs(theta_mode - recomputed_mode) < 1e-5 and
               np.abs(theta_var - recomputed_var) < 1e-5) :
                beta = beta_
                alpha = alpha_
                break
    if(beta == -1):
        print(" !!! No valid roots in alphaBetaFromModeVar")

    return alpha, beta