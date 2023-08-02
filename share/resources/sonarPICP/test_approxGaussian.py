import numpy as np
import scipy.stats
import approxGaussian

def expectationTheta_monteCarlo(params):
    """ 
        Compute expectation for theta (scale beta distribution) with monte carlo (for testing purpose) 

        Parameters
        ----------
        params : dict
                 contains data on the pdf of point expressed in local spherical coords

        Returns
        -------
        e_c : float
              expectation of cos(theta), theta following a scaled beta distribution
        e_s : float
              expectation of sin(theta), theta following a scaled beta distribution
        e_cs : float
               expectation of cos(theta)*sin(theta), theta following a scaled beta distribution
        e_csqr : float
                 expectation of cos(theta)^2, theta following a scaled beta distribution
        e_ssqr : float
                 expectation of sin(theta)^2, theta following a scaled beta distribution
    """

    # Generate samples 
    n_samples = 50000000
    alpha = params["theta"]["alpha"]
    beta  = params["theta"]["beta"]
    b = params["theta"]["b"]
    theta_samples = scipy.stats.uniform.rvs(loc=-0.5*b,scale=b,size=n_samples)
    f_theta_samples =  scipy.stats.beta.pdf(theta_samples,a=alpha, b=beta, loc= -0.5*b, scale = b)

    cos_theta = np.cos(theta_samples)
    sin_theta = np.sin(theta_samples)
    cos_sin_theta = np.multiply(cos_theta,sin_theta)
    cos_sqr_theta = np.multiply(cos_theta,cos_theta)
    sin_sqr_theta = np.multiply(sin_theta,sin_theta)

    num = 1./np.sum(f_theta_samples)
    e_c = np.dot(cos_theta,f_theta_samples)*num
    e_s = np.dot(sin_theta,f_theta_samples)*num
    e_cs = np.dot(cos_sin_theta, f_theta_samples)*num
    e_csqr = np.dot(cos_sqr_theta, f_theta_samples)*num
    e_ssqr = np.dot(sin_sqr_theta, f_theta_samples)*num

    return e_c, e_s, e_cs, e_csqr, e_ssqr

def expectationPsi_monteCarlo(params):
    """
        Compute expectation for psi (gaussian distribution) with monte carlo (for testing purpose) 

        Parameters
        ----------
        params : dict
                 contains data on the pdf of point expressed in local spherical coords

        Returns
        -------
        e_c : float
              expectation of cos(psi), psi following a gaussian distribution
        e_s : float
              expectation of sin(psi), psi following a gaussian distribution
        e_cs : float
               expectation of cos(psi)*sin(psi), psi following a gaussian distribution
        e_csqr : float
                 expectation of cos(psi)^2, psi following a gaussian distribution
        e_ssqr : float
                 expectation of sin(psi)^2, psi following a gaussian distribution
    """

    n_samples = 100000000
    psi_mean = params["psi"]["mean"]
    psi_std = params["psi"]["std"]
    psi_samples = scipy.stats.uniform.rvs(loc=-10.*psi_std,scale=20.*psi_std,size=n_samples)
    f_psi_samples = scipy.stats.norm.pdf(psi_samples, loc=psi_mean, scale=psi_std)

    cos_p = np.cos(psi_samples)
    sin_p = np.sin(psi_samples)
    cos_sin_p = np.multiply(cos_p, sin_p)
    cos_p_sqr = np.multiply(cos_p, cos_p)
    sin_p_sqr = np.multiply(sin_p, sin_p)

    num = 1./np.sum(f_psi_samples)
    e_c = np.dot(cos_p, f_psi_samples)*num
    e_s = np.dot(sin_p, f_psi_samples)*num
    e_cs = np.dot(cos_sin_p, f_psi_samples)*num
    e_csqr = np.dot(cos_p_sqr, f_psi_samples)*num
    e_ssqr = np.dot(sin_p_sqr, f_psi_samples)*num

    return e_c, e_s, e_cs, e_csqr, e_ssqr


def test_expectationsTheta():
    """
        Test the functions computing the expectations of trigonometric function with theta 
    """

    params = {'rho': {'mean': 49.1004, 'std': 0.2},
                  'theta': {'alpha': 14.4, 'beta': 105.6, 'b': np.deg2rad(35)},
                  'psi': {'mean': 0.4, 'std': 0.1}
            }
    
    print("Computing monte carlo expectations ....")
    mc_ec, mc_es, mc_ecs, mc_ecsqr, mc_essqr = expectationTheta_monteCarlo(params)
    print("Finished")
    ec_3, es_3, ecs_3, ecsqr_3, essqr_3 = approxGaussian.expectationTheta_closeForm(params, n_approx=3)
    ec_5, es_5, ecs_5, ecsqr_5, essqr_5 = approxGaussian. expectationTheta_closeForm(params, n_approx=5)
    ec_10, es_10, ecs_10, ecsqr_10, essqr_10 = approxGaussian.expectationTheta_closeForm(params, n_approx=100)

    print("---> Expections Theta computed with MC / Expectations closedForm with approx 3,5,10 : ")
    print(" E(cos(theta)) = {} / {} , {} , {}".format(mc_ec, ec_3, ec_5, ec_10))
    print(" E(sin(theta)) = {} / {} , {} , {}".format(mc_es, es_3, es_5, es_10))
    print(" E(cos(theta)sin(theta)) = {} / {} , {} , {}".format(mc_ecs, ecs_3, ecs_5, ecs_10))
    print(" E(cos^2(theta)) = {} / {} , {} , {}".format(mc_ecsqr, ecsqr_3, ecsqr_5, ecsqr_10))
    print(" E(sin^2(theta)) = {} / {} , {} , {}".format(mc_essqr, essqr_3, essqr_5, essqr_10))
    
    assert np.allclose(np.array([mc_ec, mc_es, mc_ecs, mc_ecsqr, mc_essqr]), 
                       np.array([ec_5, es_5, ecs_5, ecsqr_5, essqr_5]), atol=1e-5)

def test_expectationsPsi():
    """
        Test the functions computing the expectations of trigonometric function with psi
    """

    params = {'rho': {'mean': 5., 'std': 0.2},
                  'theta': {'alpha': 3.2, 'beta': 2.2, 'b': np.deg2rad(35)},
                  'psi': {'mean': 0.4, 'std': 0.1}
                  }
    print("Computing monte carlo expectations ....")
    mc_ec, mc_es, mc_ecs, mc_ecsqr, mc_essqr = expectationPsi_monteCarlo(params)
    print("Finished")
    ec, es, ecs, ecsqr, essqr = approxGaussian.expectationPsi_closeForm(params)

    print("---> Expections Psi computed with MC / Expectations closedForm : ")
    print(" E(cos(psi)) = {} / {}".format(mc_ec, ec))
    print(" E(sin(psi)) = {} / {}".format(mc_es, es))
    print(" E(cos(psi)sin(psi)) = {} / {}".format(mc_ecs, ecs))
    print(" E(cos^2(psi)) = {} / {}".format(mc_ecsqr, ecsqr))
    print(" E(sin^2(psi)) = {} / {}".format(mc_essqr, essqr))

    assert np.allclose(np.array([mc_ec, mc_es, mc_ecs, mc_ecsqr, mc_essqr]),
                       np.array([ec, es, ecs, ecsqr, essqr]), atol = 1e-5)

# ------------- MAIN ---------------------
if __name__ == "__main__":
    test_expectationsTheta()
    test_expectationsPsi()