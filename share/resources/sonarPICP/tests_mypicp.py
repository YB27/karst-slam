import mypicp_variante as mypicp
import math_utility
import betaDist
import poses_euler
import time
import scipy.spatial
import approxGaussian
import numpy as np

picp = mypicp.my_picp(8)

b = np.deg2rad(35.)
x_test_sphe = [5.,0.05,0.2]
x_test_cart = math_utility.fromSphericalToCart(x_test_sphe)
alpha,beta = betaDist.alphaBetaFromMeanVar(x_test_sphe[1], 0.02,b)
q_test = [0.01, -0.02, 0.01, 0.01, 0.01, 0.01]

std_angle = 0.01
std_xyz = 0.01
var_angle = std_angle**2
var_xyz = std_xyz**2
q_cov_test = np.array([[var_angle , 0.   , 0.   , 0.   , 0.   , 0.],
                       [0.   , var_angle , 0.   , 0.   , 0.   , 0.],
                       [0.   , 0.   , var_angle , 0.   , 0.   , 0.],
                       [0.   , 0.   , 0.   , var_xyz , 0.   , 0.],
                       [0.   , 0.   , 0.   , 0.   , var_xyz , 0.],
                       [0.   , 0.   , 0.   , 0.   , 0.   , var_xyz]])
params_prime_test = {'rho': {'mean': x_test_sphe[0], 'std': 0.1},
               'theta': {'alpha': alpha, 'beta' : beta, 'b': b},
               'psi' : {'mean' : x_test_sphe[2], 'std' : 0.1},
               'pose_mean' : q_test, 'pose_cov' : q_cov_test
              }

x_prime_test_sphe = [5.5, 0.03, 0.1]
x_prime_test_cart = math_utility.fromSphericalToCart(x_prime_test_sphe)
alpha_prime,beta_prime = betaDist.alphaBetaFromMeanVar(x_prime_test_sphe[1], 0.02, b)
q_prime_test = q_test #[0.02, 0.01, -0.01, -0.01, 0.02, -0.01]
params_test = {'rho': {'mean': x_prime_test_sphe[0], 'std': 0.1},
               'theta': {'alpha': alpha_prime, 'beta' : beta_prime, 'b': b},
               'psi' : {'mean' : x_prime_test_sphe[2], 'std' : 0.1},
               'pose_mean': q_prime_test, 'pose_cov': q_cov_test}

mode_x_test = picp.getMode_f_X(params_test)
mode_x_prime_test = picp.getMode_f_X(params_prime_test)

"""
    Test concerning pICP without gaussian approximation
    DEPRECATED : based on old version of mypicp ....
    Majority of code now use functiions ****_opt_array wihtn input arguments replaced by class member
    so that we can't directly use numericalJacobian easily

    TODO: rewrite test using the current mypicp impl.

    References
    ----------
    .. [1] "doc/MpIC_oldVersion_noApprox.pdf"
"""

def testJ_f_spherical():
    """
        Test the jacobian of f_spherical (see picp.f_spherical, picp.J_f_spherical)

        Notes
        -----
        See [1], section 5.3.6 equation (113)
    """

    x_minus_q = poses_euler.inverseComposePoseEulerPoint(q_test,x_test_cart)
    localSpheCord = math_utility.fromCartToSpherical(x_minus_q)
    f_s = picp.f_spherical(localSpheCord, params_test)
    J_closedForm = picp.J_f_spherical(f_s, localSpheCord, params_test)

    dincr = 1e-4
    J_num = math_utility.numericalJacobian(lambda x: picp.f_spherical_opt_array(x, params_test, "X"), 1, localSpheCord,
                                           np.full((3,),dincr))
    print(" ----  Test J_f_s ---- ")
    print("J_closedForm : ")
    print(J_closedForm)
    print("J_num : ")
    print(J_num)
    print(" --------------------- ")

    assert np.allclose(J_closedForm, J_num, atol=1e-5)

def testJ_f_spherical_opt():
    """
        Test the jacobian of f_spherical_opt (see picp.f_spherical_opt, picp.J_f_spherical_opt)
        (_opt indicates parallelize)

        Notes
        -----
        See [1], section 5.3.6 equation (113)
    """

    q_arr = np.tile(q_test, (3,1))
    x_minus_q = poses_euler.inverseComposePoseEulerPoint_opt(q_arr, x_test_cart)
    localSpheCord = picp.fromCartToSpherical_opt_array(x_minus_q, "X")
    f_s = picp.J_f_spherical_opt_array(localSpheCord, params_test)
    J_closedForm_opt = picp.J_f_spherical_opt_array(f_s,localSpheCord, params_test)
    J_closedForm = picp.J_f_spherical(f_s[0], localSpheCord[0], params_test)

    print(" ---- Test J_f_s_opt ---- ")
    print("J_closedForm_opt : ")
    print(J_closedForm_opt)
    print("J_closedForm : ")
    print(J_closedForm)
    print(" --------------------- ")

    assert np.allclose(J_closedForm_opt, J_closedForm, atol=1e-5)

def testJ_g_inverse():
    """
        Test the jacobian of g inverse

        Notes
        -----
        g is defined in [1], equation (47)
    """

    J_closedForm = picp.J_g_inverse(x_test_cart, np.linalg.norm(x_test_cart))

    dincr = 1e-4
    J_num = math_utility.numericalJacobian(lambda x: picp.fromCartToSpherical(x), 3, x_test_cart,
                                           [dincr, dincr, dincr])
    print(" ---- Test J_g_inverse ---- ")
    print("J_closedForm : ")
    print(J_closedForm)
    print("J_num : ")
    print(J_num)
    print(" --------------------- ")

    assert np.allclose(J_closedForm, J_num, atol=1e-5)

def testJ_g_inverse_opt():
    """
        Test the jacobian of g inverse (version parallelize)

        Notes
        -----
        g is defined in [1], equation (47)
    """

    x = np.tile(x_test_cart, (2, 1))
    J_closedForm = picp.J_g_inverse(x_test_cart, np.linalg.norm(x_test_cart))

    J_closedForm_opt = picp.J_g_inverse_opt(x, np.linalg.norm(x,axis=1))

    print(" ---- Test J_g_inverse ---- ")
    print("J_closedForm : ")
    print(J_closedForm)
    print("J_closedForm_opt : ")
    print(J_closedForm_opt)
    print(" --------------------- ")

    assert np.allclose(J_closedForm, J_closedForm_opt, atol=1e-8)

def testJ_detJ():
    """
        Test the jacobian of determinant of g^-1

        Notes
        -----
        Defined in [1], equation (114)
    """

    J_closedForm = picp.J_detJ(x_test_cart, np.linalg.norm(x_test_cart))

    dincr = 1e-4
    J_num = math_utility.numericalJacobian(lambda x: picp.jacobianDeterminant(x), 1, x_test_cart,
                                           [dincr, dincr, dincr])
    print(" ---- Test J_detJ ---- ")
    print("J_closedForm : ")
    print(J_closedForm)
    print("J_num : ")
    print(J_num)
    print(" --------------------- ")

    assert np.allclose(J_closedForm, J_num, atol=1e-5)

def testJ_detJ_opt():
    """
        Test the jacobian of determinant of g^-1 (version parralellize)

        Notes
        -----
        Defined in [1], equation (114)
    """

    x = np.tile(x_test_cart, (2, 1))
    J_closedForm = picp.J_detJ(x_test_cart, np.linalg.norm(x_test_cart))
    J_closedForm_opt = picp.J_detJ_opt(x,np.linalg.norm(x,axis=1))

    print(" ---- Test J_detJ_opt ---- ")
    print("J_closedForm : ")
    print(J_closedForm)
    print("J_closedForm_opt : ")
    print(J_closedForm_opt)
    print(" --------------------- ")

    assert np.allclose(J_closedForm, J_closedForm_opt, atol=1e-8)

def testJ_h():
    """
        Test the jacobian of H

        Notes
        -----
        H is defined in [1], equation (80)
        J_H is defined in [1], equation (112) 
    """

    print("x cart : {}".format(x_test_cart))
    print("x sphe : {}".format(x_test_sphe))
    print("params : {}".format(params_test))
    J_closedForm = picp.J_h(x_test_cart, q_test, params_test)
    dincr = 1e-4
    J_num = math_utility.numericalJacobian(lambda x: picp.h(x,q_test,params_test), 1, x_test_cart, [dincr, dincr, dincr])
    print(" ---- Test J_h ---- ")
    print("J_closedForm : ")
    print(J_closedForm)
    print("J_num : ")
    print(J_num)
    print(" --------------------- ")

    assert np.allclose(J_closedForm, J_num, atol=1e-5)

def testJ_h_opt():
    """
        Test the jacobian of H (version parralellize)

        Notes
        -----
        H is defined in [1], equation (80)
        J_H is defined in [1], equation (112) 
    """

    q_arr = np.tile(q_test, (2,1))
    J_closedForm = picp.J_h(x_test_cart, q_test, params_test)
    J_closedForm_opt = picp.J_h_opt(x_test_cart, q_arr, params_test)

    print(" ---- Test J_h_opt ----- ")
    print("J_closedForm : ")
    print(J_closedForm)
    print("J_closedForm_opt : ")
    print(J_closedForm_opt)
    print(" --------------------- ")

    assert np.allclose(J_closedForm, J_closedForm_opt, atol=1e-8)


def testJ_f_X():
    """
        Test the jacobian of f_X

        Notes
        -----
        f_X is defined in [1], equation (79)
        J_f_X is defined in [1] , equation (111)
    """

    J_closedForm = picp.J_f_X(x_test_cart, params_test)

    dincr = 1e-4
    J_num = math_utility.numericalJacobian(lambda x: picp.f_X(x,params_test), 1, x_test_cart,
                                           [dincr, dincr, dincr])
    print(" ---- Test J_f_X ---- ")
    print("J_closedForm : ")
    print(J_closedForm)
    print("J_num : ")
    print(J_num)
    print(" --------------------- ")

    assert np.allclose(J_closedForm, J_num, atol=1e-5)


def testJ_f_X_opt():
    """
         Test the jacobian of f_X (version parralellize)

        Notes
        -----
        f_X is defined in [1], equation (79)
        J_f_X is defined in [1] , equation (111)
    """

    picp.J_f_X(x_test_cart, params_test)
    picp.J_f_X_opt(x_test_cart, params_test)

    start = time.time()
    J_closedForm = picp.J_f_X(x_test_cart, params_test)
    comp_time = time.time() - start

    start = time.time()
    J_closedForm_opt = picp.J_f_X_opt(x_test_cart, params_test)
    comp_time_opt = time.time() - start

    print(" ---- Test J_f_X_opt ---- ")
    print("J_closedForm with time {}".format(comp_time))
    print(J_closedForm)
    print("J_closedForm_opt with time {}".format(comp_time_opt))
    print(J_closedForm_opt)
    print(" --------------------- ")

    assert np.allclose(J_closedForm, J_closedForm_opt, atol=1e-8)

def testJ_Fi():
    """
        Test the jacobian of cost function for the i-th term

        Notes
        -----
        Cost function is defined in [1], equation (88)
    """

    val_max = picp.f_X(mode_x_test, params_test) * picp.f_X(mode_x_prime_test, params_prime_test)
    fX = picp.f_X(mode_x_prime_test, params_test)
    fXprime = picp.f_X(mode_x_test, params_prime_test)
    print("fX, fXprime : {} , {}".format(fX,fXprime))
    print("fModeX, fModeXprime : {} , {}".format(picp.f_X(mode_x_test, params_test), picp.f_X(mode_x_prime_test, params_prime_test)))

    J_closedForm = picp.J_Fi(mode_x_test, mode_x_prime_test, params_test, params_prime_test,fX,fXprime, val_max)

    dincr = 1e-4
    eps_null = np.zeros((6,))
    J_num = math_utility.numericalJacobian(lambda x: picp.Fi(mode_x_test, mode_x_prime_test, params_test, params_prime_test, val_max, x), 1, eps_null,
                                           [dincr, dincr, dincr, dincr, dincr, dincr])
    print(" ---- Test J_Fi ---- ")
    print("J_closedForm : ")
    print(J_closedForm)
    print("J_num : ")
    print(J_num)
    print(" --------------------- ")

    assert np.allclose(J_closedForm, J_num, atol=1e-5)

def testJ_Fi_opt():
    """
        Test the jacobian of cost function for the i-th term (version parralellize)

        Notes
        -----
        Cost function is defined in [1], equation (88)
    """

    val_max = picp.f_X(mode_x_test, params_test) * picp.f_X(mode_x_prime_test, params_prime_test)
    fX = picp.f_X(mode_x_prime_test, params_test)
    fXprime = picp.f_X(mode_x_test, params_prime_test)
    print("fX, fXprime : {} , {}".format(fX,fXprime))
    print("fModeX, fModeXprime : {} , {}".format(picp.f_X(mode_x_test, params_test), picp.f_X(mode_x_prime_test, params_prime_test)))

    J_closedForm = picp.J_Fi(mode_x_test, mode_x_prime_test, params_test, params_prime_test,fX,fXprime,val_max)

    dincr = 1e-4
    eps_null = np.zeros((6,))
    J_num = math_utility.numericalJacobian(lambda x: picp.Fi(mode_x_test, mode_x_prime_test, params_test, params_prime_test, val_max,x), 1, eps_null,
                                           [dincr, dincr, dincr, dincr, dincr, dincr])
    print(" ---- Test J_Fi_opt ---- ")
    print("J_closedForm : ")
    print(J_closedForm)
    print("J_num : ")
    print(J_num)
    print(" --------------------- ")

    assert np.allclose(J_closedForm, J_num, atol=1e-5)


def testJ_F():
    """
        Test the jacobian of cost function

        Notes
        -----
        Cost function is defined in [1], equation (88)
    """

    mode_x = picp.getMode_f_X(params_test)
    mode_x_prime = picp.getMode_f_X(params_prime_test)
    fX = picp.f_X(mode_x_prime, params_test)
    fXprime = picp.f_X(mode_x, params_prime_test)
    mode_x_arr = [mode_x,mode_x]
    mode_x_prime_arr = [mode_x_prime, mode_x_prime]
    params_test_arr = [params_test, params_test]
    params_prime_test_arr = [params_prime_test, params_prime_test]
    fX_arr = [fX,fX]
    fXprime_arr = [fXprime, fXprime]
    J_closedForm = picp.J_F(mode_x_arr,
                              mode_x_prime_arr,
                              params_test_arr,
                              params_prime_test_arr,
                              fX_arr,
                              fXprime_arr)

    dincr = 1e-4
    eps_null = np.zeros((6,))
    J_num = math_utility.numericalJacobian(lambda x: picp.F(mode_x_arr, mode_x_prime_arr, params_test_arr, params_prime_test_arr, x), 2, eps_null,
                                           [dincr, dincr, dincr, dincr, dincr, dincr])
    print(" ---- Test J_F ---- ")
    print("J_closedForm : ")
    print(J_closedForm)
    print("J_num : ")
    print(J_num)
    print(" --------------------- ")

    assert np.allclose(J_closedForm, J_num, atol=1e-5)

def testComputationTime():
    n = 100
    q_cut_samples = np.array([[0.5,-0.2,0.3,1.,2.,3.]] *n)
    x = np.array([1,1,1])
    params_arr = params_test

    picp.J_compose_point(q_cut_samples[0])

    start = time.time()
    for q_ in q_cut_samples:
        y = picp.J_compose_point(q_)
        #print("y : {}".format(y))
    comp_time_g = time.time() - start
    print("time : {}".format(comp_time_g))

    picp.J_compose_point_opt(q_cut_samples)

    start = time.time()
    y_opt = picp.J_compose_point_opt(q_cut_samples)
    comp_time_g_opt = time.time() - start
    #print("yopt: {}".format(y_opt))
    print("time : {}".format(comp_time_g_opt))


    '''start = time.time()
    picp.h(x,q,params_test)
    comp_time_h = time.time() - start

    start = time.time()
    np.cos(np.linalg.norm(x))
    comp_time_cos = time.time() - start

    print("cos norm compute time, cut-8 : {} , {}".format(comp_time_cos, comp_time_cos*745.))
    print("h compute time, cut-8 : {} , {}".format(comp_time_h, comp_time_h*745.))'''


def generateDataForTest(n_points):
    """ 
        Generate the params_array with n_points 

        Parameters
        ----------
        n_points : float
                   number of points to generate
        
        Returns
        -------
        x_array : (n_points, 3) ndarray
                  points generated
        params_array : dict of array
                       contains data on the local spherical pdf of the points
    """

    params_array = {"pose_mean": [], "pose_cov": []}
    params_array["pose_mean"] = np.random.rand(n_points, 6)
    cov = np.eye(6)
    params_array["pose_cov"] = np.empty((n_points, 6, 6))
    for k in range(0, n_points):
        params_array["pose_cov"][k, :, :] = cov
    params_array["rho"] = {"mean": np.random.rand(n_points), "std": np.random.rand(n_points)}
    params_array["psi"] = {"mean": np.random.rand(n_points), "std": np.random.rand(n_points)}
    one_array = np.empty(n_points)
    one_array.fill(1.)
    b_array = 1.3 * one_array
    params_array["theta"] = {"alpha": one_array, "beta": one_array, "b": b_array}
    # print("params_array : ")
    # print(params_array)

    # Generate the x_array 
    x_array = np.random.rand(n_points, 3)

    return x_array, params_array


def testf_x_opt():
    """
        Compare computation time between parrallelize and non parrallelize version
    """

    n_points = 200

    # Generate the x_array and params_array with n_points 
    x_array, params_array = generateDataForTest(n_points)

    p = my_picp(8)
    start = time.time()
    res = p.f_X_array(x_array, params_array)
    print("f_x_opt results in {} s : ".format(time.time() - start))
    # print(res)

    # Compare with the non-array version 
    params = []
    for k in range(0, n_points):
        params.append({"rho": {"mean": params_array["rho"]["mean"][k], "std": params_array["rho"]["std"][k]},
                       "theta": {"alpha": params_array["theta"]["alpha"][k], "beta": params_array["theta"]["beta"][k],
                                 "b": params_array["theta"]["b"][k]},
                       "psi": {"mean": params_array["psi"]["mean"][k], "std": params_array["psi"]["std"][k]},
                       "pose_mean": params_array["pose_mean"][k], "pose_cov": params_array["pose_cov"][k]
                       })

    # print("Params : ")
    # print(params)

    start = time.time()
    for k in range(0, n_points):
        res = p.f_X(x_array[k, :], params[k])
        # print(res)
    print("f_x results in {} s".format(time.time() - start))


def testFunctionCost():
    """
        Compare computation time between parrallelize and non parrallelize version
    """

    n_points = 200

    # Generate the x_array and params_array with n_points
    x_array, params_array = generateDataForTest(n_points)
    vals_max = np.random.rand(n_points)

    p = picp.my_picp(8)

    start = time.time()
    vals, _, _ = p.functionCost_array(x_array, x_array, params_array, params_array, vals_max)
    comp_time = time.time() - start
    print("Comp time functionCost_array : {}".format(comp_time))
    # print("vals : ")
    # print(vals)

    # Compare with the non-array version 
    params = []
    for k in range(0, n_points):
        params.append({"rho": {"mean": params_array["rho"]["mean"][k], "std": params_array["rho"]["std"][k]},
                       "theta": {"alpha": params_array["theta"]["alpha"][k], "beta": params_array["theta"]["beta"][k],
                                 "b": params_array["theta"]["b"][k]},
                       "psi": {"mean": params_array["psi"]["mean"][k], "std": params_array["psi"]["std"][k]},
                       "pose_mean": params_array["pose_mean"][k], "pose_cov": params_array["pose_cov"][k]
                       })
    start = time.time()
    vals, _, _ = p.functionCost(x_array, x_array, params, params, vals_max)
    comp_time = time.time() - start
    print("Comp time functionCost : {}".format(comp_time))
    # print("vals : ")
    # print(vals)


def testJf():
    """
        Test the
    """

    n_points = 200

    # Generate the x_array and params_array with n_points
    x_array, params_array = generateDataForTest(n_points)
    xprime_array, params_prime_array = generateDataForTest(n_points)
    # params_prime_array = params_array

    p = my_picp(8)
    q_array = p.CUT.convertSamplesToPDF_array(params_array["pose_mean"], params_array["pose_cov"])
    q_array_prime = p.CUT.convertSamplesToPDF_array(params_prime_array["pose_mean"], params_prime_array["pose_cov"])
    # print("q_array shape: {}".format(q_array.shape))
    # print(q_array)

    # Calls to compute cached values 
    p.initCUT_array(params_array, params_prime_array)
    f_X_array = p.f_X_array(xprime_array, params_array, "X")
    f_X_prime_array = p.f_X_array(x_array, params_prime_array, "X_prime")
    p.J_g_inverse_opt_array("X")  # cache value self.k
    p.J_g_inverse_opt_array("X_prime")
    p.J_composePose["X"] = -p.J_compose_pose_array(x_array)
    p.J_composePose["X_prime"] = p.J_compose_pose_array(xprime_array)

    p_poses = []
    p_prime_poses = []
    for pose_mean_i, cov_q_i, pose_mean_prime_i, cov_prime_q_i in zip(params_array["pose_mean"],
                                                                      params_array["pose_cov"],
                                                                      params_prime_array["pose_mean"],
                                                                      params_prime_array["pose_cov"]):
        p_i = {'pose_mean': pose_mean_i, 'pose_cov': cov_q_i}
        p_poses.append(p_i)
        p_prime_i = {'pose_mean': pose_mean_prime_i,
                     'pose_cov': cov_prime_q_i}
        p_prime_poses.append(p_prime_i)
    params_current = p.copyParams(params_array)
    params_prime_current = p.copyParams(params_prime_array)

    # Only for J_F 
    vals_max = np.random.rand(n_points)
    q_pose = {'pose_mean': np.random.rand(6), 'pose_cov': np.eye(6)}
    start = time.time()
    p.computeParamsCurrent_array(q_pose, params_array, params_prime_array, params_current, params_prime_current)
    # res = p.J_F_opt_array(params_array, params_prime_array, f_X_array, f_X_prime_array, vals_max)
    # res = p.J_F_array(params_array)
    # res = p.J_h_opt_array(q_array, params_array, "X")
    # res = p.J_detJ_opt_array("X")
    # res = p.J_g_inverse_opt_array()
    # res = p.J_f_spherical_opt_array(params_array)
    comp_time = time.time() - start
    print("Comp time computeParamsCurrent_array : {}".format(comp_time))
    # print("res shape: {}".format(res.shape))
    # print(params_current)

    # Compare with the non-array version
    params = []
    for k in range(0, n_points):
        params.append({"rho": {"mean": params_array["rho"]["mean"][k], "std": params_array["rho"]["std"][k]},
                       "theta": {"alpha": params_array["theta"]["alpha"][k], "beta": params_array["theta"]["beta"][k],
                                 "b": params_array["theta"]["b"][k]},
                       "psi": {"mean": params_array["psi"]["mean"][k], "std": params_array["psi"]["std"][k]},
                       "pose_mean": params_array["pose_mean"][k], "pose_cov": params_array["pose_cov"][k]
                       })
    params_prime = []
    for k in range(0, n_points):
        params_prime.append(
            {"rho": {"mean": params_prime_array["rho"]["mean"][k], "std": params_prime_array["rho"]["std"][k]},
             "theta": {"alpha": params_prime_array["theta"]["alpha"][k], "beta": params_prime_array["theta"]["beta"][k],
                       "b": params_prime_array["theta"]["b"][k]},
             "psi": {"mean": params_prime_array["psi"]["mean"][k], "std": params_prime_array["psi"]["std"][k]},
             "pose_mean": params_prime_array["pose_mean"][k], "pose_cov": params_prime_array["pose_cov"][k]
             })

    comp_time = 0
    '''for k in range(0,n_points):
        #print("q_array slice shape: {}".format(q_array[:,k,:].shape))
        #print(q_array[:,k,:])
        x_minus_q = poses_euler.inverseComposePoseEulerPoint_opt(q_array[:,k,:], xprime_array[k])

        locSphericalCoords = p.fromCartToSpherical_opt(x_minus_q)

        f_s = p.f_spherical_opt(locSphericalCoords, params[k])

        start = time.time()
        #res = p.J_f_X_opt(x_array[k], params[k])
        res = p.J_h_opt(xprime_array[k], q_array[:,k,:], params[k])
        #res = p.J_f_spherical_opt(f_s,locSphericalCoords,params[k])
        #res = p.J_g_inverse_opt(x_minus_q, locSphericalCoords[:,0])
        #res= p.J_detJ_opt(x_minus_q, locSphericalCoords[:,0])
        comp_time += time.time() - start
        print("res shape :{}".format(res.shape))
        print(res)
    print("Comp time J_F_opt : {}".format(comp_time))'''

    params_current = p.copyParams(params_array)
    params_prime_current = p.copyParams(params_prime_array)
    start = time.time()
    p.computeParamsCurrent(p_poses, p_prime_poses, q_pose, params_current, params_prime_current)
    # res = p.J_F_opt(x_array, xprime_array, params, params_prime, f_X_array, f_X_prime_array, vals_max)
    comp_time = time.time() - start
    # print("res shape :{}".format(res.shape))
    # print(params_current)
    print("Comp time computeParamsCurrent : {}".format(comp_time))

def testForAssociation():
    n_points_x = 4
    n_points_xprime = 5
    ''' Generate the x_array and params_array with n_points '''
    x_array, params_array = generateDataForTest(n_points_x)
    xprime_array, params_prime_array = generateDataForTest(n_points_xprime)

    p = mypicp.my_picp(4)
    p.initCUT_array(params_array, params_prime_array)
    f_X_matrix = p.f_X_array_association(xprime_array, params_array, "X")
    f_X_prime_matrix = p.f_X_array_association(x_array, params_prime_array, "X_prime")
    print("f_X_matrix : ")
    print(f_X_matrix)
    print("f_X_prime_matrix : ")
    print(f_X_prime_matrix)

    ''' Compare with the non-array version '''
    params = []
    for k in range(0, n_points_x):
        params.append({"rho": {"mean": params_array["rho"]["mean"][k], "std": params_array["rho"]["std"][k]},
                       "theta": {"alpha": params_array["theta"]["alpha"][k], "beta": params_array["theta"]["beta"][k],
                                 "b": params_array["theta"]["b"][k]},
                       "psi": {"mean": params_array["psi"]["mean"][k], "std": params_array["psi"]["std"][k]},
                       "pose_mean": params_array["pose_mean"][k], "pose_cov": params_array["pose_cov"][k]
                       })
    params_prime = []
    for k in range(0, n_points_xprime):
        params_prime.append(
            {"rho": {"mean": params_prime_array["rho"]["mean"][k], "std": params_prime_array["rho"]["std"][k]},
             "theta": {"alpha": params_prime_array["theta"]["alpha"][k], "beta": params_prime_array["theta"]["beta"][k],
                       "b": params_prime_array["theta"]["b"][k]},
             "psi": {"mean": params_prime_array["psi"]["mean"][k], "std": params_prime_array["psi"]["std"][k]},
             "pose_mean": params_prime_array["pose_mean"][k], "pose_cov": params_prime_array["pose_cov"][k]
             })
    print("f_X : ")
    f_X_vals = np.empty((n_points_x, n_points_xprime))
    for i in range(0, len(params)):
        for j in range(0, len(xprime_array)):
            fX = p.f_X(xprime_array[j], params[i])  # f_Xi(X'j)
            print("i,j : {}".format(str(i) + "," + str(j) + "-->" + str(fX)))
            f_X_vals[i,j] = fX

    print("f_X' : ")
    f_Xprime_vals = np.empty((n_points_xprime, n_points_x))
    for i in range(0, len(x_array)):
        for j in range(0, len(params_prime)):
            fX_prime = p.f_X(x_array[i], params_prime[j])  # f_X'j(Xi)
            print("i,j : {}".format(str(i) + "," + str(j) + "-->" + str(fX_prime)))
            f_Xprime_vals[i,j] = fX_prime
    
    assert np.allclose(f_X_matrix, f_X_vals, atol=1e-5) and \
           np.allclose(f_X_prime_matrix, f_Xprime_vals, atol=1e-5)

def functionCost_forTest(eps, q, picp, modeX, modeXprime, params, params_prime):
    exp_epsilon_R, exp_epsilon_t = math_utility.exp_SE3(eps)
    qincr = np.concatenate((scipy.spatial.transform.Rotation.from_matrix(exp_epsilon_R).as_euler('ZYX'), exp_epsilon_t))
    #qnew = {'pose_mean': poses_euler.composePoseEuler(q['pose_mean'], qincr), 'pose_cov': q['pose_cov']}

    params_current = picp.copyParams(params)
    params_current_prime = picp.copyParams(params_prime)
    picp.computeParamsCurrent_array(q, params, params_prime, params_current, params_current_prime)
    picp.initCUT_array(params_current, params_current_prime)

    modeX_current = poses_euler.inverseComposePoseEulerPoint(q['pose_mean'], modeX)
    modeXprime_current = poses_euler.composePoseEulerPoint(q['pose_mean'], modeXprime)

    modeX_eps = poses_euler.inverseComposePoseEulerPoint(qincr, modeX)
    modeXprime_eps = poses_euler.composePoseEulerPoint(qincr, modeXprime)

    vals_max = picp.computeValsMax(modeX_current, modeXprime_current, params_current,
                                   params_current_prime)
    fval, _, _ = picp.functionCost_array(modeX_eps, modeXprime_eps, params_current, params_current_prime, vals_max)
    return fval

def test_J_F_opt_array():
    n_points = 3
    x_array, params_array = generateDataForTest(n_points)
    xprime_array, params_prime_array = generateDataForTest(n_points)
    q = {'pose_mean': np.random.rand(6), 'pose_cov': np.eye(6)}

    picp = mypicp.my_picp(8)


    params_current = picp.copyParams(params_array)
    params_current_prime = picp.copyParams(params_prime_array)
    picp.computeParamsCurrent_array(q, params_array, params_prime_array, params_current, params_current_prime)
    picp.initCUT_array(params_current, params_current_prime)
    modeX_current = poses_euler.inverseComposePoseEulerPoint(q['pose_mean'], x_array)
    modeXprime_current = poses_euler.composePoseEulerPoint(q['pose_mean'], xprime_array)

    vals_max = picp.computeValsMax(modeX_current, modeXprime_current, params_current,
                                   params_current_prime)
    fval, fX_array, fXprime_array = picp.functionCost_array(x_array, xprime_array, params_current, params_current_prime, vals_max)

    fval_test = functionCost_forTest(np.zeros((6,)), q, picp, x_array, xprime_array, params_array, params_prime_array)

    print("fval : ")
    print(fval)
    print("fval_test :")
    print(fval_test)

    picp.J_composePose["X"] = -picp.J_compose_pose_array(x_array)
    picp.J_composePose["X_prime"] = picp.J_compose_pose_array(xprime_array)
    J_f_comp = picp.J_F_opt_array(params_current, params_current_prime,
                                  fX_array, fXprime_array, vals_max)
    print("J_f_comp : ")
    print(J_f_comp)

    dincr = 1e-5
    eps_null = np.zeros((6,))
    J_num = math_utility.numericalJacobian(
        lambda x: functionCost_forTest(x, q, picp, x_array, xprime_array, params_array, params_prime_array), n_points, eps_null,
        np.full(6, dincr))

    print("J_num : ")
    print(J_num)

    assert np.allclose(J_f_comp, J_num, atol=1e-5)

def testJacobianCombine():
    n_points = 3
    x_array, params_array = generateDataForTest(n_points)
    xprime_array, params_prime_array = generateDataForTest(n_points)
    q = {'pose_mean': np.random.rand(6), 'pose_cov': np.eye(6)}

    p = []
    p_prime = []
    for i in range(0, n_points):
        p.append({"rho" : {"mean": params_array["rho"]["mean"][i], "std":params_array["rho"]["std"][i] },
                  "theta" : {"alpha": params_array["theta"]["alpha"][i], "beta": params_array["theta"]["beta"][i], "b": params_array["theta"]["b"][i]},
                  "psi": {"mean": params_array["psi"]["mean"][i], "std": params_array["psi"]["std"][i]},
                  "pose_mean": params_array["pose_mean"][i], "pose_cov": params_array["pose_cov"][i]})

        p_prime.append({"rho": {"mean": params_prime_array["rho"]["mean"][i], "std": params_prime_array["rho"]["std"][i]},
                  "theta": {"alpha": params_prime_array["theta"]["alpha"][i], "beta": params_prime_array["theta"]["beta"][i],
                            "b": params_prime_array["theta"]["b"][i]},
                  "psi": {"mean": params_prime_array["psi"]["mean"][i], "std": params_prime_array["psi"]["std"][i]},
                  "pose_mean": params_prime_array["pose_mean"][i], "pose_cov": params_prime_array["pose_cov"][i]})

    a_pts = []
    c_pts = []
    for params in p:
        local_cart = approxGaussian.approximateToGaussian_closedForm(params)
        cart_inScanRefFrame = poses_euler.composePosePDFEulerPoint(params, local_cart)
        a_pts.append(cart_inScanRefFrame)
    for params in p_prime:
        local_cart = approxGaussian.approximateToGaussian_closedForm(params)
        c_pts.append(poses_euler.composePosePDFEulerPoint(params, local_cart))
    n_aPts = len(a_pts)
    n_cPts = len(c_pts)
    a_pts_array = {"mean": np.empty((n_aPts, 3)), "cov": np.empty((n_aPts, 3, 3))}
    c_pts_array = {"mean": np.empty((n_cPts, 3)), "cov": np.empty((n_cPts, 3, 3))}
    for k in range(0, n_aPts):
        a_pts_array["mean"][k] = a_pts[k]["mean"]
        a_pts_array["cov"][k] = a_pts[k]["cov"]
    for k in range(0, n_cPts):
        c_pts_array["mean"][k] = c_pts[k]["mean"]
        c_pts_array["cov"][k] = c_pts[k]["cov"]


    alpha = 0.75
    picp = mypicp.my_picp(8)

    C_x = np.zeros((n_points, 3, 3))
    math_utility.vectToSkewMatrixArray(c_pts_array["mean"], C_x)

    params_current = picp.copyParams(params_array)
    params_current_prime = picp.copyParams(params_prime_array)
    picp.computeParamsCurrent_array(q, params_array, params_prime_array, params_current, params_current_prime)
    picp.initCUT_array(params_current, params_current_prime)
    modeX_current = poses_euler.inverseComposePoseEulerPoint(q['pose_mean'], x_array)
    modeXprime_current = poses_euler.composePoseEulerPoint(q['pose_mean'], xprime_array)

    vals_max = picp.computeValsMax(modeX_current, modeXprime_current, params_current,
                                   params_current_prime)

    omega, K, errors_mean, errors_cov_inv = picp.intermediateValues(q, a_pts_array, c_pts_array, C_x)
    fval_comp, fval_mahalanobis_comp, fXarray_comp, fXprime_array_comp = picp.functionCost_array_combine(alpha, x_array, xprime_array,
                                                                                                         params_current, params_current_prime, vals_max,
                                                                                                         errors_mean, errors_cov_inv)

    picp.J_composePose["X"] = -picp.J_compose_pose_array(x_array)
    picp.J_composePose["X_prime"] = picp.J_compose_pose_array(xprime_array)
    J_comp = picp.J_F_array_combine(alpha, fval_mahalanobis_comp, errors_mean, omega, K, C_x, params_array, params_prime_array,
                                    fXarray_comp, fXprime_array_comp, vals_max)

    dincr = 1e-5
    eps_null = np.zeros((6,))
    fval_num = picp.newFunctionCost_combine(np.zeros((6,)),q, x_array, xprime_array, params_array, params_prime_array, a_pts_array, c_pts_array, C_x)
    J_num = math_utility.numericalJacobian(
                        lambda x: picp.newFunctionCost_combine(x, q, x_array, xprime_array, params_array, params_prime_array,
                                                                      a_pts_array, c_pts_array, C_x),
                        n_points, eps_null,
                        np.full(6, dincr))

    print("fval_comp / fval_num = {} / {}".format(fval_comp, fval_num))
    print("J_comp : ")
    print(J_comp)
    print("J_num : ")
    print(J_num)

    assert np.allclose(J_comp, J_num, atol=1e-5)

# ------------- MAIN ---------------------
if __name__ == "__main__":
    testForAssociation()
    #testJacobianCombine()