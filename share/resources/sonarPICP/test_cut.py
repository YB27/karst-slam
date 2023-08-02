import numpy as np
import math
import cut

def testWithCos():
    """ 
        Test with the example 5.1.2 given in [1]

        References
        ----------
        .. [1] "Conjugate unscented transformation: Applications to estimation and control, Adurthi, Nagavenkat and Singla, Puneet and Singh, Tarunraj, 2018"
    """

    print("Test with the Exemple 5.1.2 in the original paper of CUT")
    # gt_value = -0.543583844
    cut4_paper_value = -0.5492
    cut6_paper_value = -0.5419
    cut8_paper_value = -0.5430
    paper_values = [cut4_paper_value, cut6_paper_value, cut8_paper_value]

    mean = np.zeros((6,))
    covariance = np.eye(6)
    cut4 = cut.CUT(approxDegree=4, n=6)
    cut6 = cut.CUT(approxDegree=6, n=6)
    cut8 = cut.CUT(approxDegree=8, n=6)
    cut4_value = cut4.compute_opt(lambda x: np.cos(np.linalg.norm(x,axis=1)), mean, covariance)
    cut6_value = cut6.compute_opt(lambda x: np.cos(np.linalg.norm(x,axis=1)), mean, covariance)
    cut8_value = cut8.compute_opt(lambda x: np.cos(np.linalg.norm(x,axis=1)), mean, covariance)
    computed_values = [cut4_value, cut6_value, cut8_value]

    assert np.allclose(paper_values, computed_values, atol=1e-3) 

def testDimension3():
    """
        Test for dimension 3 on integral of a polynom (here x^8) with a unitary centered normal distribution
    """

    gt_value = 315
    cut4 = cut.CUT(approxDegree=4, n=3)
    cut6 = cut.CUT(approxDegree=6, n=3)
    cut8 = cut.CUT(approxDegree=8, n=3)

    mean = np.zeros((3,))
    covariance = np.eye(3)
    cut4_value = cut4.compute_opt(lambda x: np.power(x[:,0],8) + np.power(x[:,1],8) + np.power(x[:,2],8), mean, covariance)
    cut6_value = cut6.compute_opt(lambda x: np.power(x[:,0],8) + np.power(x[:,1],8) + np.power(x[:,2],8), mean, covariance)
    cut8_value = cut8.compute_opt(lambda x: np.power(x[:,0],8) + np.power(x[:,1],8) + np.power(x[:,2],8), mean, covariance)

    print("Gt value : ")
    print(gt_value)
    print("cut4 value : ")
    print(cut4_value)
    print("cut6_value : ")
    print(cut6_value)
    print("cut8_value : ")
    print(cut8_value)
    
    assert math.isclose(gt_value, cut8_value, abs_tol=1e-4)

def testUT():
    """
        Test Unscented Transform
    """
    
    np.random.seed(1)
    n = 6
    mat = np.random.rand(n,n)
    cov = mat.T@mat

    func = lambda x : 2.*x

    cov_expected = 4.*cov

    ut = cut.UT(n,3)
    cov_computed = ut.computeCovariance(func, cov)

    assert np.allclose(cov_expected, cov_computed, atol=1e-5)


#------------- MAIN ---------------------
if __name__ =="__main__":
    testWithCos()
    testDimension3()
    testUT()