import time
import numpy as np
import scipy.linalg.matfuncs
import scipy.spatial.transform
import scipy.stats
from itertools import combinations, product
import matplotlib as plot
import timeit
from scipy.linalg.decomp import eigh

""" 
    Conjugate Unscented Transform (CUT) method detailed in [1] in the context of ICP for non-gaussian uncertainty 
    Note (14/10/2021) : Currently not used as it requires some more work to use in ICP

    References
    ----------
    .. [1] "Conjugate unscented transformation: Applications to estimation and control, Adurthi, Nagavenkat and Singla, Puneet and Singh, Tarunraj, 2018"
"""

def sqrtm_psd(A, check_finite=True):
    """
    Modified from https://github.com/scipy/scipy/pull/3556/commits/e3313a729eab8f060e280c8b1285ae64cd711b2c

    Matrix square root of a positive semi-definite matrix.
    Parameters
    ----------
    A : (N, N) array_like
        Positive semi-definite matrix whose square root to evaluate.
    check_finite : boolean, optional
        Whether to check that the input matrices contain only finite numbers.
        Disabling may give a performance gain, but may result in problems
        (crashes, non-termination) if the inputs do contain infinities or NaNs.
    
    Returns
    -------
    sqrtm : (N, N) ndarray
        Value of the sqrt function at `A`.

    See also
    --------
    sqrtm : Matrix square root without the psd restriction.
    
    Examples
    --------
    >>> from scipy import linalg
    >>> a = np.ones((4, 4))
    >>> r = linalg.sqrtm_psd(a)
    >>> r
    array([[ 0.5,  0.5,  0.5,  0.5],
           [ 0.5,  0.5,  0.5,  0.5],
           [ 0.5,  0.5,  0.5,  0.5],
           [ 0.5,  0.5,  0.5,  0.5]])
    >>> r.dot(r)
    array([[ 1.,  1.,  1.,  1.],
           [ 1.,  1.,  1.,  1.],
           [ 1.,  1.,  1.,  1.],
           [ 1.,  1.,  1.,  1.]])
    """

    A = np.asarray(A)
    w, v = scipy.linalg.decomp.eigh(A, check_finite=check_finite)
    w = np.sqrt(np.maximum(w, 0))
    return (v * w).dot(v.T)

def sqrtm_psd_array(A):
    """
        Modified version of sqrtm_psd with a stack of matrice ie 3D array n X m X m
        Matrix square root for an array of positive semi-definite matrices.

        Parameters
        ----------
        A : (N, M, M) ndarray
            Array of Positive semi-definite matrices whose square root to evaluate.

        Returns
        -------
        (N, M, M) ndarray
                         Array of matrices square root

        See also
        --------
        sqrtm_psd : No array original version
    """

    # Seems that scipy.eigh is faster  and outweight the vectorization allowed by numpy.eigh ....
    A = np.asarray(A)
    w, v = np.linalg.eigh(A)
    w = np.sqrt(np.maximum(w, 0))
    return np.einsum('...ij,...j,...hj->...ih',v,w,v,optimize=True)

"""" TODO : For better perf, take in account that for some dimension some weights are unused 
    (ex for dim 3, w5/r5 are omitted) """

class CUT:
    """
        Class implementing the Conjugate Unscented Transform [1] for computing Expectation over multivariate gaussian.
        
        
        Notes
        -----
        List of ranges and weights for n=6 : [w_1, w_2, ... ]
        Directly taken from the paper tables or from provided equations

         w_0 value are not explicitly given for the cut-6 and cut-8 cases
        For cut-4 :
            It is 0 (for n > 2, here n = 6)
        For cut-6 :
            Just compute it using the equation (42)
        For cut-8 :
            Generate the same formula from E(x^2_i) = 1 which gives
            w_o = 1 - 2n*w_1 - 2^n*w_2 - 2n(n-1)*w_3 - 2^n*w_4 - n_1*w_5 - n*2^n*w_6
            with n_1 = 4n(n-1)(n-2)/3
    """

    # Surely not the best way to stock all weights in the same struct :/ 
    cut_w = {"4": {"3":None, "6":None}, "6": {"3":None, "6":None}, "8": {"3":None, "6":None}}
    cut_w["4"]["3"] = [0., 0.16, 0.005]
    cut_w["4"]["6"] = [0., 0.0625      , 0.00390625]
    cut_w["6"]["3"] = [0.312478973, 0.0290351301, 0.0633844605, 0.0005195469]
    cut_w["6"]["6"] = [0.06746372199999998, 0.0365072564, 0.0069487173, 0.0008288549]
    cut_w["8"]["3"] = [0.259591709, 0.0246319934, 0.081510094, 0.00976723555, 0.0057724893, 0, 0.0002794729]
    cut_w["8"]["6"] = [0.08827161775999999, 0.0061728395, 0.0069134430, 0.0041152263, 0.0002183265,  0.00065104166, 0.00007849171]
    
    cut_r = {"4": {"3":None, "6":None}, "6": {"3":None, "6":None}, "8": {"3":None, "6":None}}
    cut_r["4"]["3"] = [0., np.sqrt(2.5), np.sqrt(5)]
    cut_r["4"]["6"] = [2.           , np.sqrt(2.)]
    cut_r["6"]["3"] = [2.3587090379, 1.1198362859, 3.1421303838]
    cut_r["6"]["6"] = [1.9488352799, 1.1445968942, 2.9068006056]
    cut_r["8"]["3"] = [2.2551372655, 0.7174531274, 1.8430194370, 1.5584810327, 0, 1.3055615004]
    cut_r["8"]["6"] = [2.4494897427,  0.8938246941221211,  1.7320508075,  1.531963037906212, 2., 1.0954451150]

    cut_Nsamples = {"4": {"3":None, "6":None}, "6": {"3":None, "6":None}, "8": {"3":None, "6":None}}
    cut_Nsamples["4"]["3"] = 15
    cut_Nsamples["6"]["3"] = 27
    cut_Nsamples["8"]["3"] = 59
    cut_Nsamples["4"]["6"] = 77
    cut_Nsamples["6"]["6"] = 137
    cut_Nsamples["8"]["6"] = 745

    def __init__(self, approxDegree=8, n=6):
        """
            Constructor

            Parameters
            ----------
            approxDegree : int
                           Approximation degree. Can be 4, 6 or 8.
            n : int
                Dimension of the problem

            Notes
            -----
            The higher the approxDegree, the better accuracy at the expense of computation time
        """

        if(n != 3 and n!= 6):
            print("Error in CUT. Can only be set for dimension 3 and 6")

        self.approxDegree = approxDegree
        self.n = n
        self.h = 3 # Scaling for the scaled conjugate axes s^n
        if (n == 3):
            self.h = 2.74

        self.list_c_n = []
        self.list_c_2 = []
        self.list_c_3 = []
        self.list_s_n = []
        self.scaledSamplesArray = {"X":None, "X_prime":None} # cache
   
        self.generate_c_6(0, np.zeros((n,)), self.list_c_n)
        self.list_c_2 = self.generate_c_n(2)
        self.list_c_3 = self.generate_c_n(3)
        self.list_s_n = self.generate_s_6()

        # Get the samples and weights for the reduced centered normal 
        approxDegree_str = str(approxDegree)
        n_str = str(n)
        self.weights = self.cut_w[approxDegree_str][n_str]
        self.nSamples = self.cut_Nsamples[approxDegree_str][n_str]
        if (approxDegree == 4):             
            self.normalized_samples = np.array(self.generate_cut4_x(self.cut_r[approxDegree_str][n_str]))
            self.weights_array = np.concatenate(([self.weights[0]],self.weights[1]*np.ones((1,2*n)),self.weights[2]*np.ones((1,2**n))), axis=None)
        elif (approxDegree == 6):
            self.normalized_samples = np.array(self.generate_cut6_x(self.cut_r[approxDegree_str][n_str]))
            self.weights_array = np.concatenate(([self.weights[0]],self.weights[1]*np.ones((1,2*n)),self.weights[2]*np.ones((1,2**n)), self.weights[3]*np.ones((1,2*n*(n-1)))), axis=None )
        elif (approxDegree == 8):
            self.normalized_samples = np.array(self.generate_cut8_x(self.cut_r[approxDegree_str][n_str]))
            n1 = 4*n*(n-1)*(n-2)//3
            self.weights_array = np.concatenate(([self.weights[0]],self.weights[1]*np.ones((1,2*n)),self.weights[2]*np.ones((1,2**n)), self.weights[3]*np.ones((1,2*n*(n-1))), self.weights[4]*np.ones((1,2**n)),self.weights[5]*np.ones((1, n1)), self.weights[6]*np.ones((1,n*2**n))), axis=None)
        else:
            print("Error in CUT. Can only be set with approxDegree in {4,6,8}")

    def generate_c_6(self, index, current_c, list_c):
        """ 
            Recursively generate an array of array corresponding to the conjugate axes c^6 for n=6. Used for all CUT 

            Parameters
            ----------
            index : int 
                    Index to be modified as a +1 or -1
            current_c : (6) array
                        current in-construction conjugate axe
            list_c : (64, 6) ndarray
                     array of conjugate axes c^6

            Notes
            -----
            Conjugate axes c^i are defined in [1], section 4
            The final list_c dimension is (2^m (m among n), n) with m = n = 6
        """

        # A conjugate axis has been generated. Stop here
        if (index > self.n - 1):
            list_c.append(current_c.copy())
        else:
            new_c_p = current_c.copy()
            new_c_p[index] = 1
            self.generate_c_6(index + 1, new_c_p, list_c)
            new_c_m = current_c.copy()
            new_c_m[index] = -1
            self.generate_c_6(index + 1, new_c_m, list_c)

    def generate_c_n(self, approxDegree):
        """ 
            Generate an array of array corresponding to the conjugate axes c^2 for n=6. Used for CUT-6 and CUT-8

            Parameters
            ----------
            approxDegree : int
                           Approximation degree. Should be 6 or 8.

            Returns
            -------
            list_c : (40, 6) ndarray
                     Array of conjugate axis c^2

            Notes
            -----
            Conjugate axes s^i are defined in [1], section 4
            The return dimension is (2^m (m among n), n) with m = 2 and n = 6
        """

        list_c = []
        non_zero_indexes = list(combinations(np.arange(self.n), approxDegree))
        values = list(product([1,-1],repeat=approxDegree))
        for indexes in non_zero_indexes:
            current_c = np.zeros((self.n,))
            for val in values:
                j = 0
                for i in indexes:
                    current_c[i] = val[j]
                    j = j + 1
                list_c.append(current_c.copy())

        return list_c

    def generate_s_6(self):
        """ 
            Generate an array of array corresponding to the scaled conjugate axes s^6 for n=6. Used for CUT-8

            Returns
            -------
            list_s : (384, 6) ndarray

            Notes
            -----
            Scaled conjugate axes s^i are defined in [1], section 4
            The return dimension is (n*2^m (m among n), n) with m = 6 and n = 6
        """

        list_s = []
        for c in self.list_c_n:
            for i in range(0, self.n):
                s = c.copy()
                s[i] = self.h*s[i]
                list_s.append(s)

        return list_s

    def generate_cut4_x(self, cut_6_r): 
        """ 
            Generate the list of samples x = [x_sigma, x_c] for CUT-4 and n=6

            Parameters
            ----------
            cut_6_r : (2) array
                      scales r1 and r2 for CUT-4

            Returns
            ------
            x : (77, 6) ndarray
                Sigma points for CUT-4

            Notes 
            -----
            The scales r1, r2 in cut_6_r for CUT-4 are defined in [1], equation (28)
            The sigma points are defined following [1], Table 6. The dimension is (1 + 2*n + 2^n, n) with n = 6
        """

        x = []

        # Center 
        x.append(np.zeros((self.n,)))

        # For x_sigma (principal axes)
        for i in range(0, self.n):
            x_sample = np.zeros((self.n,))
            x_sample[i] = cut_6_r[0]
            x.append(x_sample.copy())
            x_sample[i] = -cut_6_r[0]
            x.append(x_sample.copy())

        # For x_c (conjugated axes) 
        for c in self.list_c_n:
            x.append(cut_6_r[1]*np.array(c))

        return x

    def generate_cut6_x(self, cut_6_r):
        """ 
            List of samples x = [x_sigma, x_c] for CUT-6 and n=6 

            Parameters
            ----------
            cut_6_r : (3) array
                      scales r1, r2, r3 for CUT-6

            Returns
            -------
            x : (137, 6) ndarray
                Sigma points for CUT-6

            Notes 
            -----
            The scales r1, r2, r3 in cut_6_r for CUT-6 are given in [1], Table 27
            The sigma points are defined following [1], Table 11. The dimension is (1 + 2*n^2 + 2^n, n) with n = 6
        """

        x = []

        # The first 2n + 2**n elements are the same (with different r values) 
        x.extend(self.generate_cut4_x(cut_6_r))

        for c in self.list_c_2:
            x.append(cut_6_r[2]*c)

        return x

    def generate_cut8_x(self, cut_6_r):
        """ 
             List of samples x = [x_sigma, x_c] for CUT-8 and n=6 

            Parameters
            ----------
            cut_6_r : (3) array
                      scales r1, r2, r3, r4, r5, r6 for CUT-8

            Returns
            -------
            x : (745, 6) ndarray
                Sigma points for CUT-8

            Notes 
            -----
            The scales r1, r2, r3, r4, r5, r6 in cut_6_r for CUT-8 are given in [1], Table 28
            The sigma points are defined following [1], Table 16. 
            The dimension is (1 + n2^n + 2^n + n_1 + 2n(n-1) + 2^n + 2n, n) with n_1 = 4n(n-1)(n-2)/3 and n = 6 
        """
        
        # The first 2n + 2**n elements are the same (with different r values)
        x = self.generate_cut6_x(cut_6_r)

        for c in self.list_c_n:
            x.append(cut_6_r[3]*c)

        for c in self.list_c_3:
            x.append(cut_6_r[4]*c)

        for s in self.list_s_n:
            x.append(cut_6_r[5]*s)

        return x

    def convertSamplesToPDF(self, mean, covariance):
        """ 
            Convert the samples computed for the reduced centered normal to the normal pdf with (mean,covariance) 

            Parameters
            ----------
            mean : (n) array
                   mean of the n-multivariate normal random variable
            covariance : (n,n) ndarray
                         n X n covariance matrix of the n-multivariate normal random variable

            Returns
            -------
            samples : (m,n) ndarray
                      For CUT-m, m samples of the n-multivariate normal pdf. 

            Notes
            -----
            The formula used for generating the samples $s'$ is $s' = \Sigma_^{\frac{1}{2}}s + m$ where s is a sample from $\mathcal{N}(0, I)$
            and $m,\Sigma$ the mean/covariance of the normal pdf.

            The number of samples depends on the CUT setting.
            CUT-4 -> m = 76
            CUT-6 -> m = 137
            CUT-8 -> m = 745 
        """

        # Pb with sqrtm which sometimes gives complex values (very small imaginary part) 
        # so use directly diagonalisation to enforce the positivity of the covariance matrix 
        U, S, VT = scipy.linalg.svd(covariance)
        sqrt_S = np.diag(np.sqrt(S))
        cov_sqrt_T = U@sqrt_S@VT

        return self.normalized_samples@cov_sqrt_T + mean

    def convertSamplesToPDF_array(self, mean_array, covariance_array):
        """  
            Convert the samples computed for the reduced centered normal to each normal pdfs with 
            (mean,covariance) in mean_array,covariance_array 

            Parameters
            ----------
            mean_array : (p,n) array
                         mean of the n-multivariate normal random variable
            covariance_array : (p,n) ndarray
                         n X n covariance matrix of the n-multivariate normal random variable

            Returns
            -------
            samples : (m,n) ndarray
                      For CUT-m, m samples of the n-multivariate normal pdf. 

            Notes
            -----
            The formula used for generating the samples $s'$ is $s' = \Sigma_^{\frac{1}{2}}s + m$ where s is a sample from $\mathcal{N}(0, I)$
            and $m,\Sigma$ the mean/covariance of the normal pdf.

            The number of samples depends on the CUT setting.
            CUT-4 -> m = 76
            CUT-6 -> m = 137
            CUT-8 -> m = 745 
        """
        cov_sqrt_T = sqrtm_psd_array(covariance_array)

        return np.einsum('ij,...jh->i...h',self.normalized_samples,cov_sqrt_T,optimize=True) + mean_array[np.newaxis,:]

    def compute_opt(self, func, mean, covariance):
        """ 
            Expectation of func computed using CUT with a normal pdf (mean, covariance) 

            Parameters
            ----------
            func : function 
                   Function for which the expectation is computed
            mean : (n) array
                   mean of the n-multivariate normal pdf
            covariance : (n,n) ndarray
                         covariance matrix of the n-multivariate normal pdf

            Returns
            -------
            expectation : float
                          Expectation value
            comp_time : float
                        Computation time
            
            Notes
            -----
            Simply computed as a weigthed sum of the function evaluated at particular points (sigma points)
        """

        # Convert the samples to provided normal
        scaled_samples = self.convertSamplesToPDF(mean, covariance)
        func_values = func(scaled_samples).T

        expectation = self.computeExpectation_(func_values)

        return expectation

    def computeScaledSamplesArray(self, mean_array, covariance_array, type):
        """
        """
        self.scaledSamplesArray[type] = self.convertSamplesToPDF_array(mean_array, covariance_array)

    def compute_opt_array(self, func, type):
        """ 
            Expectation of func computed using CUT with normal pdfs 

            Parameters
            ----------
            func : function 
                   Function for which the expectation is computed

            Returns
            -------
            expectations : (n) array
                           Expectation values
            
            Notes
            -----
            Simply computed as a weigthed sum of the function evaluated at particular points (sigma points)
        """

        # Compute at once all the func values 
        # Result is 2D array of dimension n_points X n_samples before transpose 
        func_values = func(self.scaledSamplesArray[type]).T

        # Compute expectations for all vector
        # Result is vector with expection for each point 
        expectations = self.computeExpectation_(func_values)

        return expectations

    def computeExpectation_(self, func_values):
        """
            Compute the expectation as the weighted sums of one or more function evaluations at sigma points

            Parameters
            ----------
            func_values : (N,M) ndarray 
                          array of N functions values at the sigma points
            
            Returns
            -------
            float or float array
                                Weighted sums for each function 
        """
        return func_values@self.weights_array

class UT:
    """ 
        Implementation of the classic Unscented Transform (UT) which is a special case of CUT.
        Used to compute the mean/covariance of a pdf going through a non linear function. 

        Notes
        -----
        Brief review of UT can be read in [1], section 3
    """

    def __init__(self, n, k):
        """
            Constructor

            Parameters
            ----------
            n : int
                Dimension of the problem
            k : int
                Tuning parameter
        """
        self.n = n
        self.k = k
        self.weights = None
        self.normalizedSamples = None

        self.computeNormalizedWeights(n, k)
        self.computeNormalizedSamples(n, k)

    def computeNormalizedSamples(self, n, k):
        """
            Compute samples from the reduced centered normal pdf

            Parameters
            ----------
            n : int
                Dimension of the problem
            k : int
                Tuning parameter
        """
        self.normalizedSamples = np.empty((2*n+1, n))
        K = np.sqrt(n+k)*np.eye(n)
        self.normalizedSamples[0] = np.zeros((1, n))
        self.normalizedSamples[1:n+1, :] = K
        self.normalizedSamples[n+1:2*n+1, :] = -K

    def computeNormalizedWeights(self, n, k):
        """
            Compute weights from the reduced centered normal pdf

            Parameters
            ----------
            n : int
                Dimension of the problem
            k : int
                Tuning parameter
        """
        w = 1. / (2. * (n + k))
        self.weights = np.full(2*n+1, w)
        self.weights[0] = 2.*k*w

    def computeSamples(self, covariance):
        """ 
            Convert the samples computed for the reduced centered normal to the normal pdf with (mean,covariance) 

            Parameters
            ----------
            covariance : (n,n) ndarray
                         n X n covariance matrix of the n-multivariate normal random variable

            Returns
            -------
            samples : (2*n+1,n) ndarray
                      2*n + 1 samples of the n-multivariate normal pdf.  
        """
        
        U, S, VT = scipy.linalg.svd(covariance)
        sqrt_S = np.diag(np.sqrt(S))
        cov_sqrt_T = U @ sqrt_S @ VT
        return self.normalizedSamples @ cov_sqrt_T

    def computeCovariance(self, func, covariance):
        """
            Compute covariance after function evaluation

            Parameters
            ----------
            func : function
                   Function in which the pdf is passed
            covariance : (n,n) ndarray
                         covariance matrix of the input pdf 

            Returns
            -------
            cov : (n,n) ndarray
                  output covariance matrix resulting from the function evaluation
        """
        sigma_points = self.computeSamples(covariance)
        cov = np.zeros((self.n,self.n))
        for pt, w in zip(sigma_points, self.weights):
            sample = func(pt)
            cov += w*np.outer(sample,sample)
        return cov