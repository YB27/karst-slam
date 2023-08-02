import poses_euler
import poses_quat
import numba as nb
import math_utility
import cut
import numpy as np
import scipy.linalg.matfuncs
import scipy.spatial.transform
import scipy.stats
import time
import timeit
import pptk
import dataAssociation
import gaussianPICP
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import mpl_toolkits.mplot3d as plt3d
from mayavi import mlab
import math_utility
import betaDist

class my_picp :
    """
        Variant of MpICP using directly the true probability distribution of points and not a gaussian approximation
        Not used currently. Need some more work. Refer to the section 5.3 in [1] in the doc folder.
        This is the second approach (eq (102)).

        TODO : Change the array layouts to avoid reshape(order='F') and instead use directly shape() (avoid copy)
    
        References
        ----------
        .. [1] "doc/MpIC_oldVersion_noApprox.pdf"
        .. [2] "Conjugate unscented transformation: Applications to estimation and control, Adurthi, Nagavenkat and Singla, Puneet and Singh, Tarunraj, 2018"
        .. [3] METHODS FOR NON-LINEAR LEAST SQUARES PROBLEMS, K. Madsen, H.B. Nielsen, O. Tingleff, 2004
    """
    
    def __init__(self, n_cut):
        self.n_cut = n_cut
        self.CUT = cut.CUT(n_cut)

        # Cached values for both X and X_prime 
        # (all are dict with entries "X" and "X_prime" 
        self.x_minus_q_array = {"X":None, "X_prime":None}
        self.rho_array = {"X":None, "X_prime":None}
        self.rho_sqr = {"X":None, "X_prime":None}
        self.c = {"X":None, "X_prime":None} 
        self.c_sqr = {"X":None, "X_prime":None}
        self.k_cube = {"X":None, "X_prime":None}
        self.localSphericalCoords_array = {"X":None, "X_prime":None}
        self.f_s_array = {"X":None, "X_prime":None}
        self.detJ_array = {"X":None, "X_prime":None}
        self.J_composePose = {"X":None, "X_prime":None}

        self.picpMaxIter = 20
        self.errorConvThresh = 1e-6

    def getMode_f_X(self, params):
        """ 
            Get the mode of X pdf from its spherical coords in params 

            Parameters
            ----------
            params : dict of data
                     contains data related to the local spherical pdf coords of a point

            Returns
            -------
            global_mode : (6) array
                          mode of the point in global cartesian coords
        """

        theta_mode = betaDist.beta_mode(params["theta"]["alpha"], params["theta"]["beta"], params["theta"]["b"])
        local_mode = math_utility.fromSphericalToCart([params["rho"]["mean"], theta_mode, params["psi"]["mean"]])
        global_mode = poses_euler.composePoseEulerPoint(params["pose_mean"], local_mode)
        return global_mode

    def getModes(self, measures):
        """ 
            Get the mode of all 3D points from their spherical coords
            Same as getMode_f_X but for multiple points at once

            Parameters
            ----------
            measures : (n) array of params (dict of data)
                       array of data related to the local spherical pdf coords of a point

            Returns
            -------
            modes : (n,3) array
                    array of points mode in global cartesian coords
        """

        modes = []
        for meas in measures:
            modes.append(self.getMode_f_X(meas))
        return np.array(modes)

    def initCUT_array(self, params_array, params_prime_array):
        """ 
            Initialize the samples for CUT 
        
            Parameters
            ----------
            params_array : array of params (dict of data)
                           contains data related to the local spherical pdf coords of the first point cloud
            params_prime_array :  array of params (dict of data)
                                  contains data related to the local spherical pdf coords of the second point cloud
        
            Notes
            -----
            See [1], section 5.3.2 and [2]
        """ 

        self.CUT.computeScaledSamplesArray(params_array["pose_mean"], params_array["pose_cov"], "X")
        self.CUT.computeScaledSamplesArray(params_prime_array["pose_mean"], params_prime_array["pose_cov"], "X_prime")

    def fromCartToSpherical_opt(self, x):
        """ 
            Express a point (in cart coords) in spherical coords

            Parameters
            ----------
            x : (n,3) ndarray
                array of 3d points in cartesian coords

            Returns
            -------
            x_sphe : (n,3) ndarray
                     array of 3d points in spherical coords

            Notes
            -----
            Corresponds to g^-1 in [1], equation (47) 
        """
        
        rho = np.linalg.norm(x, axis=1)
        theta = np.arcsin(x[:, 2] / rho)
        phi = np.arctan2(x[:, 1], x[:, 0])
        return np.column_stack((rho, theta, phi))

    def fromCartToSpherical_opt_array(self, type):
        """ 
            Express an array of points (in cart coords) in spherical coords

            Parameters
            ----------
            type : str
                   point from the first cloud ("X") or second cloud ("X_prime")

            Notes
            -----
            Corresponds to g^-1 in [1], equation (47)
        """
        
        self.rho_array[type] = np.linalg.norm(self.x_minus_q_array[type], axis=2)
        theta = np.arcsin(self.x_minus_q_array[type][:, :,  2] / self.rho_array[type])
        phi = np.arctan2(self.x_minus_q_array[type][:, :,  1], self.x_minus_q_array[type][:, :, 0])
        self.localSphericalCoords_array[type] = np.hstack((self.rho_array[type][:, np.newaxis], theta[:, np.newaxis], phi[:, np.newaxis]))

    def fromCartToSpherical_opt_array_association(self, type):
        """ 
            Similar as fromCartToSpherical_opt_array but used for the association phase 
            (tensors with one extra dimension compared to the normal version)

            Parameters
            ----------
            type : str
                   point from the first cloud ("X") or second cloud ("X_prime")
        """

        self.rho_array[type] = np.linalg.norm(self.x_minus_q_array[type], axis=3)
        theta = np.arcsin(self.x_minus_q_array[type][:, :, :, 2] / self.rho_array[type])
        phi = np.arctan2(self.x_minus_q_array[type][:, :, :, 1], self.x_minus_q_array[type][:, :, :, 0])
        self.localSphericalCoords_array[type] = np.hstack(
            (self.rho_array[type][:, np.newaxis], theta[:, np.newaxis], phi[:, np.newaxis]))

    def f_rho(self, rho, params):
        """ 
            Pdf of gamma law (can be approximated by a gaussian)

            Parameters
            ----------
            rho : float or (n) array
                  range value(s) in spherical coords
            params : dict (or array of dict) with ["mean"] and ["cov"] keys
                     contains data related to the range pdf

            Return
            ------
            val : float or (n) array
                  pdf value(s) at rho

            Notes
            -----
            See [1], equation (50)        
        """

        return scipy.stats.norm.pdf(rho, loc=params["mean"], scale=params["std"])

    def f_psi(self, psi, params):
        """ 
            Pdf of a gaussian

            Parameters
            ----------
            psi : float or (n) array
                  azimuthal angle(s) in spherical coords
            params :dict (or array of dict) with ["mean"] and ["cov"] keys
                     contains data related to the azimuthal angle pdf

            Returns
            -------
            val : float or (n) array
                  pdf value(s) at psi

            Notes
            -----
            See [1], equation (52) 
        """
        return scipy.stats.norm.pdf(psi, loc=params["mean"], scale = params["std"])

    def f_theta(self, theta, params):
        """ 
            Pdf of scaled beta

            Parameters
            ----------
            theta : float or (n) array
                    elevation angle(s) in spherical coords 
            params : dict (or array of dict) with ["alpha"], ["beta"] and ["b"] keys
                     contains data related to the elevation angle pdf

            Returns
            -------
            val : float or (n) array
                  pdf value(s) at theta

            Notes
            -----
            See [1], equation (51)
        """

        f_t = scipy.stats.beta.pdf(theta, a=params["alpha"], b=params["beta"], loc= -0.5*params["b"], scale = params["b"])
        
        # For distribution with alpha < 1 and/or beta < 1, the mode is in -b/2 or b/2 ie in this case f_theta(+-b/2) is infinite !
        # Replace inf with a high finite value
        isInf = np.isinf(f_t)
        if(isInf.any()):
            f_t[isInf] = 1e8
        return f_t

    def f_spherical_opt(self, x_spherical, params):
        """ 
            Pdf evaluation of points in spherical coords
            Defined as f_spherical = f_rho*f_theta*f_psi
        
            Parameters
            ----------
            x_spherical : (n,3) array
                          spherical coords of points
            params : dict of data
                     contains data related to the local spherical pdf coords of a point

            Returns
            -------
            f_s : (n) array
                  pdf evaluation

            Notes
            -----
            See [1], equation (74)
        """

        f_s = self.f_rho(x_spherical[:,0], params["rho"]) * self.f_theta(x_spherical[:,1], params["theta"]) * self.f_psi(x_spherical[:,2], params["psi"])
        return f_s

    def f_spherical_opt_array(self, params_array, type):
        """ 
            Pdf evaluation of points in spherical coords
            Defined as f_spherical = f_rho*f_theta*f_psi,
            Similar to f_spherical_opt but used to efficiently compute pdf for each CUT samples

            Parameters
            ----------
            params_array : dict of data
                           contains data related to the local spherical pdf coords of points
            type : str
                   point from the first cloud ("X") or second cloud ("X_prime")

            Returns
            -------
            f_s : (m,n) ndarray
                 pdf evaluation at each CUT sample for each point

            Notes
            -----
            The input x_array is a 3D array of dim m X n X 3
            The results of each pdf is a m X n matrix
            m = number of samples for CUT and n = number of points
            See [1], equation (74)
        """

        return self.f_rho(self.localSphericalCoords_array[type][:,0,:], params_array["rho"]) * \
               self.f_theta(self.localSphericalCoords_array[type][:,1,:], params_array["theta"]) *\
               self.f_psi(self.localSphericalCoords_array[type][:,2,:], params_array["psi"])

    def f_spherical_opt_array_association(self, params_array, type):
        """ 
            Pdf evaluation of points in spherical coords
            Defined as f_spherical = f_rho*f_theta*f_psi,
            Compute for all pair X_i, X'_j 
            
            Parameters
            ----------
            params_array : dict of data
                           contains data related to the local spherical pdf coords of points
            type : str
                   point from the first cloud ("X") or second cloud ("X_prime")

            Notes
            -----
            Output is a 3D array m X I x J 
            m = number of samples for CUT, I = number of X, J = number of X' 
            See [1], equation (74)
        """

        nPoints = params_array['pose_mean'].shape[0]
        f_s = np.empty((self.localSphericalCoords_array[type].shape[0],
                        nPoints,
                        self.localSphericalCoords_array[type].shape[3]))

        #TODO : store the dimensions of the problem (nSamples cute, nPoints...)
        for i in range(0, nPoints):
             f_s[:,i,:] = self.f_rho(self.localSphericalCoords_array[type][:,0,i,:], {"mean": params_array["rho"]["mean"][i],
                                                                                      "std": params_array["rho"]["std"][i]})* \
                          self.f_theta(self.localSphericalCoords_array[type][:, 1, i, :],
                                     {"alpha": params_array["theta"]["alpha"][i],
                                      "beta": params_array["theta"]["beta"][i],
                                      "b" : params_array["theta"]["b"][i]})* \
                          self.f_psi(self.localSphericalCoords_array[type][:, 2, i, :],
                                     {"mean": params_array["psi"]["mean"][i],
                                      "std": params_array["psi"]["std"][i]})

        return f_s

    def jacobianDeterminant_opt(self, x):
        """ 
            Absolute value of the determinant

            Parameters
            ----------
            x : (n,3) array
                points in cartesian coords
            
            Returns
            -------
            det : (n) array
                  array of jacobian determinant

            Notes
            -----
            See [1], equation (77)
        """

        return 1. / (np.linalg.norm(x,axis=1) * np.linalg.norm(x[:,0:2],axis=1))

    def jacobianDeterminant_opt_array(self, type):
        """ 
            Absolute value of the determinant for array
            
            Parameters
            ----------  
            type : str
                   point from the first cloud ("X") or second cloud ("X_prime")

            Returns
            -------
            det : (m,n) array
                  array of jacobian determinant (n being the size of first or second point cloud depending on "type", m the CUT samples)

            Notes
            -----
            See [1], equation (77)
        """

        self.c[type] = np.linalg.norm(self.x_minus_q_array[type][:, :, 0:2], axis=2)
        return 1. / (self.rho_array[type] * self.c[type])

    def jacobianDeterminant_opt_array_association(self, type):
        """ 
            Absolute value of the determinant for the association step
            
            Parameters
            ----------  
            type : str
                   point from the first cloud ("X") or second cloud ("X_prime")

            Returns
            -------
            det : array
                  array of jacobian determinant (n being the size of first or second point cloud depending on "type")

            Notes
            -----
            See [1], equation (77)
        """

        return 1. / (self.rho_array[type] * np.linalg.norm(self.x_minus_q_array[type][:, :, :,0:2], axis=3))

    def h_opt(self, x, q, params):
        """ 
            pdf of a 3D point measured by a sonar is the expectation of H(x,q) over the pose pdf \eta_p

            H(x,q) = f_(rho,theta,phi)(g^-1(x_minus_q)) |J|(x,q) 
            
            Parameters
            ----------
            x : (n,3) array
                array of points in cartesian coords
            q : (6) array
                relative pose between points cloud in euler + 3d
            params : dict of data
                     contains data related to the local spherical pdf coords of  points

            Returns
            -------
            h : (n) array
                values of h(x,q)

            Notes
            -----
            See [1], section 5.3.1 and equation (80)
        """

        x_minus_q = poses_euler.inverseComposePoseEulerPoint_opt(q, x)

        # f_(rho,theta,psi)(g^-1(x_minus_q)) 
        locSphericalCoords = self.fromCartToSpherical_opt(x_minus_q)
        f_s = self.f_spherical_opt(locSphericalCoords, params)

        # Determinant of the jacobian 
        J_det = self.jacobianDeterminant_opt(x_minus_q)
        prod = f_s * J_det
        return prod

    def h_opt_array(self, x_array, q, params_array, type):
        """ 
            Compute H directly with 3d array input (all the point X_i + all the samples q_k) 
            
            Parameters
            ----------
            x_array : (n,3) array
                array of points in cartesian coords
            q : (m,6) array
                relative (CUT) sample poses between points cloud in euler + 3d
            params_array : dict of data
                           contains data related to the local spherical pdf coords of points
            type : str
                   point from the first cloud ("X") or second cloud ("X_prime")

            Returns
            -------
            h : (m,n) array
                values of h(x,q)

            Notes
            -----
            See [1], section 5.3.1 and equation (80)
        """
        
        # qi,k is the k-th samples for the i-th point
        # 3D array , with the k-th slice in the third dimension being the matrix [[x1 - q1,k] ... [xn - qn,k]]
        self.x_minus_q_array[type] = poses_euler.inverseComposePoseEulerPoint_opt_array(q, x_array)

        # f_(rho,theta,psi)(g^-1(x_minus_q)) 
        self.fromCartToSpherical_opt_array(type)

        self.f_s_array[type] = self.f_spherical_opt_array(params_array, type)

        # Determinant of the jacobian 
        self.detJ_array[type] = self.jacobianDeterminant_opt_array(type)

        prod = self.f_s_array[type] * self.detJ_array[type]

        return prod

    def h_opt_array_association(self, x_array, q, params_array, type):
        """ 
            Compute h directly with 3d array input (all the point X_i + all the samples q_k)
            
            Parameters
            ----------
            x_array : (n,3) array
                array of points in cartesian coords
            q : (m,6) array
                relative (CUT) sample poses between points cloud in euler + 3d
            params_array : dict of data
                           contains data related to the local spherical pdf coords of points
            type : str
                   point from the first cloud ("X") or second cloud ("X_prime")

            Returns
            -------
            h : (m,n) array
                values of h(x,q)

            Notes
            -----
            See [1], section 5.3.1 and equation (80)
        """
        
        # qi,k is the k-th samples for the i-th point
        # 3D array , with the k-th slice in the third dimension being the matrix [[x1 - q1,k] ... [xn - qn,k]]
        self.x_minus_q_array[type] = poses_euler.inverseComposePoseEulerPoint_opt_array_association(q, x_array)

        # f_(rho,theta,psi)(g^-1(x_minus_q))
        self.fromCartToSpherical_opt_array_association(type)

        f_s_array = self.f_spherical_opt_array_association(params_array, type)

        # Determinant of the jacobian 
        detJ_array = self.jacobianDeterminant_opt_array_association(type)

        prod = f_s_array * detJ_array

        return prod

    def f_X(self, x, params):
        """ 
            pdf of X : f_X(x) = E_q(h(q,x)) 
            
            Parameters
            ----------
            x : (3) array
                point in cartesian coords
            params : dict of data
                     contains data related to the local spherical pdf coords of point

            Returns
            -------
            val : float
                  pdf value f_X(x)

            Notes
            -----
            See [1], equation (79)
        """

        val = self.CUT.compute_opt(lambda q: self.h_opt(x, q, params), params["pose_mean"], params["pose_cov"])
        return val

    def f_X_array(self, x_array, params_array, type):
        """ 
            pdf of X : f_X(x) = E_q(h(q,x)) for array
            
            Parameters
            ----------
            x_array : (n,3) array
                      points in cartesian coords
            params_array : dict of data
                          contains data related to the local spherical pdf coords of points
            type : str
                   point from the first cloud ("X") or second cloud ("X_prime")

            Returns
            -------
            val : (n) array
                  pdf value f_X(x_array)

            Notes
            -----
            See [1], equation (79)
        """

        vals = self.CUT.compute_opt_array(lambda q: self.h_opt_array(x_array, q, params_array, type), type)
        return vals

    def f_X_array_association(self, x_array, params_array, type):
        """ 
            Return matrix IxJ [f_Xj(Xi)] ,  I = x_array.shape[0] , J = params_array length

            Parameters
            ----------
            x_array : (n,3) array
                      points in cartesian coords
            params_array : dict of data
                          contains data related to the local spherical pdf coords of points
            type : str
                   point from the first cloud ("X") or second cloud ("X_prime")
            
            Returns
            -------
            val : (n) array
                  pdf value f_X(x_array)
        """

        vals = self.CUT.compute_opt_array(lambda q: self.h_opt_array_association(x_array, q, params_array, type), type)
        return vals

    def functionCost_array(self, modeX, modeXprime, params, params_prime, vals_max):
        """ 
            Compute f = [(f_X(q + mean(X')) * f_X'(mean(X) - q) - max_fx*max_fx')] 

            Parameters
            ----------
            modeX : (n,3) array
                    modes of associated first point cloud
            modeXprime : (n,3) array
                         modes of associated second point cloud
            params : dict of data
                     contains data related to the local spherical pdf coords of point from first point cloud (X)
            params_prime : dict of data
                     contains data related to the local spherical pdf coords of point from second point cloud (Xprime)
            vals_max : (n) array
                       maximum values for f_X * f_X' 

            Returns
            -------
            functionCost : (n) array
                           cost function evaluated at modeX, modeXprime
            fX_array : (n) array
                       pdf values of f_X(modeXprime)
            fXprime_array : (n) array
                            pdf values of f_Xprime(modeX)
            Notes
            -----
            See [1], equation (101)
        """
        
        fX_array = self.f_X_array(modeXprime, params, "X")
        fXprime_array = self.f_X_array(modeX, params_prime, "X_prime")
        functionCost = np.log(vals_max) - np.log(fX_array * fXprime_array)

        return functionCost, fX_array, fXprime_array

    def functionCost_array_association(self, modeX, modeXprime, params, params_prime, vals_max_matrix):
        """
            Return the results for all combination of X and X' 
            All the outputs are 2D matrix nModeX X nModeXprime

            Parameters
            ----------
            modeX : (n,3) array
                    modes of associated first point cloud
            modeXprime : (m,3) array
                         modes of associated second point cloud
            params : dict of data
                     contains data related to the local spherical pdf coords of point from first point cloud (X)
            params_prime : dict of data
                     contains data related to the local spherical pdf coords of point from second point cloud (Xprime)
            vals_max : (m,n) array
                       maximum values for f_X * f_X' 

            Returns
            -------
            functionCost : (m,n) array
                           cost function evaluated at modeX, modeXprime
            fX_array : (m,n) array
                       pdf values of f_X(modeXprime)
            fXprime_array : (m,n) array
                            pdf values of f_Xprime(modeX)

            Notes
            -----
            See [1], equation (101)
        """
        
        fX_matrix = self.f_X_array_association(modeXprime, params, "X")
        fXprime_matrix = self.f_X_array_association(modeX, params_prime, "X_prime")
        return ((fX_matrix.T * fXprime_matrix)/ vals_max_matrix) - 1., fX_matrix, fXprime_matrix

    def J_compose_pose_array(self, x_array):
        """ 
            Jacobian of pose-point composition wrt pose.

            Parameters
            ----------
            x_array : (n,3) array
                       array of points in cartesian coords

            Returns
            -------
            jacobian : (n,3,6) ndarray
                       jacobian matrix evaluated at each point

            Notes
            -----
            See [1], equation (12) 
        """
        
        J_c = np.zeros((x_array.shape[0],3,6))
        math_utility.vectToSkewMatrixArray(-x_array, J_c[:,0:3,0:3])
        #TODO : better way to do ?
        ones = np.ones((x_array.shape[0],))
        J_c[:, 0, 3] = ones
        J_c[:, 1, 4] = ones
        J_c[:, 2, 5] = ones
        return J_c

    def J_compose_point_opt_array(self, q_array):
        """ 
            Jacobian of pose-point composition wrt point

            Parameters
            ----------
            q_array : (m,n,6) ndarray
                      poses in euler + 3d. (m = CUT samples, n = number of points)

            Returns
            -------
            jacobian : (m,n,3,3) ndarray
                        jacobian matrix evaluated at each pose sample for each point

            Notes
            -----
            See [1], equation (13) 
        """

        q_posesEuler_reshaped = q_array.reshape((q_array.shape[0] * q_array.shape[1], 6), order='F')
        rot_T = scipy.spatial.transform.Rotation.from_euler('ZYX', q_posesEuler_reshaped[:, 0:3]).inv()
        matrixRot = rot_T.as_matrix().reshape((q_array.shape[0] , q_array.shape[1], 3, 3),order='F')
        return matrixRot

    def J_f_spherical_opt_array(self, params_array, type):
        """ 
            Compute Jacobian of f_spherical 
            
            Parameters
            ----------
            params_array : dict of data
                          contains data related to the local spherical pdf coords of points
            type : str
                   point from the first cloud ("X") or second cloud ("X_prime")        
            
            Returns
            -------
            jacobian : (m, 3, n) ndarray
                      jacobian matrices

            Notes
            -----
            See [1], equation (113)
        """

        # Output is a 3d array of dimension n_samples X 3 X n_points 
        b_half = 0.5*params_array["theta"]["b"]
        rho_var_inv = 1. / np.power(params_array["rho"]["std"],2)
        psi_var_inv = 1. / np.power(params_array["psi"]["std"],2)
        rho_col = ((params_array["rho"]["mean"] - self.localSphericalCoords_array[type][:,0,:]) * rho_var_inv)[:,np.newaxis]
        theta_col = ((params_array["theta"]["alpha"] - 1.) / (b_half + self.localSphericalCoords_array[type][:,1,:]) - (params_array["theta"]["beta"] - 1.) / (b_half - self.localSphericalCoords_array[type][:,1,:]))[:,np.newaxis]
        psi_col = ((params_array["psi"]["mean"] - self.localSphericalCoords_array[type][:,2,:]) * psi_var_inv)[:,np.newaxis]
        return np.einsum('...j,...ji->...ji',self.f_s_array[type],np.hstack((rho_col, theta_col, psi_col)).transpose((0,2,1)),optimize=True)

    def J_g_inverse_opt_array(self, type):
        """ 
            Compute the Jacobian of g^-1 

            Parameters
            ----------
            type : str
                   point from the first cloud ("X") or second cloud ("X_prime")     

            Returns
            -------
            jacobian : (m,n,3,3) ndarray
                        jacobian matrices

            Notes
            -----
            See [1], equation (179) 
        """

        self.rho_sqr[type] =  np.power(self.rho_array[type], 2)
        rho_inv = 1. / self.rho_array[type]
        self.c_sqr[type] = np.power(self.c[type],2)
        c_sqr_inv = 1. / self.c_sqr[type]
        k = 1. / (self.c[type] * self.rho_sqr[type])
        self.k_cube[type] = -k * rho_inv * c_sqr_inv

        A = np.zeros((self.rho_array[type].shape[0],self.rho_array[type].shape[1],3,3))

        A[:,:,0,:] = np.einsum('ki...,ki->ki...', self.x_minus_q_array[type], rho_inv, optimize=True)#(self.x_minus_q_array[:,:,0] * rho_inv)[:,np.newaxis]
        A[:,:,1,0] = np.einsum('ki,ki,ki->ki', self.x_minus_q_array[type][:,:,0], self.x_minus_q_array[type][:,:,2], -k, optimize=True)#-self.x_minus_q_array[:,0,:] * self.x_minus_q_array[:,2,:] * k
        A[:,:,1,1] = np.einsum('ki,ki,ki->ki', self.x_minus_q_array[type][:,:,1], self.x_minus_q_array[type][:,:,2], -k, optimize=True)#-self.x_minus_q_array[:,1,:] * self.x_minus_q_array[:,2,:] * k
        A[:,:,1,2] = self.c_sqr[type] * k
        A[:,:,2,0] = np.einsum('ki...,ki->ki', self.x_minus_q_array[type][:,:,1], -c_sqr_inv, optimize=True)#-self.x_minus_q_array[:,1,:] * c_inv
        A[:,:,2,1] = np.einsum('ki...,ki->ki', self.x_minus_q_array[type][:,:,0], c_sqr_inv, optimize=True)#self.x_minus_q_array[:,0,:] * c_inv

        return A

    def J_detJ_opt_array(self, type):
        """ 
            Define the Jacobian of the determinant of J_g^1 

            Parameters
            ----------
            type : str
                   point from the first cloud ("X") or second cloud ("X_prime")

            Returns
            -------
            jacobian : (m,n,3) ndarray
                       jacobians

            Notes
            -----
            See [1], equation (77) 
        """

        c2 = self.rho_sqr[type] + self.c_sqr[type]  #c1 + rho_sqr

        A = np.empty((self.x_minus_q_array[type].shape[0], self.x_minus_q_array[type].shape[1], 3))
        A[:,:,0:2] = np.einsum('ki,ki...->ki...', c2, self.x_minus_q_array[type][:,:,0:2], optimize=True)# c2 * self.x_minus_q_array[:,:,0:2]
        A[:,:,2] = np.einsum('ki,ki->ki', self.c_sqr[type], self.x_minus_q_array[type][:,:,2], optimize=True) #c1 *  self.x_minus_q_array[:,:,2]
        return np.einsum('ki,ki...->ki...',self.k_cube[type], A, optimize=True)

    def J_h_opt_array(self, q, params_array, type):
        """ 
            Compute the jacobian of H relative to X 
            
            Parameters 
            ----------
            q : (m,n,6) ndarray
                 poses in euler + 3d. (m = CUT samples, n = number of points)
            params_array : dict of data
                          contains data related to the local spherical pdf coords of points
            type : str
                   point from the first cloud ("X") or second cloud ("X_prime")

            Returns
            -------
            jacobian : (m,n,3) ndarray
                        jacobians

            Notes
            -----
            See [1], equation (112) 
        """
        
        # J_f_spherical 
        J_f_s = self.J_f_spherical_opt_array(params_array, type)

        # J_g^-1
        J_g_inv = self.J_g_inverse_opt_array(type)

        # J_(detJ_g^-1) 
        JdetJ = self.J_detJ_opt_array(type)

        # J_compose_point 
        J_c_p = self.J_compose_point_opt_array(q)

        # TODO : avoid transpose by directly setting the right shape in J_g_inv computation 
        JdetJ_Jfs = np.einsum('ki,ki...->ki...', self.detJ_array[type], J_f_s, optimize=True)

        C = np.einsum('kij,kij...->ki...', JdetJ_Jfs, J_g_inv, optimize = True) + np.einsum('ki,ki...->ki...', self.f_s_array[type], JdetJ, optimize=True)
        return np.einsum('kij,kij...->ki...', C, J_c_p,optimize=True)

    def J_f_X_array(self,params_array, type):
        """ 
            Jacobian of f_x computed using CUT

            Parameters
            ----------
            params_array : dict of data
                          contains data related to the local spherical pdf coords of points
            type : str
                   point from the first cloud ("X") or second cloud ("X_prime")

            Returns
            -------
            jacobian : (n,3) ndarray
                        jacobians
            Notes
            ----- 
            See [1], equation (111) 
        """

        val = self.CUT.compute_opt_array(lambda q: self.J_h_opt_array(q, params_array, type), type)
        return val.T

    def J_F_opt_array(self, params_array, params_prime_array , f_X_array, f_Xprime_array, vals_max):
        """ 
            Jacobian of F = fTf 
            
            Parameters
            ----------
            params_array : dict of data
                          contains data related to the local spherical pdf coords of first point cloud
            params_prime_array : dict of data
                                 contains data related to the local spherical pdf coords of second point cloud
            f_X_array : (n) ndarray
                        values of f_X
            f_Xprime_array : (n) ndarray
                             values of f_Xprime
            vals_max : (n) array
                       maximum values for f_X*f_Xprime

            Returns
            -------
            jacobian : (n,3) ndarray
                        jacobians

            Notes
            -----
            See [1], equation (102) 
        """ 
        
        J_fX_array = self.J_f_X_array(params_array, "X")
        J_fXprime_array = self.J_f_X_array(params_prime_array, "X_prime")

        K = 1. / (f_X_array * f_Xprime_array + 1e-6)
        J_fX_array = np.einsum('ij, i->ij', J_fX_array, K)
        J_fXprime_array = np.einsum('ij, i->ij', J_fXprime_array, K)

        J = np.einsum('ij,ijk->ik', np.einsum('i,i...->i...', f_Xprime_array, J_fX_array, optimize=True), self.J_composePose["X_prime"], optimize=True) +\
               np.einsum('ij,ijk->ik', np.einsum('i,i...->i...', f_X_array, J_fXprime_array, optimize=True), self.J_composePose["X"], optimize=True)
        return J

    def computeValsMax(self, modeX, modeXprime, params, params_prime):
        """ 
            Compute maximul values of pdf (used for normalisation)

            Parameters
            ----------
            modeX : (n,3) array
                    modes of associated first point cloud
            modeXprime : (n,3) array
                         modes of associated second point cloud
            params : dict of data
                     contains data related to the local spherical pdf coords of point from first point cloud (X)
            params_prime : dict of data
                     contains data related to the local spherical pdf coords of point from second point cloud (Xprime)
            
            Returns
            -------
            vals_max : (n) array
                       maximum values for f_X * f_X' 

            Notes
            ----- 
            Corresponds simply to the first term in [1], equation (101)
            ie f_ai(a_mi)*f_ci(c_mi) 
        """

        max_fX_array = self.f_X_array(modeX, params, "X")
        max_fXprime_array = self.f_X_array(modeXprime, params_prime, "X_prime")
        return max_fX_array*max_fXprime_array

    def computeValsMax_association(self, modeX, modeXprime, params, params_prime):
        """
            (For association step) Compute maximul values of pdf (used for normalisation) 

            Parameters
            ----------
            modeX : (n,3) array
                    modes of associated first point cloud
            modeXprime : (m,3) array
                         modes of associated second point cloud
            params : dict of data
                     contains data related to the local spherical pdf coords of point from first point cloud (X)
            params_prime : dict of data
                     contains data related to the local spherical pdf coords of point from second point cloud (Xprime)
            
            Returns
            -------
            vals_max : (m,n) ndarray
                        maximum values for f_X * f_X' 

            Notes
            -----
            Corresponds simply to the first term in [1], equation (101)
            ie f_ai(a_mi)*f_ci(c_mi) 
        """

        max_fX_array = self.f_X_array(modeX, params, "X")
        max_fXprime_array = self.f_X_array(modeXprime, params_prime, "X_prime")
        return np.outer(max_fX_array,max_fXprime_array)

    def computeParamsCurrent(self, p, p_prime, q, params_current, params_prime_current):
        """ 
            Compute a^{(k)}_{mi} and c^{(k)}_{mi}
            
            Parameters
            ----------
            p : (n) array
                pose at which point has been measured in first point cloud
            p_prime : (n) array
                      pose at which point has been measured in second point cloud
            params_currents : dict of data
                     contains data related to the local spherical pdf coords of point from first point cloud (X)
            params_prime_current : dict of data
                     contains data related to the local spherical pdf coords of point from second point cloud (Xprime)

            Notes
            -----
            Defined in [1], equations (99-100)
        """
        
        for k in range(0, len(p)):
            # a_{mi} - \bar{q}^{(k)}
            p_i_minus_q = poses_euler.inverseComposePosePDFEuler(p[k], q)

            # \bar{q}^{(k)} + c_{mi}
            p_prime_i_plus_q = poses_euler.composePosePDFEuler(q, p_prime[k])

            params_current['pose_mean'][k] = p_i_minus_q['pose_mean']
            params_current['pose_cov'][k] = p_i_minus_q['pose_cov']

            params_prime_current['pose_mean'][k] = p_prime_i_plus_q['pose_mean']
            params_prime_current['pose_cov'][k] = p_prime_i_plus_q['pose_cov']

    def computeParamsCurrent_array(self, q, params, params_prime, params_current, params_prime_current):
        """ 
            Compute a^{(k)}_{mi} and c^{(k)}_{mi} 
            
            Parameters
            ----------
            q : dict with ["pose_mean"] and ["pose_cov"]
                relative pose between the two point clouds
            params : dict of data
                     contains data related to the local spherical pdf coords of point from first point cloud (X)
            params_prime : dict of data
                     contains data related to the local spherical pdf coords of point from second point cloud (Xprime)
            params_currents : dict of data
                     contains data related to the local spherical pdf coords of point from first point cloud (X)
            params_prime_current : dict of data
                     contains data related to the local spherical pdf coords of point from second point cloud (Xprime)
            
            Notes
            -----
            Defined in [1], equations (99-100)
        """
        
        # a_{mi} - \bar{q}^{(k)}
        p_i_minus_q = poses_euler.inverseComposePosePDFEuler_array(params, q)

        # \bar{q}^{(k)} + c_{mi}
        p_prime_i_plus_q = poses_euler.composePosePDFEuler_array(q, params_prime)

        params_current['pose_mean'] =  p_i_minus_q['pose_mean']
        params_current['pose_cov'] = p_i_minus_q['pose_cov']
        params_prime_current['pose_mean'] = p_prime_i_plus_q['pose_mean']
        params_prime_current['pose_cov'] = p_prime_i_plus_q['pose_cov']

    def copyParams(self, params):
        """ 
            Copy some parameters
            
            Parameters
            ----------
            params : dict of data
                     contains data related to the local spherical pdf coords of points

            Returns
            params_copy : dict of data
                         copied data
        """
        
        entries = ["rho", "theta", "psi", "pose_mean", "pose_cov"]
        params_current = {}
        for ent in entries:
            params_current[ent] = params[ent].copy()
        return params_current

    def getParamsByIndexes(self,params, idxs):
        """ 
            Get the parameters corresponding to the point index

            Parameters
            ----------
            params : dict of data
                      contains data related to the local spherical pdf coords of points
            idxs : (n) array of int
                    indexes to extract
            
            Returns
            -------
            params_subset : dict of data
                            a subset of params 
        """
        
        return {'rho': {'mean': params['rho']['mean'][idxs],
                                  'std': params['rho']['std'][idxs]},
                          'theta': {'alpha': params['theta']['alpha'][idxs],
                                    'beta': params['theta']['beta'][idxs],
                                    'b': params['theta']['b'][idxs]},
                          'psi': {'mean': params['psi']['mean'][idxs], 'std': params['psi']['std'][idxs]},
                          'pose_mean': params['pose_mean'][idxs,:], 'pose_cov':params['pose_cov'][idxs,:,:]}

    def association_bruteForce(self, q, modeX, modeXprime, params, params_prime, associationThreshold):
        """ 
            Association step with brute force strategy 

            Parameters
            ----------
            modeX : (n,3) array
                    modes of associated first point cloud
            modeXprime : (m,3) array
                         modes of associated second point cloud
            params : dict of data
                     contains data related to the local spherical pdf coords of point from first point cloud (X)
            params_prime : dict of data
                     contains data related to the local spherical pdf coords of point from second point cloud (Xprime)
            associationThreshold : float
                                   threshold for considering a valid association
            
            Returns 
            -------
            idx_pointX : (n) array of int
                         indexes of associated points in the first point cloud
            idx_pointXprime : (n) array of int
                         indexes of associated points in the second point cloud
        """
        
        modeX_current = poses_euler.inverseComposePoseEulerPoint(q['pose_mean'], modeX)
        modeXprime_current = poses_euler.composePoseEulerPoint(q['pose_mean'], modeXprime)

        # Full copy without references 
        params_current = self.copyParams(params)
        params_prime_current = self.copyParams(params_prime)

        self.computeParamsCurrent_array(q, params, params_prime, params_current, params_prime_current)
        self.initCUT_array(params_current, params_prime_current)
        vals_max_association = self.computeValsMax_association(modeX_current, modeXprime_current, params_current,
                                                               params_prime_current)
        costMatrix, _,_ = self.functionCost_array_association(modeX, modeXprime, params_current,
                                                                                    params_prime_current,
                                                                                    vals_max_association)

        # Note the fabs(costMatrix) 
        idxs_pointX, idxs_pointXprime, indiv_compatible_A = dataAssociation.dataAssociation_withDistanceMatrix(np.fabs(costMatrix),
                                                                                           associationThreshold)
        return idxs_pointX, idxs_pointXprime

    def association_approxGaussian_(self, q, modeX, modeXprime, params, params_prime, firstScanGaussianApprox,
                                   secondScanGaussianApprox):
        """ 
            Assocation step using the gaussian approximation for finding a first coarse "compatibility set" 
            Then use Brute force on those 
            
            Parameters
            ----------
            modeX : (n,3) array
                    modes of associated first point cloud
            modeXprime : (m,3) array
                         modes of associated second point cloud
            params : dict of data
                     contains data related to the local spherical pdf coords of point from first point cloud (X)
            params_prime : dict of data
                     contains data related to the local spherical pdf coords of point from second point cloud (Xprime)
            firstScanGaussianApprox : (n) array of pdf
                                      pdf of gaussian approximation for the first point cloud
            secondScanGaussianApprox : (m) array of pdf
                                      pdf of gaussian approximation for the second point cloud 

            Returns
            -------
            list_pair_indexes : dict with ["dist"], ["idx1"] and ["idx2"] keys
                                contains tuple of associations with the distance and indexes in each point cloud                    
        """

        print('--> association_approxGaussian')
        associationThreshold = 5 #7.81  # 95% CI with 3 dof --> Use a higher threshold here than for GaussianPICP
        _, _, indiv_compat_A = dataAssociation.dataAssociation(dataAssociation.mahalanobisMetric,
                                                               firstScanGaussianApprox,
                                                               poses_euler.composePosePDFEulerPoint_array(q,
                                                                                                          secondScanGaussianApprox),
                                                               associationThreshold)

        # Now use brute force on the compatible sets. 
        # TODO : Use a grid algo as in Palomer2016 (wait for c++ impl, not needed in the python code ?)
        modeX_current = poses_euler.inverseComposePoseEulerPoint(q['pose_mean'], modeX)
        modeXprime_current = poses_euler.composePoseEulerPoint(q['pose_mean'], modeXprime)

        # Full copy without references 
        params_current = self.copyParams(params)
        params_prime_current = self.copyParams(params_prime)

        list_pair_indexes = []
        for i in range(0, len(indiv_compat_A)):
            compat_list = list(indiv_compat_A[i])
            if (compat_list):
                print("idx " + str(i))
                params_ = self.getParamsByIndexes(params, i)
                params_['rho']['mean'] = np.array([params_['rho']['mean']])
                params_['rho']['std'] = np.array([params_['rho']['std']])
                params_['psi']['mean'] = np.array([params_['psi']['mean']])
                params_['psi']['std'] = np.array([params_['psi']['std']])
                params_['theta']['alpha'] = np.array([params_['theta']['alpha']])
                params_['theta']['beta'] = np.array([params_['theta']['beta']])
                params_['theta']['b'] = np.array([params_['theta']['b']])
                params_['pose_mean'] = np.reshape(params_['pose_mean'], (1, 6))
                params_['pose_cov'] = np.reshape(params_['pose_cov'], (1, 6, 6))

                params_prime_ = self.getParamsByIndexes(params_prime, compat_list)
                params_current_ = self.getParamsByIndexes(params_current, i)
                params_current_['rho']['mean'] = np.array([params_current_['rho']['mean']])
                params_current_['rho']['std'] = np.array([params_current_['rho']['std']])
                params_current_['psi']['mean'] = np.array([params_current_['psi']['mean']])
                params_current_['psi']['std'] = np.array([params_current_['psi']['std']])
                params_current_['theta']['alpha'] = np.array([params_current_['theta']['alpha']])
                params_current_['theta']['beta'] = np.array([params_current_['theta']['beta']])
                params_current_['theta']['b'] = np.array([params_current_['theta']['b']])
                params_current_['pose_mean'] = np.reshape(params_current_['pose_mean'], (1, 6))
                params_current_['pose_cov'] = np.reshape(params_current_['pose_cov'], (1, 6, 6))

                params_prime_current_ = self.getParamsByIndexes(params_prime_current, compat_list)
                self.computeParamsCurrent_array(q, params_, params_prime_, params_current_, params_prime_current_)
                self.initCUT_array(params_current_, params_prime_current_)
                vals_max_association = self.computeValsMax_association(modeX_current[i].reshape((1, 3)),
                                                                       modeXprime_current[compat_list], params_current_,
                                                                       params_prime_current_)
                costMatrix, _,_ = self.functionCost_array_association(modeX[i].reshape((1, 3)),
                                                                                            modeXprime[compat_list],
                                                                                            params_current_,
                                                                                            params_prime_current_,
                                                                                            vals_max_association)
                costMatrix = np.fabs(costMatrix)
                for j in range(0, len(compat_list)):
                    list_pair_indexes.append({'dist': costMatrix[0][j], 'idx1': i, 'idx2': compat_list[j]})

        return list_pair_indexes

    def association_approxGaussian(self, q, modeX, modeXprime, params, params_prime, firstScanGaussianApprox, secondScanGaussianApprox):
        """ 
            Association based on gaussian approximation to filter possible associations 

            Parameters
            ----------
            modeX : (n,3) array
                    modes of associated first point cloud
            modeXprime : (m,3) array
                         modes of associated second point cloud
            params : dict of data
                     contains data related to the local spherical pdf coords of point from first point cloud (X)
            params_prime : dict of data
                     contains data related to the local spherical pdf coords of point from second point cloud (Xprime)
            firstScanGaussianApprox : (n) array of pdf
                                      pdf of gaussian approximation for the first point cloud
            secondScanGaussianApprox : (m) array of pdf
                                      pdf of gaussian approximation for the second point cloud 

            Return
            ------
            idx_pointX : (n) array of int
                         indexes of associated points in the first point cloud
            idx_pointXprime : (n) array of int
                         indexes of associated points in the second point cloud
        """
        
        list_pair_indexes = self.association_approxGaussian_( q, modeX, modeXprime, params, params_prime, firstScanGaussianApprox, secondScanGaussianApprox)

        sorted_list_pair_indexes = sorted(list_pair_indexes, key=lambda pair : pair['dist'])

        associationThreshold = 0.99
        alreadyAssociatedPoint2 = set()
        alreadyAssociatedPoint1 = set()
        idxs_pointCloud_1 = []
        idxs_pointCloud_2 = []
        for pair_indexes in sorted_list_pair_indexes:
            idx1 = pair_indexes['idx1']
            idx2 = pair_indexes['idx2']
            if (pair_indexes['idx2'] not in alreadyAssociatedPoint2 and
                pair_indexes['idx1'] not in alreadyAssociatedPoint1 and
                pair_indexes['dist'] < associationThreshold):
                    idxs_pointCloud_1.append(idx1)
                    idxs_pointCloud_2.append(idx2)
                    alreadyAssociatedPoint1.add(idx1)
                    alreadyAssociatedPoint2.add(idx2)
                    print("({},{}) with dist = {}".format(idx1, idx2, pair_indexes['dist']))
        return np.array(idxs_pointCloud_1), np.array(idxs_pointCloud_2)

    def pICP_display_mayavi(self,fig, modeX, modeXprime, modeXprime_current, modeXprime_current_GT,
                            idxs_pointX_assoc =None, idxs_pointXprime_assoc=None, modeX_planeProjections=None):
        """ 
            Display the pIC state at current iteration 

            Parameters
            ----------
            modeX : (n,3) array
                    modes of associated first point cloud (initial)
            modeXprime : (m,3) array 
                         modes of associated second point cloud (initial)
            modeXprime_current : (m,3) array 
                         modes of associated second point cloud (with current estimated relative pose q)
            modeXprime_current_GT : (m,3) array 
                         modes of associated second point cloud (with ground-truth relative pose q_gt)
            idxs_pointX_assoc : (l) array of int 
                                indexes of associated points in the first point cloud
            idxs_pointXprime_assoc : (l) array of int 
                                indexes of associated points in the second point cloud 
            modeX_planeProjections : (n,array) array
                                     projections of first point cloud mode in tangent planes at second point cloud 
                                    (related to point-to-plane association)                    
        """ 
        
        scale_factor = 0.3
        mlab.clf()
        mlab.points3d(modeX[:, 0], modeX[:, 1], modeX[:, 2], color=(1, 0, 0),
                      scale_factor=scale_factor)
        mlab.points3d(modeXprime[:, 0], modeXprime[:, 1], modeXprime[:, 2], color=(0, 0, 1),
                      scale_factor=scale_factor)
        #for a_pt, arcIdx in zip(a_pts_array['mean'], a_pts_array['arcIdx']):
        #    mlab.text3d(a_pt[0], a_pt[1], a_pt[2], str(arcIdx), figure=fig, color=(0, 0, 0), scale=scale_factor)

        mlab.points3d(modeXprime_current[:,0], modeXprime_current[:,1], modeXprime_current[:,2], color=(0,1,0), scale_factor=scale_factor)
        mlab.points3d(modeXprime_current_GT[:, 0], modeXprime_current_GT[:, 1], modeXprime_current_GT[:, 2], color=(1, 1, 0),
                      scale_factor=scale_factor)

        if (modeX_planeProjections is not None):
            mlab.points3d(modeX_planeProjections[:, 0], modeX_planeProjections[:, 1], modeX_planeProjections[:, 2], color=(1, 0.64, 0),
                          scale_factor=scale_factor)
            for pt, pt_trans in zip(modeX[idxs_pointX_assoc], modeX_planeProjections):
                mlab.plot3d([pt[0], pt_trans[0]], [pt[1], pt_trans[1]], [pt[2], pt_trans[2]], color=(0,0,0))

        if (idxs_pointX_assoc is not None):
            for pt_A, pt_N in zip(modeX[idxs_pointX_assoc], modeXprime_current[idxs_pointXprime_assoc]):
                mlab.plot3d([pt_A[0], pt_N[0]], [pt_A[1], pt_N[1]], [pt_A[2], pt_N[2]], color=(0, 0, 0))

        for pt, idx in zip(modeX[idxs_pointX_assoc], idxs_pointX_assoc):
            mlab.text3d(pt[0], pt[1], pt[2], str(idx), figure=fig, color=(0,0,0),scale=scale_factor)

        axes = mlab.axes(figure=fig, color=(0, 0, 0), nb_labels=4)
        axes.title_text_property.color = (0.0, 0.0, 0.0)
        axes.title_text_property.font_family = 'times'
        axes.label_text_property.color = (0.0, 0.0, 0.0)
        axes.label_text_property.font_family = 'times'
        axes.axes.font_factor = 0.5

        mlab.gcf().scene.parallel_projection = True
        mlab.orientation_axes()
        mlab.show(stop=True)

    def projectPointToTangentPlanes(self,firstCloud, secondCloud, normalsInFirstScan_refFrame):
        """ 
            Project point to tangent plane of associated points.
            Only used in point-to-plane association.

            Parameters
            ----------
            firstCloud : (n,3) ndarray 
                         associated first point cloud ("X")
            secondCloud : (n,3) ndarray
                          associated second point cloud ("Xprime")
            normalsInFirstScan_refFrame : (n,3) ndarray
                                          normals at each point of the second cloud

            Returns
            -------
            projectedPoints : (n,3) ndarray
                              first point cloud projected to the tangent plane of their associated point in the second cloud
        """
        
        vecs = firstCloud - secondCloud
        dot = np.einsum('ij,ij->i', vecs, normalsInFirstScan_refFrame)
        multiply = np.multiply(normalsInFirstScan_refFrame,dot[:, np.newaxis])
        return secondCloud + multiply

    def pointToPlaneAssociations(self,firstCloud, secondCloud,
                                 params_firstCloud, normalsInFirstScan_refFrame):
        """ 
            Point-to-plane association  

            Parameters
            ----------
            firstCloud : (n,3) ndarray 
                         associated first point cloud ("X")
            secondCloud : (n,3) ndarray
                          associated second point cloud ("Xprime")
            params_firstCloud : dict of data
                                contains data related to the local spherical pdf coords of point from first point cloud (X)
            normalsInFirstScan_refFrame : (n,3) ndarray
                                          normals at each point of the second cloud

            Returns
            -------
            projectedPoints : (n,3) ndarray
                              first point cloud projected to the tangent plane of their associated point in the second cloud
            params_translated : dict of data
                                contains data related to the local spherical pdf coords of projected points

            Notes
            -----
            See [1], section 4.1
        """ 

        # Project the points of the second cloud onto the tangent planes at point in the first cloud 
        projectedPoints = self.projectPointToTangentPlanes(firstCloud, secondCloud, normalsInFirstScan_refFrame)

        # Compute the corresponding translations on the tangent planes (ie the vector 'firstCloudPoint-projectedPoint') 
        translationsInTangentPlanes = projectedPoints - firstCloud

        # We now move the first point to the projected point while keeping its parameters (ie local spheric measurements 
        # It means that we have to translate the pose at which it was measured        
        #  TODO : Take in account for the translation variance.  
    
        params_translated = self.copyParams(params_firstCloud)
        params_translated['pose_mean'][:,3:6] = params_firstCloud['pose_mean'][:,3:6] + translationsInTangentPlanes

        return projectedPoints, params_translated

    def pICP(self, q0, modeX, modeXprime, params, params_prime,  errorConvThresh, picpMaxIter,
             normalsInFirstScan_refFrame = None,  pointToPlane=False, firstScanGaussianApprox=None, secondScanGaussianApprox=None):
        """ 
            Main function executing the pIC algorithm 

            Parameters
            ----------
            q0 : dict with ["pose_mean"] and ["pose_cov"]
                 initial relative pose between the two point cloud
            modeX : (n,3) array
                    modes of associated first point cloud 
            modeXprime : (m,3) array 
                         modes of associated second point cloud
            params : dict of data
                     contains data related to the local spherical pdf coords of point from first point cloud (X)
            params_prime : dict of data
                     contains data related to the local spherical pdf coords of point from second point cloud (Xprime)
            errorConvThresh : float
                              termination threshold on error
            picpMaxIter : int
                          maximum number of pICP iteration
            normalsInFirstScan_refFrame : (n,3) ndarray
                                          normals at each point of the second cloud
            pointToPlane : bool
                           If true, use point-to-plane association. Otherwise, default to point-to-point association.
            firstScanGaussianApprox : (n) array of pdf
                                      pdf of gaussian approximation for the first point cloud
            secondScanGaussianApprox : (m) array of pdf
                                      pdf of gaussian approximation for the second point cloud 

            Returns
            -------
            q : dict with ["pose_mean"] and ["pose_cov"]
                estimated relative pose between the two point cloud
            path : dict of arrays with "time", "cost", "pose" keys
                   contains details on the iterations (computation time, cost function value, intermediate poses) 

            Notes
            -----
            See [1], Algorithm 2
        """ 
        
        tau = 3
        e1 = 1e-8
        e2 = 1e-8
        maxIter = 30
        associationThreshold = 0.98

        q=q0

        q_GT = np.array([0., 0., 0., 15., 0. ,0.])

        # Compute init error 
        # Not necessary at all, so comment it to speed up
        modeX_current = poses_euler.inverseComposePoseEulerPoint(q['pose_mean'], modeX)
        modeXprime_current = poses_euler.composePoseEulerPoint(q['pose_mean'], modeXprime)
        modeXprime_current_GT = poses_euler.composePoseEulerPoint(q_GT, modeXprime)

        if(firstScanGaussianApprox == None):
            idxs_pointX, idxs_pointXprime = self.association_bruteForce(q, modeX, modeXprime, params, params_prime, associationThreshold)
        else:
            idxs_pointX, idxs_pointXprime = self.association_approxGaussian(q, modeX, modeXprime, params, params_prime, firstScanGaussianApprox, secondScanGaussianApprox)

        params_assoc = self.getParamsByIndexes(params, idxs_pointX)
        params_prime_assoc = self.getParamsByIndexes(params_prime, idxs_pointXprime)
        params_current = self.copyParams(params_assoc)
        params_prime_current = self.copyParams(params_prime_assoc)
        self.computeParamsCurrent_array(q, params_assoc, params_prime_assoc, params_current, params_prime_current)
        self.initCUT_array(params_current, params_prime_current)
        vals_max = self.computeValsMax(modeX_current[idxs_pointX,:], modeXprime_current[idxs_pointXprime,:], params_current, params_prime_current)
        fval_init,_,_ = self.functionCost_array(modeX[idxs_pointX,:], modeXprime[idxs_pointXprime,:], params_current, params_prime_current,vals_max)
        print("Init cost : {}".format(np.linalg.norm(fval_init)**2))
        path = {'time': [0], 'pose': [q], 'cost': [np.linalg.norm(fval_init)**2]}

        if (pointToPlane):
            modeX_planeProjections, params_planeProjections = self.pointToPlaneAssociations(modeX[idxs_pointX, :],
                                                                                       modeXprime_current[idxs_pointXprime, :],
                                                                                       self.getParamsByIndexes(params,
                                                                                                               idxs_pointX),
                                                                                       normalsInFirstScan_refFrame[idxs_pointX,:])


        fig = mlab.figure(bgcolor=(1, 1, 1))
        '''if (pointToPlane):
            self.pICP_display_mayavi(fig, modeX, modeXprime, modeXprime_current, modeXprime_current_GT,
                                     idxs_pointX, idxs_pointXprime, modeX_planeProjections)
        else:
            self.pICP_display_mayavi(fig, modeX, modeXprime, modeXprime_current, modeXprime_current_GT,
                                     idxs_pointX, idxs_pointXprime)'''

        iter = 0
        hasFinished = False
        while(not hasFinished):
            start = time.time()
            iter += 1

            print("-----------------------------")
            print("MYPICP iteration " + str(iter))
            print("-----------------------------")

            if (firstScanGaussianApprox == None):
                idxs_pointX, idxs_pointXprime = self.association_bruteForce(q, modeX, modeXprime, params, params_prime,associationThreshold)
            else:
                idxs_pointX, idxs_pointXprime = self.association_approxGaussian(q, modeX, modeXprime, params, params_prime, firstScanGaussianApprox, secondScanGaussianApprox)

            print("Associations : ")
            print(idxs_pointX)
            print(idxs_pointXprime)

            # If using the pointToPlane association, associate the point in the second cloud (modeXprime) to their 
            #    projections on the tangent plane to the surface at the associated point in the first cloud (modeX)
            #    In practice, we currently translate the point in the first cloud to the projected points 
            #    (ie the projected points has the pdf of the point in the first cloud) (To be tested !)
            if(pointToPlane):
                modeX_planeProjections, params_planeProjections = self.pointToPlaneAssociations(modeX[idxs_pointX, :], modeXprime_current[idxs_pointXprime, :],
                                                                                           self.getParamsByIndexes(params, idxs_pointX), normalsInFirstScan_refFrame[idxs_pointX,:])
                q, error, _ = self.LM_variante_array(q, modeX_planeProjections, modeXprime[idxs_pointXprime, :],
                                                     params_planeProjections,
                                                     self.getParamsByIndexes(params_prime, idxs_pointXprime),
                                                     tau=tau, e1=e1, e2=e2, maxIter=maxIter)
            else:
                q, error, _ = self.LM_variante_array(q, modeX[idxs_pointX,:], modeXprime[idxs_pointXprime,:],
                                                 self.getParamsByIndexes(params, idxs_pointX),
                                                 self.getParamsByIndexes(params_prime, idxs_pointXprime),
                                                 tau=tau, e1=e1, e2=e2, maxIter=maxIter)

            # IMU constraints 
            alpha_z = 2 * 2 * 0.1
            alpha_pitch = 2*2*np.deg2rad(0.1)
            alpha_roll = alpha_pitch
            if(q['pose_mean'][5] - 0. > alpha_z):
                print('Bound z !')
                q['pose_mean'][5] = alpha_z
            if(q['pose_mean'][5] - 0. < -alpha_z):
                print('Bound z !')
                q['pose_mean'][5] = -alpha_z
            if (q['pose_mean'][1] - 0. > alpha_pitch):
                print('Bound pitch !')
                q['pose_mean'][1] = alpha_pitch
            if (q['pose_mean'][1] - 0. < -alpha_pitch):
                print('Bound pitch !')
                q['pose_mean'][1] = -alpha_pitch
            if (q['pose_mean'][2] - 0. > alpha_roll):
                print('Bound roll !')
                q['pose_mean'][2] = alpha_roll
            if (q['pose_mean'][2] - 0. < -alpha_roll):
                print('Bound roll !')
                q['pose_mean'][2] = -alpha_roll

            compTime = time.time() - start
            path['time'].append(path['time'][len(path['time']) - 1] + compTime)
            path['cost'].append(error)
            path['pose'].append(q)
            print("Error : {}".format(error))
            print("qopt : {}".format(q))

            #modeX_current = poses_euler.inverseComposePoseEulerPoint(q['pose_mean'], modeX)
            modeXprime_current = poses_euler.composePoseEulerPoint(q['pose_mean'], modeXprime)

            if (pointToPlane):
                self.pICP_display_mayavi(fig, modeX, modeXprime, modeXprime_current, modeXprime_current_GT,
                                         idxs_pointX, idxs_pointXprime, modeX_planeProjections)
            else:
                self.pICP_display_mayavi(fig, modeX, modeXprime, modeXprime_current, modeXprime_current_GT,
                                         idxs_pointX, idxs_pointXprime)


            if (error < self.errorConvThresh or iter == self.picpMaxIter):
                hasFinished = True
                print(" ---> MYPICP finished in " + str(iter) + " iterations and error " + str(error))
                print("Final qopt : {}".format(q))

        return q, path

    def LM_variante_array(self, q0, modeX, modeXprime, params, params_prime, tau=3, e1=1e-8, e2=1e-8, maxIter=20):
        """ 
            Levendberg-Marquardt Impl adapted to our problem.
            Main differences with a classic LM algo are on the precomputing of some data and
            the incrementation of poses in se(3)

            Parameters
            ----------
            q0 : dict with ["pose_mean"] and ["pose_cov"]
                 initial relative pose between the two point cloud
            modeX : (n,3) array
                    modes of associated first point cloud 
            modeXprime : (m,3) array 
                         modes of associated second point cloud
            params : dict of data
                     contains data related to the local spherical pdf coords of point from first point cloud (X)
            params_prime : dict of data
                     contains data related to the local spherical pdf coords of point from second point cloud (Xprime)
            tau : float
                  internal parameter related to the initial value of the damping parameter lambda
            e1 : float
                 termination threshold on the L-inf norm of the gradient 
            e2 : float
                 termination threshold on the change in q
            maxIter : int
                      maximum number of iteration

            Returns
            -------
            q : (n) array
                optimized value
            Fval : float
                    final cost function value
            path : dict with 'time', 'cost', 'pose'
                    info at each iteration
        """
        
        q = q0

        self.J_composePose["X"] = -self.J_compose_pose_array(modeX)
        self.J_composePose["X_prime"] = self.J_compose_pose_array(modeXprime)

        #######################################################################################
        # X_mean(0) = X_mean - q0
        # X'_mean(0) = q0 + X'_mean(0)
        #######################################################################################
        modeX_current = poses_euler.inverseComposePoseEulerPoint(q['pose_mean'], modeX)
        modeXprime_current = poses_euler.composePoseEulerPoint(q['pose_mean'], modeXprime)

        #######################################################################################
        # Get the poses Pi, P'i for each point Xi, X'i
        # Pi is the pose of the robot relative to the scan reference frame when Xi was measured
        #######################################################################################
        p = []
        p_prime = []
        for mean_q_i, cov_q_i, mean_q_prime_i, cov_prime_q_i in zip(params["pose_mean"], params["pose_cov"], params_prime["pose_mean"], params_prime["pose_cov"]):
            p_i = {'pose_mean': mean_q_i, 'pose_cov': cov_q_i}
            p.append(p_i)
            p_prime_i = {'pose_mean': mean_q_prime_i,
                         'pose_cov': cov_prime_q_i}
            p_prime.append(p_prime_i)

        #######################################################################################
        # Compute Qi = Pi - q0, Q'i = q0 + P'i
        # Replace Pi,P'i by Qi,Q'i in the parameters (corresponding to the original version, cf LM)
        #######################################################################################
        ''' Full copy without references '''
        params_current = self.copyParams(params)
        params_prime_current = self.copyParams(params_prime)

        self.computeParamsCurrent_array(q,params, params_prime, params_current, params_prime_current)
        self.initCUT_array(params_current, params_prime_current)

        #######################################################################################
        # Compute the maximum values of the pdfs Xi(0) and X'i(0) (for normalization)
        # ie f_Xi(0)(modeXi(0)), f_X'i(0)(modeX'i(0))
        #######################################################################################

        vals_max = self.computeValsMax(modeX_current, modeXprime_current, params_current, params_prime_current)
        fvals, fX_array, fXprime_array = self.functionCost_array(modeX, modeXprime, params_current,
                                                                 params_prime_current, vals_max)
        J_f = self.J_F_opt_array(params_current, params_prime_current, fX_array, fXprime_array, vals_max)

        '''----------------------------------------------------------------------------'''
        ''' TODO : Avoid recomputing and update the self values with the right indexes '''
        '''----------------------------------------------------------------------------'''
        '''nPoint = len(idxs_pointX)
        idxs_flatten = nPoint*idxs_pointXprime + idxs_pointX
        fvals = costMatrix.ravel()[idxs_flatten] #costMatrix[idxs_pointXprime][idxs_pointX].ravel()
        J_f = self.J_F_opt_array(params_current, params_prime_current,
                                 fX_matrix.ravel()[idxs_flatten],#fX_matrix[idxs_pointXprime][idxs_pointX].ravel(),
                                 fXprime_matrix.ravel()[idxs_flatten],#fXprime_matrix[idxs_pointXprime][idxs_pointX].ravel(),
                                 vals_max_association.ravel()[idxs_flatten])#vals_max_association[idxs_pointXprime][idxs_pointX].ravel())'''

        Fvals = np.linalg.norm(fvals)**2
        #print("First Fvals : {}".format(Fvals))

        J_fT = J_f.T
        H = J_fT@J_f
        g = J_fT@fvals

        ''' Keep data at each iterations (used for plots) '''
        path = {'time': [0], 'pose': [q], 'cost': [Fvals]}

        ''' Check g norm '''
        hasConverged = (np.linalg.norm(g, np.inf) <= e1)
        if(hasConverged):
            print(" Ending condition : norm_inf(g) = {} <= e1".format(g))

        lambda_val = tau * np.amax(np.diagonal(H))
        iter = 0
        v = 2.

        #print("[LM] Init finished. Start iterations ...")
        I_6 = np.eye(6)
        while(not hasConverged and iter < maxIter):
            start = time.time()
            iter += 1
            #print("-- Current iteration : {}".format(iter))
            #print("Lambda : {}".format(lambda_val))
            epsilon = -np.linalg.solve(H + lambda_val*I_6, g)
            #print("  epsilon* : {}".format(epsilon))
            epsilon_norm = np.linalg.norm(epsilon)
            if(epsilon_norm < (e2**2)):
                hasConverged = True
                print("Ending condition on epsilon norm: {} < {}".format(epsilon, e2**2))
            else:
                ''' Increment '''
                exp_epsilon_R, exp_epsilon_t = math_utility.exp_SE3(epsilon)
                qincr = np.concatenate((scipy.spatial.transform.Rotation.from_matrix(exp_epsilon_R).as_euler('ZYX'), exp_epsilon_t))
                qnew = {'pose_mean' : poses_euler.composePoseEuler(q['pose_mean'], qincr), 'pose_cov': q['pose_cov']}

                self.computeParamsCurrent_array(qnew, params, params_prime, params_current, params_prime_current)
                self.initCUT_array(params_current, params_prime_current)

                modeX_current = poses_euler.inverseComposePoseEulerPoint(qnew['pose_mean'], modeX)
                modeXprime_current = poses_euler.composePoseEulerPoint(qnew['pose_mean'], modeXprime)

                vals_max = self.computeValsMax(modeX_current, modeXprime_current, params_current,
                                               params_prime_current)
                fvals, fX_array, fXprime_array = self.functionCost_array(modeX, modeXprime, params_current,
                                                                         params_prime_current, vals_max)

                Fvals_new = np.linalg.norm(fvals)**2

                tmp = lambda_val*epsilon - g
                denom = 0.5*np.dot(tmp.flatten(), epsilon.flatten())
                l = (Fvals - Fvals_new)/denom
                #print("l : {}".format(l))
                if(l > 0):
                    #print("Accept increment q_incr : {}".format(qincr))
                    q = qnew

                    Fvals = Fvals_new
                    #print("Current Fval : {}".format(Fvals))

                    J_f = self.J_F_opt_array(params_current, params_prime_current,
                                             fX_array, fXprime_array, vals_max)

                    J_fT = J_f.T
                    H = J_fT@J_f
                    g = J_fT@fvals

                    ''' Check g norm '''
                    norm_g = np.linalg.norm(g, np.inf)
                    hasConverged = (norm_g <= e1)
                    if (hasConverged):
                        print(" Ending condition : norm_inf(g) = {} <= e1".format(g))

                    lambda_val *= np.max([0.33, 1. - (2.*l - 1.)**3.])
                    v = 2.

                    compTime = time.time() - start
                    path['time'].append( path['time'][len(path['time'])-1] + compTime )
                    path['cost'].append(Fvals)
                    path['pose'].append(q)
                else:
                    lambda_val *= v
                    v *= 2.

        #print("-- LM finished --")
        #print("Final sqr error : {}".format(Fvals))
        #print("Iterations : {}".format(iter))
        #print("Final pose : {}".format(q))
        return q, Fvals, path

    def DogLeg_variante_array(self, delta0, q0, modeX, modeXprime, params, params_prime):
        """
            DogLeg implementation. Should normally used the LM algo for optimization.
            This was implemented for testing purpose.

            Parameters
            ----------
            delta0 : float
                     initial trust region radius
            q0 : dict with ["pose_mean"] and ["pose_cov"]
                 initial relative pose between the two point cloud
            modeX : (n,3) array
                    modes of associated first point cloud 
            modeXprime : (m,3) array 
                         modes of associated second point cloud
            params : dict of data
                     contains data related to the local spherical pdf coords of point from first point cloud (X)
            params_prime : dict of data
                     contains data related to the local spherical pdf coords of point from second point cloud (Xprime)

            Returns
            -------
            q : (n) array
                optimized value
            path : dict with 'time', 'cost', 'pose'
                   info at each iteration

            Notes
            -----
            Implemented as proposed in [3], section 3.3
        """
        
        e1 = 1e-8
        e2 = 1e-8
        e3 = 1e-8
        maxIter = 50

        # print("[LM] Init ...")
        q = q0
        delta = delta0

        # Precompute J_composePose
        eself.J_composePos["X"] = -self.J_compose_pose_array(modeX)
        self.J_composePose["X_prime"] = self.J_compose_pose_array(modeXprime)

        #######################################################################################
        # X_mean(0) = X_mean - q0
        # X'_mean(0) = q0 + X'_mean(0)
        #######################################################################################
        modeX_current = poses_euler.inverseComposePoseEulerPoint(q['pose_mean'], modeX)
        modeXprime_current = poses_euler.composePoseEulerPoint(q['pose_mean'], modeXprime)

        #######################################################################################
        # Get the poses Pi, P'i for each point Xi, X'i
        # Pi is the pose of the robot relative to the scan reference frame when Xi was measured
        #######################################################################################
        p = []
        p_prime = []
        for mean_q_i, cov_q_i, mean_q_prime_i, cov_prime_q_i in zip(params["pose_mean"], params["pose_cov"],
                                                                    params_prime["pose_mean"], params_prime["pose_cov"]):
            p_i = {'pose_mean': mean_q_i, 'pose_cov': cov_q_i}
            p.append(p_i)
            p_prime_i = {'pose_mean': mean_q_prime_i,
                         'pose_cov': cov_prime_q_i}
            p_prime.append(p_prime_i)
        # print("At start P : {}".format(p))

        #######################################################################################
        # Compute Qi = Pi - q0, Q'i = q0 + P'i
        # Replace Pi,P'i by Qi,Q'i in the parameters (corresponding to the original version, cf LM)
        #######################################################################################
        # TODO : Refactor this to speed up
        # Full copy without references 
        params_current = self.copyParams(params)
        params_prime_current = self.copyParams(params_prime)

        self.computeParamsCurrent_array(q, params, params_prime, params_current, params_prime_current)

        self.initCUT_array(params_current, params_prime_current)

        #######################################################################################
        # Compute the maximum values of the pdfs Xi(0) and X'i(0) (for normalization)
        # ie f_Xi(0)(modeXi(0)), f_X'i(0)(modeX'i(0))
        #######################################################################################
        vals_max = self.computeValsMax(modeX_current, modeXprime_current, params_current, params_prime_current)

        fvals, fX_array, fXprime_array = self.functionCost_array(modeX, modeXprime, params_current,
                                                                 params_prime_current, vals_max)

        J_f = self.J_F_opt_array(params_current, params_prime_current, fX_array, fXprime_array, vals_max)
        Fvals = np.linalg.norm(fvals) ** 2

        J_fT = J_f.T
        H = J_fT @ J_f
        g = J_fT @ fvals

        # Keep data at each iterations 
        path = {'time': [0], 'pose': [q], 'cost': [Fvals]}

        hasConverged = (np.linalg.norm(fvals,np.inf) < e3) or (np.linalg.norm(g,np.inf) < e1)
        iter = 0
        I_6 = np.eye(6)
        eps = 1e-6
        while(not hasConverged and iter < maxIter):
            start = time.time()
            iter += 1
            print("-- Current iteration : {}".format(iter))
            norm_g = np.linalg.norm(g)
            alpha = (norm_g/np.linalg.norm(J_f@g))**2
            h_sd = -alpha*g # steepest descent
            h_gn = -np.linalg.solve(H + eps*I_6,g)# Gauss-Newton
            # Compute h_dl (dogleg)
            if(np.linalg.norm(h_gn) <= delta):
                h_dl = h_gn
                denum = Fvals
            elif(np.linalg.norm(alpha*h_sd) >= delta):
                norm_h_sd = np.linalg.norm(h_sd)
                h_dl = (delta/norm_h_sd)*h_sd
                denum = delta*(2*norm_h_sd - delta)/(2*alpha)
            else:
                a = alpha*h_sd
                c = np.dot(a, h_gn - a)
                norm_b_minus_a_sqr = np.linalg.norm(h_gn - a) ** 2
                if(c <= 0):
                    beta = (-c + np.sqrt(c**2 + norm_b_minus_a_sqr * (delta**2 - np.linalg.norm(a)**2) ))/ norm_b_minus_a_sqr
                else:
                    beta = (delta**2 - np.linalg.norm(a)**2)/(c + np.sqrt(c**2 + norm_b_minus_a_sqr * (delta**2 - np.linalg.norm(a)**2) ))
                h_dl = alpha*h_sd + beta*(h_gn - alpha*h_sd)
                denum = 0.5*alpha*(1 - beta)**2 * norm_g**2 + beta*(2-beta)*Fvals

            if(np.linalg.norm(h_dl) <= e2**2):
                hasConverged = True
                print("Has converged with ||h_dl|| <= e2")
            else:
                # Increment
                exp_epsilon_R, exp_epsilon_t = math_utility.exp_SE3(h_dl)
                qincr = np.concatenate(
                    (scipy.spatial.transform.Rotation.from_matrix(exp_epsilon_R).as_euler('ZYX'), exp_epsilon_t))
                # print("qincr : {}".format(qincr))

                qnew = {'pose_mean': poses_euler.composePoseEuler(q['pose_mean'], qincr), 'pose_cov': q['pose_cov']}

                self.computeParamsCurrent_array(qnew, params, params_prime, params_current, params_prime_current)
                self.initCUT_array(params_current, params_prime_current)

                modeX_current = poses_euler.inverseComposePoseEulerPoint(qnew['pose_mean'], modeX)
                modeXprime_current = poses_euler.composePoseEulerPoint(qnew['pose_mean'], modeXprime)
                vals_max = self.computeValsMax(modeX_current, modeXprime_current, params_current, params_prime_current)

                fvals, fX_array, fXprime_array = self.functionCost_array(modeX, modeXprime, params_current,
                                                                         params_prime_current, vals_max)

                Fnew = np.linalg.norm(fvals) ** 2

                l = (Fvals - Fnew)/denum
                print("l : {}".format(l))
                if(l > 0):
                    q = qnew
                    Fvals = Fnew

                    J_f = self.J_F_opt_array(params_current, params_prime_current, fX_array, fXprime_array, vals_max)
                    J_fT = J_f.T
                    H = J_fT @ J_f
                    g = J_fT @ fvals

                    hasConverged = (np.linalg.norm(fvals,np.inf) < e3) or (np.linalg.norm(g,np.inf) < e1)

                    compTime = time.time() - start
                    path['time'].append(path['time'][len(path['time']) - 1] + compTime)
                    path['cost'].append(Fvals)
                    path['pose'].append(q)

                if(l > 0.75):
                    delta = np.max([delta, 3*np.linalg.norm(h_dl)])
                elif(l<0.25):
                    delta *= 0.5
                    hasConverged = (delta <= e2*(np.linalg.norm(q["pose_mean"]) + e2))

        return q, path