import numpy as np
import scipy as scp
import scipy.spatial.transform
import poses_euler
import poses_quat
import poses2D
import math_utility
import math
import time
import PICP_display
import dataAssociation
import multiprocessing as mp
from abc import ABC, abstractmethod
import betaDist

# Generators of SO(3)
G = np.zeros((6,3,3))
G[0] = np.array([[0,0,0],
                 [0,0,-1],
                 [0,1,0]])
G[1] = np.array([[0, 0, 1],
                 [0, 0, 0],
                 [-1, 0, 0]])
G[2] = np.array([[0, -1, 0],
                 [1, 0, 0],
                 [0, 0, 0]])

class gaussianPICP(ABC):
    """ 
        Base class for 3D Gaussian pIC 
        Notations based on [1]
        
        References
        ----------
        .. [1] Yohan Breux and André Mas and Lionel Lapierre, "On-manifold Probabilistic ICP : Application to Underwater Karst
        Exploration"
        .. [2]  METHODS FOR NON-LINEAR LEAST SQUARES PROBLEMS, K. Madsen, H.B. Nielsen, O. Tingleff, 2004
        .. [3] Mallios, A.; Ridao, P.; Ribas, D.; Hernández, E. Scan matching SLAM in underwater environments.  
               Auton. Robot. 2014, 36, 181–198
    """
    
    def __init__(self, a_pts_array, c_pts_array):
        self.a_pts_array = a_pts_array
        self.a_pts_array_current = None
        self.a_pts_array_new = None
        self.c_pts_array = c_pts_array
        self.a_pts_array_assoc = a_pts_array
        self.c_pts_array_assoc = c_pts_array
        self.errors_mean = None
        self.errors_cov_inv = None
        self.associationType = "pointToPoint"
        self.normalsInFirstScan_refFrame = {"mean": None, "cov": None}
        self.showDisplay = True
        self.picpMaxIter = 40
        self.errorConvThresh = 1e-6
        self.LM_maxIter = 100
        self.tau = 1
        self.e1 = 1e-8
        self.e2 = 1e-8
        self.chi2_p = 0.5 #0.5
        self.representationDim = 6

    def initializeData(self):
        pass

    def computeOmega(self, q):
        pass

    def compute_ni(self, q, ci, cov_ci):
        pass

    @abstractmethod
    def compute_nPts(self, q):
        """ 
            Point of the second cloud after being applied the transformation q 
            ie n_i = q + c_i

            Parameters
            ----------
            q : dict with "pose_mean" ((6) array) and "pose_cov" ((6,6) ndarray)
                pose pdf 
        """
        pass

    def costFunction(self, q):
        """ 
            Cost function defined as the sum of mahalanobis distances between each associated points

            Parameters
            ----------
            q : dict with "pose_mean" ((6) array) and "pose_cov" ((6,6) ndarray)
                pose pdf

            Returns
            ------- 
            fval : float
                   cost function value

            Notes
            -----
            Cost function defined in [1], equation (30)   
        """
        
        fval = np.einsum('ki,ki', self.errors_mean, np.einsum('kij,kj->ki', self.errors_cov_inv, self.errors_mean))
        return np.sqrt(fval)

    def costFunctionJacobian(self, q, fval):
        """ 
            Compute the cost function jacobian numerically (default implementation)

            Parameters
            ----------
            q : dict with "pose_mean" ((6) array) and "pose_cov" ((6,6) ndarray)
                pose pdf
            fval : float
                   cost function value
            
            Returns
            -------
            jacobian : (1,n) ndarray
                      1 X n jacobian matrix where n is the SE(3) representation dimension (6 for euler, 7 for quat)
        """

        # By default, compute the jacobian numerically 
        return math_utility.numericalJacobian_pdf(self.costFunction, 1, q, np.full((self.representationDim,), 1e-4))

    def costFunctionHessian_q(self, q):
        """
            Comptue the cost function hessian numerically (default impl)

            Parameters
            ----------
            q : dict with "pose_mean" ((6) array) and "pose_cov" ((6,6) ndarray)
                pose pdf

            Returns
            -------
            hessian : (n,n) ndarray
                      n X n hessian matrix where n is the SE(3) representation dimension (6 for euler, 7 for quat)
        """

        # By default, compute the hessian w.r.t to q numerically
        return math_utility.numericalHessian_pdf(self.costFunction, 1, q, np.full((self.representationDim,), 1e-4))

    def costFunction_zi(self, q, ai, ci, cov_ai, cov_ci):
        """ 
            Cost function with points (ai,ci) = z_i as parameters 

            Parameters
            ----------
            q : dict with "pose_mean" ((6) array) and "pose_cov" ((6,6) ndarray)
                pose pdf
            ai : (n) array
                 nD mean point from the first point cloud
            ci : (n) array
                 nD mean point from the second point cloud
            cov_ai : (n,n) ndarray
                     covariance matrix of ai
            cov_ci : (n,n) ndarray
                     covariance matrix of ci

            Returns
            -------
            fval : float
                   cost function for only one association pair (ai, ci)
        """

        ni = self.compute_ni(q, ci, cov_ci)
        ei_mean = ni["mean"] - ai
        ei_cov_inv = np.linalg.inv(cov_ai + ni["cov"])
        fval = np.dot(ei_mean, ei_cov_inv@ei_mean)
        return np.sqrt(fval)

    def costFunctionHessian_zi(self, q, ai , ci, cov_ai, cov_ci):
        """ 
            Compute the second derivative df²/dqdz_i (diag block of the hessian)
            for the i-th points numerically (default impl)
            
            Parameters
            ----------
            q : dict with "pose_mean" ((6) array) and "pose_cov" ((6,6) ndarray)
                pose pdf
            ai : (n) array
                 nD mean point from the first point cloud
            ci : (n) array
                 nD mean point from the second point cloud
            cov_ai : (n,n) ndarray
                     covariance matrix of ai
            cov_ci : (n,n) ndarray
                     covariance matrix of ci

            Returns
            -------
            H_zi : (n,2*n) ndarray
                   Hessian matrix of the cost function relative to the points (ai, ci) = zi
                   n is the SE(3) representation dimension (6 for euler, 7 for quat)
        """

        double_dim = 2*self.dim

        H_zi = np.zeros((self.representationDim, double_dim))
        H_zi[:,0:self.dim] = math_utility.numericalJacobian(lambda a: math_utility.numericalJacobian_pdf(lambda x: self.costFunction_zi(x, a, ci, cov_ai, cov_ci), 1, q,
                                                                                       np.full((self.representationDim,),1e-8)).T,
                                              self.representationDim, ai, np.full((self.dim,),1e-8))
        H_zi[:,self.dim:double_dim] = math_utility.numericalJacobian(lambda c: math_utility.numericalJacobian_pdf(lambda x: self.costFunction_zi(x, ai, c, cov_ai, cov_ci), 1, q,
                                                                                       np.full((self.representationDim,),1e-8)).T,
                                              self.representationDim, ci, np.full((self.dim,),1e-8))

        return H_zi

    def costFunctionHessian_z(self, q):
        """ 
            Compute the second derivative df²/dqdz (diag block of the hessian) 

            Parameters
            ----------
            q : dict with "pose_mean" ((6) array) and "pose_cov" ((6,6) ndarray)
                pose pdf
            
            Returns
            -------
            H_z : (n, 2*n) ndarray
                  Hessian matrix relative to the points (ai,ci) = zi
            
            Notes
            -----
            Corresponds to H^F_z in [1], equation (47)
        """

        n = self.a_pts_array_assoc["mean"].shape[0]
        H_z = np.empty((n, self.representationDim, 2*self.dim))
        i=0
        for ai, ci, cov_ai, cov_ci in zip(self.a_pts_array_assoc["mean"], self.c_pts_array_assoc["mean"], 
                                          self.a_pts_array_assoc["cov"], self.c_pts_array_assoc["cov"]):
            H_zi = self.costFunctionHessian_zi(q, ai, ci, cov_ai, cov_ci)
            H_z[i,:,:] = H_zi
            i+=1
        
        return H_z

    def computeSigmaZ(self):
        """ 
            Compute the covariance related to points (z)

            Returns
            -------
            Sigma_z : (m, 2*n, 2*n) ndarray
                      array of m covariance matrices Sigma_zi
                      n is the SE(3) representation dimension (6 for euler, 7 for quat) 

            Notes
            -----
            Corresponds to [1], equation (48) (with a reshaping (2*n*m, 2*n*m) -> (m, 2*n ,2*n)) 
        """
        
        double_dim = 2 * self.dim
        n = self.a_pts_array_assoc["mean"].shape[0]
        Sigma_z = np.zeros((n, double_dim, double_dim))
        Sigma_z[:, 0:self.dim, 0:self.dim] = self.a_pts_array_assoc["cov"]
        Sigma_z[:, self.dim:double_dim, self.dim:double_dim] = self.c_pts_array_assoc["cov"]
        return Sigma_z

    def compute_errors(self, q):
        """ 
            Compute errors (mean and covariance)

            Parameters
            ----------
            q : dict with "pose_mean" ((6) array) and "pose_cov" ((6,6) ndarray)
                pose pdf

            Notes
            -----
            Defined in [1], equations (27)(28)(29)
        """
        
        n_pts_array = self.compute_nPts(q)
        self.errors_mean = n_pts_array["mean"] - self.a_pts_array_assoc["mean"]
        self.errors_cov_inv = np.linalg.inv(self.a_pts_array_assoc["cov"] + n_pts_array["cov"])

    @abstractmethod
    def updateForLM(self, q):
        pass

    @abstractmethod
    def incrementLM(self, q):
        pass

    def pointToPlaneAssociations(self, nPts, assocIdxs):
        """ 
            point-to-plane association based on normals at each point on the first point cloud 

            Parameters
            ----------
            nPts : (m, n) ndarray
                   n = q + c mean points
            assocIdxs : (m) int array
                        Indexes of associated points in the first scan (ie the points ai)
                   
            Notes
            -----
            See [1], section 4.1 for details.
        """
        
        # Project the points of the second cloud onto the tangent planes at point in the first cloud
        vecs = self.a_pts_array_assoc - nPts
        dot = np.einsum('ij,ij->i', vecs, self.normalsInFirstScan_refFrame["mean"][assocIdxs])
        multiply = np.multiply(self.normalsInFirstScan_refFrame["mean"][assocIdxs], dot[:, np.newaxis])
        return nPts + multiply

    def LevenbergMarquardt(self, q0):
        """ 
            Levendberg Marquardt algorithm slightly adapted by using precomputed values 
            
            Parameters
            ----------
            q0 : dict with "pose_mean" ((6) array) and "pose_cov" ((6,6) ndarray) keys
                 initial pose to be optimized

            Returns
            -------
            q : dict with "pose_mean" ((6) array) and "pose_cov" ((6,6) ndarray) keys
                optimized pose 
            Fval : float
                   final cost function value
            path : dict of arrays with "time", "cost", "pose" keys
                   contains details on the iterations (computation time, cost function value, intermediate poses) 

            Notes
            -----
            Based on the version proposed in [2]
        """ 

        q = q0
        n = len(q0["pose_mean"])
        identity = np.eye(n)

        self.initializeData()
        self.a_pts_array_current = self.a_pts_array
        self.updateForLM(q)

        fval = self.costFunction(q)
        J_f = self.costFunctionJacobian(q, fval)

        Fval = np.linalg.norm(fval) ** 2

        J_fT = np.transpose(J_f)
        H = np.matmul(J_fT, J_f)
        g = fval * J_fT

        # Check g norm 
        hasConverged = (np.linalg.norm(g, np.inf) <= self.e1)
        #if (hasConverged):
        #   print(" Ending condition : norm_inf(g) = {} <= e1".format(g))

        lambda_val = self.tau * np.amax(np.diagonal(H))
        iter = 0
        v = 2.

        # Keep data at each iterations 
        path = {'time': [0], 'pose': [q], 'cost': [Fval]}

        while (not hasConverged and iter < self.LM_maxIter):
            start = time.time()
            iter += 1
            H_inv = np.linalg.inv(H + lambda_val * identity)
            epsilon_mean = -np.matmul(H_inv, g).flatten()
            epsilon_mean_norm = np.linalg.norm(epsilon_mean)
            if (epsilon_mean_norm < (self.e2 ** 2)):
                hasConverged = True
                #print("Ending condition on epsilon norm: {} < {}".format(epsilon_mean, self.e2 ** 2))
            else:
                # Increment 
                qnew = self.incrementLM(q, epsilon_mean)

                self.updateForLM(qnew)

                fval = self.costFunction(qnew)
                Fval_new = np.linalg.norm(fval) ** 2

                tmp = lambda_val * epsilon_mean - g.flatten()
                denom = 0.5 * np.dot(tmp.flatten(), epsilon_mean)
                l = (Fval - Fval_new) / denom
                if (l > 0):
                    q = qnew
                    Fval = Fval_new

                    J_f = self.costFunctionJacobian(q, fval)
                    J_fT = np.transpose(J_f)
                    H = np.matmul(J_fT, J_f)
                    g = fval * J_fT

                    # Check g norm 
                    norm_g = np.linalg.norm(g, np.inf)
                    hasConverged = (norm_g <= self.e1)
                    #if (hasConverged):
                    #    print(" Ending condition : norm_inf(g) = {} <= e1".format(g))

                    lambda_val *= np.max([0.33, 1. - (2. * l - 1.) ** 3.])
                    v = 2.
                    compTime = time.time() - start
                    path['time'].append(path['time'][len(path['time']) - 1] + compTime)
                    path['cost'].append(Fval)
                    path['pose'].append(q)
                else:
                    lambda_val *= v
                    v *= 2.

        # print("-- LM finished --")
        # print("Final sqr error : {}".format(Fval))
        # print("Iterations : {}".format(iter))
        # print("Final pose : {}".format(x))
        return q, Fval, path

    def displayCurrentState(self, n_pts, assoc_index_N = None, n_gt_pts=None, n_init_pts=None, normals=None):
        """ 
            Display the pIC state (original / GT / transformed with current estimated pose point clouds)

            Parameters
            ----------
            n_pts : dict of arrays with "mean" ((n, 3) array) and "cov" ((n, 3,3) ndarray) keys
                    points of the second cloud after current transformation ie n_i = q + c_i
            assoc_index_N : (n) array of int , optional
                            indexes of points in n_pts which have a valid association with the other point cloud
            n_gt_pts : dict of arrays with "mean" ((n, 3) array) and "cov" ((n,3,3) ndarray) keys , optional
                       Ground-truth points of the second cloud after transformation with groudh truth pose ie n_i = q_gt + c_i
            n_init_pts : dict of arrays with "mean" ((n,3) array) and "cov" ((n,3,3) ndarray) keys , optional
                         initial points of the second cloud after transformation with initial pose ie n_i = q0 + c_i
            normals : dict of arrays of "mean" ((n,3) array) and "cov" ((n,3,3) array)
                      normals pdf at points in the first poitn cloud 
        """

        if(self.showDisplay):
            self.viewer.display(self.a_pts_array, self.c_pts_array, n_pts,
                                n_gt_pts, n_init_pts, self.a_pts_array_assoc, assoc_index_N, normals)

    def PICP(self, q0, q_gt_mean=None):
        """ 
            Main function : run the full pIC algo as described in Algorithm 2 of the paper

            Parameters
            ----------
            q0 : dict with "pose_mean" ((6) array) and "pose_cov" ((6,6) ndarray) keys
                 initial pose to be optimized
            q_gt_mean : (6) array
                        Ground-truth mean pose between the two point clouds

            Returns
            -------
            q : dict with "pose_mean" ((6) array) and "pose_cov" ((6,6) ndarray) keys
                optimized pose 
            path : dict of arrays with "time", "cost", "pose" keys
                   contains details on the iterations (computation time, cost function value, intermediate poses) 
        """

        # Create the display window if required
        if(self.showDisplay):
            self.viewer = PICP_display.PICP_display(showAssociations = True)
            self.viewer.associationType = self.associationType
        
        associationThreshold = scipy.stats.chi2.ppf(1. - self.chi2_p, df=self.dim)

        # Generate the ground truth second point cloud 
        # (second point transformed with the GT transformation)
        n_gt_points = None
        if (q_gt_mean is not None):
            n_gt_points = poses_euler.composePoseEulerPoint(q_gt_mean, self.c_pts_array["mean"])
        q = q0

        # Init
        path = {'time': [0], 'pose': [q0], 'cost': [100000]}
        iter = 0
        hasFinished = False

        # Main loop 
        while (not hasFinished):
            start = time.time()
            iter += 1

            self.c_pts_array_assoc = self.c_pts_array
            self.a_pts_array_assoc = self.a_pts_array
            
            # Precompute useful variables 
            self.initializeData()
            self.computeOmega(q)
            n_pts = self.compute_nPts(q)
            
            if(iter == 1):
                n_init_pts = n_pts

            # Display 
            self.displayCurrentState(n_pts, None, n_gt_points, n_init_pts)

            # Association step 
            idxs_pointsA, idxs_pointsN, indiv_compat_A = dataAssociation.dataAssociation(
                dataAssociation.mahalanobisMetric,
                self.a_pts_array, n_pts,
                associationThreshold)

            # No association found ?
            if(idxs_pointsN.shape[0] == 0):
                print("No association --> Stop PICP")
                hasFinished = True
                break

            # Subset of point clouds limited to associated points
            self.c_pts_array_assoc = {"mean": self.c_pts_array["mean"][idxs_pointsN], "cov": self.c_pts_array["cov"][idxs_pointsN]}
            self.a_pts_array_assoc = {"mean": self.a_pts_array["mean"][idxs_pointsA],
                                      "cov": self.a_pts_array["cov"][idxs_pointsA]}

            # In the case of point-to-plane assocation, 
            # the associated point is replaced by its projection on the tangent plane 
            if(self.associationType == "pointToPlane"):
                self.a_pts_array_assoc = self.pointToPlaneAssociations(self.a_pts_array_assoc["mean"],
                                                                       n_pts["mean"][idxs_pointsN],
                                                                       n_pts["cov"][idxs_pointsN],
                                                                       self.normalsInFirstScan_refFrame["mean"][idxs_pointsA, :],
                                                                       self.normalsInFirstScan_refFrame["cov"][idxs_pointsA, :, :]
                                                                               )
            # Optimization step
            q, error, _ = self.LevenbergMarquardt(q)

            # Display
            if (self.associationType == "pointToPlane"):
                self.displayCurrentState(n_pts, idxs_pointsN, n_gt_points, self.normalsInFirstScan_refFrame["mean"][idxs_pointsA, :])
            else:
                self.displayCurrentState(n_pts, idxs_pointsN, n_gt_points)

            # Store info on the current iteration
            compTime = time.time() - start
            path['time'].append(path['time'][len(path['time']) - 1] + compTime)
            path['cost'].append(error)
            path['pose'].append(q)

            # Check the terminating conditions
            if (error < self.errorConvThresh or iter == self.picpMaxIter):
                '''if (self.associationType == "pointToPlane"):
                    self.displayCurrentState(n_pts, idxs_pointsN, n_gt_points, n_init_pts, self.normalsInFirstScan_refFrame["mean"][idxs_pointsA, :])
                else:
                    self.displayCurrentState(n_pts, idxs_pointsN, n_gt_points, n_init_pts)'''
                hasFinished = True

        return q, path

    def LM_covariance_num(self, q):
        """ 
            Covariance of the estimated pose computed numerically 

            Parameters
            ----------
            q : dict with "pose_mean" ((6) array) and "pose_cov" ((6,6) ndarray) keys
                pose pdf

            Returns
            -------
            cov : (6,6) ndarray
                  Covariance matrix of the pose

            Notes
            -----
            In particular, used for testing the closed form expressions
        """ 
        
        H_q = self.costFunctionHessian_q(q)
        H_q_inv = np.linalg.inv(H_q)
        H_z = self.costFunctionHessian_z(q)
        Sigma_z = self.computeSigmaZ()

        # Sum_i H_q,zi Sigma_zi H_q,zi^T 
        A = np.sum(np.einsum('kij, kjl->kil', H_z, np.einsum("kij,klj->kil", Sigma_z, H_z)), axis=0)
        return H_q_inv @ A @ H_q_inv

class gaussianPICP_2D(gaussianPICP):
    """ 
        Specialization of the base class in the 2D case with normally distributed points
        This corresponds to the work propose by [3] 
    """
    
    def __init__(self, a_pts_array, c_pts_array):
        gaussianPICP.__init__(self, a_pts_array, c_pts_array)
        self.dim = 2
        self.representationDim = 3

    def compute_ni(self, q, ci, cov_ci):
        """
            Compute the point pdf n_i defined by n_i = q + c_i

            Parameters
            ----------
            q : dict with "pose_mean" ((3) array) and "pose_cov" ((3,3) ndarray) keys
                current estimated pose pdf between the two point clouds
            ci : (2) array
                 point in the second point cloud
            cov_ci : (2,2) ndarray
                     covariance of c_i
            
            Returns
            -------
            n_i : dict with "mean" ((2) array) and "cov" ((2,2) ndarray)
                  n_i = q + c_i 
        """
        return poses2D.composePosePDFPoint(q, {"mean":ci, "cov": cov_ci})

    def compute_nPts(self, q):
        ""
        return poses2D.composePosePDFPoint_array(q, self.c_pts_array_assoc)

    def costFunction(self, q):
        """ 
            Cost function defined as the sum of mahalanobis distances between each associated points

            Parameters
            ----------
            q : dict with "pose_mean" ((3) array) and "pose_cov" ((3,3) ndarray)
                pose pdf

            Returns
            ------- 
            fval : float
                   cost function value

            Notes
            -----
            Cost function defined in [1], equation (30)   
        """
        
        self.updateForLM(q)
        return super().costFunction(q)

    def updateForLM(self, q):
        """
            Update intermediate values for the LM optimization

            Paramters
            ---------
            q : dict with "pose_mean" ((3) array) and "pose_cov" ((3,3) ndarray)
                pose pdf
        """

        self.compute_errors(q)

    def incrementLM(self, q, eps):
        """
            Pose increments in the LM algorithm

            Parameters
            ----------
            q : dict with "pose_mean" ((3) array) and "pose_cov" ((3,3) ndarray)
                pose pdf
            eps : (3) array
                  pose increment obtained in the last LM iteration
            
            Returns
            ------
            q' : dict with "pose_mean" ((3) array) and "pose_cov" ((3,3) ndarray)
                           incremented pose q' = q + eps
                           
        """
        return {"pose_mean": poses2D.composePose(q["pose_mean"], eps),
                "pose_cov": q["pose_cov"]}

class gaussianPICP_Euler(gaussianPICP):
    """ 
        Specialization for the generic 3D case using Euler angles for representing rotations 
    """

    def __init__(self, a_pts_array, c_pts_array):
        gaussianPICP.__init__(self, a_pts_array, c_pts_array)
        self.dim = 3
        self.representationDim = 6

    def compute_ni(self, q, ci, cov_ci):
        """
            Compute the point pdf n_i defined by n_i = q + c_i

            Parameters
            ----------
            q : dict with "pose_mean" ((6) array) and "pose_cov" ((6,6) ndarray) keys
                current estimated pose pdf between the two point clouds
            ci : (3) array
                 point in the second point cloud
            cov_ci : (3,3) ndarray
                     covariance of c_i
            
            Returns
            -------
            n_i : dict with "mean" ((3) array) and "cov" ((3,3) ndarray)
                  n_i = q + c_i 
        """
        return poses_euler.composePosePDFEulerPoint(q, {"mean":ci, "cov": cov_ci})

    def compute_nPts(self, q):
        """ 
            Point of the second cloud after being applied the transformation q 
            ie n_i = q + c_i

            Parameters
            ----------
            q : dict with "pose_mean" ((6) array) and "pose_cov" ((6,6) ndarray)
                pose pdf 
        """
        return poses_euler.composePosePDFEulerPoint_array(q, self.c_pts_array_assoc)

    def costFunction(self, q):
        """ 
            Cost function defined as the sum of mahalanobis distances between each associated points

            Parameters
            ----------
            q : dict with "pose_mean" ((6) array) and "pose_cov" ((6,6) ndarray)
                pose pdf

            Returns
            ------- 
            fval : float
                   cost function value

            Notes
            -----
            Cost function defined in [1], equation (30)   
        """

        self.updateForLM(q)
        return super().costFunction(q)

    def updateForLM(self, q):
        """
            Update intermediate values for the LM optimization

            Paramters
            ---------
            q : dict with "pose_mean" ((6) array) and "pose_cov" ((6,6) ndarray)
                pose pdf
        """

        self.compute_errors(q)

    def incrementLM(self, q, eps):
        """
            Pose increments in the LM algorithm

            Parameters
            ----------
            q : dict with "pose_mean" ((6) array) and "pose_cov" ((6,6) ndarray)
                pose pdf
            eps : (6) array
                  pose increment obtained in the last LM iteration
            
            Returns
            ------
            q' : dict with "pose_mean" ((6) array) and "pose_cov" ((6,6) ndarray)
                           incremented pose q' = q + eps
                           
        """

        return {"pose_mean": poses_euler.composePoseEuler(q["pose_mean"], eps),
                "pose_cov": q["pose_cov"]}

class gaussianPICP_Quaternion(gaussianPICP):
    """ 
        Specialization for the generic 3D case using Quaternions for representing rotations
    """

    def __init__(self,a_pts_array, c_pts_array):
        gaussianPICP.__init__(self,a_pts_array, c_pts_array)
        self.dim = 3
        self.representationDim = 7

    def compute_ni(self, q, ci, cov_ci):
        """
            Compute the point pdf n_i defined by n_i = q + c_i

            Parameters
            ----------
            q : dict with "pose_mean" ((7) array) and "pose_cov" ((7,7) ndarray) keys
                current estimated pose pdf between the two point clouds
            ci : (3) array
                 point in the second point cloud
            cov_ci : (3,3) ndarray
                     covariance of c_i
            
            Returns
            -------
            n_i : dict with "mean" ((3) array) and "cov" ((3,3) ndarray)
                  n_i = q + c_i 
        """

        return poses_quat.composePosePDFQuatPoint(q, {"mean": ci, "cov": cov_ci})

    def compute_nPts(self, q):
        """ 
            Point of the second cloud after being applied the transformation q 
            ie n_i = q + c_i

            Parameters
            ----------
            q : dict with "pose_mean" ((7) array) and "pose_cov" ((7,7) ndarray)
                pose pdf 
        """

        return poses_quat.composePosePDFQuatPoint(q, self.c_pts_array_assoc)

    def costFunction(self, q):
        """ 
            Cost function defined as the sum of mahalanobis distances between each associated points

            Parameters
            ----------
            q : dict with "pose_mean" ((7) array) and "pose_cov" ((7,7) ndarray)
                pose pdf

            Returns
            ------- 
            fval : float
                   cost function value

            Notes
            -----
            Cost function defined in [1], equation (30)   
        """

        q_mean_norm = q["pose_mean"].copy()

        # Quaternion are unit vectors !
        q_mean_norm[0:4] /= np.linalg.norm(q["pose_mean"][0:4])
        q_norm = {"pose_mean": q_mean_norm, "pose_cov": q["pose_cov"]}
        self.updateForLM(q_norm)
        return super().costFunction(q_norm)

    def updateForLM(self, q):
        """
            Update intermediate values for the LM optimization

            Paramters
            ---------
            q : dict with "pose_mean" ((7) array) and "pose_cov" ((7,7) ndarray)
                pose pdf
        """

        self.compute_errors(q)

    def incrementLM(self, q, eps):
        """
            Pose increments in the LM algorithm

            Parameters
            ----------
            q : dict with "pose_mean" ((7) array) and "pose_cov" ((7,7) ndarray)
                pose pdf
            eps : (7) array
                  pose increment obtained in the last LM iteration
            
            Returns
            ------
            q' : dict with "pose_mean" ((7) array) and "pose_cov" ((7,7) ndarray)
                           incremented pose q' = q + eps      
        """

        qnew_mean = q["pose_mean"] + eps
        norm = np.linalg.norm(qnew_mean[0:4])
        # Quaternion are unit vectors ! So need to reproject into the space of quaternion
        qnew_mean[0:4] /= norm
        return {"pose_mean": qnew_mean, "pose_cov": q["pose_cov"]}

class gaussianPICP_se(gaussianPICP):
    """ 
        Specialization for the generic 3D case using se(3) for on-manifold pose optimization 
    """
    
    def __init__(self,a_pts_array, c_pts_array):
        gaussianPICP.__init__(self, a_pts_array, c_pts_array)
        self.dim = 3
        self.representationDim = 6
        
        # C_x is a 3D tensor with slices being [c_i]_x, 
        # c_i being the i-th points in c_pts_array
        self.C_x = None
        
        # Corresponds to U_c in equation (14)(15) in [1]
        # where c is a point in c_pts_array
        self.U = None
        
        # Intermediate variable defined by K = Sigma_ei_inv * R , where 
        # - Sigma_ei_inv is the inverse of the error covariance ei of the i-th associated pair
        # - R is the rotation matrix of the current transformation pose q
        self.K = None
        
        # Defined in [1], equation (38) 
        self.omega = None
        
        # Rotation matrix of the current transformation pose q
        self.R = None

    def initializeData(self):
        """
            Initialize some internal values 
        """

        self.compute_C_x()
        self.compute_U()

    def compute_C_x(self):
        """ 
            Compute the C_x tensor (n,3,3) with (3,3) slices [c_i]_x, 
            c_i being the i-th points in c_pts_array

            Notes
            -----
            []_x operator is defined in [1], section 3.1 
            This is used to compute U_c as defined in [1], equation (14)(15)
            See also compute_U
        """

        self.C_x = np.zeros((self.c_pts_array_assoc["mean"].shape[0], 3, 3))
        math_utility.vectToSkewMatrixArray(self.c_pts_array_assoc["mean"], self.C_x)

    def compute_U(self):
        """ 
            Compute U_c ((n,3,6) ndarray) which is the array of U_ci for each ci in c_pts_array

            Notes
            -----
            Corresponds to each U_ci in [1], equation (14)(15)
        """
        
        n = self.C_x.shape[0]
        self.U = np.zeros((n, 3, 6))
        ones = np.full(n, 1.)
        self.U[:, :, 0:3] = -self.C_x
        self.U[:, 0, 3] = ones
        self.U[:, 1, 4] = ones
        self.U[:, 2, 5] = ones

    def compute_nPts(self, q):
        """ 
            Point of the second cloud after being applied the transformation q 
            ie n_i = q + c_i

            Parameters
            ----------
            q : dict with "pose_mean" ((6) array) and "pose_cov" ((6,6) ndarray)
                pose pdf 
        """

        return {"mean": poses_euler.composePoseEulerPoint(q["pose_mean"], self.c_pts_array_assoc["mean"]),
                "cov": np.einsum('ij,kjl->kil', self.R, np.einsum('kij,jl->kil', self.omega, self.R.T))}
    
    def computeOmega(self, q):
        """ 
            Compute Omega (intermediate term used to compute errors covariance)

            Notes
            -----
            Omega is defined in [1], equation (38)
        """
        
        self.R = scipy.spatial.transform.Rotation.from_euler('ZYX', q["pose_mean"][0:3]).as_matrix()

        # Omega_i  = Sigma_c_i + Sigma_q_22 - [c_i]_x Sigma_q_11 [c_i]_x
        A_ = np.einsum('ij,kjl->kil', q["pose_cov"][0:3, 0:3], self.C_x)
        A = A_ + q["pose_cov"][0:3, 3:6]
        B = np.einsum('kij, kjl-> kil', self.C_x, A)
        C = np.einsum('ij,kjl->kil', q["pose_cov"][3:6, 0:3], self.C_x)
        self.omega = self.c_pts_array_assoc["cov"] + q["pose_cov"][3:6, 3:6] - B + C

    def computeK(self):
        """ 
            Compute intermediate variable defined by K = Sigma_ei_inv * R , where 
            - Sigma_ei_inv is the inverse of the error covariance ei of the i-th associated pair
            - R is the rotation matrix of the current transformation pose q
        """

        self.K = np.einsum('kij,jl->kil', self.errors_cov_inv, self.R)

    def updateForLM(self, q):
        """
            Update intermediate values for the LM optimization

            Paramters
            ---------
            q : dict with "pose_mean" ((6) array) and "pose_cov" ((6,6) ndarray)
                pose pdf
        """

        self.computeOmega(q)
        self.compute_errors(q)
        self.computeK()

    def costFunctionJacobian(self, q, fval):
        """" 
            Jacobian of function cost taking into account variations of Sigma_e

            Parameters
            ----------
            q : dict with "pose_mean" ((6) array) and "pose_cov" ((6,6) ndarray)
                pose pdf
            fval : float
                   current evaluation of the cost function

            Notes
            -----
            Implementation of the formula in [1],  section 4.2 Proposition 1
        """
        
        n = self.errors_mean.shape[0]
        J = np.empty((1, 6))
        J[:, 3:6] = 2. * np.einsum('kj,kjl->l', self.errors_mean, self.K)[None, :]
        eT_K = np.einsum('kj, kjl->kl', self.errors_mean, self.K)
        A = np.zeros((n, 3, 3))
        for i in range(0, 3):
            A_ = np.einsum('kij,jl->kil', self.omega, G[i])
            A_T = np.transpose(A_, (0, 2, 1))
            A[:, :, i] = np.einsum('kji,ki->kj', A_ + A_T, eT_K)
        C = A - 2. * self.C_x
        J[:, 0:3] = np.sum(np.einsum('kj,kjl->kl', eT_K, C), axis=0)
        return J / (2. * fval)
    
    def incrementLM(self, q, eps):
        """
            Pose increments in the LM algorithm

            Parameters
            ----------
            q : dict with "pose_mean" ((6) array) and "pose_cov" ((6,6) ndarray)
                pose pdf
            eps : (6) array
                  pose increment obtained in the last LM iteration
            
            Returns
            ------
            q' : dict with "pose_mean" ((6) array) and "pose_cov" ((6,6) ndarray)
                           incremented pose q' = q + eps
                           
        """
           
        exp_epsilon_R, exp_epsilon_t = math_utility.exp_SE3(eps)

        qincr_mean = np.concatenate(
            (scipy.spatial.transform.Rotation.from_matrix(exp_epsilon_R).as_euler('ZYX'), exp_epsilon_t.flatten()))

        return {'pose_mean': poses_euler.composePoseEuler(q['pose_mean'], qincr_mean),
                'pose_cov': q['pose_cov']}
    
    def hessianCost_q(self, w_hat, xivec):
        """ 
            Second derivative of the function cost w.r.t the transformation pose q 

            Parameters
            ----------
            w_hat : (3,3) ndarray
                    element of so(3) (Lie algebra of SO(3))
            xivec : (6) array
                    element of se(3) (Lie algebra of SE(3))

            Notes
            -----    
            Corresponds to H^f_q in [1], equation (47).
            The formula is based on the formula (49) and using (21)(22). 
            w_hat = [w]_x where w is the rotational part of an element of se(3)
        """

        w_hat_sqr = w_hat @ w_hat

        # Lambda_w as defined in [1], equation (50)
        Lambda_w = np.einsum('kij,jl->kil', self.omega, w_hat)
        
        # Lambda_w + Lambda_w^T (appears in [1], equation (52))
        Lambda_w_plus_transpose = Lambda_w + np.transpose(Lambda_w, (0, 2, 1))

        # Defined in [1], equation (52)
        K_w = np.einsum('kij, klj->kil', Lambda_w_plus_transpose, self.K)
        
        # K_w * e, where e is the array of mean errors 
        K_w_e = np.einsum('kij,kj->ki', K_w, self.errors_mean)

        # Defined in [1], equation (51)
        D_w = 2 * np.einsum('ij,kjl->kil', w_hat, Lambda_w) - np.einsum('kij,jl->kil', self.omega, w_hat_sqr) - np.einsum(
            'ij,kjl->kil', w_hat_sqr, self.omega)

        # e^T * K
        eT_K = np.einsum('kj, kjl->kl', self.errors_mean, self.K)

        # U * xivec, where xivec is an element of Lie algebra se(3)
        U_xi = np.einsum('kij,j->ki', self.U, xivec)
        
        # R*U_xi
        RU_xi = np.einsum('ij,kj->ki', self.R, U_xi)

        # \hat{w} * c_ + \tau, where \tau is the "translation" part of xivec
        w_c_tau = np.einsum('kij, j->ki', self.U[:, :, 0:3], xivec[0:3]) + xivec[3:6]
        
        # w * w_c_tau
        w_w_c_tau = np.einsum('ij,kj->ki', w_hat, w_c_tau)
        
        # D_w * K^T
        D_wKT = np.einsum('kij, klj->kil', D_w, self.K)
        
        # D_wKT * e
        D_wKTe = np.einsum('kij,kj->ki', D_wKT, self.errors_mean)

        # K_w * R
        K_wR = np.einsum('kij,jl->kil', K_w, self.R)
        
        # K_w * R * (2 * K_w * e + 3 * U_xi)
        v_3 = np.einsum('kij,kj->ki', K_wR, 2. * K_w_e + 3. * U_xi)
        
        # Add the terms appearing in [1], equation (49)
        A = np.einsum('kj,kj->k', eT_K, 2. * w_w_c_tau + D_wKTe + v_3)
        RU_xiT_K = np.einsum('kj,kjl->kl', RU_xi, self.K)
        B = np.einsum('kj,kj->k', RU_xiT_K, 2. * U_xi + K_w_e)

        return A + B

    def hessianMatrixCost_z(self, q):
        """ 
            Second derivative of the function cost df²/dqdz

            Parameters
            ----------
            q : dict with "pose_mean" ((6) array) and "pose_cov" ((6,6) ndarray)
                pose pdf

            Notes
            -----
            Corresponds to H^f_q,z in [1], equation (47).
            The formula are given in [1], Appendix B.2.2
        """

        # count of points 
        n = self.errors_mean.shape[0]
        H_z = np.zeros((n, 6, 6))

        # A corresponds to A_1 + A_2 in [1], equation (128)
        # Here first set A_2 as in [1], equation (139) 
        A = np.einsum('kji,kjl->kil', self.U, np.einsum('kji,jl->kil', self.K, self.R))
        
        # B corresponds to A-3 + A_4 in [1], equation (128)
        B = np.zeros((n, 6, 3))

        # Corresponds to [1], equation (137)
        dU = np.zeros((3, 6))
        for i in range(0, 6):
            # xivec \in se(3)
            xivec = np.zeros((6,))
            xivec[i] = 1.
            
            # Defined in [1], equation (50) with here w the "rotational" part of current xivec 
            # so that here \hat{w} is a generator of SO(3)
            Lambda_w = np.einsum('kij,jl->kil', self.omega, G[i])
            
            # Lambda_w + Lambda_w^T
            Lambda_w_plus_transpose = Lambda_w + np.transpose(Lambda_w, (0, 2, 1))

            # Defined in [1], equation (52)
            K_w = np.einsum('kij, klj->kil', Lambda_w_plus_transpose, self.K)

            # e^T * K_w^T  = (K_w * e)^T
            eT_KwT = np.einsum('kj, kij ->ki', self.errors_mean, K_w)

            # K * K_w
            K_Kw = np.einsum('kij, kjl->kil', self.K, K_w)

            # xivec^T * U^T
            xivec_UT = np.einsum('j,kij->ki', xivec, self.U)

            # Derivative w.r.t points a. Corresponds to the [1], equation (126)
            H_z[:, i, 0:3] = -(
                        np.einsum('kj,kij->ki', 2. * xivec_UT + eT_KwT, self.K) + np.einsum('kj,klj->kl', self.errors_mean, K_Kw))

            # Derivative w.r.t points c 
            
            # K*U
            KU = np.einsum('kij,kjl->kil', self.K, self.U)
            
            # (K * K_w)^T
            K_Kw_T = np.transpose(K_Kw, (0, 2, 1))
            
            # First set A_3 ([1], equation (131)) as it is independent from xivec 
            B[:, i, :] = np.einsum('kj,kji->ki', self.errors_mean, np.einsum('kij,jl->kil', K_Kw + K_Kw_T, self.R))
            for j in range(0, 3):
                # equation (137)
                dU[:, 0:3] = -G[j]

                # First term in [1], equation (136) 
                K1 = np.einsum('ij,kjl->kil', dU, np.einsum('ij,klj->kil', q["pose_cov"], self.U))
                
                # Seconde term in [1], equation (136)
                K2 = np.einsum('kij,jl->kil', self.U, np.einsum('ij, lj->il', q["pose_cov"], dU))
                
                # [1], Equation (136)
                dOmega = K1 + K2
                
                # [1], Equation (135)
                dSigma = np.einsum('ij,kjl->kil', self.R, np.einsum('kij,jl->kil', dOmega, self.R.T))
                
                # Compute A_1 (eq from [1], equations (134) to (138))
                sigmaInv_dSigma = np.einsum('kij,kjl->kil', self.errors_cov_inv, dSigma)
                dKU = np.einsum('kij, jl->kil', self.K, dU) - np.einsum('kij,kjl->kil', sigmaInv_dSigma, KU)
                A_ = np.einsum('...j,...j', self.errors_mean, dKU[:, :, i])
                
                # Add 
                A[:, i, j] += A_

                # Compute [1], equation (141)
                K_Lambda_w = np.einsum('kij,kjl->kil', self.K, Lambda_w_plus_transpose)
                dLambda_w_ = np.einsum('kij,jl->kil', dOmega, G[i])
                dLambda_w = dLambda_w_ + np.transpose(dLambda_w_, (0, 2, 1))
                dKwT = -np.einsum('kij,kjl->kil', sigmaInv_dSigma, K_Lambda_w) + np.einsum('kij,kjl->kil', self.K, dLambda_w)
                dKwT_RT_sigmaInv = np.einsum('kij,klj->kil', dKwT, self.K) - np.einsum('kji,kjl->kil', K_Kw,
                                                                                  np.einsum('kij,kjl->kil', dSigma,
                                                                                            self.errors_cov_inv))
                B[:, i, j] += np.einsum('...j,...j', self.errors_mean,
                                        np.einsum('kij, kj->ki', dKwT_RT_sigmaInv, self.errors_mean))

        H_z[:, :, 3:6] = 2. * A + B

        return H_z

    def hessianMatrixCost_q(self):
        """ 
            Hessian w.r.t q (ie H_q in [1]]) of cost function 

            Returns
            -------
            H : (6,6) ndarray
                Hessian matrix
        """

        n = self.errors_mean.shape[0]
        self.H_q = np.empty((n, 6, 6))

        # Compute diagonal elements first with simplified expressions 
        for i in range(0, 6):
            xivec = np.zeros((6,))
            xivec[i] = 1.
            self.H_q[:, i, i] = self.hessianCost_q(G[i], xivec)

        # Off diagonal elements with full expressions 
        for i in range(0, 6):
            for j in range(0, i):
                xivec = np.zeros((6,))
                xivec[i] = 1.
                xivec[j] = 1.
                # Exploit the symmetric bilinearity of the hessian (cf [1], section 3.3) 
                self.H_q[:, i, j] = 0.5 * (
                            self.hessianCost_q(G[i] + G[j], xivec) - self.H_q[:, i, i] - self.H_q[:, j,j])
                self.H_q[:, j, i] = self.H_q[:, i, j]

        return np.sum(self.H_q, axis=0)

    def hessianMatrixCost(self, q):
        """ 
            Compute the full hessian matrix

            Parameters
            ----------
            q : dict with "pose_mean" ((6) array) and "pose_cov" ((6,6) ndarray)
                pose pdf
        """

        self.H_q = self.hessianMatrixCost_q()
        self.H_z = self.hessianMatrixCost_z(q)
 
    def LM_covariance_(self, Sigma_z):
        """ 
            Covariance of the estimated pose obtained after optimization (Internal)

            Parameters
            ----------
            Sigma_z : (2*n, 2*n) ndarray
                      Concatenated covariances of points as defined in [1], equation (48)

            Returns
            -------
            cov : (6,6) ndarray
                  pose covariance
        """

        H_q_inv = np.linalg.inv(self.H_q)

        # Sum_i H_q,zi Sigma_zi H_q,zi^T ([1], equation (46)) 
        A = np.sum(np.einsum('kij, kjl->kil', self.H_z, np.einsum("kij,klj->kil", Sigma_z, self.H_z)), axis=0)
        cov = H_q_inv @ A @ H_q_inv

        return cov

    def LM_covariance(self, q):
        """ 
            Covariance of the estimated pose obtained after optimization 

            Parameters
            ----------
            q : dict with "pose_mean" ((6) array) and "pose_cov" ((6,6) ndarray)
                pose pdf

            Returns
            -------
            cov : (6,6) ndarray
                  pose covariance
        """

        self.hessianMatrixCost(q)

        # Covariance of the data
        Sigma_z = np.zeros((self.H_z.shape[0], 6, 6))
        Sigma_z[:, 0:3, 0:3] = self.a_pts_array_assoc["cov"]
        Sigma_z[:, 3:6, 3:6] = self.c_pts_array_assoc["cov"]

        return self.LM_covariance_(Sigma_z)

    def jacobianProjectionToPlane_wrt_n(self, normal_means):
        """ 
            Jacobian of the projection to the normal plane w.r.t the point
            
            Parameters
            ----------
            normal_means : (n,3) ndarray
                            normal means at each point of the second cloud (c_i)

            Returns
            -------
            jacobian : (n,3,3) ndarray
                       Jacobian matrix for each point n_i

            Notes
            -----
            Defined in [1], equation (79)
        """

        # dai_dni = I_3 - vi vi^T , ai = firstCloudPt, ni= secondCloudPt after composition, vi = normal mean
        return np.eye(3) - np.einsum("ki,kj->kij", normal_means, normal_means)

    def jacobianNormalNormalization(self, normal_means):
        """ 
            Jacobian of the normal normalization 

            Parameters
            ----------
            normal_means : (n,3) ndarray
                           normal means at each point of the second cloud (c_i)

            Returns
            -------
            jacobian : (n,3,3) ndarray
                       jacobian matrix for each normal

            Notes
            -----
            Defined in [1], equation (76)
        """

        nx_sqr = np.power(normal_means[:, 0], 2)
        ny_sqr = np.power(normal_means[:, 1], 2)
        nz_sqr = np.power(normal_means[:, 2], 2)
        nx_ny = normal_means[:,0] * normal_means[:,1]
        nx_nz = normal_means[:,0] * normal_means[:,2]
        ny_nz = normal_means[:,1] * normal_means[:,2]

        K = 1./np.power(np.linalg.norm(normal_means[:,0:4],axis=1), 3)
        jacobian_normalization = np.empty((normal_means.shape[0], 3, 3))
        jacobian_normalization[:,0,0] = ny_sqr + nz_sqr
        jacobian_normalization[:, 0, 1] = -nx_ny
        jacobian_normalization[:, 0, 2] = -nx_nz
        jacobian_normalization[:, 1, 0] = -nx_ny
        jacobian_normalization[:, 1, 1] = nx_sqr + nz_sqr
        jacobian_normalization[:, 1, 2] = -ny_nz
        jacobian_normalization[:, 2, 0] = -nx_nz
        jacobian_normalization[:, 2, 1] = -ny_nz
        jacobian_normalization[:, 2, 2] = nx_sqr + ny_sqr

        return K[:,None,None]*jacobian_normalization

    def jacobianProjectionToPlane_wrt_normal(self, normal_means, vecs, dot):
        """ 
            Jacobian of the projection to the normal plane w.r.t the normal 

            Parameters
            ----------
            normal_means : (n,3) ndarray
                           normal means at each point of the second cloud (c_i)
            vecs : (n,3) ndarray
                   errors of associated points ie n_i - a_i
            dot : (n) array
                  dot product between each vec in vecs and normal means ie vi^T(ni - ai)

            Returns
            -------
            jacobian : (n,3,3) ndarray
                        jacobian matrix for each normal

            Notes
            -----
            Defined in [1], equation (80)
        """

        # dai_dvi = vi(ni - ai)^T + vi^T(ni - ai)I_3 , here dot = vi^T(ni -ai) and vecs = ni - ai
        # (vi unnormalized)
        # The final jacobian is dai_dvi * dvi_norm_dvi (jacobian_norm)
        A = np.zeros((normal_means.shape[0],3,3))
        A[:,0,0] = dot
        A[:,1,1] = dot
        A[:,2,2] = dot

        J = -(np.einsum("ki,kj->kij", normal_means, vecs) + A)

        jacobian_norm = self.jacobianNormalNormalization(normal_means)

        return np.einsum("kij,kjl->kil", J, jacobian_norm)

    def pointToPlaneAssociations(self, firstCloud_means, secondCloud_means, secondCloud_covs, normal_means, normal_covs):
        """ 
            Projection of point to point cloud tangent planes (for point-to-plane association)

            Parameters
            ----------
            firstCloud_means : (n,3) ndarray
                               point clouds a_i
            secondCloud_means : (n,3) ndarray
                                point clouds c_i
            secondCloud_covs : (n,3,3) ndarray
                                covariance matrices of the second point cloud
            normal_means : (n,3) ndarray
                           normal means at each point of the second cloud (c_i)
            normal_covs : (n,3,3) ndarray
                           normal convariances at each point of the second cloud (c_i)

            Returns
            -------
            projectedPoints : dict of arrays with "mean" ((n,3) ndarray) and "cov" ((n,3,3) ndarray)
                              projected points along the normal

            Notes
            -----
            Explained in [1], section 4.1

        """

        vecs = secondCloud_means - firstCloud_means
        dot = np.einsum('ij,ij->i', vecs, normal_means)
        multiply = np.multiply(normal_means, dot[:, np.newaxis])

        jacobian_n = self.jacobianProjectionToPlane_wrt_n(normal_means)
        jacobian_v = self.jacobianProjectionToPlane_wrt_normal(normal_means,
                                                          vecs,
                                                          dot)
        cov = np.einsum("kij,kjl->kil", jacobian_n, np.einsum("kij,klj->kil", secondCloud_covs, jacobian_n)) + \
              np.einsum("kij,kjl->kil", jacobian_v, np.einsum("kij,klj->kil", normal_covs, jacobian_v))

        return {"mean": secondCloud_means - multiply, "cov": cov}