import sys
import numpy as np
import scipy
import gaussianPICP
import approxGaussian
import mypicp
import poses_euler
import poses2D
import time
import math_utility
import plotPICP
import PICP_display
import data_io
import copy
from dataPICP import *
import betaDist
import simulatedDataGeneration as sdg
import multiprocessing as mp

# Fix the rng seed 
np.random.seed(1)

# Uncomment to raise all errors (debugging)
#np.seterr(all='raise')

class sonarPICPSimulation():
    """ 
        Class used to apply and compare the MpIC algo to other pIC algorithm on simulated data
        and then also extended to use data obtained with real data (Girona dataset)
        TODO: Clearly need better naming and cleaning as this file was updated continously for testing ideas ....

        References
        ----------
        .. [1] Mallios, A.; Vidal, E.; Campos, R.; Carreras, M. Underwater caves sonar data set. Int. J. Robot. Res. 2017,
               36, 1247–1251
        .. [2]  Breux, Yohan, André Mas, and Lionel Lapierre.
                "On-manifold Probabilistic ICP: Application to Underwater Karst Exploration." (2021) 
        .. [3] "doc/MpIC_oldVersion_noApprox.pdf"
    """ 

    def __init__(self):
        """
            Constructor
        """

        self.rho_std = 0.1
        self.b = np.deg2rad(35.)
        self.maxTheta = 0.5 * self.b
        self.psi_std = 0.01
        self.theta_var = 0.001
        self.horizontalSonarPoseOnRobot = {"pose_mean": np.full((6,), 0.),  #np.array([np.pi, 0., 0., 0.1, 0., -0.42]),
                                           "pose_cov": np.zeros((6,6))}
        self.ignoreUniformDist = True
        # Use twice the variance as it corresponds to the odometry variance (between two independant measures)
        self.useAbsoluteDepthPitchRoll=False
        self.var_depth_sensor = 2.*0.000277778 # 3-sigma= 5 cm
        self.var_IMU_pitch = np.deg2rad(2.*0.027777778) # 3-sigma = 0.5 degree 
        self.var_IMU_roll = np.deg2rad(2.*0.027777778) # 3-sigma = 0.5 degree
        self.showDisplayDebug = False
        self.sdg = sdg.simulatedDataGenerator()

    def correctCovAbsoluteDepthPitchRoll(self, covariance):
        """
            Correction to take into account for absolute covariance on depth, pitch and roll

            Parameters
            ----------
            covariance : (6,6) ndarray
                         covariance matrix

            Notes
            -----
            Normally, the covariance are expressed relatively to the previous pose.
            In the case of depth, the depth sensor provides absolute data so that no need to accumulate incertainty.
            Similarly, the IMU gives stable absolute value for the ptich and roll.
        """

        # remove all covariances with to z
        covariance['pose_cov'][:, 5] = 0. 
        covariance['pose_cov'][5, :] = 0.
        # remove all covariances with to pitch
        covariance['pose_cov'][:, 1] = 0.
        covariance['pose_cov'][1, :] = 0.
        # remove all covariances with to roll
        covariance['pose_cov'][:, 2] = 0. 
        covariance['pose_cov'][2, :] = 0.
        covariance['pose_cov'][5,5] = self.var_depth_sensor
        covariance['pose_cov'][1,1] = self.var_IMU_pitch
        covariance['pose_cov'][2,2] = self.var_IMU_roll

    def loadTrajAndOdo_loopClosure(self, trajFile, trajGTFile):
        """
            Load the trajectory and odometries for simulation with loop closure

            Parameters
            ----------
            trajFile : str
                       file containing the trajectories
            trajGTFile : str
                         file containing the GT trajectories

            Returns
            -------
            q_gt : dict with ["pose_mean"] and ["pose_cov"]    
                   GT relative pose pdf between two scans
            q0 : dict with ["pose_mean"] and ["pose_cov"]
                 initial relative pose pdf between two scans
            firstScan_refFrame : dict with ["pose_mean"] and ["pose_cov"]
                                 reference frame (pose) pdf of the first scan
            secondScan_refFrame : dict with ["pose_mean"] and ["pose_cov"]
                                  reference frame (pose) pdf of the second scan                    
            firstScan_poses_relative : array of poses pdf   
                                       array of first scan robot poses relative to the first scan reference frame
            secondScan_poses_relative : array of poses pdf   
                                        array of second scan robot poses relative to the second scan reference frame 
        """

        trajectory = data_io.loadTrajectoryFromFile(trajFile)
        trajectory_gt = data_io.loadTrajectoryFromFile(trajGTFile)

        middle_idx = int(np.rint(len(trajectory_gt) / 4))

        timeStampsKeys = list(trajectory.keys())
        trajPoses = list(trajectory.values())
        trajPoses_gt = list(trajectory_gt.values())
        q_gt, q0, firstScan_poses_relative, secondScan_poses_relative = self.generateScansPoseRelativeToReferenceFrame_usingTrajectory(middle_idx,
                                                                                                                                       timeStampsKeys,
                                                                                                                                       trajPoses,
                                                                                                                                       trajPoses_gt)

        firstScan_refFrame = {'pose_mean': trajPoses[middle_idx]['pose_mean'], 'pose_cov': np.zeros((6, 6))}
        secondScan_refFrame = {'pose_mean': trajPoses[3 * middle_idx]['pose_mean'], 'pose_cov': np.zeros((6, 6))}

        return q0, q_gt, trajectory, trajectory_gt, firstScan_refFrame, secondScan_refFrame, firstScan_poses_relative, secondScan_poses_relative

    def loadTrajAndOdo(self, bodyPoses_firstScan_file, bodyPoses_secondScan_file, refFrameTimestamps_file):
        """
            Load the trajectories and odometries

            Parameters
            ----------
            bodyPoses_firstScan_file : str
                                       file containing the robot poses in the first scan
            bodyPoses_secondScan_file : str
                                       file containing the robot poses in the second scan
            refFrameTimestamps_file : str
                                      file containing the timestamps of the two reference frame (pose) for the two scans

            Returns 
            -------
            q0 : dict with ["pose_mean"] and ["pose_cov"]
                 initial relative pose pdf between two scans
            firstScan_refFrame : dict with ["pose_mean"] and ["pose_cov"]
                                 reference frame (pose) pdf of the first scan
            secondScan_refFrame : dict with ["pose_mean"] and ["pose_cov"]
                                  reference frame (pose) pdf of the second scan                    
            firstScan_poses_relative : array of poses pdf   
                                       array of first scan robot poses relative to the first scan reference frame
            secondScan_poses_relative : array of poses pdf   
                                        array of second scan robot poses relative to the second scan reference frame 
        """

        bodyPoses_firstScan = data_io.loadTrajectoryFromFile(bodyPoses_firstScan_file)
        bodyPoses_secondScan = data_io.loadTrajectoryFromFile(bodyPoses_secondScan_file)

        with open(refFrameTimestamps_file, "r") as file:
            firstScan_refFrame_timestamp = int(file.readline())
            secondScan_refFrame_timestamp = int(file.readline())

        q0, firstScan_poses_relative, secondScan_poses_relative = self.generateScansPoseRelativeToReferenceFrame_realData(firstScan_refFrame_timestamp,
                                                                                                                          secondScan_refFrame_timestamp,
                                                                                                                          bodyPoses_firstScan,
                                                                                                                          bodyPoses_secondScan)

        firstScan_refFrame = {'pose_mean': bodyPoses_firstScan[firstScan_refFrame_timestamp]['pose_mean'], 'pose_cov': np.zeros((6, 6))}
        secondScan_refFrame = {'pose_mean': bodyPoses_secondScan[secondScan_refFrame_timestamp]['pose_mean'], 'pose_cov': np.zeros((6, 6))}

        return q0, firstScan_refFrame, secondScan_refFrame, firstScan_poses_relative, secondScan_poses_relative

    def generateRelativePosesFromTrajectory(self, timeStamps, trajectory, middle_idx):
        """
            Compute the robot poses inside a scan expressed relatively to the scan reference frame

            Parameters
            ----------
            timeStamps : (n) array
                         timestamp of each robot pose
            trajectory : (n) array of dict
                         array of robot pose pdf in global frame
            middle_idx : int
                         index of the reference frame in the trajectory array
            
            Returns
            -------
            poses_relative : (n) array of dict
                             array of robot pose pdf relative to the reference frame
        """
        s
        poses_relative = {}
        refFramePDF = trajectory[middle_idx]
        for ts, poseInFirstFrame in zip(timeStamps, trajectory):
            robotPosePDF = poses_euler.inverseComposePosePDFEuler(poseInFirstFrame, refFramePDF)
            poses_relative[ts] = poses_euler.composePosePDFEuler(robotPosePDF, self.horizontalSonarPoseOnRobot)
        return poses_relative

    def generateRelativePosesFromTrajectory_realData(self, bodyPoses, refFrame_timestamp):
        """
            Compute the robot poses inside a scan expressed relatively to the scan reference frame
            (version using real data from girona dataset)

            Parameters
            ----------
            timeStamps : (n) array
                         timestamp of each robot pose
            bodyPoses : (n) array of dict
                         array of robot pose pdf in global frame
            refFrame_timestamp : int
                                 timestamp of the reference frame in the bodyPoses array
            
            Returns
            -------
            poses_relative : (n) array of dict
                             array of robot pose pdf relative to the reference frame
            
            Notes
            -----
            Function first implemented to use data from the dataset provided by [1]
        """

        poses_relative = {}
        refFramePDF = bodyPoses[refFrame_timestamp]
        for ts, posePDF in bodyPoses.items():
            robotPosePDF = poses_euler.inverseComposePosePDFEuler(posePDF, refFramePDF)
            poses_relative[ts] = poses_euler.composePosePDFEuler(robotPosePDF, self.horizontalSonarPoseOnRobot)

        return poses_relative

    def generateScansPoseRelativeToReferenceFrame_realData(self, firstScan_refFrame_timestamp, secondScan_refFrame_timestamp, bodyPoses_firstScan, bodyPoses_secondScan):
        """ 
            Generate the pdf poses relative to their (scan-centered) reference frame 

            Parameters
            ----------
            firstScan_refFrame_timestamp : int
                                           timestamp of the first scan reference frame
            secondScan_refFrame_timestamp : int
                                            timestamp of the second scan reference frame
            bodyposes_firstScan : (n) array of dict
                                  array of pose pdf of the robot poses in the first scan
            bodyPoses_secondScan : (n) array of dict
                                   array of pose pdf of the robot poses in the second scan 

            Returns
            -------
            q0 : dict with ["pose_mean"] and ["pose_cov"]
                 initial relative pose pdf between the two scan reference frame
            firstScan_poses_relative : (n) array of dict
                                        array of robot pose pdf expressed relatively to the first scan reference frame
            secondScan_poses_relative : (n) array of dict
                                        array of robot pose pdf expressed relatively to the second scan reference frame
        """

        firstScan_poses_relative = self.generateRelativePosesFromTrajectory_realData(bodyPoses_firstScan, firstScan_refFrame_timestamp)
        secondScan_poses_relative = self.generateRelativePosesFromTrajectory_realData(bodyPoses_secondScan, secondScan_refFrame_timestamp)

        q0 = poses_euler.inverseComposePosePDFEuler(bodyPoses_secondScan[secondScan_refFrame_timestamp], bodyPoses_firstScan[firstScan_refFrame_timestamp])
    
        print("q0 cov first")
        print(q0["pose_cov"])
        if(self.useAbsoluteDepthPitchRoll and q0["pose_mean"].shape[0] == 6):
            self.correctCovAbsoluteDepthPitchRoll(q0)

        return q0, firstScan_poses_relative, secondScan_poses_relative

    ''' The middle_idx is the index separating data of the first and second scan '''
    def generateScansPoseRelativeToReferenceFrame_usingTrajectory(self, middle_idx, timeStamps, trajectory, trajectory_gt):
        """ 
            Generate the pdf poses relative to their (scan-centered) reference frame 

            Parameters
            ----------
            middle_idx : int
                         index of the reference frame in the trajectory array
            timeStamps : (n) array
                         array of timestamps
            trajectory : (n) array of dict
                         array of robot pose pdf
            trajectory_gt : (n) array of dict
                            array of ground truth robot pose pdf

            Returns
            -------
            q_gt : dict with ["pose_mean"] and ["pose_cov"]
                   Ground truth relative pose pdf between the two scan reference framet
            q0 : dict with ["pose_mean"] and ["pose_cov"]
                 initial relative pose pdf between the two scan reference frame
            firstScan_poses_relative : (n) array of dict
                                        array of robot pose pdf expressed relatively to the first scan reference frame
            secondScan_poses_relative : (n) array of dict
                                        array of robot pose pdf expressed relatively to the second scan reference frame
        """

        # As the middle idx is exactly half the scan in the simulated case, the two scans total size is 4*middle idx
        firstScan_poses_relative = self.generateRelativePosesFromTrajectory(timeStamps[0:2 * middle_idx], trajectory[0:2 * middle_idx], middle_idx)
        secondScan_poses_relative = self.generateRelativePosesFromTrajectory(timeStamps[2 * middle_idx: 4 * middle_idx], trajectory[2 * middle_idx: 4 * middle_idx],
                                                               middle_idx)

        q0 = poses_euler.inverseComposePosePDFEuler(trajectory[3*middle_idx], trajectory[middle_idx])

        q_gt = poses_euler.inverseComposePosePDFEuler(trajectory_gt[3*middle_idx], trajectory_gt[middle_idx])

        return q_gt, q0, firstScan_poses_relative, secondScan_poses_relative

    def normalRotationMatrix(self, normal):
        """
            Generate a rotation matrix mapping e_x (base vector) to the given normal

            Parameters
            ----------
            normal : (3) array
                     normal vector (normalized)
            
            Returns
            -------
            R : (3,3) array
                rotation matrix
        """

        R = np.empty((3,3))
        R[:,0] = normal
        R[:,1] = np.cross(normal, [0,0,1])
        R[:,2] = np.cross(normal, R[:,1])

        return R

    def precompute_approxGaussian_pts(self, firstScanMeas, secondScanMeas):
        """
            Precompute gaussian approximation for the points pdf 

            Parameters
            ----------
            firstScanMeas : (n) array of dict
                            array of parameters defining point pdf in local spherical coords (first scan)
            secondScanMeas : (m) array of dict
                             array of parameters defining point pdf in local spherical coords (second scan)

            Returns
            -------
            a_pts_array : (n) array of dict with ["mean"] and ["cov"]
                          array of gaussian pdf of each point in the first scan
            c_pts_array : (m) array of dict with ["mean"] and ["cov"]
                          array of gaussian pdf of each point in the second scan
            compTime_preprocessGaussian : float
                                          computational time
            a_array_L : (n) array of dict with ["mean"] and ["cov"]                            
                        array of gaussian pdf of each point in the first scan expressed in the sonar local coordinates at the time of measure.
            c_array_L : (m) array of dict with ["mean"] and ["cov"]                            
                        array of gaussian pdf of each point in the second scan expressed in the sonar local coordinates at the time of measure.

            Notes
            -----
            See [2], section 5.4 and approxGaussian.py
        """

        start = time.time()
        a_pts = []
        c_pts = []
        a_array_L = []
        c_array_L = []
        for params in firstScanMeas:
            if( (self.ignoreUniformDist and (params["theta"]["alpha"]!= 1 or params["theta"]["beta"] !=1)) or
                not self.ignoreUniformDist):
                local_cart = approxGaussian.approximateToGaussian_closedForm(params)
                a_array_L.append(local_cart)
                cart_inScanRefFrame = poses_euler.composePosePDFEulerPoint(params, local_cart)
                a_pts.append(cart_inScanRefFrame)
        for params in secondScanMeas:
            if ((self.ignoreUniformDist and (params["theta"]["alpha"]!= 1 or params["theta"]["beta"] !=1)) or
               not self.ignoreUniformDist):
                local_cart = approxGaussian.approximateToGaussian_closedForm(params)
                c_array_L.append(local_cart)
                c_pts.append(poses_euler.composePosePDFEulerPoint(params, local_cart))
        n_aPts = len(a_pts)
        n_cPts = len(c_pts)
        a_pts_array = {"mean": np.empty((n_aPts, 3)), "cov": np.empty((n_aPts, 3, 3))}
        c_pts_array = {"mean": np.empty((n_cPts, 3)), "cov": np.empty((n_cPts, 3, 3))}

        # Generalized ICP : add covariance in the surface plane
        # Cov value should be dependent on range
        #cov_inPlane = np.zeros((3,3))
        #v = 0.05
        #cov_inPlane[1,1] = v ** 2
        #cov_inPlane[2,2] = v ** 2

        for k in range(0, n_aPts):
            #R_a = self.normalRotationMatrix(normals_firstFrame["mean"][k])
            a_pts_array["mean"][k] = a_pts[k]["mean"]
            a_pts_array["cov"][k] = a_pts[k]["cov"] #+ R_a@cov_inPlane@np.transpose(R_a)
        for k in range(0, n_cPts):
            #R_c = self.normalRotationMatrix(normals_secondFrame["mean"][k])
            c_pts_array["mean"][k] = c_pts[k]["mean"]
            c_pts_array["cov"][k] = c_pts[k]["cov"] #+ R_c@cov_inPlane@np.transpose(R_c)
        compTime_preprocessGaussian = time.time() - start

        return a_pts_array, c_pts_array, compTime_preprocessGaussian, a_array_L, c_array_L

    def precompute_approxGaussian2D_pts(self, firstScanMeas, secondScanMeas):
        """
             Precompute gaussian approximation for the points pdf (2D case)

            Parameters
            ----------
            firstScanMeas : (n) array of dict
                            array of parameters defining point pdf in local spherical coords (first scan)
            secondScanMeas : (m) array of dict
                             array of parameters defining point pdf in local spherical coords (second scan)

            Returns
            -------
            a_pts_array : (n) array of dict with ["mean"] and ["cov"]
                          array of gaussian pdf of each point in the first scan
            c_pts_array : (n) array of dict with ["mean"] and ["cov"]
                          array of gaussian pdf of each point in the second scan
        """

        a_pts = []
        c_pts = []

        #TODO set this code in a separate function (currently copy/paste for the two scans)
        # Also put in approxGaussian.py not here
        for params in firstScanMeas:
            # Get 2D pose 
            pose = poses2D.fromPosePDF3DEuler(params)

            cov_polarPoint = np.zeros((2,2))
            cov_polarPoint[0,0] = params["rho"]["std"]**2
            cov_polarPoint[1, 1] = params["psi"]["std"] ** 2
            ''' Just ignore theta (ie theta=0) '''
            cos_psi = np.cos(params['psi']["mean"])
            sin_psi = np.sin(params['psi']["mean"])
            local_cart = {"mean": None, "cov": None}
            local_cart["mean"] = params['rho']["mean"]*np.array([cos_psi, sin_psi])
            jacobian_polarToCart = np.array([[cos_psi, -local_cart["mean"][1]],
                                             [sin_psi, local_cart["mean"][0]]])
            local_cart["cov"] = jacobian_polarToCart@cov_polarPoint@jacobian_polarToCart.T
            global_cart = poses2D.composePosePDFPoint(pose, local_cart)
            a_pts.append(global_cart)
        for params in secondScanMeas:
            # Get 2D pose 
            pose = poses2D.fromPosePDF3DEuler(params)

            cov_polarPoint = np.zeros((2, 2))
            cov_polarPoint[0, 0] = params["rho"]["std"] ** 2
            cov_polarPoint[1, 1] = params["psi"]["std"] ** 2
            ''' Just ignore theta (ie theta=0) '''
            cos_psi = np.cos(params['psi']["mean"])
            sin_psi = np.sin(params['psi']["mean"])
            local_cart = {"mean": None, "cov": None}
            local_cart["mean"] = params['rho']["mean"] *np.array([cos_psi, sin_psi])
            jacobian_polarToCart = np.array([[cos_psi, -local_cart["mean"][1]],
                                             [sin_psi, local_cart["mean"][0]]])
            local_cart["cov"] = jacobian_polarToCart @ cov_polarPoint @ jacobian_polarToCart.T
            c_pts.append(poses2D.composePosePDFPoint(pose, local_cart))

        n_aPts = len(a_pts)
        n_cPts = len(c_pts)
        a_pts_array = {"mean": np.empty((n_aPts, 2)), "cov": np.empty((n_aPts, 2, 2))}
        c_pts_array = {"mean": np.empty((n_cPts, 2)), "cov": np.empty((n_cPts, 2, 2))}
        for k in range(0, n_aPts):
            a_pts_array["mean"][k] = a_pts[k]["mean"]
            a_pts_array["cov"][k] = a_pts[k]["cov"]
        for k in range(0, n_cPts):
            c_pts_array["mean"][k] = c_pts[k]["mean"]
            c_pts_array["cov"][k] = c_pts[k]["cov"]
        return a_pts_array, c_pts_array

    def experiment_girona2D(self, q0, q_gt_mean, firstScanMeas, secondScanMeas, onlyOptimization, **kwargs):
        """
            Experiment in the 2D case (as in the Girona work).
            It consists in estimating the relative between two scans.

            Parameters
            ----------
            q0 : dict with ["pose_mean"] and ["pose_cov"]
                 initial relative pose pdf between the two scan reference frame
            q_gt_mean : (6) array
                        Ground truth relative pose between the two scan reference framet
            firstScanMeas : (n) array of dict
                            array of parameters defining point pdf in local spherical coords (first scan)
            secondScanMeas : (m) array of dict
                             array of parameters defining point pdf in local spherical coords (second scan)
            onlyOptimization : bool
                               if true, skip the association step of pICP

            Returns
            -------
            q_opt : dict with ["pose_mean"] and ["pose_cov"]
                    estimated relative pose pdf between the two scan reference frame
            path_gaussian : dict of arrays 
                            contains details on pICP iterations (see gaussianPICP.py)
            a_pts_array : (n) array of dict with ["mean"] and ["cov"]
                          array of gaussian pdf of each point in the first scan
            c_pts_array : (m) array of dict with ["mean"] and ["cov"]
                          array of gaussian pdf of each point in the second scan

            Notes
            -----
            See [1] and references therein for Girona work 
        """

        a_pts_array, c_pts_array = self.precompute_approxGaussian2D_pts(firstScanMeas, secondScanMeas)

        gPICP = gaussianPICP.gaussianPICP_2D(a_pts_array, c_pts_array)
        gPICP.picpMaxIter = kwargs["picpMaxIter"]
        q0_2D = poses2D.fromPosePDF3DEuler(q0)
        print("q0_2D cov : ")
        print(q0_2D["pose_cov"])
        if (not onlyOptimization):
            gPICP.showDisplay = self.showDisplayDebug
            q_opt, path_gaussian = gPICP.PICP(q0_2D, None)
        else:
            q_opt, _, path_gaussian = gPICP.LevenbergMarquardt(q0_2D, maxIter=100)

        # Convert back to 3D 
        pathPose3D = []
        for p in path_gaussian["pose"]:
            pathPose3D.append(poses2D.fromPosePDF2DTo3D(p, q0))

        path_gaussian["pose"] = pathPose3D
        q_opt["pose_cov"] = gPICP.LM_covariance_num(q_opt)
        q_opt = poses2D.fromPosePDF2DTo3D(q_opt, q0)
        print("2DpIC q_opt cov : ")
        print(q_opt["pose_cov"])

        # Take into account the computing time for gaussian approximation 
        # Not fair as it is not optimized .... 
        return q_opt, path_gaussian, a_pts_array, c_pts_array

    def experiment_gaussian(self, q0, q_gt_mean, firstScanMeas, secondScanMeas, onlyOptimization, **kwargs):
        """
            Experiment in the 3D case with gaussian approximation.
            It consists in estimating the relative between two scans.

            Parameters
            ----------
            q0 : dict with ["pose_mean"] and ["pose_cov"]
                 initial relative pose pdf between the two scan reference frame
            q_gt_mean : (6) array
                        Ground truth relative pose between the two scan reference framet
            firstScanMeas : (n) array of dict
                            array of parameters defining point pdf in local spherical coords (first scan)
            secondScanMeas : (m) array of dict
                             array of parameters defining point pdf in local spherical coords (second scan)
            onlyOptimization : bool
                               if true, skip the association step of pICP

            Returns
            -------
            q_opt : dict with ["pose_mean"] and ["pose_cov"]
                    estimated relative pose pdf between the two scan reference frame
            path_gaussian : dict of arrays 
                            contains details on pICP iterations (see gaussianPICP.py)
            a_pts_array : (n) array of dict with ["mean"] and ["cov"]
                          array of gaussian pdf of each point in the first scan
            c_pts_array : (m) array of dict with ["mean"] and ["cov"]
                          array of gaussian pdf of each point in the second scan

            Notes
            -----
            Experiment as in [2]. Also see gaussianPICP.py
        """
        
        a_pts_array, c_pts_array,_,_,_ = self.precompute_approxGaussian_pts(firstScanMeas, secondScanMeas)

        gPICP = gaussianPICP.gaussianPICP_se(a_pts_array, c_pts_array)
        gPICP.picpMaxIter = kwargs["picpMaxIter"]
        gPICP.associationType = kwargs["associationType"]

        if(not onlyOptimization):
            gPICP.showDisplay = self.showDisplayDebug
            gPICP.normalsInFirstScan_refFrame = kwargs["normals_firstFrame"]
            q_opt, path_gaussian = gPICP.PICP(q0, q_gt_mean)
        else:
            q_opt, _, path_gaussian = gPICP.LevenbergMarquardt(q0, maxIter=100)

        # Compute the covariance 
        cov = gPICP.LM_covariance(q_opt)
        q_opt["pose_cov"] = cov

        # Uncomment to fix z, pitch and roll and set related variances
        q_opt['pose_mean'][5] = q0["pose_mean"][5]
        q_opt['pose_mean'][2] = q0["pose_mean"][2]
        q_opt['pose_mean'][1] = q0["pose_mean"][1]

        # Take into account the computing time for gaussian approximation 
        # Not fair as it is not optimized .... 
        return q_opt, path_gaussian, a_pts_array, c_pts_array

    def experiment_mypicp(self, q0, firstScanMeas, secondScanMeas, onlyOptimization, **kwargs):
        """
            Experiment in the 3D case without gaussian approximation 

            Parameters
            ----------
            q0 : dict with ["pose_mean"] and ["pose_cov"]
                 initial relative pose pdf between the two scan reference frame
            firstScanMeas : (n) array of dict
                            array of parameters defining point pdf in local spherical coords (first scan)
            secondScanMeas : (m) array of dict
                             array of parameters defining point pdf in local spherical coords (second scan)
            onlyOptimization : bool
                               if true, skip the association step of pICP

            Returns
            -------
            q_opt : dict with ["pose_mean"] and ["pose_cov"]
                    estimated relative pose pdf between the two scan reference frame
            path : dict of arrays 
                   contains details on pICP iterations

            Notes
            -----
            See [3] and mypicp_variante.py
        """

        picp = mypicp.my_picp(kwargs['nSamplesCUT'])
        picp.picpMaxIter = kwargs["picpMaxIter"]
        if(kwargs['updateTest']):
            picp.useUpdateMeasurePose = True

        a_pts_array, c_pts_array, _, a_array_L, c_array_L = self.precompute_approxGaussian_pts(firstScanMeas,
                                                                                               secondScanMeas)
        firstScanMeas_forArray = data_io.convertForArrayVersion(firstScanMeas)
        secondScanMeas_forArray = data_io.convertForArrayVersion(secondScanMeas)

        modeX = picp.getModes_array(firstScanMeas_forArray)
        modeXprime = picp.getModes_array(secondScanMeas_forArray)


        if(not onlyOptimization):
            q_opt, path_mypicp = picp.pICP(q0, modeX, modeXprime, firstScanMeas_forArray,
                                                      secondScanMeas_forArray, errorConvThresh=1e-6,
                                                      picpMaxIter=kwargs['picpMaxIter'],  normalsInFirstScan_refFrame=kwargs["normals"], pointToPlane=kwargs["pointToPlane"],  firstScanGaussianApprox = a_pts_array,
                                                      secondScanGaussianApprox = c_pts_array)
        else:
            # Here, suppose perfect association ! 
            q_opt, _, path_mypicp = picp.LM_variante_array(q0, modeX, modeXprime,
                                                           firstScanMeas_forArray, secondScanMeas_forArray,
                                                           tau=3, e1=1e-8, e2=1e-8, maxIter=40, firstScanGaussianApprox=a_array_L,
                                                           secondScanGaussianApprox=c_array_L, firstScanMeas= a_pts_array, secondScanMeas=c_pts_array)

        # Uncomment to fix z, pitch and roll 
        q_opt['pose_mean'][5] = q0["pose_mean"][5]
        q_opt['pose_mean'][2] = q0["pose_mean"][2]
        q_opt['pose_mean'][1] = q0["pose_mean"][1]

        return q_opt, path_mypicp

    def picpTrial(self, q0, q_gt_mean, firstScanMeas, secondScanMeas, method, onlyOptimization, **kwargs):
        """
            Experiments on pICP algorithms
            
            Parameters
            ----------
            q0 : dict with ["pose_mean"] and ["pose_cov"]
                 initial relative pose pdf between the two scan reference frame
            q_gt_mean : (6) array
                        Ground truth relative pose between the two scan reference framet
            firstScanMeas : (n) array of dict
                            array of parameters defining point pdf in local spherical coords (first scan)
            secondScanMeas : (m) array of dict
                             array of parameters defining point pdf in local spherical coords (second scan)
            method : str
                     name of the algorithm to use. Can be "mypicp" ([3]), "gaussian"([2]) or "girona2D"([1])
            onlyOptimization : bool
                               if true, skip the association step of pICP   
            
            Returns
            -------
            dataPICP : dataPICP
                       object containing the data related to pICP results. See dataPICP.py
        """

        if(method == "mypicp"):
            qopt, path = self.experiment_mypicp(q0, firstScanMeas, secondScanMeas, onlyOptimization, **kwargs)
        elif(method == "gaussian"):
            qopt, path, _, _ = self.experiment_gaussian(q0, q_gt_mean, firstScanMeas, secondScanMeas, onlyOptimization, **kwargs)
        elif(method == "girona2D"):
            qopt, path, _, _ = self.experiment_girona2D(q0, q_gt_mean, firstScanMeas, secondScanMeas, onlyOptimization, **kwargs)

        distancesSE3 = []
        if(q_gt_mean is not None):
            for pose in path['pose']:
                distancesSE3.append(math_utility.distanceSE3(pose['pose_mean'], q_gt_mean))

        return dataPICP(qopt, path, distancesSE3)

    def experiment_oneTrial_mypicp(self, q_gt, q0, firstScanMeas, secondScanMeas, normals_firstFrame, normals_secondFrame, onlyOptimization):
        """
            One run of an experiment of pICP without gaussian approximation

            Parameters
            ----------
            q_gt : dict with ["pose_mean"] and ["pose_cov"]
                 Ground truth relative pose pdf between the two scan reference frame
            q0 : dict with ["pose_mean"] and ["pose_cov"]
                 initial relative pose pdf between the two scan reference frame
            firstScanMeas : (n) array of dict
                            array of parameters defining point pdf in local spherical coords (first scan)
            secondScanMeas : (m) array of dict
                             array of parameters defining point pdf in local spherical coords (second scan)
            normals_firstFrame : (n) array of dict
                                 normals of the first scan 
            normals_secondFrame : (n) array of dict
                                  normals of the second scan
            onlyOptimization : bool
                               if true, skip the association step of pICP   
            
            Returns
            -------
            res : dict with ["dist"] and ["q_opt"]
                  results with the distance between estimated pose and GT and the estimated mean pose q_opt

            Notes
            -----
            See [3] and mypicp_variante.py 
        """

        self.ignoreUniformDist = True
        data_mypicp_8 = self.picpTrial(q0,
                                       q_gt["pose_mean"], firstScanMeas, secondScanMeas,
                                       "mypicp", onlyOptimization, picpMaxIter=7, nSamplesCUT=8, updateTest=False,
                                       normals=normals_firstFrame)
        dist_mypicp = math_utility.distanceSE3(data_mypicp_8.q_opt["pose_mean"], q_gt["pose_mean"])

        return {"dist" : dist_mypicp, "q_opt" :data_mypicp_8.q_opt["pose_mean"]}

    def experiment_comparison_oneTrial(self, q_gt, q0, firstScanMeas, secondScanMeas, normals_firstFrame, normals_secondFrame, onlyOptimization):
        """
            One run of experiment with the same data with different pICP algorithms for comparisoan

            Parameters
            ----------
            q_gt : dict with ["pose_mean"] and ["pose_cov"]
                 Ground truth relative pose pdf between the two scan reference frame
            q0 : dict with ["pose_mean"] and ["pose_cov"]
                 initial relative pose pdf between the two scan reference frame
            firstScanMeas : (n) array of dict
                            array of parameters defining point pdf in local spherical coords (first scan)
            secondScanMeas : (m) array of dict
                             array of parameters defining point pdf in local spherical coords (second scan)
            normals_firstFrame : (n) array of dict
                                 normals of the first scan 
            normals_secondFrame : (n) array of dict
                                  normals of the second scan
            onlyOptimization : bool
                               if true, skip the association step of pICP  

            Returns
            -------
            res : dict
                  contains the results for each algo 
        """

        picpIter = 40

        dist_init = math_utility.distanceSE3(q0["pose_mean"], q_gt["pose_mean"])

        # 2D (based on Girona papers) 
        data_2D_girona = self.picpTrial(q0, q_gt["pose_mean"], firstScanMeas, secondScanMeas,
                                       "girona2D", onlyOptimization, picpMaxIter=picpIter, normals_firstFrame=normals_firstFrame, normals_secondFrame=normals_secondFrame)
        dist2D = math_utility.distanceSE3(data_2D_girona.q_opt["pose_mean"], q_gt["pose_mean"])

        # MpIC (point-to-point)
        self.ignoreUniformDist = True
        data_gaussian = self.picpTrial(q0, q_gt["pose_mean"], firstScanMeas, secondScanMeas,
                                       "gaussian", onlyOptimization, picpMaxIter=picpIter, normals_firstFrame=normals_firstFrame,
                                       normals_secondFrame=normals_secondFrame, associationType = "pointToPoint")
        dist3D = math_utility.distanceSE3(data_gaussian.q_opt["pose_mean"], q_gt["pose_mean"])

        # MpIC with uniform arcs 
        self.ignoreUniformDist = False
        data_gaussian_allArcs = self.picpTrial(q0, q_gt["pose_mean"], firstScanMeas, secondScanMeas,
                                       "gaussian", onlyOptimization, picpMaxIter=picpIter, normals_firstFrame=normals_firstFrame,
                                        normals_secondFrame=normals_secondFrame, associationType = "pointToPoint")
        dist3D_allArcs = math_utility.distanceSE3(data_gaussian_allArcs.q_opt["pose_mean"], q_gt["pose_mean"])


        # MpIC (point-to-plane) starting with point-to-point MpIC results 
        self.ignoreUniformDist = True
        data_gaussian_plane_startPoint = self.picpTrial(data_gaussian.q_opt, q_gt["pose_mean"], firstScanMeas, secondScanMeas,
                                             "gaussian", onlyOptimization, picpMaxIter=picpIter,
                                             normals_firstFrame=normals_firstFrame,
                                             normals_secondFrame=normals_secondFrame, associationType="pointToPlane")
        dist3D_plane_startPoint = math_utility.distanceSE3(data_gaussian_plane_startPoint.q_opt["pose_mean"], q_gt["pose_mean"])

        data_gaussian = data_2D_girona
        data_gaussian_allArcs = data_2D_girona
        data_gaussian_plane_startPoint = data_2D_girona
        dist3D = dist2D
        dist3D_allArcs = dist2D
        dist3D_plane_startPoint = dist2D
        return {"q_gt":q_gt["pose_mean"],
                "q0": q0["pose_mean"],
                "qopt_2D": data_2D_girona.q_opt["pose_mean"],
                "qopt_3D": data_gaussian.q_opt["pose_mean"],
                "qopt_3D_allArcs":data_gaussian_allArcs.q_opt["pose_mean"],
                "qopt_3D_plane_startPoint": data_gaussian_plane_startPoint.q_opt["pose_mean"],
                "dist_init": dist_init,
                "dist2D": dist2D,
                "dist3D": dist3D,
                "dist3D_allArcs": dist3D_allArcs,
                "dist3D_plane_startPoint": dist3D_plane_startPoint
                }

    def experiment_comparison_oneTrial_simulatedKarstData(self, trajectoryFile, trajectoryGTFile,
                                                          horizontalSonarDataFile, estimatedNormalsFile, saveFolder, onlyOptimization=False):
        """
            Experiment with simulated karst data

            Parameters
            ----------
            trajectoryFile : str
                             file containing the trajectory
            trajectoryGTFile : str
                               file containing the ground truth trajectory
            horizontalSonarDataFile : str
                                      file containing data on the horizontal sonar data
            estimatedNormalsFile : str
                                   file containing data on the normals pdf
            saveFolder : str
                         folder where the results are saved
            onlyOptimization : bool
                               if true, skip the association step of pICP  

            Returns
            -------
            res : dict
                  contains the results for each algo  
        """

        print("Traj file : {}".format(trajectoryFile))
        # Simulate Karst data 
        # Note that q0 here is the tranformation between the first and second scan reference frame 
        # --> Need to take its inverse for the transformation between the point clouds 
        q0, q_gt, trajectory, trajectory_gt, firstScan_refFrame, secondScan_refFrame, firstScan_poses_relative, secondScan_poses_relative =self.loadTrajAndOdo_loopClosure(
                                                               trajectoryFile, trajectoryGTFile)
        firstScanMeas, secondScanMeas, nonUniformIdxs = data_io.loadHorizontalSonarSimulatedData(horizontalSonarDataFile,
                                                                         firstScan_poses_relative,
                                                                         secondScan_poses_relative,
                                                                         self.rho_std, self.psi_std, self.b)

        normals_firstFrame, normal_secondFrame = data_io.loadNormals(estimatedNormalsFile, firstScan_refFrame, secondScan_refFrame,
                                                             nonUniformIdxs)

        # Initial transformation 
        if (onlyOptimization):
            saveFolder += "onlyOptimization/"

        res = self.experiment_comparison_oneTrial(q_gt, q0, firstScanMeas, secondScanMeas, normals_firstFrame, normal_secondFrame,onlyOptimization)
        res["scdRefFrame_z"]     = secondScan_refFrame["pose_mean"][5]
        res["scdRefFrame_pitch"] = secondScan_refFrame["pose_mean"][1]
        res["scdRefFrame_roll"]  = secondScan_refFrame["pose_mean"][2]

        return res

    def experiment_comparison_oneTrial_simulatedKarstData_mypicp(self, trajectoryFile, trajectoryGTFile,
                                                          horizontalSonarDataFile, estimatedNormalsFile, saveFolder,
                                                          q0_mean,
                                                          onlyOptimization=False):
        """
            Experiment with simulated karst data with no gaussian approximation

            Parameters
            ----------
            trajectoryFile : str
                             file containing the trajectory
            trajectoryGTFile : str
                               file containing the ground truth trajectory
            horizontalSonarDataFile : str
                                      file containing data on the horizontal sonar data
            estimatedNormalsFile : str
                                   file containing data on the normals pdf
            saveFolder : str
                         folder where the results are saved
            q0_mean : (6) array
                      initial relative pose 
            onlyOptimization : bool
                               if true, skip the association step of pICP  

            Returns
            -------
            res : dict
                  contains the results for each algo  
        """

        print("Traj file : {}".format(trajectoryFile))
        # Simulate Karst data 
        # Note that q0 here is the tranformation between the first and second scan reference frame 
        # --> Need to take its inverse for the transformation between the point clouds 
        q0, q_gt, trajectory, trajectory_gt, firstScan_refFrame, secondScan_refFrame, firstScan_poses_relative, secondScan_poses_relative = self.loadTrajAndOdo_loopClosure(
            trajectoryFile, trajectoryGTFile)
        firstScanMeas, secondScanMeas, nonUniformIdxs = data_io.loadHorizontalSonarSimulatedData(horizontalSonarDataFile,
                                                                                              firstScan_poses_relative,
                                                                                              secondScan_poses_relative,
                                                                                              self.rho_std, self.psi_std, self.b)

        normals_firstFrame, normal_secondFrame = data_io.loadNormals(estimatedNormalsFile, firstScan_refFrame,
                                                                    secondScan_refFrame,
                                                                    nonUniformIdxs)

        # Initial transformation 
        # saveFolder = "/home/breux/pid-workspace/packages/karst_slam/share/resources/experiment_sonarPICP_karst/"
        if (onlyOptimization):
            saveFolder += "onlyOptimization/"

        q0["pose_mean"] = q0_mean
        res = self.experiment_oneTrial_mypicp(q_gt, q0, firstScanMeas, secondScanMeas, normals_firstFrame,
                                                  normal_secondFrame, onlyOptimization)
        res["scdRefFrame_z"] = secondScan_refFrame["pose_mean"][5]
        res["scdRefFrame_pitch"] = secondScan_refFrame["pose_mean"][1]
        res["scdRefFrame_roll"] = secondScan_refFrame["pose_mean"][2]

        return res

    def experiment_comparison_oneTrial_simulatedKarstData_loopClosure(self,
                                                                      trajectoryFile, trajectoryGTFile,
                                                                      horizontalSonarDataFile,
                                                                      estimatedNormalsFile,
                                                                      onlyOptimization=False):
        """
            Experiment with simulated karst data 

            Parameters
            ----------
            trajectoryFile : str
                             file containing the trajectory
            trajectoryGTFile : str
                               file containing the ground truth trajectory
            horizontalSonarDataFile : str
                                      file containing data on the horizontal sonar data
            estimatedNormalsFile : str
                                   file containing data on the normals pdf
            saveFolder : str
                         folder where the results are saved
            onlyOptimization : bool
                               if true, skip the association step of pICP  

            Returns
            -------
            res : dict
                  contains the results for each algo  
        """

        # Simulate Karst data 
        # Note that q0 here is the tranformation between the first and second scan reference frame 
        # --> Need to take its inverse for the transformation between the point clouds 
        q0, q_gt, trajectory, trajectory_gt, firstScan_refFrame, secondScan_refFrame, firstScan_poses_relative, secondScan_poses_relative = self.loadTrajAndOdo_loopClosure(trajectoryFile, trajectoryGTFile)
        firstScanMeas, secondScanMeas, _ = data_io.loadHorizontalSonarSimulatedData(horizontalSonarDataFile,
                                                                                    firstScan_poses_relative,
                                                                                    secondScan_poses_relative,
                                                                                    self.rho_std, self.psi_std, self.b)

        normals = data_io.loadNormals(estimatedNormalsFile, firstScan_refFrame, len(firstScanMeas))  # expressed in firstScan_refFrame

        # Initial transformation 
        saveFolder = "/home/breux/pid-workspace/packages/karst_slam/share/resources/experiment_sonarPICP_karst_loopClosure/"
        if (onlyOptimization):
            saveFolder += "onlyOptimization/"

        return self.experiment_comparison_oneTrial(q_gt, q0, firstScanMeas, secondScanMeas, normals,
                                            onlyOptimization)

    ##############################################################################################################
    #                                   SIMULATED DATA
    ##############################################################################################################

    def experiment_comparison_oneTrial_simulatedData(self, rng_seed, onlyOptimization = False):
        """
            Experiment (one run)

            Parameters
            ----------
            rng_seed : int
                       seed for random number generator
            onlyOptimization : bool
                               if true, skip the association step of pICP

            Returns
            -------
            res : dict
                  contains the results for each algo
        """

        # Simulate data 
        q_gt, q0, firstScanMeas, secondScanMeas, trajectory = self.sdg.simulateData(rng_seed)

        # Initial transformation
        saveFolder = "experiment_sonarPICP/"
        if(onlyOptimization):
            saveFolder += "onlyOptimization/"

        self.experiment_comparison_oneTrial(q_gt, q0, firstScanMeas, secondScanMeas, onlyOptimization)


    def experiment_comparison_simulatedData(self, nTrial, onlyOptimization = False):
        """
            Experiment with several runs for different pICP algo

            Parameters
            ----------
            nTrial : int
                     number of experiment run
            onlyOptimization : bool
                               if true, skip the association step of pICP

            Returns
            -------
            res : dict
                  contains the results for each algo
        """

        for i in range(81, nTrial):
            print("Trial {}".format(i))
            self.experiment_comparison_oneTrial_simulatedData(rng_seed=i, onlyOptimization=onlyOptimization)

    def experiment_simulation(self):
        """
            Experiment 
        """

        nExp = 50
        sps.experiment_comparison_oneTrial_simulatedData(nExp, True)

        folder = "/home/breux/pid-workspace/packages/karst_slam/share/resources/experiment_sonarPICP/onlyOptimization/"
        '''plotPICP.plotBoxPlot_comparisonDistribution(folders= [folder + "gaussian/",
                                                              folder + "gaussian_afterML5/",
                                                              folder + "gaussian_afterML10/",
                                                              folder + "gaussian_afterML20/"
                                                              ],
                                                    colors=['r', 'g', 'b', 'm', 'c'],
                                                    labels=["approxGaussian", "approxGaussian_afterML5",  "approxGaussian_afterML10",
                                                            "approxGaussian_afterML20"])'''

        plotPICP.plotGraphs_sonarPICP_simulation(folder + "gaussian/exp_gaussian_" + str(nExp) + ".txt",
                                                 [#folder + "gaussian_afterML5/exp_gaussian_" + str(nExp) + ".txt",
                                                  #folder + "gaussian_afterML10/exp_gaussian_" + str(nExp) + ".txt",
                                                  folder + "gaussian_afterML20/exp_gaussian_" + str(nExp) + ".txt",
                                                  folder + "halfGaussian/exp_halfGaussian_" + str(nExp) + ".txt"],
                                                 colors=['r', 'g', 'b', 'm', 'c', 'orange', 'black'],
                                                 labels=["approxGaussian", "gaussian_ML20", "halfGaussian"])
                                                 
    ##############################################################################################################
    ##############################################################################################################

    def experiment_karstDonut_internal_mp(self, input_folder, folders_sorted):
        """
            Experiment on the karst donut model (multiprocessing to speed up computation)

            Parameters
            ----------
            input_folder : str
                           folder containing the data obtained by the simulation in the karst donut (cf c++ code)
            folders_sorted : array of str
                             array of folder name (one for each pair of scans)

            Returns
            -------
            res : array of results
                  array containing the results of experiments for each pair of scans (see experiment_comparison_oneTrial for the details)

            Notes
            -----
            In the C++ code where the data simulation is done, each data related to a pair of scan [i,i+1] is saved in a folder named "i"
            We then estimate the relative poses for each pair of scan.
            Here, parallelize the processing of several pair of scan to speed up the computation.
        """

        pool = mp.Pool(mp.cpu_count())
        arglist = []
        for f in folders_sorted:
            current_folder = input_folder + "/" + f + "/"
            arglist.append((current_folder + "bodyPoses.txt", #trajectoryFile
                            current_folder + "bodyPoses_gt.txt", #trajectoryGTFile
                            current_folder + "horizontalSonarEstimatedData.txt", #horizontalSonarDataFile
                            current_folder + "res_estimatedNormalsNew.txt", #estimatedNormalsFile
                            current_folder,
                            False) ) #onlyOptimization

        res = pool.starmap(self.experiment_comparison_oneTrial_simulatedKarstData, arglist)
        pool.close()
        pool.join()

        return res

    def experiment_karstDonut_internal(self, input_folder, folders_sorted):
        """
            Experiment on the karst donut model 

            Parameters
            ----------
            input_folder : str
                           folder containing the data obtained by the simulation in the karst donut (cf c++ code)
            folders_sorted : array of str
                             array of folder name (one for each pair of scans)

            Returns
            -------
            res : array of results
                  array containing the results of experiments for each pair of scans (see experiment_comparison_oneTrial for the details)

            Notes
            -----
            In the C++ code where the data simulation is done, each data related to a pair of scan [i,i+1] is saved in a folder named "i"
            We then estimate the relative poses for each pair of scan.
        """
        
        res = []
        for f in folders_sorted:
            current_folder = input_folder + "/" + f + "/"
            res.append(self.experiment_comparison_oneTrial_simulatedKarstData(current_folder + "bodyPoses.txt", #trajectoryFile
                                                                              current_folder + "bodyPoses_gt.txt", #trajectoryGTFile
                                                                              current_folder + "horizontalSonarEstimatedData.txt", #horizontalSonarDataFile
                                                                              current_folder + "res_estimatedNormalsNew.txt", #estimatedNormalsFile
                                                                              current_folder,
                                                                              False))
        return res

    def experiment_karstDonut_mypicp_internal_mp(self, input_folder, folders_sorted, poses_init):
        """
            Experiment on the karst donut model (multiprocessing to speed up computation) without gaussian approximation

            Parameters
            ----------
            input_folder : str
                           folder containing the data obtained by the simulation in the karst donut (cf c++ code)
            folders_sorted : array of str
                             array of folder name (one for each pair of scans)
            poses_init : array of dict with ["pose_mean"] and ["pose_cov"]
                         array of initial relative poses

            Returns
            -------
            res : array of results
                  array containing the results of experiments for each pair of scans (see experiment_comparison_oneTrial for the details)

            Notes
            -----
            See [3] for pICP without gaussian approximation.
            In the C++ code where the data simulation is done, each data related to a pair of scan [i,i+1] is saved in a folder named "i"
            We then estimate the relative poses for each pair of scan.
            Here, parallelize the processing of several pair of scan to speed up the computation.
        """

        pool = mp.Pool(mp.cpu_count())
        arglist = []
        for f,q0 in zip(folders_sorted,poses_init):
            current_folder = input_folder + "/" + f + "/"
            arglist.append((current_folder + "bodyPoses.txt",  # trajectoryFile
                            current_folder + "bodyPoses_gt.txt",  # trajectoryGTFile
                            current_folder + "horizontalSonarEstimatedData.txt",  # horizontalSonarDataFile
                            current_folder + "res_estimatedNormalsNew.txt",  # estimatedNormalsFile
                            current_folder,
                            q0,
                            False))  # onlyOptimization


        res = pool.starmap(self.experiment_comparison_oneTrial_simulatedKarstData_mypicp, arglist)
        pool.close()
        pool.join()

        return res

    def experiment_karstDonut_mypicp_internal(self, input_folder, folders_sorted, dinit, poses_init):
        """
            Experiment on the karst donut model (multiprocessing to speed up computation) without gaussian approximation

            Parameters
            ----------
            input_folder : str
                           folder containing the data obtained by the simulation in the karst donut (cf c++ code)
            folders_sorted : array of str
                             array of folder name (one for each pair of scans)
            dinit : array of float
                    distances of relative poses with GT and initial poses
            poses_init : array of dict with ["pose_mean"] and ["pose_cov"]
                         array of initial relative poses

            Returns
            -------
            res : array of results
                  array containing the results of experiments for each pair of scans (see experiment_comparison_oneTrial for the details)

            Notes
            -----
            See [3] for pICP without gaussian approximation.
            In the C++ code where the data simulation is done, each data related to a pair of scan [i,i+1] is saved in a folder named "i"
            We then estimate the relative poses for each pair of scan.
        """

        res = []
        print("size folders/poses_init : {} ,{}".format(len(folders_sorted), len(poses_init)))
        for f,q0,d in zip(folders_sorted,poses_init,dinit):
            current_folder = input_folder + "/" + f + "/"
            res_ = self.experiment_comparison_oneTrial_simulatedKarstData_mypicp(current_folder + "bodyPoses.txt",
                                                                              # trajectoryFile
                                                                              current_folder + "bodyPoses_gt.txt",
                                                                              # trajectoryGTFile
                                                                              current_folder + "horizontalSonarEstimatedData.txt",
                                                                              # horizontalSonarDataFile
                                                                              current_folder + "res_estimatedNormalsNew.txt",
                                                                              # estimatedNormalsFile
                                                                              current_folder,
                                                                              q0,
                                                                              False)
            res.append(res_)
            print("dinit   : {}".format(d))
            print("dmypicp : {}".format(res_["dist"]))
        return res

    def experiment_karstDonut_mypicp(self, input_folder, multithread=True):
        """
            Experiment on the karst donut model without gaussian approximation.
            The results are saved in the input_folder

            Parameters
            ----------
            input_folder : str
                           folder containing the data obtained by the simulation in the karst donut (cf c++ code)
            multithread : bool
                          if true, parallelize computation

            Notes
            -----
            See [3] for pICP without gaussian approximation.
            In the C++ code where the data simulation is done, each data related to a pair of scan [i,i+1] is saved in a folder named "i"
            We then estimate the relative poses for each pair of scan.
        """

        # Sort the folders 
        folders_sorted = data_io.getSubFoldersSortedList(input_folder)

        # Load initial transformation from MpIC results 
        dinit, poses_init = data_io.loadDistancesPosesFromFile(folder + "/dist3D_chi2_0.5.txt")

        if (multithread):
            res = self.experiment_karstDonut_mypicp_internal_mp(input_folder, folders_sorted, poses_init)
        else:
            res = self.experiment_karstDonut_mypicp_internal(input_folder, folders_sorted, dinit, poses_init)

        #TODO the chi2 value in the saved file is set manually here :/ ---> error prone
        # should set it automatically !
        fileName = input_folder + "/dist_mypicp_chi2_0.5.txt"
        with open(fileName, "w") as file:
            file.write("# distance to q_gt, pose q \n")
            for r in res:
                file.write(str(r["dist"]) + ",")
                writePoseFunc(file, r["q_opt"])

    def experiment_karstDonut(self, input_folder, multithread=True):
        """
            Experiment with simulation on karst donut 
            Results are saved in the input_folder

            Parameters
            ----------
            input_folder : str
                           folder containing the data obtained by the simulation in the karst donut (cf c++ code)
            multithread : bool
                          if true, parallelize computation

            Notes
            -----
            In the C++ code where the data simulation is done, each data related to a pair of scan [i,i+1] is saved in a folder named "i"
            We then estimate the relative poses for each pair of scan.
        """
        
        # Sort the folders 
        folders_sorted = data_io.getSubFoldersSortedList(input_folder)

        if(multithread):
            res = self.experiment_karstDonut_internal_mp(input_folder, folders_sorted)
        else:
            res = self.experiment_karstDonut_internal(input_folder, folders_sorted)

        L = ["dist_init",
             "dist2D",
             "dist3D",
             "dist3D_allArcs",
             "dist3D_plane_startPoint",
             ]
        Q = ["q0",
             "qopt_2D",
             "qopt_3D",
             "qopt_3D_allArcs",
             "qopt_3D_plane_startPoint",
            ]

        for l,q in zip(L,Q):
            #TODO the chi2 value in the saved file is set manually here :/ ---> error prone
            # should set it automatically !
            fileName = input_folder + "/" + l + "_chi2_0.5.txt"
            with open(fileName, "w") as file:
                file.write("# distance to q_gt, pose q \n")
                for res_ in res:
                    file.write(str(res_[l]) + ",")
                    data_io.writePoseFunc(file, res_[q])
        with open(input_folder + "/q_gt.txt","w") as file:
            for res_ in res:
                data_io.writePoseFunc(file, res_["q_gt"])

    def experiment_simulationKarst_loopClosureSim(self, folder):
        """
            Experiment 

            Parameters
            ----------
            folder : str
                     folder containing the simulated data
        """

        trajectoryFile = folder + "/bodyPoses.txt"  # trajectoryFile
        trajectoryGTFile = folder + "/bodyPoses_gt.txt"  # trajectoryGTFile
        horizontalSonarDataFile = folder + "/horizontalSonarEstimatedData.txt"  # horizontalSonarDataFile
        estimatedNormalsFile = folder + "/res_estimatedNormalsNew.txt"  # estimatedNormalsFile

        # Simulate Karst data 
        # Note that q0 here is the transformation between the first and second scan reference frame 
        # --> Need to take its inverse for the transformation between the point clouds 
        q0, q_gt, trajectory, trajectory_gt, firstScan_refFrame, secondScan_refFrame, firstScan_poses_relative, secondScan_poses_relative = self.loadTrajAndOdo_loopClosure(
            trajectoryFile, trajectoryGTFile)
        firstScanMeas, secondScanMeas, nonUniformIdxs = data_io.loadHorizontalSonarSimulatedData(horizontalSonarDataFile,
                                                                                                firstScan_poses_relative,
                                                                                                secondScan_poses_relative,
                                                                                                self.rho_std, self.psi_std, self.b)

        normals_firstFrame, normal_secondFrame = data_io.loadNormals(estimatedNormalsFile, firstScan_refFrame,
                                                                      secondScan_refFrame,
                                                                      nonUniformIdxs)

        # Don't use q0 but add a higher noise to q_gt to simulate something similar in order to a loopClosure
        nIter = 50

        # Diag cov. Use absolute cov for z, pitch and roll
        cov = np.zeros((6, 6))
        cov[5, 5] = q0["pose_cov"][5, 5]
        cov[1, 1] = q0["pose_cov"][1, 1]
        cov[2, 2] = q0["pose_cov"][2, 2]
        cov[0, 0] = 3.5 * q0["pose_cov"][0, 0]
        cov[3, 3] = 3.5 * q0["pose_cov"][3, 3]
        cov[4, 4] = 3.5 * q0["pose_cov"][4, 4]
        q0["pose_cov"] = cov

        argList = []
        q0_means_generated = np.random.multivariate_normal(q_gt["pose_mean"], cov, size=nIter)

        '''for q0 in q0_means_generated:
            self.experiment_comparison_oneTrial(q_gt,
                                                {"pose_mean": q0, "pose_cov":cov},
                                                firstScanMeas,
                                                secondScanMeas,
                                                normals_firstFrame,
                                                normal_secondFrame,
                                                False)'''

        for q0_mean in q0_means_generated:
            argList.append((q_gt,
                            {"pose_mean": q0_mean, "pose_cov":cov},
                            firstScanMeas,
                            secondScanMeas,
                            normals_firstFrame,
                            normal_secondFrame,
                            False))  # onlyOptimization

        pool = mp.Pool(mp.cpu_count())
        res_all = pool.starmap(self.experiment_comparison_oneTrial, argList)
        pool.close()
        pool.join()

        #TODO the chi2 value in the saved file is set manually here :/ ---> error prone
        # should set it automatically !
        with open(folder + "/loopClosureSimu_chi2_0.05.txt","w") as file:
            file.write("#dinit, d2d, d3D, d3D_allArcs, d3D_plane_startPoint\n")
            for res in res_all:
                file.write(
                    "{0},{1},{2},{3},{4}\n".format(str(res["dist_init"]), str(res["dist2D"]), str(res["dist3D"]),
                                                       str(res["dist3D_allArcs"]), str(res["dist3D_plane_startPoint"])))

    
    def experiment_oneScanGraph(self,folder,scanNumber):
        """
            Plot graph of distance (q0, qopt) wrt picp iteration for a single pair of scan

            Parameters
            ----------
            folder : str
                     folder containing the data for each pair of scans
            scanNumber : int
                         index of the pair of scan
        """

        current_folder = folder + "/" + str(scanNumber) + "/"
        res, dataPICPList = self.experiment_comparison_oneTrial_simulatedKarstData( trajectoryFile=current_folder + "bodyPoses.txt",
                                                                                    trajectoryGTFile=current_folder + "bodyPoses_gt.txt",
                                                                                    horizontalSonarDataFile=current_folder + "horizontalSonarEstimatedData.txt",
                                                                                    estimatedNormalsFile=current_folder + "res_estimatedNormalsNew.txt",
                                                                                    saveFolder=current_folder,
                                                                                    onlyOptimization=False)
        plotPICP.plotGraphs_DistanceWRTIteration(0, dataPICPList, ["2D", "3Dp", "3DpA", "3Dpl"], ['-','-','-','-'], 4, True, folder + "/graphScan_" + str(scanNumber))

    def pIC_realData(self,folder, method):
        """
            Experiment using the girona dataset

            Parameters
            ----------
            folder : str
                     folder containing the data (robot pose pdf etc ...)
            method : str
                     name of the pICP algo to use

            Notes
            -----
            See [1] for the Girona dataset
        """

        bodyPose_firstFrame_file = folder + "/bodyPoses_firstScan.txt"
        bodyPose_secondFrame_file = folder + "/bodyPoses_secondScan.txt"
        refFrame_timestamps_file = folder + "/refFrames_timestamps.txt"
        horizontalSonarDataFile = folder + "/horizontalSonarEstimatedData.txt"  # horizontalSonarDataFile
        estimatedNormalsFile = folder + "/res_estimatedNormalsNew.txt"
        q0, firstScan_refFrame, secondScan_refFrame, firstScan_poses_relative, secondScan_poses_relative = self.loadTrajAndOdo(bodyPose_firstFrame_file, 
                                                                                                                                           bodyPose_secondFrame_file, refFrame_timestamps_file)
        firstScanMeas, secondScanMeas, nonUniformIdxs = data_io.loadHorizontalSonarSimulatedData(horizontalSonarDataFile,
                                                                                                firstScan_poses_relative,
                                                                                                secondScan_poses_relative,
                                                                                                self.rho_std, self.psi_std, self.b)

        normals_firstFrame, normals_secondFrame = data_io.loadNormals(estimatedNormalsFile, firstScan_refFrame,
                                                                  secondScan_refFrame,
                                                                  nonUniformIdxs)

        picpIter = 40

        print("q0 cov: ")
        print(q0["pose_cov"])

        if(method == "all"):
            with open(folder + "/pICresults_dr.txt","a") as file:
                data_io.writePosePDF(file, q0)

            print("[sonarPIC_simulation.py::pIC_realData] Scan Matching with 2DpIC ... ")
            self.ignoreUniformDist = False
            data = self.picpTrial(q0, None, firstScanMeas, secondScanMeas,
                                        "girona2D", False, picpMaxIter=picpIter, normals_firstFrame=normals_firstFrame, normals_secondFrame=normals_secondFrame)
            with open(folder + "/pICresults_2DpIC.txt", "a") as file:
                data_io.writePosePDF(file, data.q_opt) #data_io.writePoseFunc(file, data.q_opt["pose_mean"])

            print("[sonarPIC_simulation.py::pIC_realData] Scan Matching with MpIC ... ")
            self.showDisplayDebug = False
            self.ignoreUniformDist = True
            data_gaussian = self.picpTrial(q0, None, firstScanMeas, secondScanMeas,
                                        "gaussian", False, picpMaxIter=picpIter, normals_firstFrame=normals_firstFrame,
                                        normals_secondFrame=normals_secondFrame, associationType = "pointToPoint")
            with open(folder + "/pICresults_MpIC.txt","a") as file:
                data_io.writePosePDF(file, data_gaussian.q_opt) #data_io.writePoseFunc(file, data_gaussian.q_opt["pose_mean"])

            print("[sonarPIC_simulation.py::pIC_realData] Scan Matching with MpICa ... ")
            self.ignoreUniformDist = False
            data = self.picpTrial(q0, None, firstScanMeas, secondScanMeas,
                                        "gaussian", False, picpMaxIter=picpIter, normals_firstFrame=normals_firstFrame,
                                            normals_secondFrame=normals_secondFrame, associationType = "pointToPoint")
            with open(folder + "/pICresults_MpICa.txt","a") as file:
                data_io.writePosePDF(file, data.q_opt) #data_io.writePoseFunc(file, data.q_opt["pose_mean"])

            print("[sonarPIC_simulation.py::pIC_realData] Scan Matching with MpICp ... ")
            self.ignoreUniformDist = True
            data = self.picpTrial(q0, None, firstScanMeas, secondScanMeas,
                                  "gaussian", False, picpMaxIter=picpIter,
                                  normals_firstFrame=normals_firstFrame,
                                  normals_secondFrame=normals_secondFrame, associationType="pointToPlane")
            with open(folder + "/pICresults_MpICp.txt","a") as file:
                data_io.writePosePDF(file, data.q_opt) #data_io.writePoseFunc(file, data.q_opt["pose_mean"])
        else:
            if(method == "2DpIC"):
                self.ignoreUniformDist = False
                data = self.picpTrial(q0, None, firstScanMeas, secondScanMeas,
                                        "girona2D", False, picpMaxIter=picpIter, normals_firstFrame=normals_firstFrame, normals_secondFrame=normals_secondFrame)
            elif(method == "MpIC"):
                self.ignoreUniformDist = True
                data = self.picpTrial(q0, None, firstScanMeas, secondScanMeas,
                                        "gaussian", False, picpMaxIter=picpIter, normals_firstFrame=normals_firstFrame,
                                        normals_secondFrame=normals_secondFrame, associationType = "pointToPoint")
            elif(method == "MpICa"):
                self.ignoreUniformDist = False
                data = self.picpTrial(q0, None, firstScanMeas, secondScanMeas,
                                      "gaussian", False, picpMaxIter=picpIter, normals_firstFrame=normals_firstFrame,
                                            normals_secondFrame=normals_secondFrame, associationType = "pointToPoint")
            elif(method == "MpICp"):
                self.ignoreUniformDist = True
                data = self.picpTrial(q0, None, firstScanMeas, secondScanMeas,
                                      "gaussian", False, picpMaxIter=picpIter, normals_firstFrame=normals_firstFrame,
                                      normals_secondFrame=normals_secondFrame, associationType = "pointToPoint")

                data = self.picpTrial(data.q_opt, None, firstScanMeas, secondScanMeas,
                                                "gaussian", False, picpMaxIter=picpIter,
                                                normals_firstFrame=normals_firstFrame,
                                                normals_secondFrame=normals_secondFrame, associationType="pointToPlane")

            with open(folder + "/pICresults.txt","a") as file:
                data_io.writePoseFunc(file, data.q_opt["pose_mean"])

def picWithRealData():
    """
        Main function for computing relative poses between two scans with girona dataset 

        Notes
        -----
        See [1] for the Girona dataset
    """

    sps = sonarPICPSimulation()
    sps.horizontalSonarPoseOnRobot = {"pose_mean": np.array([np.pi, 0., 0., 0.1, 0., -0.42]),
                                      "pose_cov": np.zeros((6, 6))}
    folder = sys.argv[1]
    method = sys.argv[2]
    sps.pIC_realData(folder, method)

def expSimu():
    sps = sonarPICPSimulation()
    sps.horizontalSonarPoseOnRobot = {"pose_mean": np.full((6,), 0.),
                                      "pose_cov": np.zeros((6, 6))}
    folder = "/home/breux/surfaceMeasureData/expKarstDonut_200steps"
    # sps.experiment_oneScanGraph(folder,50)
    # sps.experiment_karstDonut_mypicp(folder, multithread=False)
    #sps.experiment_karstDonut(folder)
    #sps.experiment_simulationKarst_loopClosureSim(folder + "/15")
    plotPICP.plotComparisonDistributionsRatioICP(folder)
    plotPICP.plotDistancesICP_scatter(folder)

#------------- MAIN ---------------------
if __name__ == "__main__":
    #expSimu()
    picWithRealData()
