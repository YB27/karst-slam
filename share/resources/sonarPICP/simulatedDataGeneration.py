import numpy as np

class simulatedDataGenerator():
    """ 
        Class used for genering data used in [1], section 6.1, figure 8

        References
        ----------
        .. [1] Yohan Breux and André Mas and Lionel Lapierre, "On-manifold Probabilistic ICP : Application to Underwater Karst
               Exploration"
    """
     
    default_incr_pose_cov = 5.*np.array([[0.00001, 0., 0., 0., 0., 0.],
                    [0., 0.00001, 0., 0., 0., 0.],
                    [0., 0., 0.00001, 0., 0., 0.],
                    [0., 0., 0., 0.0001, 0., 0.],
                    [0., 0., 0., 0., 0.0001, 0.],
                    [0., 0., 0., 0., 0., 0.0001]])

    def __init__(self, n_meas=100, dx=0.1, deuler=0, incr_pose_cov= default_incr_pose_cov):
        """
            Constructor

            Parameters
            ----------
            n_meas : int
                     number of points to simulate
            dx : float
                 relative displacement along x between each trajectory pose
            deuler : float
                     relative angle between each trajectory pose
            incr_pose_cov : (6,6) ndarray
                            covariance of the relative displacement between two successive poses in Trajectory
        """

        # Number of points in the simulated environnment
        self.n_meas = n_meas
        # Relative displacement along x between each trajectory pose
        self.dx = dx
        # Relative angle between each trajectory pose
        self.deuler = deuler
        # Covariance of the relative displacement between two successive poses in Trajectory
        self.incr_pose_cov = incr_pose_cov
    
    def generateTrajectory(self):
        """ 
            Generate the robot trajectory

            Returns
            -------
            trajectory : array of pose pdf
                         generated noisy robot trajectory as an array of pose pdf
            trajectory_gt : array of pose pdf
                            ground truth robot trajectory as an array of pose pdf
            q_gt_mean : (6) array
                        Ground truth relative mean pose between the two scans reference frame
            q0 : dict with ["pose_mean"] and ["pose_cov"]
                 initial relative pose between the two scans reference frame
            firstScan_poses_relative : array of poses pdf   
                                       array of first scan robot poses relative to the first scan reference frame
            secondScan_poses_relative : array of poses pdf   
                                        array of second scan robot poses relative to the second scan reference frame
        """

        # Noisy trajectory
        trajectory = []

        # Ground-truth trajectory
        trajectory_gt = []

        current_pose_pdf_gt = {'pose_mean': np.zeros((6,)), 'pose_cov': np.zeros((6, 6))}
        current_pose_pdf    = {'pose_mean': np.zeros((6,)), 'pose_cov': np.zeros((6, 6))}
        mean_incr = np.array([self.deuler, 0., 0., self.dx, 0., 0.])
        trajectory.append(current_pose_pdf)
        trajectory_gt.append(current_pose_pdf_gt)

        relativePoseToPrevious = []
        relativePoseToPrevious_gt = []
        for i in range(1, 2*self.n_meas):
            incr_pose_pdf_gt = {'pose_mean': np.random.multivariate_normal(mean_incr, self.incr_pose_cov), 'pose_cov': self.incr_pose_cov}
            current_pose_pdf_gt = poses_euler.composePosePDFEuler(current_pose_pdf_gt, incr_pose_pdf_gt)
            trajectory_gt.append(current_pose_pdf_gt)
            relativePoseToPrevious_gt.append(incr_pose_pdf_gt)

            incr_pose_pdf = {'pose_mean': mean_incr, 'pose_cov': self.incr_pose_cov}
            current_pose_pdf = poses_euler.composePosePDFEuler(current_pose_pdf, incr_pose_pdf)

            trajectory.append(current_pose_pdf)
            relativePoseToPrevious.append({'pose_mean': incr_pose_pdf["pose_mean"], 'pose_cov': incr_pose_pdf["pose_cov"]})

        middle_idx = int(np.rint(self.n_meas / 2))

        q_gt_mean, q0, firstScan_poses_relative, secondScan_poses_relative = self.generateScansPoseRelativeToReferenceFrame(middle_idx,
                                                                                                                            relativePoseToPrevious,
                                                                                                                            relativePoseToPrevious_gt)

        return trajectory, trajectory_gt, q_gt_mean, q0, firstScan_poses_relative, secondScan_poses_relative

    def generateRelativePoses(self, relativePoseToPrevious, middle_idx):
        """
            Generate the relative poses to the current considered scan reference frame (index middle idx)

            Parameters
            ----------
            relativePoseToPrevious : array of pose pdf
                                     relative poses of robot pose n wrt pose n-1 (odometry)
            middle_idx : int
                         middle idx which corresponds to the index of the reference frame

            Returns
            -------
            poses_relative : array of pose pdf
                             poses of the current scan expressed in the scan reference frame
        """

        poses_relative = []
        curPosePdf = {'pose_mean': np.zeros((6,)), 'pose_cov': np.zeros((6, 6))}
        poses_relative.append(curPosePdf)
        for relative_pose in relativePoseToPrevious[middle_idx : ]:
            curPosePdf = poses_euler.composePosePDFEuler(curPosePdf, relative_pose)
            if(self.useAbsoluteDepthPitchRoll):
                self.correctCovAbsoluteDepthPitchRoll(curPosePdf["pose_cov"])
            poses_relative.append(poses_euler.composePosePDFEuler(curPosePdf, self.horizontalSonarPoseOnRobot))

        curPosePdf = {'pose_mean': np.zeros((6,)), 'pose_cov': np.zeros((6, 6))}
        for relative_pose in reversed(relativePoseToPrevious[0:middle_idx]):
            curPosePdf = poses_euler.composePosePDFEuler(curPosePdf, poses_euler.inversePosePDFEuler(relative_pose))
            if(self.useAbsoluteDepthPitchRoll):
                self.correctCovAbsoluteDepthPitchRoll(curPosePdf["pose_cov"])
            poses_relative.insert(0, poses_euler.composePosePDFEuler(curPosePdf, self.horizontalSonarPoseOnRobot))

        return poses_relative

    def generateScansPoseRelativeToReferenceFrame(self, middle_idx, relativePoseToPrevious, relativePoseToPrevious_gt):
        """ 
            Generate the pdf poses relative to their (scan-centered) reference frame 

            Parameters
            ----------
            middle_idx : int
                         middle idx which corresponds to the index of the reference frame
            relativePoseToPrevious : array of pose pdf
                                     (noisy) relative poses of robot pose n wrt pose n-1 (odometry)
            relativePoseToPrevious_gt : array of pose pdf
                                        Ground truth relative poses of robot pose n wrt pose n-1 (odometry)
            
            Returns
            -------
            q_gt_mean : (6) array
                        Ground truth relative mean pose between the two scans reference frame 
            q0 : dict with ["pose_mean"] and ["pose_cov"]
                 initial relative pose between the two scans reference frame
            firstScan_poses_relative : array of poses pdf   
                                       array of first scan robot poses relative to the first scan reference frame
            secondScan_poses_relative : array of poses pdf   
                                        array of second scan robot poses relative to the second scan reference frame
        """

        firstScan_poses_relative  = self.generateRelativePoses(relativePoseToPrevious[0:2*middle_idx ], middle_idx)
        secondScan_poses_relative = self.generateRelativePoses(relativePoseToPrevious[2*middle_idx: 4 * middle_idx], middle_idx)

        q0 = {"pose_mean": np.zeros((6,)), "pose_cov": np.zeros((6,6))}
        for relativePose in relativePoseToPrevious[middle_idx:3*middle_idx]:
            q0 = poses_euler.composePosePDFEuler(q0, relativePose)
            if(self.useAbsoluteDepthPitchRoll):
                self.correctCovAbsoluteDepthPitchRoll(q0["pose_cov"])

        q_gt = {"pose_mean": np.zeros((6,)), "pose_cov": np.zeros((6, 6))}
        for relativePose in relativePoseToPrevious_gt[middle_idx:3 * middle_idx]:
            q_gt = poses_euler.composePosePDFEuler(q_gt, relativePose)
            if(self.useAbsoluteDepthPitchRoll):
                self.correctCovAbsoluteDepthPitchRoll(q_gt["pose_cov"])

        return q_gt["pose_mean"], q0, firstScan_poses_relative, secondScan_poses_relative

    def generatePoints(self):
        """ 
            Generate the point clouds 

            Returns
            -------
            points : (n,3) array
                     generated point cloud
        """

        # Generate alternatively points to the right and left
        points = []
        for i in range(0, self.n_meas):
            if(i%2):
                point = [i*self.dx,
                         np.random.uniform(self.point_y_min, self.point_y_max),
                         np.random.uniform(self.point_z_min, self.point_z_max)]
            else:
                point = [i*self.dx,
                         np.random.uniform(-self.point_y_max, -self.point_y_min),
                         np.random.uniform(self.point_z_min, self.point_z_max)]
            points.append(point)
        return np.array(points)
    
    def generateParameters(self, local_sphe_point, posePDF):
        """
            Generate data on the simulated measure of the point by a sonar

            Parameters
            ----------
            local_sphe_point : (3) array
                               spherical coord of a point in the sonar local frame
            posePDF : dict with ["pose_mean"] and ["pose_cov"]
                      pose of the robot at which the point has been measured

            Returns
            -------
            params : dict of data
                     contains data related to spherical coords pdf  

        """

        alpha, beta = betaDist.alphaBetaFromModeVar(local_sphe_point[1], self.theta_var, self.b)

        ''' Noisy measures '''
        rho_noisy = np.random.normal(loc=local_sphe_point[0], scale=self.rho_std)
        psi_noisy = np.random.normal(loc=local_sphe_point[2], scale=self.psi_std)

        params = {'rho': {'mean': rho_noisy, 'std': self.rho_std},
                  'theta': {'alpha': alpha, 'beta': beta, 'b': self.b},
                  'psi': {'mean': psi_noisy, 'std': self.psi_std},
                  'pose_mean': posePDF['pose_mean'], 'pose_cov': posePDF['pose_cov']
                 }
        return params

    def computeMeasurements(self, trajectory_gt, firstScan_poses_relative, secondScan_poses_relative, points):
        """ 
            Compute the sonars measurements (for two successive scans) 
            relative to the simulated trajectory and point clouds 

            Parameters
            ----------
            trajectory_gt : array of poses pdf
                            Ground truth robot trajectory
            firstScan_poses_relative : array of poses pdf   
                                       array of first scan robot poses relative to the first scan reference frame
            secondScan_poses_relative : array of poses pdf   
                                        array of second scan robot poses relative to the second scan reference frame
            points : (n,3) array
                     point cloud (simulated environment)

            Returns
            -------
            firstScanMeasure : array of dict of data
                               array of params for the first scan (see generateParameters())
            secondScanMeasure : array of dict of data
                               array of params for the second scan (see generateParameters())
        """
        
        firstScanMeasure = []
        secondScanMeasure = []

        # For each robot pose in its trajectory 
        # (note that here, each point is observed for two robot poses, one for each scan)
        for (pose1_gt, pose2_gt, pose1_relative, pose2_relative, point) in zip(trajectory_gt[0:self.n_meas], trajectory_gt[self.n_meas:2*self.n_meas], firstScan_poses_relative, secondScan_poses_relative, points):
            # Local positions of the observed point in the sonar frame (in cartesian coords)
            # Observed during the first scan
            local_cart_point1 = poses_euler.inverseComposePoseEulerPoint(pose1_gt['pose_mean'], point)
            # Observed during the second scan 
            local_cart_point2 = poses_euler.inverseComposePoseEulerPoint(pose2_gt['pose_mean'], point)

            # Same but convert to spherical coordinates
            local_sphe_point1 = math_utility.fromCartToSpherical(local_cart_point1)
            local_sphe_point2 = math_utility.fromCartToSpherical(local_cart_point2)

            # Check that observations of the point at the given trajectory poses are valid
            # (inside the sonar beam)
            # TODO : Very cheap way to do this and could be way better. 
            # But not need to spend time here as it was initially used for quick testing.
            # Better use the simulation on 3d models with ray tracing ! 
            if(np.abs(local_sphe_point1[1]) < self.maxTheta and
               np.abs(local_sphe_point2[1]) < self.maxTheta):
                firstScanMeasure.append(self.generateParameters(local_sphe_point1, pose1_relative))
                secondScanMeasure.append(self.generateParameters(local_sphe_point2, pose2_relative))
            else:
                print(" !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ")
                print("Point is outside one arc")
                print("Thetas : {},{}".format(local_sphe_point1[1], local_sphe_point2[1]))
                print(" !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ")

        return firstScanMeasure, secondScanMeasure   

    def simulateData(self, rng_seed=3):
        """
            Main function simulating a robot trajectory, an environment (fixed set of points) and the simulated measurements from sonar 

            Parameters
            ----------
            rng_seed : int
                       seed for the rng generator
            
            Returns
            -------
            q_gt_mean : (6) array
                        Ground truth relative mean pose between the two scans reference frame 
            q0 : dict with ["pose_mean"] and ["pose_cov"]
                 initial relative pose between the two scans reference frame
            firstScanMeasure : array of dict of data
                               array of params for the first scan (see generateParameters())
            secondScanMeasure : array of dict of data
                               array of params for the second scan (see generateParameters())
            trajectory : array of pose pdf
                         generated noisy robot trajectory as an array of pose pdf
        """
        # Fix to a given seed (allow repeatability)
        np.random.seed(rng_seed)

        # Generate a noisy linear trajectory 
        trajectory, trajectory_gt, q_gt_mean, q0, firstScan_poses_relative, secondScan_poses_relative = self.generateTrajectory()

        # Generate the set of environment points
        points = self.generatePoints()

        # Compute the corresponding measurements for each point
        firstScanMeas, secondScanMeas = self.computeMeasurements(trajectory_gt,
                                                                 firstScan_poses_relative,
                                                                 secondScan_poses_relative,
                                                                 points)

        return q_gt_mean, q0, firstScanMeas, secondScanMeas, trajectory