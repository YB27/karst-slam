import numpy as np
import scipy as scp
import math_utility
import gaussianPICP
import poses_euler
import poses_quat
import plotPICP
import dataAssociation
import time
import multiprocessing as mp

import gaussianPICP
from dataPICP import *

"""
    Code for comparing results obtained with different pose representation
    (3D + Euler, 3D + Quaternion, se(3)) by the optimization step of a classical pIC method 
    with normally distributed 3D points. (eg association is supposed known).
    It corresponds to [1], section 6.1 (Figure 8)

    TODO : refactor with classes and inheritence (also for poses euler and quat !)

    References
    ----------
    .. [1] Yohan Breux and André Mas and Lionel Lapierre, 
           "On-manifold Probabilistic ICP : Application to Underwater Karst Exploration"
"""

def generatePointClouds(q_gt_mean, n, cov_scale):
    """ 
        Generate random point clouds for the experiment 

        Parameters
        ----------
        q_gt_mean : (6) array
                    Ground truth relative mean pose between the point clouds
        n : int
            Number of points to generate in each point cloud
        cov_scale : float
                    scale to control the magnitude of the covariances

        Returns
        -------
        a_pts_array : dict with "mean" ((n,3) ndarray) and "cov" ((n,3,3) ndarray)
                      First point pdf cloud generated
        c_pts_array : dict with "mean" ((n,3) ndarray) and "cov" ((n,3,3) ndarray)
                      Second point pdf cloud generated
    """

    # We use here the same notation as in our paper [1], section 4.
    c_pts_array = {"mean": None, "cov": None}
    a_pts_array = {"mean": None, "cov": None}

    # Generate means 
    points = []
    d = 10
    for i in range(0, n):
        point = [np.random.uniform(-d, d), np.random.uniform(-d, d), np.random.uniform(-d, d)]
        points.append(point)
    c_pts_array["mean"] = np.array(points)

    # Generate covariance 
    c_pts_array["cov"] = np.empty((n, 3, 3))
    for i in range(0, n):
        mat = cov_scale*np.random.rand(3,3)
        c_pts_array["cov"][i] = mat.T@mat

    # Sample the ground truth points from c_pts_array
    c_gt_points = np.empty((n, 3))
    i = 0
    for mean_c, cov_c in zip(c_pts_array["mean"], c_pts_array["cov"]):
        c_gt_points[i] = np.random.multivariate_normal(mean_c, cov_c)
        i += 1

    # Compute the second cloud from the ground truth point of the first cloud 
    rot = scp.spatial.transform.Rotation.from_euler('ZYX', q_gt_mean[0:3])
    a_samples = rot.apply(c_gt_points) + q_gt_mean[3:6]

    # Generate the second point cloud distributions 
    a_pts_array["cov"] = np.empty((n, 3, 3))
    a_pts_array["mean"] = np.empty((n, 3))
    for i in range(0, n):
        mat = cov_scale * np.random.rand(3, 3)
        a_pts_array["cov"][i] = mat.T @ mat
        a_pts_array["mean"][i] = np.random.multivariate_normal(a_samples[i], a_pts_array["cov"][i])

    return a_pts_array, c_pts_array

def optimize(q_gt_mean, q0, maxIter, picp):
    """
        Optimization to estimate the relative pose between point clouds knowing the associations

        Parameters
        ----------
        q_gt_mean : (6) array
                    Ground truth relative mean pose between the point clouds
        q0 : dict with "pose_mean" ((6) array) and "pose_cov" ((6,6) ndarray)
             Initial relative pose pdf
        picp : gaussianPICP 
               PICP algorithm

        Returns
        -------
        res : dataPICP
              PICP results data. See dataPICP.py
    """

    # The association step is omitted (points already corresponding) so only 
    # go for the optimization step  
    q, Fval, path = picp.LevenbergMarquardt(q0)

    # Compute distances at each step to the ground truth transformation 
    distancesSE3 = []
    for pose in path['pose']:
        # Ugly. Will be changed when refactoring with classes 
        if (q_gt_mean.shape[0] == 6):
            distancesSE3.append(poses_euler.distanceSE3(pose['pose_mean'], q_gt_mean))
        else:
            distancesSE3.append(poses_quat.distanceSE3(pose['pose_mean'], q_gt_mean))
    return dataPICP(q, path, distancesSE3)

def experiment(rng_seed, n, cov_scale_point, cov_scale_q, maxIter):
    """ 
        Main function  : compute pIC for the different representation 

        Parameters
        ----------
        n : int
            Number of points to generate for the point clouds 
        cov_scale_point : float
                          scale to control the magnitude of the covariances for the points
        cov_scale_q : float
                      scale to control the magnitude of the covariances for the initial relative pose
        maxIter : int
                  Maximum number of iterations for the PICP optimization step

        Returns 
        -------
        d0 : float
             SE(3) distance between initial pose q0 and ground truth pose
        d_euler : float
                  SE(3) distance between results obtained with gaussianPICP_Euler and ground truth pose
        d_quat : float
                 SE(3) distance between results obtained with gaussianPICP_Quaternion and ground truth pose
        d_se3 : float
                SE(3) distance between results obtained with gaussianPICP_se and ground truth pose
        cost_euler : float
                     Final cost function value (sum of mahalanobis distances)
        cost_quat : float
                    Final cost function value (sum of mahalanobis distances)
        cost_se3 : float
                   Final cost function value (sum of mahalanobis distances)    
    """
    np.random.seed(rng_seed)
    # For numerical jacobian in Euler and Quaternion cases 
    increments_euler = np.full((6,), 1e-5)
    increments_quat = np.full((7,), 1e-5)

    # Generate the simulated data 
    mat = cov_scale_q*np.random.rand(6, 6)
    q_euler_cov = mat.T @ mat
    q0_euler_mean = np.random.rand(6)
    q0_euler = {"pose_mean": q0_euler_mean, "pose_cov": q_euler_cov}
    q0_quat = poses_euler.fromPosePDFEulerToPosePDFQuat(q0_euler)

    # Sample initial transformation
    q_gt_euler_mean = np.random.multivariate_normal(q0_euler_mean, q_euler_cov)
    q_gt_quat_mean = poses_euler.fromPoseEulerToPoseQuat(q_gt_euler_mean)

    a_pts_array, c_pts_array = generatePointClouds(q_gt_euler_mean, n, cov_scale_point)

    # Distance initiale 
    d0 = poses_euler.distanceSE3(q0_euler["pose_mean"], q_gt_euler_mean)

    datas = []
    labels = []

    # Euler
    picp_euler = gaussianPICP.gaussianPICP_Euler(a_pts_array, c_pts_array)
    dataEuler = optimize(q_gt_euler_mean,
                           q0_euler,
                           maxIter,
                           picp_euler)
    datas.append(dataEuler)
    labels.append("Euler")

    # Quaternion
    picp_quat = gaussianPICP.gaussianPICP_Quaternion(a_pts_array, c_pts_array)
    dataQuaternion = optimize(q_gt_quat_mean,
                                q0_quat,
                                maxIter,
                                picp_quat)

    datas.append(dataQuaternion)
    labels.append("Quaternion")

    # se(3)
    picp_se = gaussianPICP.gaussianPICP_se(a_pts_array, c_pts_array)
    data_se = optimize(q_gt_euler_mean,
                         q0_euler,
                         maxIter,
                         picp_se)
    datas.append(data_se)
    labels.append("SE")

    # Display and Save results 
    #colors = ['r','g','b']
    #plotPICP.plotGraphs_DistanceWRTIteration(0, datas, colors, labels,linestyles=['-','-'], linewidth=1, alpha=0.6)
    #plt.show()

    return d0, datas[0].distancesSE3[-1], datas[1].distancesSE3[-1], datas[2].distancesSE3[-1], datas[0].path["cost"][-1], datas[1].path["cost"][-1], datas[2].path["cost"][-1]

def experiment_nRuns_mp(nTrial, n, cov_scale_point, cov_scale_q, maxIter):
    # For multithreading
    pool = mp.Pool(mp.cpu_count())
    res = pool.starmap(experiment, [(i, n, cov_scale_point, cov_scale_q, maxIter) for i in range(nTrial)])
    pool.close()
    pool.join()

    print(res)

    d0, d_euler, d_quat, d_se, _, _, _ = zip(*res)
    d0 = np.array(d0)
    d_euler = np.array(d_euler)
    d_quat = np.array(d_quat)
    d_se = np.array(d_se)
    d_euler_rel = d0 / d_euler
    d_quat_rel = d0 / d_quat
    d_se_rel = d0 / d_se

    # Save the results
    folder = "../experiment_gaussianPICP_compare/"
    with open(folder + "experiment_data_PICP_d0_mp.txt", "w") as file:
        for d in d0:
            file.write(str(d) + "\n")
    with open(folder + "experiment_data_PICP_deuler_mp.txt", "w") as file:
        for d in d_euler:
            file.write(str(d) + "\n")
    with open(folder + "experiment_data_PICP_dquat_mp.txt", "w") as file:
        for d in d_quat:
            file.write(str(d) + "\n")
    with open(folder + "experiment_data_PICP_dse_mp.txt", "w") as file:
        for d in d_se:
            file.write(str(d) + "\n")
    with open(folder + "experiment_data_PICP_euler_rel_mp.txt", "w") as file:
        for d in d_euler_rel:
            file.write(str(d) + "\n")
    with open(folder + "experiment_data_PICP_quat_rel_mp.txt", "w") as file:
        for d in d_quat_rel:
            file.write(str(d) + "\n")
    with open(folder + "experiment_data_PICP_se_rel_mp.txt", "w") as file:
        for d in d_se_rel:
            file.write(str(d) + "\n")

def experiment_nRuns(nTrial, n, cov_scale_point, cov_scale_q, maxIter):
    """ 
        Run several experiments with different point clouds and save results as 
        relative results (ration initial distance / optimized distance) in files

        Parameters
        ----------
        nTrial : int
                 Number of experiments to run
        n : int
            Number of points to generate for the point clouds 
        cov_scale_point : float
                          scale to control the magnitude of the covariances for the points
        cov_scale_q : float
                      scale to control the magnitude of the covariances for the initial relative pose 
        maxIter : int
                  Maximum number of iterations for the PICP optimization step
    """

    #For multithreading
    #pool = mp.Pool(mp.cpu_count())
    #res = pool.map(self.experiment, [(n, cov_scale_point, cov_scale_q, max_iter) for _ in range(nTrial)])

    res = {"d_0": [], "d_quat": [], "d_euler": [], "d_se" : [], "d_quat_rel": [], "d_euler_rel": [], "d_se_rel": []}
    for i in range(0, nTrial):
        print("Experiment i : {}".format(i))
        d0, d_euler, d_quat, d_se, cost_euler, cost_quat, cost_se = experiment(i, n, cov_scale_point, cov_scale_q, maxIter)
        res["d_0"].append(d0)
        res["d_euler"].append(d_euler)
        res["d_quat"].append(d_quat)
        res["d_se"].append(d_se)
        res["d_quat_rel"].append(d0 / d_quat)
        res["d_euler_rel"].append(d0 / d_euler)
        res["d_se_rel"].append(d0 / d_se)

    # Save the results
    folder = "../experiment_gaussianPICP_compare/"
    with open(folder + "experiment_data_PICP_d0.txt","w") as file:
        for d in res["d_0"]:
            file.write(str(d) + "\n")
    with open(folder + "experiment_data_PICP_deuler.txt","w") as file:
        for d in res["d_euler"]:
            file.write(str(d) + "\n")
    with open(folder + "experiment_data_PICP_dquat.txt","w") as file:
        for d in res["d_quat"]:
            file.write(str(d) + "\n")
    with open(folder + "experiment_data_PICP_dse.txt","w") as file:
        for d in res["d_se"]:
            file.write(str(d) + "\n")
    with open(folder + "experiment_data_PICP_euler_rel.txt","w") as file:
        for d in res["d_euler_rel"]:
            file.write(str(d) + "\n")
    with open(folder + "experiment_data_PICP_quat_rel.txt","w") as file:
        for d in res["d_quat_rel"]:
            file.write(str(d) + "\n")
    with open(folder + "experiment_data_PICP_se_rel.txt","w") as file:
        for d in res["d_se_rel"]:
            file.write(str(d) + "\n")

# ------------- MAIN ---------------------
if __name__ == "__main__":
    #d0, d_euler, d_quat, d_se, cost_euler, cost_quat, cost_se = experiment(100, 0.5, 0.5, 100)

    #experiment_nRuns(6, 100, 0.25, 0.25, 100)
    #experiment_nRuns_mp(500, 100, 0.5, 0.5, 100)

    #d_euler_relatif = np.genfromtxt("experiment_data_PICP_euler.txt")
    #d_quat_relatif = np.genfromtxt("experiment_data_PICP_quat.txt",)
    #d_se_relatif = np.genfromtxt("experiment_data_PICP_se.txt")

    plotPICP.plotBoxPlot_comparisonRepresentation_("../experiment_gaussianPICP_compare/experiment_data_PICP_d0_mp.txt",
                                                   "../experiment_gaussianPICP_compare/experiment_data_PICP_deuler_mp.txt",
                                                   "../experiment_gaussianPICP_compare/experiment_data_PICP_dquat_mp.txt",
                                                   "../experiment_gaussianPICP_compare/experiment_data_PICP_dse_mp.txt",
                                                  True,
                                                  "comparisonRepresentation")