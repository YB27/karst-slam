import numpy as np
import scipy.stats.distributions

""" 
    Methods for data association step of ICP algorithms
    See [1], section 4.1 for details

    References
    ----------
    .. [1] Yohan Breux and André Mas and Lionel Lapierre, "On-manifold Probabilistic ICP : Application to Underwater Karst
    Exploration"
"""

def mahalanobisMetric(points1, points2):
    """
        Compute the mahalanobis distance between each pair of point from two point clouds

        Parameters
        ----------
        points1 : dict with key "mean" ((m,n) ndarray) and "cov" ( (m,n,n) ndarray)
                  first point pdf cloud of size m
        points2 : dict with key "mean" ((l,n) ndarray) and "cov" ( (l,n,n) ndarray)
                  second point pdf cloud of size l

        Returns
        -------
        distances : (m,l) ndarray
                    Matrix of mahalanobis distances. 
                    Lines (resp.) columns corresponds to points of the first (resp. second) point cloud 
    """

    n = points1['mean'].shape[1]
    nPoints1 = points1['mean'].shape[0]
    nPoints2 = points2['mean'].shape[0]

    # Avoid singular covariance matrices !
    smallCov = 1e-6*np.eye(n)

    diff = np.empty((nPoints1, nPoints2, n))
    meanCovs_inv = np.empty((nPoints1, nPoints2, n, n))

    # Add small cov to avoid singular covariance matrices !
    smallCov = 1e-6*np.eye(n)
    for i in range(0, nPoints1):
        diff[i,:,:] = (points2['mean'] - points1['mean'][i])
        meanCovs_inv[i,:,:,:] = 0.5*(np.linalg.inv(points2['cov'] + smallCov) + np.linalg.inv(points1['cov'][i] + smallCov))

    res = np.einsum('ijk,ijk->ij', diff, np.einsum('ijkl,ijl->ijk', meanCovs_inv, diff))
    return np.sqrt(res)

def dataAssociation_withDistanceMatrix(distancesMatrix, threshold):
    """ 
        Associate corresponding points from two point clouds 
        based on the pre-computed distances matrix for each pair of points. 

        TODO : Implement a look-up approach on octree as described in 
        Albert Palomer, Pere Ridao, and David Ribas. “Multibeam 3D underwater SLAM with probabilis-
        tic registration”. In: Sensors 16.4 (2016)

        Parameters
        ---------- 
        distancesMatrix : (n,m) ndarray
                           Matrix of distances between each pair of two point clouds
        threshold : float
                    distance threshold to consider two points as compatible

        Returns
        -------
        idxs_pointCloud_1 : (l) array
                            Indexes of associated point in the first point cloud 
        idxs_pointCloud_2 : (l) array
                            Indexes of associated point in the second point cloud  
        indiv_compatible_A : list of set
                             Set of indexes of the second point cloud for each point in the first point cloud
                             Corresponds to the set A_i in [1], section 4.1
    """
    
    idxs_pointCloud_1 = []
    idxs_pointCloud_2 = []
    indiv_compatible_A = [set() for i in range(distancesMatrix.shape[0])]

    # Sort per distance in increasing order 
    # https://stackoverflow.com/questions/29734660/python-numpy-keep-a-list-of-indices-of-a-sorted-2d-array 
    orderedIndexes = np.vstack(np.unravel_index(np.argsort(distancesMatrix, axis=None, kind='mergesort'), distancesMatrix.shape)).T

    alreadyAssociatedPoint2 = set()
    alreadyAssociatedPoint1 = set()
    for assocIndexes in orderedIndexes:
        compatible = (distancesMatrix[assocIndexes[0], assocIndexes[1]] < threshold)
        if(compatible):
             indiv_compatible_A[assocIndexes[0]].add(assocIndexes[1])

        if (assocIndexes[1] not in alreadyAssociatedPoint2 and
            assocIndexes[0] not in alreadyAssociatedPoint1 and
            compatible):
            idxs_pointCloud_1.append(assocIndexes[0])
            idxs_pointCloud_2.append(assocIndexes[1])
            alreadyAssociatedPoint1.add(assocIndexes[0])
            alreadyAssociatedPoint2.add(assocIndexes[1])

    return np.array(idxs_pointCloud_1), np.array(idxs_pointCloud_2), indiv_compatible_A 

def dataAssociation(func_metric, pointCloud_1, pointCloud_2, threshold, **kwargs):
    """ 
        Associate point from a point cloud to another 
        Here, points are probabilistic and depending on it, we use different metric 

        Parameters
        ----------
        func_metric : function
                      function computing the metric
        pointCloud_1 : dict with key "mean" ((m,n) ndarray) and "cov" ( (m,n,n) ndarray)
                       first point pdf cloud of size m
        pointCloud_2 : dict with key "mean" ((l,n) ndarray) and "cov" ( (l,n,n) ndarray)
                       second point pdf cloud of size l
        threshold : float
                    distance threshold to consider two points as compatible

        Returns
        -------
        idxs_pointCloud_1 : (l) array
                            Indexes of associated point in the first point cloud 
        idxs_pointCloud_2 : (l) array
                            Indexes of associated point in the second point cloud  
        indiv_compatible_A : list of set
                             Set of indexes of the second point cloud for each point in the first point cloud
                             Corresponds to the set A_i in [1], section 4.1
    """

    # Return the distances (matrix nPoint1 X nPoint2) 
    if(len(kwargs) == 0):
        distances = func_metric(pointCloud_1, pointCloud_2)
    else:
        distances = func_metric(**kwargs)

    return dataAssociation_withDistanceMatrix(distances, threshold)