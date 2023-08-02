import numpy as np
import scipy 
import dataAssociation

def testMahalanobisMetric():
    """
        Test the mahalanobis metric used for data association
    """

    A1 = np.random.rand(3,3) - 0.5
    A2 = np.random.rand(3,3) - 0.5
    cov1 = A1.T@A1 
    cov2 = A2.T@A2
    points1 = {'mean': np.array([[1.,2.,3.],
                                 [4.,5.,6.],
                                 [4.,5.,6.]]), 'cov': np.array([cov1, cov2, cov1])}

    points2 = {'mean': np.array([[6.,5.,4.],
                                 [3.,2.,1.]]), 'cov': np.array([cov2, cov1])}

    computed_array = np.empty((3,2))
    smallCov = 1e-6*np.eye(3)
    for i in range(0, points1['mean'].shape[0]):
        for j in range(0, points2['mean'].shape[0]):
            diff = points1['mean'][i] - points2['mean'][j]
            meanCov = 0.5*(np.linalg.inv(points1['cov'][i] + smallCov) + np.linalg.inv(points2['cov'][j] + smallCov))
            d_mahala = np.dot(diff, meanCov@diff)
            computed_array[i,j] = np.sqrt(d_mahala)
    
    print("Computed_array : ")
    print(computed_array)

    print("Result with function mahalanobisMetric :")
    print(dataAssociation.mahalanobisMetric(points1, points2))

    assert np.allclose(computed_array, dataAssociation.mahalanobisMetric(points1, points2), atol=1e-5)

def testDataAssoc():
    """
        Test the data association
    """
    
    points1 = {'mean': np.array([[1., 1., 1.],
                                 [2., 2., 2.],
                                 [3,3,3]]), 'cov': np.array([0.4 * np.eye(3),0.4  * np.eye(3),0.4  * np.eye(3)])}

    points2 = {'mean': np.array([[3.1,3.1,3.1],
                                 [1.1, 0.9, 1.1],
                                 [2.1,1.9,1.9]]), 'cov': np.array([0.4  * np.eye(3), 0.4*np.eye(3),0.4*np.eye(3)])}

    threshold = scipy.stats.distributions.chi2.ppf(0.95,df=3)
    idxs_pts1, idxs_pts2, indiv_comptability = dataAssociation.dataAssociation(dataAssociation.mahalanobisMetric, points1, points2, threshold)

    assert np.array_equal(idxs_pts1, np.array([0,1,2])) and \
           np.array_equal(idxs_pts2, np.array([1,2,0]))

# ------------- MAIN ---------------------
if __name__ == "__main__":
    #testDataAssoc()
    testMahalanobisMetric()