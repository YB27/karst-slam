#ifndef DEF_CURVILINEAR_ABSCISSA_SE3_H
#define DEF_CURVILINEAR_ABSCISSA_SE3_H

#include <mrpt/poses/CPose3DPDFGaussian.h>

/**
 * Functions to compute the approximated length of a curve in SE(3)
 * Here we suppose that movement between two instants t and t+1 are small enough to be approximated by the distance on the tangent plane
 *
 * To be exact, one should compute the geodesic distance between the successive poses $\Gamma_k$ and $\Gamma_{k+1}$.
 * The geodesic is given by
 * $ \forall t \in [0,1],\  Q(t) = \Gamma_k exp\left( t log\left(\Gamma_{k}^{-1}\Gamma_{k+1}\right) \right)$
 */
namespace karst_slam{namespace scanMerging {

/**
 * Left-invariant riemaniann metric on SE(3)
 */
const Eigen::Matrix<double,6,6> G = (Eigen::Matrix<double,6,6>() << 1., 0., 0., 0., 0., 0.,
                                                                    0., 1., 0., 0., 0., 0.,
                                                                    0., 0., 1., 0., 0., 0.,
                                                                    0., 0., 0., 2., 0., 0.,
                                                                    0., 0., 0., 0., 2., 0.,
                                                                    0., 0., 0., 0., 0., 2.).finished();
/**
 * Compute the distance increment between two successsives poses
 * 
 * @param current_pose
 * @param previous_pose
 * @return distance between the poses
 */
static double localCurvilinearAbscissa(const mrpt::poses::CPose3D& current_pose,
                                const mrpt::poses::CPose3D& previous_pose)
{
    // The curve length between two successives poses A,B in SE(3) is approximated here by
    // s = sqrt(log(A^-1*B)^T*G*log(A^-1*B)) where G = [I_3 0; 0 2*I_3] is a left-invariant riemaniann metric on SE(3)
    mrpt::poses::CPose3D invA_mut_B;
    invA_mut_B.inverseComposeFrom(current_pose, previous_pose);

    mrpt::math::CArrayDouble<6> log;
    invA_mut_B.ln(log);

    return sqrt((log.transpose()*G*log)(0));
}

// Currently, don't take into account incertitude
// This could be justified as the training data and prediction are made on the same poses
// (rethink this argument to be sure !)
/**
 * \overload
 */
static double localCurvilinearAbscissa(const mrpt::poses::CPose3DPDFGaussian& current_pose,
                                       const mrpt::poses::CPose3DPDFGaussian& previous_pose)
{
    return localCurvilinearAbscissa(current_pose.mean, previous_pose.mean);
}

}} // end namespaces

#endif // DEF_CURVILINEAR_ABSCISSA_SE3_H
