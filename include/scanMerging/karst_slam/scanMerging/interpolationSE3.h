#ifndef DEF_INTERPOLATION_SE3_H
#define DEF_INTERPOLATION_SE3_H

#include "karst_slam/compare.h"
#include "trajectoryPose.h"
#include <unsupported/Eigen/MatrixFunctions>
#include <boost/math/interpolators/barycentric_rational.hpp>
#include <set>
#include <map>

namespace karst_slam{namespace scanMerging {

enum INTERPOLATION_TYPE{LINEAR = 0,
                        SPLINE = 1};

/**
 * Linear interpolation between two given 3D poses
 * 
 * @param current_pose 
 * @param previous_pose 
 * @param t parameter in [0,1]
 * @return interpolated 3D pose
 * */ 
mrpt::poses::CPose3D linearInterpolateSE3(const mrpt::poses::CPose3D& current_pose,
                                          const mrpt::poses::CPose3D& previous_pose,
                                          double t);

/**
 * Linear interpolation in SE(3) for two successives poses pdf
 * 
 * @param current_pose
 * @param previous_pose
 * @param t parameter in [0,1]
 * @return interpolated 3D pose pdf
 */
mrpt::poses::CPose3DPDFGaussian linearInterpolateSE3(const mrpt::poses::CPose3DPDFGaussian& current_pose,
                                                     const mrpt::poses::CPose3DPDFGaussian& previous_pose,
                                                     double t);

/**
 * Linear interpolation of 3d pose covariance matrix
 * 
 * @param current_cov
 * @param previous_cov
 * @param t parameter in [0,1]
 * @result interpolated covariance matrix
 */ 
mrpt::math::CMatrixDouble66 linearInterpolateCovariance(const mrpt::math::CMatrixDouble66& current_cov,
                                                        const mrpt::math::CMatrixDouble66& previous_cov,
                                                        double t);
/** 
 * Interpolate linearly poses inside a given 3D trajectory (vector of poses pdf) at given timestamps
 * 
 * @param timeStamps timestamps for which an interpolated pose must be computed
 * @param poses 3d trajectory (map with timestamp as key)
 * @param onlyInterpolatedValues If true, only return the interpolated poses without the original trajectory poses.
 * @return trajectory with the interpolated poses (map with timestamps as key)
*/
std::map<uint64_t,trajectoryPose<mrpt::poses::CPose3DPDFGaussian>> linearInterpolateTrajectorySE3AtGivenPoses(const std::vector<uint64_t>& timeStamps,
                                                                                                              const std::map<uint64_t, trajectoryPose<mrpt::poses::CPose3DPDFGaussian>>& poses,
                                                                                                              bool onlyInterpolatedValues = false);

// Note : Only valid if between any poses, the rotation do not cross pi (e.g 2*pi - eps and eps for any eps)
// because exp is surjective (exp is bijective if limited to |w| < pi, w \in so(3))
/**
 * Interpolate poses with spline inside a given 3D trajectory (vector of poses pdf) at given timestamps
 * 
 * @param timeStamps timestamps for which an interpolated pose must be computed 
 * @param poses 3d trajectory (map with timestamp as key)
 * @param onlyInterpolatedValues If true, only return the interpolated poses without the original trajectory poses.
 * @return trajectory with the interpolated poses (map with timestamps as key)
 */ 
std::map<uint64_t,trajectoryPose<mrpt::poses::CPose3DPDFGaussian>> splineInterpolateTrajectorySE3AtGivenPoses(const std::vector<uint64_t>& timeStamps,
                                                                                                              const std::map<uint64_t, trajectoryPose<mrpt::poses::CPose3DPDFGaussian>>& poses,
                                                                                                              bool onlyInterpolatedValues = false);

/**
 * Interpolate poses inside a given 3D trajectory (vector of poses pdf) at given timestamps
 * 
 * @param timeStamps timestamps for which an interpolated pose must be computed 
 * @param poses 3d trajectory (map with timestamp as key)
 * @param type type of interpolation (LINEAR or SPLINE)
 * @param onlyInterpolatedValues If true, only return the interpolated poses without the original trajectory poses.
 * @return trajectory with the interpolated poses (map with timestamps as key)
 */ 
std::map<uint64_t, trajectoryPose<mrpt::poses::CPose3DPDFGaussian>> interpolateTrajectorySE3AtGivenPoses(const std::vector<uint64_t>& timeStamps,
                                                                                                         const std::map<uint64_t, trajectoryPose<mrpt::poses::CPose3DPDFGaussian>>& poses,
                                                                                                         INTERPOLATION_TYPE type,
                                                                                                         bool onlyInterpolatedValues = false);

}}

#endif // DEF_INTERPOLATION_SE3_H