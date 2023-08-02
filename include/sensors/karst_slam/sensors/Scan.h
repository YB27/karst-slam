#ifndef SCAN_H
#define SCAN_H

#include "karst_slam/typedefs.h"
#include <mrpt/poses/CPointPDFGaussian.h>
#include <mrpt/math/CMatrix.h>
#include <map>

namespace karst_slam{namespace sensors{

/** Handy struct which contains a sonar measure (range and angle if beam width not ignored) */
struct sonarMeasure
{
    sonarMeasure() = default;
    sonarMeasure(double range_,
                 double phi_,
                 double theta_ = 0.)
    {
        range = range_;
        phi   = phi_;
        theta = theta_;
    }

    double range; //!< Sonar range in meters
    double phi; //!< Sonar rotation angle (beam angle) 
    double theta = 0.; // Used only when ignoreBeamWidth is false
};

/** Tuple containing a pair of cosinus/sinus of an angle */
struct CosSin
{
    CosSin() = default;
    CosSin(double angle_)
    {
        angle = angle_;
        cos = std::cos(angle);
        sin = std::sin(angle);
    }

    double angle;
    double cos;
    double sin;
};

/** Structure used to compute the covariance matrix of scan points in local cartesian coords */
struct CartesianCov_lut
{
    CartesianCov_lut() = default;
    CartesianCov_lut(const mrpt::math::CMatrixDouble33& sphericalCov,
                     const double& cos_yaw,
                     const double& sin_yaw)
    {
        // Compute [a_ij] = J*Cov_spheric*J^T with J jacobian evaluated with theta (pitch) = 0
        // as a function of r
        // Here precompute coefficients of the polynoms a_ij(r) for efficiency
        double cc = cos_yaw*cos_yaw, ss = sin_yaw*sin_yaw, sc = sin_yaw*cos_yaw;
        double sigma_r_sqr     = sphericalCov(0,0),
               sigma_yaw_sqr   = sphericalCov(1,1),
               sigma_pitch_sqr = sphericalCov(2,2),
               sigma_r_yaw     = sphericalCov(0,1),
               sigma_r_pitch   = sphericalCov(0,2),
               sigma_yaw_pitch = sphericalCov(1,2);

        // a_11 = a - b*r + c*r^2
        // a_12 = d + e*r + f*r^2
        // a_13 =     g*r - h*r^2
        // a_22 = i + b*r + j*r^2
        // a_23 =     k*r + l*r^2
        // a_33 =           sigma_pitch_sqr*r^2
        precomp_coeffs[0] = cc*sigma_r_sqr;
        precomp_coeffs[1] = 2.*sc*sigma_r_yaw;
        precomp_coeffs[2] = ss*sigma_yaw_sqr;
        precomp_coeffs[3] = sc*sigma_r_sqr;
        precomp_coeffs[4] = (cc - ss)*sigma_r_yaw;
        precomp_coeffs[5] = sc*sigma_yaw_sqr;
        precomp_coeffs[6] = cos_yaw*sigma_r_pitch;
        precomp_coeffs[7] = sin_yaw*sigma_yaw_pitch;
        precomp_coeffs[8] = ss*sigma_r_sqr;
        precomp_coeffs[9] = cc*sigma_yaw_sqr;
        precomp_coeffs[10] = sin_yaw*sigma_r_pitch;
        precomp_coeffs[11] = cos_yaw*sigma_yaw_sqr;
        precomp_coeffs[12] = sigma_pitch_sqr;
    }

    /** Compute the covariance at the given value r using the precomputed coefficients */
    mrpt::math::CMatrixDouble33 getCovariance(const double& r) const
    {
        mrpt::math::CMatrixDouble33 cov = Eigen::Matrix<double,3,3>::Zero();
        double rr = r*r;
        cov(0,0) = precomp_coeffs[0] - precomp_coeffs[1]*r + precomp_coeffs[2]*rr;
        cov(0,1) = precomp_coeffs[3] + precomp_coeffs[4]*r + precomp_coeffs[5]*rr;
        cov(0,2) = precomp_coeffs[6]*r - precomp_coeffs[7]*rr;
        cov(1,1) = precomp_coeffs[8] + precomp_coeffs[1]*r + precomp_coeffs[9]*rr;
        cov(1,2) = precomp_coeffs[10]*r + precomp_coeffs[11]*rr;
        cov(2,2) = precomp_coeffs[12]*rr;
        cov(1,0) = cov(0,1);
        cov(2,0) = cov(0,2);
        cov(2,1) = cov(1,2);

        return cov;
    }

private:
    double precomp_coeffs[13]; //!< Coefficients of polynoms in r appearing in the covariance definition.
};

/**
 * Convert the raw measure in the sonar frame in spherical coords to cartesian coords
 * 
 * @param measure sonar measure (ie in spherical coords in sonar local frame)
 * @param sphericalCov covariance matrix of the measure
 * @return 3D point pdf expressed in cartesian coords in sonar local frame
 */
mrpt::poses::CPointPDFGaussian fromSonarCoordsToCartesian(const sonarMeasure& measure,
                                                          const Eigen::Matrix3d& sphericalCov);

/**
 * Convert the raw measure in the sonar frame (in spherical coords) to its (cartesian) coordinates in
 * the robot frame
 * 
 * @param measure sonar measure (ie in spherical coords in sonar local frame)
 * @param robotGlobalPose Robot pose
 * @param sensorPoseOnRobot pose of the sonar in the robot body frame
 * @param sphericalCov covariance matrix of the measure
 * @return point pdf expressed in cartesian coords in the robot frame
 */
mrpt::poses::CPointPDFGaussian fromSonarLocalCoordsToGlobalCoords(const sonarMeasure& measure,
                                                                       const mrpt::poses::CPose3D& robotGlobalPose,
                                                                       const mrpt::poses::CPose3D& sensorPoseOnRobot,
                                                                       const Eigen::Matrix3d& sphericalCov);

/**
 * Convert the raw measure in the sonar frame (in spherical coords) to its (cartesian) coordinates in
 * the robot frame
 * 
 * @param measure sonar measure (ie in spherical coords in sonar local frame)
 * @param sensorPoseOnRobot pose of the sonar in the robot body frame
 * @param sphericalCov covariance matrix of the measure
 * @return point pdf expressed in cartesian coords in the robot frame
 */
mrpt::poses::CPointPDFGaussian fromSonarLocalCoordsToRobotLocalCoords(const sonarMeasure& measure,
                                                                      const mrpt::poses::CPose3D& sensorPoseOnRobot,
                                                                      const Eigen::Matrix3d& sphericalCov);

}} // end namespaces

#endif // SCAN_H
