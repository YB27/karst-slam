#ifndef DEF_SIMULATED_POINT_AT_THETA_H
#define DEF_SIMULATED_POINT_AT_THETA_H

#include <mrpt/poses/CPoint3D.h>
#include "karst_slam/sensors/Scan.h"

namespace karst_slam{namespace scanMerging{

/** Sampled point from the horizontal sonar beam */
struct pointThetaSimu
{
  pointThetaSimu() = default;

  pointThetaSimu(const mrpt::poses::CPoint3D& point_,
                 const double theta_,
                 int theta_idx_)
  {
      point     = point_;
      theta     = theta_;
      theta_idx = theta_idx_;
  }

  /** Generate points by sampling from the beam 
   * 
   * @param measure sonar measure (range, angle)
   * @param robotGlobalPose  robot pose in global frame
   * @param sensorPoseOnRobot sonar pose relative to the robot body frame
   * @param nThetaSamples number of sampled point from the beam
   * @param stepThetaSample step angle between each sample
   * @param sphericalCov covariance of spherical coordinates
   */
  static std::vector<pointThetaSimu> generatePointsOnArc(const karst_slam::sensors::sonarMeasure& measure,
                                                         const mrpt::poses::CPose3D& robotGlobalPose,
                                                         const mrpt::poses::CPose3D& sensorPoseOnRobot,
                                                         int nThetaSamples,
                                                         double beamWidth,
                                                         double stepThetaSample,
                                                         const Eigen::Matrix3d& sphericalCov)
  {
      // Output
      std::vector<pointThetaSimu> potentialPointsOnArc;
      potentialPointsOnArc.reserve(nThetaSamples);

      double theta = -0.5*beamWidth;
      karst_slam::sensors::sonarMeasure simulatedMeasure = measure;
      for(int i = 0; i < nThetaSamples; i++)
      {
          simulatedMeasure.theta = mrpt::utils::DEG2RAD(theta);
          potentialPointsOnArc.push_back(pointThetaSimu(sensors::fromSonarLocalCoordsToGlobalCoords(simulatedMeasure,
                                                                                                         robotGlobalPose,
                                                                                                         sensorPoseOnRobot,
                                                                                                         sphericalCov).mean,                                               
                                                                                                    simulatedMeasure.theta,
                                                                                                    i));
          theta += stepThetaSample;
      }

      return potentialPointsOnArc;
  }

  mrpt::poses::CPoint3D point; //!< 3D point
  double theta; //!< Corresponding theta in the beam
  int theta_idx; //!< Corresponding index
};

}} // end namespaces
#endif //DEF_SIMULATED_POINT_AT_THETA_H
