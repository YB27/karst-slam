#include "karst_slam/sensors/Scan.h"

mrpt::poses::CPointPDFGaussian karst_slam::sensors::fromSonarCoordsToCartesian(const karst_slam::sensors::sonarMeasure& measure,
                                                                               const Eigen::Matrix3d& sphericalCov)
{
    double range = measure.range, pitch = measure.theta, yaw = measure.phi;
    double cos_y = cos(yaw), cos_p = cos(pitch),
           sin_y = sin(yaw), sin_p = sin(pitch);
    CartesianCov_lut cc(sphericalCov, cos_y, sin_y);
    Eigen::Matrix3d cart_cov = cc.getCovariance(range);
    return mrpt::poses::CPointPDFGaussian(mrpt::poses::CPoint3D(range*cos_p*cos_y,range*cos_p*sin_y,range*sin_p), cart_cov);
}


mrpt::poses::CPointPDFGaussian karst_slam::sensors::fromSonarLocalCoordsToGlobalCoords(const karst_slam::sensors::sonarMeasure& measure,
                                                                                            const mrpt::poses::CPose3D& robotGlobalPose,
                                                                                            const mrpt::poses::CPose3D& sensorPoseOnRobot,
                                                                                            const Eigen::Matrix3d& sphericalCov)
{
    mrpt::poses::CPointPDFGaussian point_globalFrame = karst_slam::sensors::fromSonarCoordsToCartesian(measure,
                                                                                                       sphericalCov); 
    point_globalFrame.changeCoordinatesReference(robotGlobalPose + sensorPoseOnRobot);

    return point_globalFrame;
}

mrpt::poses::CPointPDFGaussian karst_slam::sensors::fromSonarLocalCoordsToRobotLocalCoords(const karst_slam::sensors::sonarMeasure& measure,
                                                                                           const mrpt::poses::CPose3D& sensorPoseOnRobot,
                                                                                           const Eigen::Matrix3d& sphericalCov)
{
    mrpt::poses::CPointPDFGaussian point_robotLocalFrame = karst_slam::sensors::fromSonarCoordsToCartesian(measure,
                                                                                                           sphericalCov); 
    point_robotLocalFrame.changeCoordinatesReference(sensorPoseOnRobot);

    return point_robotLocalFrame;
}
