#ifndef DEF_SURFACE_VALIDATION_DATA_H
#define DEF_SURFACE_VALIDATION_DATA_H

#include "surfaceData.h"
#include "simulatedPointAtTheta.h"
#include <mrpt/utils/COutputLogger.h>
#include <vector>
#include <string>

// Forward dcl
namespace mrpt
{
    namespace poses
    {
        class CPose3D;
        class CPose3DPDFGaussian;
        class CPoint3D;
    }
}
namespace karst_slam
{
    namespace scanMerging
    {
        class ellipticCylinder;
    }
}

namespace karst_slam{namespace scanMerging{
/** This class represents the validation data for the gaussian process regression (horizontal sonar data) */
class surfaceValidationData : public surfaceData<surfaceDatum_arcEstimation>, public mrpt::utils::COutputLogger
{
public:
    /** Default constructor */
    surfaceValidationData();

    /** Default destructor */
    ~surfaceValidationData() = default;

    /**
     * Compute the local polar coordinates (in the Frenet frame) of points relative to the guide curve
     * See Yohan Breux and André Mas and Lionel Lapierre, "On-manifold Probabilistic ICP : Application to Underwater Karst Exploration", section 5.2.1
     *
     * @param priorCylinder Prior elliptic cylinder
     * @param isLearningMeanFunc flag indicating if the prior environment surface is learned (by default should be false, just for testing)
     * @param sd [out] polar coords in the local Frenet frame relative to the guide curve (generally the cylinder axis)
     *
     * @todo Should have a class for curves acting as "guide" (intermediate for learning the surface)
     * This function should then be moved there
     */
    static void polarCoordsProjOnGuide(const ellipticCylinder& priorCylinder, 
                                       bool isLearningMeanFunc,  
                                       surfaceDatum& sd);

    /**
     * Generate data for elevation angle estimation from raw horizontal sonar data 
     * (first version as in  Breux, Yohan, and Lionel Lapierre. 
     *  "Elevation Angle Estimations of Wide-Beam Acoustic Sonar Measurements 
     *  for Autonomous Underwater Karst Exploration." Sensors 20.14 (2020): 4028.)
     * 
     * @param robotTraj Vector of global robot poses
     * @param horizontalSonarPoints_theta Vector containing , for each measure, the sampled data for different values of theta (beam pitch)
     * @param verticalSonarPose Vertical sonar pose in the robot frame
     * @param priorCylinder Prior cylinder
     * @return SurfaceValidationData
     */
    static surfaceValidationData generateFromHorizontalSonar(const std::vector<trajectoryPose<mrpt::poses::CPose3DPDFGaussian>>& robotTraj,
                                                             const std::vector<std::vector<pointThetaSimu>>& horizontalSonarPoints_theta,
                                                             const mrpt::poses::CPose3D &verticalSonarPose,
                                                             const ellipticCylinder &priorCylinder);

    /**
     * Generate data for elevation angle estimation from raw horizontal sonar data 
     * (second version with reference guide as in 
     *  Yohan Breux and André Mas and Lionel Lapierre, "On-manifold Probabilistic ICP : Application to Underwater Karst Exploration")
     * 
     * @param horizontalSonarPoints_theta Vector containing , for each measure, the sampled data for different values of theta (beam pitch)
     * @param priorCylinder Prior cylinder
     * @param isLearningMeanFunc flag indicating if the prior environment surface is learned (by default should be false, just for testing)
     * @param min_abscissa minimum curvilinear abscissa of the robot trajectory poses
     * @param max_abscissa maximum curvilinear abscissa of the robot trajectory poses 
     * @return SurfaceValidationData
     */
    static surfaceValidationData generateFromHorizontalSonar_guide(const std::vector<std::vector<pointThetaSimu> > &horizontalSonarPoints_theta,
                                                                   const ellipticCylinder& priorCylinder,
                                                                   bool isLearningMeanFunc,
                                                                   double min_abscissa, double max_abscissa);

    /**
     * Compute the prior range for a measure (r = f(pose,yaw))
     * 
     * @param priorCylinder Prior cylinder
     * @param pose Pose of the sensor for the measure
     * @param yaw Sensor yaw angle for the measure
     * @return Prior range
     */
    static double priorRange(const ellipticCylinder& priorCylinder,
                             const mrpt::poses::CPose3D& pose,
                             double yaw);

    /**
     * Save to file
     * 
     * @param fileName File
     */
    void save(const std::string& fileName) const override;

    /**
     * Load from file
     * 
     * @param fileName File
     */
    void load(const std::string& fileName) override;

    inline static mrpt::math::TPoint3D crossProduct(const mrpt::math::TPoint3D& a,
                                                    const mrpt::math::TPoint3D& b)
    {
        return mrpt::math::TPoint3D( a.y*b.z - a.z*b.y,
                                     a.z*b.x- a.x*b.z,
                                     a.x*b.y - a.y*b.x);
    }

protected:
    /**
     * Find the interval(s) for which a 3D point (pointSonar) belongs to the vertical sonar plane.
     * 
     * @param bodyPoses Vector of global robot poses
     * @param pointSonar 3D point (a sample from the horizontal sonar arc)
     * @param verticalSonarPoseOnRobot Vertical sonar pose in the robot frame
     * @return Vector of index corresponding to the interval start
     */
    static std::vector<int> findInterval(const std::vector<trajectoryPose<mrpt::poses::CPose3DPDFGaussian>>& bodyPoses,
                                         const mrpt::poses::CPoint3D& pointSonar,
                                         const mrpt::poses::CPose3D& verticalSonarPoseOnRobot);

    /**
     * Compute the linear interpolation variable t between two poses such that at this poses,
     * the 3D point pointSonar belongs to the vertical sonar plane.
     * 
     * @param bodyPose_1 Start pose of the interval
     * @param bodyPose_2 End pose of the interval
     * @param verticalSonarPose Vertical sonar pose in the robot frame
     * @param pointSonar 3D point (a sample from the horizontal sonar arc)
     * @return interpolation value t (between 0 and 1)
     */
    static double linearInterpolation_intersection(const mrpt::poses::CPose3DPDFGaussian& bodyPose_1,
                                                   const mrpt::poses::CPose3DPDFGaussian& bodyPose_2,
                                                   const mrpt::poses::CPose3D& verticalSonarPose,
                                                   const mrpt::poses::CPoint3D& pointSonar);

    /**
     * Linear interpolation between two poses
     * 
     * @param pose_prev Start pose (t=0)
     * @param pose_next End pose (t=1)
     * @param t Interpolation value
     * @param interp Interpolated pose
     * @return False in invalid t (not between 0 and 1)
     */
    static bool poseLinearInterpolation(const mrpt::poses::CPose3D& pose_prev,
                                        const mrpt::poses::CPose3D& pose_next,
                                        const double t,
                                        mrpt::poses::CPose3D& interp);

    /**
     * Compute the curvilinear abscissa corresponding to an interpolated pose
     * 
     * @param curvilinearAbscissa_prev Curvilinear abscissa of the start pose
     * @param bodyPose_prev Start pose
     * @param bodyPose_next End pose
     * @param pointSonar 3D point (a sample from the horizontal sonar arc)
     * @param t Interpolation value
     * @param verticalSonarPose Vertical sonar pose in the robot frame
     * @param interp_pose Interpolated pose
     * @param s [out] curvilinear abscissa for the interpolated pose
     * @param yaw [out] vertical sonar yaw angle for the interpolated pose
     * @param r [out] range for the interpolated pose
     * @return True if success, false otherwise
     */
    static bool intersectionCurvilinearAbscissa(const double curvilinearAbscissa_prev,
                                                const mrpt::poses::CPose3DPDFGaussian& bodyPose_prev,
                                                const mrpt::poses::CPose3DPDFGaussian& bodyPose_next,
                                                const mrpt::poses::CPoint3D& pointSonar,
                                                const double t,
                                                const mrpt::poses::CPose3D& verticalSonarPose,
                                                mrpt::poses::CPose3D& interp_pose,
                                                double& s,
                                                double& yaw,
                                                double& r);
};

}} // end namespaces
#endif //DEF_SURFACE_VALIDATION_DATA_H
