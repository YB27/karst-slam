#ifndef DEF_ELLIPTIC_CYLINDER_H
#define DEF_ELLIPTIC_CYLINDER_H

#include <opencv/cv.h>
#include <mrpt/utils/COutputLogger.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPointPDFGaussian.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>

namespace karst_slam{namespace scanMerging{

/** Auxiliary data structure used for the LM non linear least square fitting */
struct auxData_fitNLLS
{
  auxData_fitNLLS(const std::vector<mrpt::poses::CPointPDFGaussian> points_,
                  const mrpt::poses::CPose3D& verticalSonarPoseOnRobot,
                  const std::vector<mrpt::poses::CPose3DPDFGaussian> bodyPoses,
                  const mrpt::poses::CPose3D& curEstimatedPose_)
  {
      points = points_;
      curEstimatedPose = curEstimatedPose_;
      sonarPoses.reserve(bodyPoses.size());
      for(const mrpt::poses::CPose3DPDFGaussian& bodyPose : bodyPoses)
         sonarPoses.push_back(bodyPose.mean + verticalSonarPoseOnRobot);
  }

  auxData_fitNLLS(const std::vector<mrpt::poses::CPointPDFGaussian> points_,
                  const mrpt::poses::CPose3D& verticalSonarPoseOnRobot,
                  const std::vector<mrpt::poses::CPose3D> bodyPoses,
                  const mrpt::poses::CPose3D& curEstimatedPose_)
  {
      points = points_;
      curEstimatedPose = curEstimatedPose_;
      sonarPoses.reserve(bodyPoses.size());
      for(const mrpt::poses::CPose3D& bodyPose : bodyPoses)
         sonarPoses.push_back(bodyPose + verticalSonarPoseOnRobot);
  }

  std::vector<mrpt::poses::CPointPDFGaussian> points; ///< Vector of 3D points
  std::vector<mrpt::poses::CPose3D> sonarPoses; ///< Vector of 3D poses of the horizontal sonar sonar
  mrpt::poses::CPose3D curEstimatedPose; ///< Estimated 6D pose at current estimation
  mrpt::poses::CPose3D tpEstimatedPose; ///< Temporary stored possible next estimated 6D pose
  bool isImprovement = true; ///< True is improvement in the current iteration
};

/**
 * This class represent an elliptic cylinder. It is mainly used to fit 3D points to an finite or infinite elliptic cylinder
 * \todo : Refactor the code, better separating finite and infinite cylinder part.
 */
class ellipticCylinder : public mrpt::utils::COutputLogger
{
public :
    /** Default constructor */
    ellipticCylinder() = default;

    /**
     * Constructor
     * 
     * @param pose 6D Pose of the cylinder (center)
     * @param halfLongAxis Ellipse major axis half-length
     * @param halfSmallAxis Ellipse minor axis half-length
     * @param length Length of the cylinder (along the cylinder axis)
     */
    ellipticCylinder(const mrpt::poses::CPose3D& pose,
                     double halfLongAxis,
                     double halfSmallAxis,
                     double length);

    /** Default destructor */
    ~ellipticCylinder() = default;


    /** Method taken from https://gist.github.com/JohannesMP/777bdc8e84df6ddfeaa4f0ddb1c7adb3
     *   which is a fast method to compute distance to an ellipse without non linear solver and without trigonometric operator
     *   Originally described by https://www.spaceroots.org/documents/distance/distance-to-ellipse.pdf
     * 
     * @param point_x x-coord of 2D point expressed in the ellipse frame
     * @param point_y y-coord of 2D point expressed in the ellipse frame
     * @param a Half-length of ellipse major axis
     * @param b Half-length of ellipse minor axis
     * @param nIterations Number of iterations
     * @return Distance from the 2D point to the ellipse
     */
    static double distanceToEllipse(double point_x, double point_y,
                                    double a, double b, int nIterations = 5);

    static mrpt::poses::CPose3D errorFunctionForLM_cylinderPose(const mrpt::math::CVectorDouble& cylinderVals,
                                                                const auxData_fitNLLS& auxData);

    /**
     * Error function used by the LM least square solver to fit a finite elliptic cylinder.
     * It is the RMS of orthogonal distances from a point cloud to the estimated cylinder.
     * 
     * @param cylinderVals Estimated cylinders values (in order : 6D Pose (x,y,z,yaw,pitch,roll), half-major length, half-minor length, length)
     * @param auxData Auxiliary data containing the point cloud
     * @param f RMS of orthogonal distances (vector of size 1)
     */
    static void errorFunctionForLM_finite(const mrpt::math::CVectorDouble& cylinderVals,
                                          const auxData_fitNLLS& auxData,
                                          mrpt::math::CVectorDouble& f);

    /**
     * Error function used by the LM least square solver to fit an infinite elliptic cylinder.
     * It is the RMS of orthogonal distances from a point cloud to the estimated cylinder.
     * 
     * @param cylinderVals Estimated cylinders values (in order : 6D Pose (x,y,z,yaw,pitch,roll), half-major length, half-minor length)
     * @param auxData Auxiliary data containing the point cloud
     * @param f RMS of orthogonal distances (vector of size 1)
     */
    static void errorFunctionForLM_infinite(const mrpt::math::CVectorDouble& cylinderVals,
                                            const auxData_fitNLLS& auxData,
                                            mrpt::math::CVectorDouble& f);

    /**
     * Error function used by the LM least square solver to fit an infinite elliptic cylinder.
     * It is the RMS of orthogonal distances from a point cloud to the estimated cylinder.
     * Difference with errorFunctionForLM_infinite is that we add a constraint to maximize the point distance
     * to the cylinder axis
     * 
     * @param cylinderVals Estimated cylinders values (in order : 6D Pose (x,y,z,yaw,pitch,roll), half-major length, half-minor length)
     * @param auxData Auxiliary data containing the point cloud
     * @param f RMS of orthogonal distances (vector of size 1)
     */
    static void errorFunctionForLM_infinite_constrained(const mrpt::math::CVectorDouble& cylinderVals,
                                                        const auxData_fitNLLS& auxData,
                                                        mrpt::math::CVectorDouble& f);

    /**
     * Compute the orthogonal distance to a finite cylinder.
     * Three cases are considered :
     *  - The point is inside or outside the cylinder but inbetween the two bases -> computed using distance to ellipse
     *  - The point is in front of a base ->  distance to the base
     *  - The point is in between the previous case -> projection of in the base plane + distance to ellipse + pythagore
     * 
     * @param cylinderPose 6D pose of the cylinder
     * @param halfLongAxis Ellipse major axis half-length
     * @param halfSmallAxis Ellipse minor axis half-length
     * @param length Length of the cylinder (along the cylinder axis)
     * @param P Input 3D Point
     * @return Distance of the input 3D point from the cylinder
     */
    static double orthogonalDistanceToFiniteCylinder(const mrpt::poses::CPose3D& cylinderPose,
                                                     double halfLongAxis,
                                                     double halfSmallAxis,
                                                     double length,
                                                     const mrpt::poses::CPoint3D& P);

    /**
     * Compute the orthogonal distance to an infinite cylinder.
     * Simply compute the distance to the ellipse in the orthogonal plane to the cylinder axis going through the point
     * 
     * @param cylinderPose 6D pose of the cylinder
     * @param halfLongAxis Ellipse major axis half-length
     * @param halfSmallAxis Ellipse minor axis half-length
     * @param P Input 3D Point
     * @return Distance of the input 3D point from the cylinder
     */
    static double orthogonalDistanceToInfiniteCylinder(const mrpt::poses::CPose3D& cylinderPose,
                                                       double halfLongAxis,
                                                       double halfSmallAxis,
                                                       const mrpt::poses::CPoint3D& P);

    /** Compute the distance of a pose center to the cylinder axis */
    static double distanceToCylinderAxis(const mrpt::poses::CPose3D& cylinderPose,
                                         const mrpt::poses::CPose3D& sonarPose);

    /**
     * Auxiliary function used to compute the SE(3) increment in the LM least square solver in the case of finite cylinder fitting.
     * 
     * @param newVals New cylinder values (6D pose, half-length major, half-length minor, length)
     * @param oldVals Old cylinder values
     * @param incr Increment ( increment in se(3) (6D), incr_halfLength_major, incr_halfLength_minor, incr_length)
     * @param auxData Auxiliary data
     */
    static void incrementAdder_LM_finite(mrpt::math::CVectorDouble& newVals,
                                         const mrpt::math::CVectorDouble& oldVals,
                                         const mrpt::math::CVectorDouble& incr,
                                         const auxData_fitNLLS& auxData);

    /**
     *  Auxiliary function used to compute the SE(3) increment in the LM least square solver in the case of infinite cylinder fitting.
     *
     * @param newVals New cylinder values (6D pose, half-length major, half-length minor)
     * @param oldVals Old cylinder values
     * @param incr Increment ( increment in se(3) (6D), incr_halfLength_major, incr_halfLength_minor)
     * @param auxData Auxiliary data
     */
    static void incrementAdder_LM_infinite(mrpt::math::CVectorDouble& newVals,
                                           const mrpt::math::CVectorDouble& oldVals,
                                           const mrpt::math::CVectorDouble& incr,
                                           const auxData_fitNLLS& auxData);

    /**
     * Fit this elliptic cylinder to point cloud
     * 
     * @param points point cloud
     * @param verticalSonarPoseOnRobot Pose of the vertical sonar relative to the robot body frame
     * @param bodyPoses robot trajectory (vector of robot pose pdf)
     */
    void fit(const std::vector<mrpt::poses::CPointPDFGaussian>& points,
             const mrpt::poses::CPose3D& verticalSonarPoseOnRobot,
             const std::vector<mrpt::poses::CPose3DPDFGaussian>& bodyPoses);
     /**
     * Fit this elliptic cylinder to point cloud
     * 
     * @param points point cloud
     * @param verticalSonarPoseOnRobot Pose of the vertical sonar relative to the robot body frame
     * @param bodyPoses robot trajectory (vector of robot pose)
     */
    void fit(const std::vector<mrpt::poses::CPointPDFGaussian>& points,
             const mrpt::poses::CPose3D& verticalSonarPoseOnRobot,
             const std::vector<mrpt::poses::CPose3D>& bodyPoses);

    /**
     * Distance to a point projected on an orthogonal plane of the cylinder
     * 
     * @param point_x x-coord of projected point
     * @param point_y y-coord of projected point
     * @return Distance
     */
    inline double distanceToEllipseInOrthogonalPlane(double point_x, double point_y) const
    {return distanceToEllipse(point_x, point_y, m_halfLongAxis, m_halfSmallAxis);}

    /** Return true if the givein 2d point is inside the centered non-rotated ellipse of half-axis a and b */
    inline static bool isInsideEllipse(double point_x, double point_y,
                                       double a, double b)
    {
        return (point_x*point_x/(a*a)) + (point_y*point_y/(b*b)) - 1 < 0;
    }

    /**
     * Return the center of the bottom base
     * 
     * @return 3D point corresponding to the center of the bottom base
     */
    Eigen::Vector3d bottomBaseCenter() const;

    /**
     * Return the center of the bottom base (computed by the PCA)
     * 
     * @return 3D point corresponding to the center of the bottom base
     */
    Eigen::Vector3d bottomBaseCenter_pca() const;

    /** Print info on the terminal */
    void print() const;

    /**
     * Return the distance from the position P with a sonar rotation yaw to the (finite) cylinder
     * It corresponds to the prior range used for the gaussian process regression.
     * 
     * @param P 3D Point
     * @return Distance from P to the cylinder
     */
    double getOrthogonalDistanceToFiniteCylinder(const mrpt::poses::CPoint3D& P) const;

    /**
     * Return the distance from the position P with a sonar rotation yaw to the (infinite) cylinder
     * It corresponds to the prior range used for the gaussian process regression.
     * 
     * @param P 3D Point
     * @return Distance from P to the cylinder.
     */
    double getOrthogonalDistanceToInfiniteCylinder(const mrpt::poses::CPoint3D& P) const;

    /**
     * Return the distance from a point to the (infinite) cylinder along a given direction
     * 
     * @param P 3D Point
     * @param dir Direction
     * @return Distance from P to the cylinder along the given direction. Return -1 if no intersection is found.
     */
    double getDistanceToInfiniteCylinderAlongDirection(const Eigen::Vector3d& P,
                                                       const Eigen::Vector3d& dir) const;

    /**
     * Set the 6D pose of the cylinder
     * 
     * @param pose 3D pose
     */
    void setPose(const mrpt::poses::CPose3D& pose);

    /**
     * Set the principal base of the cylinder
     * 
     * @param R Rotation matrix (each column corresponds to the cylinder axis)
     */
    void setPrincipalBase(const Eigen::Matrix3d& R);

    /**
     * Compare with another cylinder
     * 
     * @param other Other cylinder
     * @param eps Tolerance
     * @return True if the cylinder are equal up to the given tolerance
     */
    bool equalTo(const ellipticCylinder& other,
                 double eps = 1e-4) const;

    /**
     * Get the cylinder length
     * 
     * @return cylinder length
     */
    inline double getLength() const {return m_length;}

    /**
     * Get the cylinder length (computed by the pca)
     * 
     * @return cylinder length
     */
    inline double getLength_pca() const {return m_length_pca;}

    /**
     * Get the ellipse major axis half length
     * 
     * @return ellipse major axis half length
     */
    inline double getHalfLongAxis() const {return m_halfLongAxis;}

    /**
     * Get the ellipse major axis half length (computed by the pca)
     * 
     * @return ellipse major axis half length
     */
    inline double getHalfLongAxis_pca() const {return m_halfLongAxis_pca;}

    /**
     * Get the ellipse minor axis half length
     * 
     * @return ellipse minor axis half length
     */
    inline double getHalfSmallAxis() const {return m_halfSmallAxis;}

    /**
     * Get the ellipse minor axis half length (computed by the pca)
     * 
     * @return ellipse minor axis half length
     */
    inline double getHalfSmallAxis_pca() const {return m_halfSmallAxis_pca;}

    /**
     * Get the cylinder pose
     * 
     * @return cylinder pose
     */
    inline const mrpt::poses::CPose3D& getPose() const {return m_pose;}

    /**
     * Get the cylinder pose (computed by the pca)
     * 
     * @return cylinder pose
     */
    inline const mrpt::poses::CPose3D& getPose_pca() const {return m_pose_pca;}

    /**
     * Get the principal base
     * 
     * @return principal base
     */
    inline const Eigen::Matrix3d& getPrincipalBase() const {return m_principalBase;}

    inline Eigen::Vector3d getCylinderAxis() const{return m_principalBase.col(0);}

    /**
     * Get the principal base (computed by the pca)
     * 
     * @return principal base
     */
    inline const Eigen::Matrix3d& getPrincipalBase_pca() const {return m_principalBase_pca;}


    /**
     * Get the cylinder center
     * 
     * @return cylinder center
     */
    inline const Eigen::Vector3d& getCenter() const {return m_pose.m_coords;}

    /**
     * Get the cylinder center (computed by the pca)
     * 
     * @return cylinder center
     */
    inline const Eigen::Vector3d& getCenter_pca() const {return m_pose_pca.m_coords;}

    /**
     * Save the elliptic cylinder parameters
     * 
     * @param fileName
     */
    void save(const std::string& fileName) const;

    /**
     * Load the elliptic cylinder from file
     * 
     * @param fileName
     */
    void load(const std::string& fileName);


protected:
    /**
     * Non linear least square (NLLS) fitting for an infinite elliptic cylinder
     * 
     * @param auxData Input Data  
     */
    void fitNLLS_infinite(const auxData_fitNLLS& auxData);

    /**
     * Non linear least square (NLLS) fitting for a finite elliptic cylinder
     * 
     * @param auxData Input Data  
     */
    void fitNLLS_finite(const auxData_fitNLLS& auxData);

    /**
     * Compute the PCA of a point cloud and define the cylinder with its eigenvectors.
     * Used as an initial point for the non linear fitting.
     * 
     * @param points Input point cloud
     */
    void fitPCA(const std::vector<mrpt::poses::CPointPDFGaussian>& points);

    /**
     * Raw robust PCA on the 3D points using the algorithm proposed in the paper
     * "Scalable robust principal component analysis using grassmann averages, Hauberg Soren et al, IEEE transactions on pattern analysis and machine intelligence, 2015"
     * For the implementation, see https://github.com/MPI-IS/Grassmann-Averages-PCA
     * 
     * @param points Input points
     * @todo make a wrapper of Grassman-Averages-PCA !
     */
    void fitPCA_raw_grassmanAverages(const std::vector<mrpt::poses::CPointPDFGaussian>& points,
                                     Eigen::Matrix3Xd& centered_pts);

    /**
     * Project points on an orthogonal plane of the cylinder
     * 
     * @param centered_pts Centered points
     * @return Vector of projected points
     */
    std::vector<cv::Point2f> projectOnOrthogonalPlane(const Eigen::Matrix3Xd& centered_pts);

    /**
     * Find the principal axis of the cylinder from the PCA eigenvectors.
     * If the cylinder length is greater than the ellipse axis length, the first eigenvector corresponds to cylinder main axis.
     * However, it is not always the case. To determine the main axis, we compute the projections of tangent vectors (vector between consecutive points)
     * on each eigenvector. The one with the smallest projections is the main axis (tangent vector are theoreticaly orthogonal to the main axis)
     * 
     * @param dotProds Mean dot products of tangent vectors with each eigenvectors.
     */
    void setAxisFromTangentVectorsProjections(const Eigen::Vector3d& dotProds);

private:  
    double m_halfLongAxis; ///< Ellipse major axis half length
    double m_halfSmallAxis; ///< Ellipse minor axis half length
    double m_length; ///< Cylinder length
    Eigen::Matrix3d m_principalBase; ///< Principal base (axis) of the cylinder
    Eigen::Matrix3d m_principalBase_t; ///< Transpose of the principal base
    mrpt::poses::CPose3D m_pose; ///< 6D Pose of the cylinder

    double m_halfLongAxis_pca; ///< Ellipse major axis half length (pca)
    double m_halfSmallAxis_pca; ///< Ellipse minor axis half length (pca)
    double m_length_pca; ///< Cylinder length (pca)
    Eigen::Matrix3d m_principalBase_pca; ///< Principal base (axis) of the cylinder (pca)
    Eigen::Matrix3d m_principalBase_t_pca; ///< Transpose of the principal base (pca)
    mrpt::poses::CPose3D m_pose_pca; ///< 6D Pose of the cylinder (pca)
};

}} // end namespaces
#endif // DEF_ELLIPTIC_CYLINDER_H
