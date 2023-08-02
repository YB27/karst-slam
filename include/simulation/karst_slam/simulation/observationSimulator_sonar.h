#ifndef DEF_OBSERVATION_SIMULATOR_SONAR_H
#define DEF_OBSERVATION_SIMULATOR_SONAR_H

#include "karst_slam/scanMerging/simulatedPointAtTheta.h"
#include "observationSimulator.h"
#include <random>

namespace mrpt{namespace utils{class CConfigFile;}}

namespace karst_slam{namespace simulation{

/** Methods for simulating the horizontal sonar (how to sample on elevation angle (pitch)) */
enum BEAM_POINT_GENERATION{ONE_RANDOM_SAMPLE, //!< Take only one random measure inside the beam
                           FIXED_SAMPLES, //!< Take a fixed number of measures inside the beam
                           SAMPLE_AT_CENTER //!< For test or debug. Only consider theta=0 (center of the beam)
                          };

/** Structure containing the principal parameters defining a sonar */
struct sonarCharacteristics
{
    sonarCharacteristics() = default;
    sonarCharacteristics(const Eigen::Matrix3d& sphericalLocalCovariance_,
                         const double maxRange_,
                         const double beamWidth_,
                         const bool ignoreBeamWidth_,
                         int nBeamRaySamples_,
                         bool filterRanges_,
                         double filterRanges_minDelta_,
                         int nThetaSamples_,
                         double quantification_range_,
                         BEAM_POINT_GENERATION sampleMethod = ONE_RANDOM_SAMPLE)
    {
        sphericalLocalCovariance = sphericalLocalCovariance_;
        maxRange           = maxRange_;
        beamWidth          = beamWidth_;
        beamWidth_rad      = mrpt::utils::DEG2RAD(beamWidth);
        ignoreBeamWidth    = ignoreBeamWidth_;
        beamSamplingMethod = sampleMethod;
        nThetaSamples      = nThetaSamples_;
        stepThetaSample    = beamWidth/(double)nThetaSamples;
        stepThetaSample_rad = beamWidth_rad/(double)nThetaSamples;
        nBeamRaySamples = nBeamRaySamples_;
        filterRanges = filterRanges_;
        filterRanges_minDelta = filterRanges_minDelta_;
        quantification_range = quantification_range_;
    }

    Eigen::Matrix3d sphericalLocalCovariance; //!< Spherical covariance matrix
    double maxRange; //!< Maximum range
    double beamWidth; //!< Beam width in degree
    double beamWidth_rad; //!< Beam width in radian
    bool ignoreBeamWidth; //!< If true, the beam width is considered null (ie no pitch)

    // Only used if ignoreBeamWidth is false
    BEAM_POINT_GENERATION beamSamplingMethod = ONE_RANDOM_SAMPLE; //!< Sampling method
    int nThetaSamples; //!< Number of theta (pitch) sampling for estimation elevation angle
    double stepThetaSample; //!< Step for theta sampling in deg
    double stepThetaSample_rad; //!< Step for theta sampling in radian
    int nBeamRaySamples; //!< Number of rays used to simulate the beam measurements (ie vector of ranges)
    bool filterRanges; //!< If true, filter the ranges to only keep ranges with enough difference between each other. It approximates the filtering on the vector of intensity in case of a real sonar.
    double filterRanges_minDelta; //!< Used to filter ranges. Minimum difference between two ranges.
    double quantification_range; //!< Corresponds to the sonar accuracy (range covered by an intensity bin). If -1, no quantification.
};


/**
 * @brief Handy struct to represent a ray.
 */
struct ray
{
    double theta = 0.; ///< Corresponding pitch
    Eigen::Vector3f source; ///< Point from which the ray is emmited
    Eigen::Vector3f dir;///< Ray direction
};

/**
 * @brief This class simulates observations made by a sonar
 */
class observationSimulator_sonar : public observationSimulator
{
public:
    /**
     * @brief Constructor
     * @param embreeIntersector Pointer on the ray tracing engine
     * @param name Name of the sonar
     */
    observationSimulator_sonar(const std::shared_ptr<igl::embree::EmbreeIntersector>& embreeIntersector,
                               const std::string& name);

    /**
     * @brief Constructor
     * @param embreeIntersector Pointer on the ray tracing engine
     * @param name Name of the sonar
     * @param cfg Configuration file containing the sonar parameters
     */
    observationSimulator_sonar(const std::shared_ptr<igl::embree::EmbreeIntersector>& embreeIntersector,
                               const std::string& name,
                               const mrpt::utils::CConfigFile& cfg);

    /**
     * @brief Simulate an observation
     * @param robotGlobalPose Robot pose
     * @param sonar_angular_pose sonar yaw angle (expressed as in a 6D pose)
     * @return Vector of sonar measures.
     */
    std::vector<sensors::sonarMeasure> simulateObservation(const mrpt::poses::CPose3DPDFGaussian& robotGlobalPose,
                                                           const mrpt::poses::CPose3DPDFGaussian& sonar_angular_pose);

    /**
     * @brief Filter raw ranges to only keep well-separated ranges
     * @param rawMeasures
     * @return filtered measures
     */
    std::vector<sensors::sonarMeasure> filterRanges(const std::vector<sensors::sonarMeasure> &rawMeasures) const;

    std::vector<sensors::sonarMeasure> getFullSensorPlaneRawMeasures(const mrpt::poses::CPose3D& robotGlobalPose,
                                                                     int nPts = 90) const;

    std::vector<mrpt::poses::CPointPDFGaussian> getFullSensorPlaneGlobalMeasures(const mrpt::poses::CPose3D& robotGlobalPose,
                                                                                 int nPts = 90) const;
    
    std::vector<mrpt::poses::CPointPDFGaussian> getFullSensorPlaneLocalMeasures(const mrpt::poses::CPose3D& robotGlobalPose,
                                                                                int nPts = 90) const;                                                                                             

    mrpt::math::TPoint3D sectionBarycenter(const mrpt::poses::CPose3D& robotGlobalPose) const; 

    double rangeQuantification(const double range)const;

    /**
     * @brief Load the sonar parameters from a configuration file
     * @param cfg Configuration file
     */
    void loadParamFromConfigFile(const mrpt::utils::CConfigFile& cfg);

    /**
     * @brief Get the sampling index corresponding to a theta
     * @param theta Angle (rad)
     * @return Index
     */
    int getThetaIdx(double theta)const;

    /**
     * @brief Set the sonar characteristics
     * @param sc Sonar characteristics structure
     */
    inline void setSonarCharacteristics(const sonarCharacteristics& sc)
    {m_characteristics = sc;}

    inline int get_nThetaSamples()const {return m_characteristics.nThetaSamples;}
    inline double getBeamWidth() const {return m_characteristics.beamWidth;}
    inline double getStepThetaSample() const {return m_characteristics.stepThetaSample;}
    inline const Eigen::Matrix3d& getSphericalLocalCov() const 
    {return m_characteristics.sphericalLocalCovariance;}

protected:
    /**
    * @brief Get a set of rays to simulate sonar measurement (depends on the method, see BEAM_POINT_GENERATION enum)
    * The ray is considered going from the x axis.
    * @param robotGlobalPose Robot pose
    * @param sonar_angular_pose sonar angle
    * @return Vector of rays
    */
   std::vector<ray> getRays(const mrpt::poses::CPose3DPDFGaussian& robotGlobalPose,
                            const mrpt::poses::CPose3DPDFGaussian& sonar_angular_pose);

   /**
    * @brief Get the direction of the ray
    * @param pose
    * @param yaw
    * @return Direction
    */
   Eigen::Vector3f getRayDir(const mrpt::poses::CPose3D& pose,
                             const double yaw) const;

   /**
    * @brief Get a ray when the beam width is ignored
    * @param source Point from which the ray is emitted
    * @param beamGlobalPose Beam pose
    * @param yaw Yaw angle (rad)
    * @return Ray
    */
   ray getRay_noBeamWidth(const Eigen::Vector3f& source,
                          const mrpt::poses::CPose3DPDFGaussian& beamGlobalPose,
                          const double yaw) const;

   /**
    * @brief Get a ray with a given pitch inside its beam
    * @param source Point from which the ray is emitted
    * @param beamGlobalPose Beam pose
    * @param pitch Pitch angle (rad)
    * @param yaw Yaw angle (rad)
    * @return Ray
    */
   ray getRay_pitch(const Eigen::Vector3f& source,
                    const mrpt::poses::CPose3DPDFGaussian& beamGlobalPose,
                    const double pitch,
                    const double yaw);

   /**
    * @brief Get one random ray inside a beam
    * @param source Point from which the ray is emitted
    * @param beamGlobalPose Beam pose
    * @param yaw Yaw angle (rad)
    * @return Ray
    */
   ray getRay_oneRandomSample(const Eigen::Vector3f& source,
                              const mrpt::poses::CPose3DPDFGaussian& beamGlobalPose,
                              const double yaw);

   sonarCharacteristics m_characteristics;///< Sonar parameters
   std::string m_name; ///< Sonar name
   std::random_device m_rd; ///< RNG
   std::mt19937 m_gen; ///< RNG
   std::uniform_int_distribution<> m_dis_theta; ///< Uniform distribution for theta sampling
};

}} // end namespaces

#endif // DEF_OBSERVATION_SIMULATOR_SONAR_H
