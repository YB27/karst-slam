#ifndef DEF_SURFACE_GP_H
#define DEF_SURFACE_GP_H

#include "surfaceTrainingData.h"
#include "surfaceValidationData.h"
#include <mrpt/utils/COutputLogger.h>
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <functional>

// Fwrd dcl
namespace mrpt
{
    namespace poses{class CPoint3D;}
    namespace utils{class CConfigFile;}
}

namespace karst_slam {namespace scanMerging
{
    class dataForScanMerging;
    class scanMergingResults;
}}

// Currently use GPy library in a separeted python script for the GP regression
// ToDo : Use instead the C++ Limbo library (https://github.com/resibots/limbo) !
namespace karst_slam{namespace scanMerging{

/** Handy structure defining a scaled beta distribution */
struct distributionTheta
{
    distributionTheta(int theta_idx_,
                      double alpha_,
                      double beta_)
    {
        theta_idx = theta_idx_;
        alpha = alpha_;
        beta = beta_;
    }

    int theta_idx; //!< Corresponding index of this distribution (one arc of measurement can have several distribution attached to it)
    double alpha; //!< Alpha parameter of the beta distribution
    double beta; //!< Beta parameter of the beta distribution
};

/** Parameters of the kernels used in the gaussian process regression */
struct kernelParameters
{
    std::string type_s; //!< Type of kernel for curvilinear abscissa
    std::string type_yaw; //!< Type of kernel for yaw angle
    double lengthscale_s; //!< Lengthscale relative to curvilinear abscissa
    double lengthscale_yaw; //!< Lengthscale relative to angle in the sonar plan
    double variance; //!< Variance of the kernel
};

/**
 * Handy structure containing results of the gaussian process regression, mainly used for display
 * The python script generates the surface as three point of clouds :
 * One for the mean and two for the lower and higher bound (+- 3 sigmas)
 * This is only for display purpose as the computation are made in the python script
 */
struct surfaceGP_data
{   
    /** Reserve memory for all vectors */
    void reserve(int n)
    {
        meanPoints.reserve(n);
        lowBoundPoints.reserve(n);
        highBoundPoints.reserve(n);
        normals.reserve(n);
        pointsForNormals_debug.reserve(n);
    }

    /** Clear all vectors */
    void clear()
    {
        meanPoints.clear();
        lowBoundPoints.clear();
        highBoundPoints.clear();
        normals.clear();
        pointsForNormals_debug.clear();
    }

    // The python script generates the surface as three point of clouds :
    // One for the mean and two for the lower and higher bound (+- 3 sigmas)
    // This is only for display purpose as the computation are made in the python script
    std::vector<mrpt::poses::CPoint3D> meanPoints; //!< Mean of the gaussian process estimated surface
    std::vector<mrpt::poses::CPoint3D> lowBoundPoints; //!< Lower bound of the gaussian process estimated surface (mean_range - 3*sigma)
    std::vector<mrpt::poses::CPoint3D> highBoundPoints; //!< Higher bound of the gaussian process estimated surface (mean_range + 3*sigma)
    std::vector<mrpt::poses::CPointPDFGaussian> normals; //!< Estimated normals at horizontal sonar data
    std::vector<mrpt::math::TPoint3D> pointsForNormals_debug;//!< Vector of points used to compute normal data (4 points for one normal)
};

/**
 * This class is used to estimate the karst surface via gaussian process regression
 * Currently, it saves/loads data for the python script executing the gaussian process (GPy lib).
 * @todo Use the c++ LIMBo library instead of python. This will also require to implement the beta distribution fitting here
 * (simple copy/paste from the python script)
 */
class surfaceGP : public mrpt::utils::COutputLogger
{
public:
    /**
     * Constructor
     * 
     * @param cfg Configuration file
     */
    explicit surfaceGP(const mrpt::utils::CConfigFile& cfg);

    /** Default destructor */
    ~surfaceGP() = default;

    /**
     * Estimate the theta distribution along the horizontal sonar measurement arcs.
     * 
     * @param priorCylinder Prior cylinder previously estimated with the training data
     * @param curvilinearAbscissaPoseMap Map of poses
     * @param sonarPoseOnRobot Local vertical sonar pose in the robot frame
     * @param distribution Map of vector of distributions. One vector corresponds to one arc.
     * 
     * @return scan merging results (ie point pdf of the horizontal sonar data)
     */
    scanMergingResults estimateDistributions(const std::shared_ptr<dataForScanMerging>& data);

    scanMergingResults estimateDistributions(const std::map<uint64_t, trajectoryPose<mrpt::poses::CPose3DPDFGaussian>>& bodyPoses,
                                             const mrpt::poses::CPose3D& verticalSonarPoseOnRobot,
                                             const ellipticCylinder& priorCylinder);

    /**
     * Load the result file containing the gaussian process regression.
     * Generated by the python script.
     * Note that we substract the prior to all the data to use a simple gaussian process regression without prior in the python script.
     * 
     * @param fileName File
     * @param curvilinearAbscissaPoseMap Map of poses
     * @param sonarPoseOnRobot Local vertical sonar pose in the robot frame
     * @param priorCylinder Prior cylinder previously estimated with the training data
     */
    void loadSurfaceGP(const std::string& fileName,
                       const std::map<uint64_t, trajectoryPose<mrpt::poses::CPose3DPDFGaussian>> &bodyPoses,
                       const mrpt::poses::CPose3D &sonarPoseOnRobot,
                       const ellipticCylinder &priorCylinder);

    /**
     * Load the result file containing the theta distributions
     *
     * @param fileName File
     * @param distribution Map of vector of distributions. One vector corresponds to one arc.
     */
    void loadDistribution(const std::string& fileName,
                          std::map<int, std::vector<distributionTheta> > &distribution);

    /**
     * Load the file containing the normals pdf
     * 
     * @param fileName name of the file containing the normals pdf
     * @param fileName_debug name of the file containing additionnal data for debug (notably the four points used)
     * @param verticalSonarPose Pose of the vertical sonar relative to the robot body frame
     * @param normals [out] loaded normals pdf
     */
    void loadNormals(const std::string& fileName,
                     const std::string& fileName_debug,
                     const mrpt::poses::CPose3D &verticalSonarPose,
                     std::vector<mrpt::poses::CPointPDFGaussian>& normals);

    /**
     * Function only used to produce data for figure in the paper 
     * Breux, Yohan, and Lionel Lapierre. "Elevation Angle Estimations of Wide-Beam Acoustic Sonar Measurements for Autonomous Underwater Karst Exploration." Sensors 20.14 (2020): 4028.
     *
     * Add the prior data to the estimated data from generateSampledSurfaceSlices() in the python script sonar_gp.py
     * In practice, it is use to compute slices of the estimated environment surface (see figures ) 
     *
     * @param curvilinearAbscissaPoseMap map with curvilinear abscissa as keys and corresponding 3D pose as values
     * @param sonarPoseOnRobot Pose of the sonar relative to the robot body frames
     * @param priorCylinder Prior estimated surface of the environment (as an elliptic cylinder)
     */
    void addPriorToSlicesData(const std::map<int, mrpt::poses::CPose3D>& curvilinearAbscissaPoseMap,
                              const mrpt::poses::CPose3D &sonarPoseOnRobot,
                              const ellipticCylinder& priorCylinder);

     /**
     * Function only used to produce data for figure in the paper 
     * Breux, Yohan, and Lionel Lapierre. "Elevation Angle Estimations of Wide-Beam Acoustic Sonar Measurements for Autonomous Underwater Karst Exploration." Sensors 20.14 (2020): 4028.
     *
     * Add the prior data to the estimated data from generateSampledSurfaceSlices() in the python script sonar_gp.py
     * In practice, it is use to compute slices of the estimated environment surface (see figures ) 
     *
     * @param file 
     * @param sonarPoseOnRobot Pose of the sonar relative to the robot body frames
     * @param priorCylinder Prior estimated surface of the environment (as an elliptic cylinder)
     */
    void addPriorToSliceFile(const std::string& file,
                             const mrpt::poses::CPose3D &sonarPoseOnRobot,
                             const ellipticCylinder& priorCylinder);

    /**
     * Get the surface data
     * 
     * @return surface data
     */
    inline const surfaceGP_data& getSurfaceData()const {return m_surfaceData;}

    /**
     * @overload
     */
    inline surfaceGP_data getSurfaceData() {return m_surfaceData;}

    inline std::string getDataFolder() const {return m_dataFolder;}
    inline void setDataFolder(const std::string& dataFolder){m_dataFolder = dataFolder;}

    /** Set a flag to indicate if the prior is learned (eg with a MLP) 
     *  It is just for test so by default the flag should be false
     */
    inline void setIsLearningMeanFunc(bool is){m_isLearningMeanFunc = is;}

private:
    surfaceGP_data m_surfaceData; //!< Surface data (mean and lower/higher bounds)
    std::string m_pythonScriptFile; //!< Python script file for gaussian process regression and theta distributions
    kernelParameters m_kernelParams; //!< Parameters for kernels used by the gaussian process
    double m_processNoise; //!< Model is r = f(s, yaw) + epsilon where m_processNoise is the variance of epsilon
    std::string m_rangeMapFunction; //!< Inverse of a function mapping the range values from R^+ to R
    std::string m_dataFolder; //!< Data files folder
    int m_batchSize; //!< Batch size for gaussian process estimations
    bool m_loadSurfaceData; //!< If false, do not load the estimated surface points from file (to speed up)
    bool m_isLearningMeanFunc; //!< flag indicating if the prior environment surface is learned (by default should be false, just for testing)
    bool m_useOffRangePts; //!< If true, exploit also the sonar point which exceeds the maximum range
};
}} // end namespaces

#endif // DEF_SURFACE_GP_H
