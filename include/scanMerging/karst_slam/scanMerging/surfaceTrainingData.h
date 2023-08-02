#ifndef DEF_SURFACE_TRAINING_DATA_H
#define DEF_SURFACE_TRAINING_DATA_H

#include "surfaceData.h"
#include "surfaceDatum.h"
#include "surfaceValidationData.h"
#include <mrpt/utils/COutputLogger.h>

namespace karst_slam{namespace scanMerging{

/** This class represents data used for training the gaussian process regression of the surface */
class surfaceTrainingData : public surfaceData<surfaceDatum>, public mrpt::utils::COutputLogger
{
public:

    surfaceTrainingData();

    /**
     * Preprocess the gaussian process training data by removing the prior
     * 
     * @param priorCylinder Prior elliptic cylinder
     * @param curvilinearAbscissaPoseMap Pose at each index of the trajectory
     * @param verticalSonarPoseOnRobot Vertical sonar pose expressed in the robot frame
     */
    void computeTrainingDataWithPrior(const ellipticCylinder& priorCylinder,
                                      const std::map<unsigned long long, trajectoryPose<mrpt::poses::CPose3DPDFGaussian>>& poses,//const std::map<int, mrpt::poses::CPose3D>& curvilinearAbscissaPoseMap,
                                      const mrpt::poses::CPose3D& verticalSonarPoseOnRobot); 

    /**
     * Save to file
     * 
     * @param fileName File
     */
    void save(const std::string& fileName) const override;
    void saveAbscissa(const std::string& fileName_abs) const;

    /**
     * Load from file
     * 
     * @param fileName File
     */
    void load(const std::string& fileName) override;
    
    void append(const surfaceTrainingData& otherData);

    /**
     * Compute the curvilinear abscissa s 
     * 
     * @param priorCylinder  Prior estimated surface of the environment (as an elliptic cylinder)
     * @param min_abscissa [out] minimum curvilinear abscissa 
     * @param max_abscissa [out] maximum curvilinear abscissa 
     */
    void computeAbscissa(const ellipticCylinder& priorCylinder,
                         double& min_abscissa,
                         double& max_abscissa);

    inline void setIsLearningMeanFunc(bool is){m_isLearningMeanFunc = is;}

protected:
    bool m_isLearningMeanFunc; //!< flag indicating if the prior environment surface is learned (by default should be false, just for testing)
};
}}// end namespaces

#endif // DEF_SURFACE_TRAINING_DATA_H
