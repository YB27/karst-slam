#ifndef DEF_OBSERVATION_SIMULATOR_H
#define DEF_OBSERVATION_SIMULATOR_H

#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/utils/COutputLogger.h>
#include <igl/embree/EmbreeIntersector.h>
#include <memory>

namespace mrpt
{
    namespace poses
    {
        class CPose3D;
        class CPointPDFGaussian;
    }
}

namespace karst_slam{namespace simulation{

/** This is the base class for simulating sensor observations. */
class observationSimulator : public mrpt::utils::COutputLogger
{
public:
    /**
     * Constructor
     * 
     * @param embreeIntersector Used for raytracing
     */
    explicit observationSimulator(const std::shared_ptr<igl::embree::EmbreeIntersector>& embreeIntersector) :
        m_embreeIntersector(embreeIntersector){}

    /** Default dtor */
    virtual ~observationSimulator() = default;

    /**
     * Get the local sensor pose in the robot frame
     * 
     * @return Local sensor pose
     */
    inline mrpt::poses::CPose3DPDFGaussian getSensorPoseOnRobot()const 
    {return m_sensorPoseOnRobot;}

    /**
     * Set the local sensor pose in the robot frame
     * 
     * @param pose Local sensor pose
     */
    inline void setSensorPoseOnRobot(const mrpt::poses::CPose3DPDFGaussian& pose) 
    {m_sensorPoseOnRobot = pose;}

protected:
    std::shared_ptr<igl::embree::EmbreeIntersector> m_embreeIntersector; //!< Engine for ray tracing
    mrpt::poses::CPose3DPDFGaussian m_sensorPoseOnRobot; //!< Local sensor pose in the robot frame
};

}} // end namespaces

#endif // DEF_OBSERVATION_SIMULATOR_H
