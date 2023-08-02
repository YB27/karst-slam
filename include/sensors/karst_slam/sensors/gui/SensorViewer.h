#ifndef SENSOR_VIEWER_H
#define SENSOR_VIEWER_H

#include "karst_slam/gui/Viewer.h"
#include "karst_slam/typedefs.h"
#include "karst_slam/mrpt_poses_frwd.h"
#include <string>

// Forward declarations
namespace mrpt
{
    namespace utils{class CConfigFile;}
    namespace opengl{class COpenGLScenePtr;}
    namespace obs{class CObservationPtr;}
}
namespace karst_slam{namespace gui{class MainViewer;}}

namespace karst_slam{namespace gui{
/**  Generic class for sensor viewers 
 * @todo Maybe too early as finally there is only one real class used currently (MSISViewer) 
*/
class SensorViewer : public Viewer
{
public:
    explicit SensorViewer(const std::string& sensorLabel, MainViewer* mainViewer = nullptr);
    virtual void render(mrpt::opengl::COpenGLScenePtr& scene,
                        const mrpt::obs::CObservationPtr& obs,
                        const mrpt::poses::CPose3D& refFrame = mrpt::poses::CPose3D()) = 0;

    virtual void toggleVisualization(const std::string& objName) override;
    virtual void setVisibility(const std::string& objName, bool isVisible) override;
protected:
    virtual void initKeystrokes(const mrpt::utils::CConfigFile& cfg) = 0;

    std::string m_configFileSection; //!< Name of the corresponding section in config file 
    std::string m_sensorLabel; //!< Name of the sensor
    MainViewer* m_mainViewer; //!< Pointer of the mainViewer to which this sensorViewer is attached to
};
}} // end namespaces

#endif // SENSOR_VIEWER_H
