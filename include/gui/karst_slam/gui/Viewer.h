#ifndef VIEWER_H
#define VIEWER_H

#include "karst_slam/gui/KeyStroke.h"
#include <memory>

// Forward declarations
namespace mrpt
{
    namespace utils{class CConfigFile;}
    namespace opengl{class COpenGLScenePtr;}
}

namespace karst_slam{namespace gui{

/** Base class for visualization */
class Viewer
{
public:
    Viewer() = default;
    virtual ~Viewer() = default;

    /** Initialize the viewer based on the configuration in the given .ini file */
    void init(const mrpt::utils::CConfigFile& cfg);
    
    /** Initialize upon an already existing 3D openGL scene */
    void init(mrpt::opengl::COpenGLScenePtr& scene,
              const mrpt::utils::CConfigFile& cfg);

    /** Callback which hides / display the given object when triggered */
    virtual void toggleVisualization(const std::string& objName) = 0 ;

    virtual void setVisibility(const std::string& objName, bool isVisible) = 0;

    inline const keystrokeMapping& getKeystrokes()const{return m_keystrokes;}

protected:
    virtual void init_(mrpt::opengl::COpenGLScenePtr& scene,
                       const mrpt::utils::CConfigFile& cfg){}
    virtual void init_(const mrpt::utils::CConfigFile& cfg){}
    virtual void initKeystrokes(const mrpt::utils::CConfigFile& cfg) = 0;
    virtual void loadKeystroke(const mrpt::utils::CConfigFile& cfg,
                               const std::string& variableName,
                               keystroke&& ks);
    virtual void addKeystroke(keystroke&& ks);
    virtual void addKeystroke(const keystroke& ks);

    keystrokeMapping m_keystrokes;
};
}} // end namespaces

#endif // VIEWER_H
