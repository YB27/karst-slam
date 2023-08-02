#ifndef MAIN_VIEWER_H
#define MAIN_VIEWER_H

#include "karst_slam/gui/Viewer.h"

namespace mrpt
{
    namespace utils{class CConfigFile;}
    namespace gui{class CDisplayWindow3D;}
    namespace graphslam{class CWindowObserver;}
}

namespace karst_slam{namespace gui{

/** Main viewer class used to visualize the graphSLAM results */
class MainViewer : public Viewer
{
public:
    /** Constructor */
    explicit MainViewer(const std::string& name = "MainViewer");
    
    /** Destructor */
    virtual ~MainViewer();

    /** Load a keystroke from a configuration file */
    void loadKeystroke(const mrpt::utils::CConfigFile& cfg,
                       const std::string& variableName,
                       keystroke&& ks) override;

    void addKeystroke(keystroke&& ks) override;
    void addKeystroke(const keystroke& ks) override;
    void addKeystrokes(const keystrokeMapping& keystrokes);
    void queryKeyboardEvents();

    void resize(unsigned int width,unsigned int heigth);

    virtual void toggleVisualization(const std::string& objName) override ;
    virtual void setVisibility(const std::string& objName, bool isVisible) override;

    void dumpKeystrokesToConsole() const;

    /**
     * @brief Check if the window should close
     * @return True if an exit order has been thrown
     */
    inline bool isExit(){return m_exit;}

    void eventLoop();

protected:
    void init_(const mrpt::utils::CConfigFile &cfg) override;
    void initWindow(const mrpt::utils::CConfigFile &cfg);
    virtual void initScene(const mrpt::utils::CConfigFile &cfg) = 0;
    virtual void initKeystrokes(const mrpt::utils::CConfigFile& cfg) = 0;

    /**
     * @brief Set the exit flag
     */
    inline void exit(){m_exit = true;}

    mrpt::graphslam::CWindowObserver* m_winObserver;
    mrpt::gui::CDisplayWindow3D* m_win;
    std::string m_name;//! Window name
    std::string m_section;//!< Section name  in .ini configuration file
    bool m_exit; 
};

}} // end namespaces
#endif // MAIN_VIEWER_H
