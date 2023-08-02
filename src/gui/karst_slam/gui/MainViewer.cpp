#include "karst_slam/gui/MainViewer.h"
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/graphslam/misc/CWindowObserver.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/system.h>

using namespace std;
using namespace mrpt::utils;
using namespace mrpt::opengl;
using namespace mrpt::graphslam;
using namespace mrpt::gui;
using namespace karst_slam::gui;

MainViewer::MainViewer(const string &name) :
    m_name(name),
    m_exit(false),
    m_section("Display")
{}

MainViewer::~MainViewer()
{
    if(m_win)
        delete m_win;
    if(m_winObserver)
        delete m_winObserver;
}

void MainViewer::init_(const CConfigFile &cfg)
{
    initWindow(cfg);
    initScene(cfg);
}

void MainViewer::initWindow(const CConfigFile& cfg)
{
    MRPT_START
    int w = cfg.read_int(m_section, m_name + "_window_width", 800, false);
    int h = cfg.read_int(m_section, m_name + "_window_height", 600, false);
    int x_pose = cfg.read_int(m_section, m_name + "_window_x_pose", 400, false);
    int y_pose = cfg.read_int(m_section, m_name + "_window_y_pose", 200, false);

    m_winObserver = new CWindowObserver();
    m_win = new CDisplayWindow3D(m_name, w, h);
    m_win->setPos(x_pose,y_pose);
    m_winObserver->observeBegin(*m_win);
    COpenGLScenePtr &scene = m_win->get3DSceneAndLock();
    COpenGLViewportPtr main_view = scene->getViewport("main");
    m_winObserver->observeBegin( *main_view );
    m_win->unlockAccess3DScene();

    // Init the keystroke mapping
    initKeystrokes(cfg);

    MRPT_END
}

void MainViewer::addKeystroke(keystroke&& ks)
{
    MRPT_START
    m_winObserver->registerKeystroke(ks.shortcut,
                                     ks.descr);
    Viewer::addKeystroke(move(ks));
    MRPT_END
}

void MainViewer::addKeystroke(const keystroke& ks)
{
    MRPT_START
    m_winObserver->registerKeystroke(ks.shortcut,
                                     ks.descr);
    Viewer::addKeystroke(ks);
    MRPT_END
}

void MainViewer::addKeystrokes(const keystrokeMapping& keystrokes)
{
    MRPT_START
    for(const auto& pair : keystrokes)
        addKeystroke(pair.second);
    MRPT_END
}

void MainViewer::queryKeyboardEvents()
{
    MRPT_START
    map<string, bool> events;
    m_winObserver->returnEventsStruct(&events);

    // Check each keystroke
    for(const auto& pair : m_keystrokes)
    {
        if(events[pair.first])
        {
            cout << "Received Event : " << pair.first << endl;
            m_keystrokes.at(pair.first).callback();
        }
    }

    MRPT_END;
}

void MainViewer::eventLoop()
{
    while (!m_exit)
    {
        queryKeyboardEvents();
        mrpt::system::sleep(10);
    }
}

void MainViewer::resize(unsigned int width, unsigned int heigth)
{
    m_win->resize(width,heigth);
}

void MainViewer::dumpKeystrokesToConsole() const
{
    MRPT_START
    cout << " ------ " << m_name << " shortcuts --------- " << endl;
    for(const auto& pair : m_keystrokes)
        cout << " - " << pair.first << " : " << pair.second.descr << endl;
    cout << "--------------------------------------------------" << endl;
    MRPT_END
}

void MainViewer::loadKeystroke(const CConfigFile& cfg,
                               const string& variableName,
                               keystroke&& ks)
{
    MRPT_START
    m_winObserver->registerKeystroke(ks.shortcut,
                                     ks.descr);
    Viewer::loadKeystroke(cfg,variableName,move(ks));
    MRPT_END
}

void MainViewer::toggleVisualization(const string& objName)
{
    MRPT_START
    COpenGLScenePtr& scene =  m_win->get3DSceneAndLock();
    CRenderizablePtr obj  = scene->getByName(objName);
    obj->setVisibility(!obj->isVisible());
    m_win->unlockAccess3DScene();
    m_win->forceRepaint();
    MRPT_END
}

void MainViewer::setVisibility(const string &objName, bool isVisible)
{
    MRPT_START
    COpenGLScenePtr& scene =  m_win->get3DSceneAndLock();
    CRenderizablePtr obj  = scene->getByName(objName);
    obj->setVisibility(isVisible);
    m_win->unlockAccess3DScene();
    m_win->forceRepaint();
    MRPT_END
}
