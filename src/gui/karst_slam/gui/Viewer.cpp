#include "karst_slam/gui/Viewer.h"
#include <mrpt/utils/CConfigFile.h>

using namespace std;
using namespace mrpt::utils;
using namespace karst_slam::gui;

void Viewer::init(mrpt::opengl::COpenGLScenePtr& scene,
                  const mrpt::utils::CConfigFile& cfg)
{
    init_(scene,cfg);
    initKeystrokes(cfg);
}

void Viewer::init(const mrpt::utils::CConfigFile& cfg)
{
    init_(cfg);
    initKeystrokes(cfg);
}

void Viewer::loadKeystroke(const CConfigFile& cfg,
                           const string& variableName,
                           keystroke &&ks)
{
    ks.shortcut = cfg.read_string("Keystrokes",variableName,ks.shortcut,true);
    m_keystrokes.insert(make_pair(ks.shortcut, move(ks)));
}

void Viewer::addKeystroke(keystroke&& ks)
{
    ASSERT_(m_keystrokes.find(ks.shortcut) == m_keystrokes.end())
    m_keystrokes.insert(make_pair(ks.shortcut, move(ks)));
}

void Viewer::addKeystroke(const keystroke& ks)
{
    ASSERT_(m_keystrokes.find(ks.shortcut) == m_keystrokes.end())
    m_keystrokes.insert(make_pair(ks.shortcut, ks));
}
