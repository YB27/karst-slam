#include "karst_slam/graphSlamEngine/gui/GraphSlamEngineViewer.h"
#include "GraphSlamEngineViewer_impl.h"
#include "karst_slam/graphSlamEngine/GraphSlamEngine.h"
#include <mrpt/obs/CSensoryFrame.h>

using namespace std;
using namespace mrpt::utils;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace mrpt::graphslam;
using namespace karst_slam;
using namespace karst_slam::gui;
using namespace karst_slam::slam;

using node_to_obs_t = std::map<mrpt::utils::TNodeID, mrpt::obs::CSensoryFramePtr>;

GraphSlamEngineViewer::GraphSlamEngineViewer(const CConfigFile& cfg,
                                             const string& name)
{
    m_pImpl = std::make_unique<GraphSlamEngineViewer_impl>(name);
    m_pImpl->init(cfg);
}

GraphSlamEngineViewer::~GraphSlamEngineViewer() = default;

void GraphSlamEngineViewer::queryKeyboardEvents() {m_pImpl->queryKeyboardEvents();}

void GraphSlamEngineViewer::dumpKeystrokesToConsole() const {m_pImpl->dumpKeystrokesToConsole();}

bool GraphSlamEngineViewer::isExit()const {return m_pImpl->isExit();}
bool GraphSlamEngineViewer::isPause()const{return m_pImpl->isPause();}
void GraphSlamEngineViewer::pause(){m_pImpl->pause();}

void GraphSlamEngineViewer::updateFrom(Observable *s){m_pImpl->render(static_cast<GraphSlamEngine*>(s));}
