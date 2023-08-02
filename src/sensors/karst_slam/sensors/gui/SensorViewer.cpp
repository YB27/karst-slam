#include "karst_slam/sensors/gui/SensorViewer.h"
#include "karst_slam/gui/MainViewer.h"

using namespace std;
using namespace karst_slam::gui;

SensorViewer::SensorViewer(const std::string& sensorLabel, MainViewer* mainViewer):
 m_configFileSection("Display"),
 m_mainViewer(mainViewer),
 m_sensorLabel(sensorLabel)
{}

void SensorViewer::toggleVisualization(const string &objName)
{
    if(m_mainViewer)
        m_mainViewer->toggleVisualization(objName);
}

void SensorViewer::setVisibility(const string &objName, bool isVisible)
{
    if(m_mainViewer)
        m_mainViewer->setVisibility(objName,isVisible);
}
