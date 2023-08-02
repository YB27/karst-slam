#ifndef SENSOR_VIEWER_FACTORY_H
#define SENSOR_VIEWER_FACTORY_H

#include "MSISViewer.h"

#include <map>
#include <memory>

/**
  * Creates factory for sensory viewer. More precisely, for each sensor class, we associate it with the corresponding sensor viewer class.
  * Each time a new sensor class is added to the library with its associated sensorViewer class, the user must register them here by :
  * - Include the new viewer class header
  * - Adding the forward declaration of the sensor class
  * - Adding the macro CREATE_SENSOR_VIEWER_FACTORY
  * - Adding the pair to the mapFactoryViewer map.
  */

#define CREATE_SENSOR_VIEWER_FACTORY(SENSOR_CLASS,SENSOR_VIEWER_CLASS) class SensorViewerFactory_##SENSOR_CLASS : public SensorViewerFactory\
{ \
std::shared_ptr<SensorViewer> create(const std::string& sensorLabel, MainViewer* mainViewer) override {return std::make_shared<SENSOR_VIEWER_CLASS>(sensorLabel,mainViewer);} \
};

// Forward declarations
namespace karst_slam{
   namespace gui
   {
     class MainViewer;
     class SensorViewer;
   }
   namespace sensor
   {
        class MSIS_primary;
        class MSIS_secondary;
     // class newSensor;  Add here forward declaration of new sensor class and corresponding viewer class
   }
}

namespace karst_slam{namespace gui{
class SensorViewerFactory
{
public:
    virtual std::shared_ptr<SensorViewer> create(const std::string& sensorLabel, MainViewer* mainViewer) = 0;
};

// Declare and define the factories
CREATE_SENSOR_VIEWER_FACTORY(MSIS_primary,MSISViewer)
CREATE_SENSOR_VIEWER_FACTORY(MSIS_secondary,MSISViewer)
//CREATE_SENSOR_VIEWER_FACTORY(newSensor, newSensorViewer)

// Add the pair (sensorClassStr,factory)
const static std::map<std::string,std::shared_ptr<SensorViewerFactory>> mapFactoryViewer = 
{{"MSIS_primary",std::make_shared<SensorViewerFactory_MSIS_primary>()},
 {"MSIS_secondary",std::make_shared<SensorViewerFactory_MSIS_secondary>()}
 /*{"newSensor",sensorViewerFactory_newSensor()}}*/
};
}} // end namespaces

#endif //SENSOR_VIEWER_FACTORY_H
