#ifndef SENSOR_FACTORY_H
#define SENSOR_FACTORY_H

#include "sensors_includes.h"

#include <map>
#include <memory>

// Forward declarations
namespace mrpt{namespace utils{class CConfigFile;}}
/**
  * Creates factory for sensors.
  * Each time a new sensor class is added to the library, the user must register them here by :
  * - Include the new sensor class header
  * - Adding the forward declaration of the sensor class
  * - Adding the macro CREATE_SENSOR_VIEWER_FACTORY
  * - Adding the pair to the mapFactoryViewer map.
  * Please note that you also need to load the parameters.
  */
namespace karst_slam{namespace sensors{
static std::shared_ptr<Sensor> createSensorFactory(const std::string& deviceType,
                                                   const std::string& deviceName,
                                                   const mrpt::utils::CConfigFile& cfg)
{
    if(deviceType == "MSIS_primary")
    {
        std::shared_ptr<MSIS_primary> msisPtr = std::make_shared<MSIS_primary>(deviceName);
        msisPtr->loadParams(cfg);
        return msisPtr;
    }
    else if(deviceType == "MSIS_secondary")
    {
        std::shared_ptr<MSIS_secondary> msisPtr = std::make_shared<MSIS_secondary>(deviceName);
        msisPtr->loadParams(cfg);
        return msisPtr;
    }
    else
        std::cout << "[sensorFactory::create] Unknown device Type" << std::endl;

    return nullptr;
}

/** Create sensors based on the given config file */
std::map<std::string,std::shared_ptr<Sensor>> createSensors(const mrpt::utils::CConfigFile& cfg)
{
    std::map<std::string,std::shared_ptr<Sensor>> res;

    std::map<Sensor::sensor_name_str, Sensor::sensor_type_str> mapSensor = Sensor::readSensorsList(cfg);
    for(const auto& pair : mapSensor)
        res[pair.first] = createSensorFactory(pair.second, pair.first, cfg);

    return res;
}

}} // end namespaces

#endif // SENSOR_FACTORY_H
