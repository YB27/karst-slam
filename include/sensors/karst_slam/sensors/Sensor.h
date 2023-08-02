#ifndef SENSOR_H
#define SENSOR_H

#include <mrpt/utils/COutputLogger.h>

// Forward decl
namespace mrpt{namespace utils{class CConfigFile;}}

/**
 * Base class for sensor device.
 * At construction it loads the sensor parameters from a config file.
 * The loadParams function should be reimplemented in the derived class if the sensor has any params.
 */
namespace karst_slam{namespace sensors{
class Sensor : public mrpt::utils::COutputLogger
{
public:
    /**　Handy typedefs　*/
    using sensor_type_str = std::string;
    using sensor_name_str = std::string;

    /**
     * Read the list of sensors installed on the robot from the config file. It is generally used to generate objects in the main.
     * 
     * @param cfg Configuration file
     * @return map of sensors name and type 
     */
    static std::map<sensor_name_str, sensor_type_str> readSensorsList(const mrpt::utils::CConfigFile& cfg);

    Sensor(const std::string& deviceName);
    virtual ~Sensor() = default;

    virtual void loadParams(const mrpt::utils::CConfigFile& cfg) = 0;
    inline std::string getDeviceName() const {return m_deviceName;}

protected:
    std::string m_deviceName;
};
}} // end namespaces
#endif // SENSOR_H
