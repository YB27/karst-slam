#include "karst_slam/sensors/Sensor.h"
#include <mrpt/utils/CConfigFile.h>

using namespace std;
using namespace mrpt::utils;
using namespace karst_slam;
using namespace karst_slam::sensors;

Sensor::Sensor(const string &deviceName):
    COutputLogger(deviceName),
    m_deviceName(deviceName)
{
    setMinLoggingLevel(LVL_DEBUG);
}

map<Sensor::sensor_name_str, Sensor::sensor_type_str> Sensor::readSensorsList(const mrpt::utils::CConfigFile& cfg)
{
    map<sensor_name_str,sensor_type_str> res;

    vector<string> sensors_list;
    cfg.read_vector<vector<string>>("GeneralConfiguration", "sensors_list",vector<string>(), sensors_list,true);
    sensor_type_str type;
    sensor_name_str sensorLabel;
    size_t sep_pos;
    for(const string& typeSensorLabel : sensors_list)
    {
        // Sensors in the config file are written in the format TYPE:SensorLabel
        sep_pos     = typeSensorLabel.find(':');
        type        = typeSensorLabel.substr(0,sep_pos);
        sensorLabel = typeSensorLabel.substr(sep_pos+1);
        res[sensorLabel] = type;
    }

    return res;
}
