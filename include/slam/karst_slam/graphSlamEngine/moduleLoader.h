#ifndef MODULEGENERATOR_H
#define MODULEGENERATOR_H

#include "karst_slam/typedefs.h"
#include "karst_slam/sensors/Sensor.h"
#include "karst_slam/graphSlamEngine/module_includes.h"
#include <mrpt/utils/CConfigFile.h>
#include <string>
#include <memory>

// Fwd declarations and registered classes
#include "module_registration.h"

namespace karst_slam
{

/** Template function to generate shared ptr of different modules for graphSlamEngine
 * 
 * @param module_class name of the module class
 * @param sensors map of sensors on the robot
 * @param configFile
 * @return Created module 
 */
template <class T>
std::shared_ptr<T> createModulePtr(const std::string& module_class,
                                   const std::map<std::string,std::shared_ptr<sensors::Sensor>>& sensors,
                                   const mrpt::utils::CConfigFile& configFile);

/** Partial specialization for NRD */
template <>
std::shared_ptr<karst_slam::nrd::MpIC_NRD3D> createModulePtr(const std::string& module_class,
                                                             const std::map<std::string,std::shared_ptr<sensors::Sensor>>& sensors,
                                                             const mrpt::utils::CConfigFile& configFile)
{
    using namespace std;
    using namespace karst_slam::nrd;
    using namespace karst_slam::sensors;

    shared_ptr<karst_slam::nrd::MpIC_NRD3D> ptr = nullptr;
    if(module_class == "MpIC_NRD3D")
    {
        // Set the sensor used by this module
        uint64_t startTimestamp = configFile.read_uint64_t("GeneralConfiguration","startTimestamp",0,true);
        ptr = make_shared<MpIC_NRD3D>(configFile, "Sonar_Seaking", "Sonar_Micron", startTimestamp);

        ASSERT_(sensors.find("Sonar_Micron") != sensors.end());
        ptr->setMSISPtr(dynamic_pointer_cast<MSIS_primary>(sensors.at("Sonar_Micron")));

        // Set the other MSIS sensor
        // ToDo : not hard coded !
        ptr->setMSISPtr_noLoc(dynamic_pointer_cast<MSIS_secondary>(sensors.at("Sonar_Seaking")));
    }
    else
    {
        string msg = module_class + " is an invalid module. Currently only MpIC_NRD3D is usable.";
        THROW_EXCEPTION(msg);
    } 

    return ptr;
}

/** Partial specialization for ERD */
template <>
std::shared_ptr<karst_slam::erd::MpIC_ERD3D> createModulePtr(const std::string& module_class,
                                                             const std::map<std::string,std::shared_ptr<sensors::Sensor>>& sensors,
                                                             const mrpt::utils::CConfigFile& configFile)
{
    using namespace std;
    using namespace karst_slam::erd;

    shared_ptr<karst_slam::erd::MpIC_ERD3D> ptr = nullptr;
    if(module_class == "MpIC_ERD3D")
        ptr = make_shared<MpIC_ERD3D>(configFile);
    else
    {
        string msg = module_class + " is an invalid module. Currently only MpIC_ERD3D is usable.";
        THROW_EXCEPTION(msg);
    }   
    
    return ptr;
}

/** Partial specialization for GSO */
template <>
std::shared_ptr<karst_slam::gso::BaseGSO3D> createModulePtr(const std::string& module_class,
                                                            const std::map<std::string,std::shared_ptr<sensors::Sensor>>& sensors, // Unused
                                                            const mrpt::utils::CConfigFile& configFile)
{
    using namespace std;
    using namespace karst_slam::gso;

    shared_ptr<BaseGSO3D> ptr = nullptr;
    if(module_class == "GaussNewtonGSO3D")
        ptr = static_pointer_cast<BaseGSO3D>(make_shared<GaussNewtonGSO3D>(configFile));
    else if(module_class == "LevenbergMarquardtGtsamGSO3D")
        ptr = static_pointer_cast<BaseGSO3D>(make_shared<LevenbergMarquardtGtsamGSO3D>(configFile));
    else if(module_class == "DoglegGSO3D")
        ptr = static_pointer_cast<BaseGSO3D>(make_shared<DoglegGSO3D>(configFile));
    else if(module_class == "NonLinearConjugateGradientGSO3D")
        ptr = static_pointer_cast<BaseGSO3D>(make_shared<NonLinearConjugateGradientGSO3D>(configFile));
    else if(module_class == "IsamGSO3D")
        ptr = static_pointer_cast<BaseGSO3D>(make_shared<IsamGSO3D>(configFile));
    else if(module_class == "Isam2GSO3D")
        ptr = static_pointer_cast<BaseGSO3D>(make_shared<Isam2GSO3D>(configFile));
    else
    {
        string msg = module_class + " is an invalid module";
        THROW_EXCEPTION(msg);
    }

    return ptr;
}

/**
 * Load a module (NRD , ERD or GSO). Throw an exception if the required module is invalid
 * @param configFileName Path to the .ini configuration file
 * @param moduleName Class name of the module to load
 * @param defaultModule Default class name of the module to load
 */
template<class T>
void loadModule(const mrpt::utils::CConfigFile& configFile,
                const std::string& moduleName,
                const std::string& defaultModule,
                const std::map<std::string,std::shared_ptr<sensors::Sensor>>& sensors,
                std::shared_ptr<T>& module)
{
    using namespace std;
    MRPT_START
    string module_class = configFile.read_string("GeneralConfiguration", moduleName, defaultModule, false);

    module = createModulePtr<T>(module_class, sensors, configFile);
    module->loadParams(configFile);
    module->printParams();

    MRPT_END
}

/**
 * \overload
 */
template<class T>
void loadModule(const std::string& configFileName,
                const std::string& moduleName,
                const std::string& defaultModule,
                std::shared_ptr<T>& module)
{
    mrpt::utils::CConfigFile configFile(configFileName);
    loadModule(configFile, moduleName, defaultModule/*,validModules*/, module);
}

} // end namespace

#endif //MODULEGENERATOR_H
