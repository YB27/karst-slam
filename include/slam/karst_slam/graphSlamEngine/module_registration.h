#ifndef MODULE_REGISTRATION_H
#define MODULE_REGISTRATION_H

#include <vector>
#include <string>

// Forward declarations
namespace karst_slam
{
    namespace nrd
    {
        class NodeRegistrationDeciderInterface;
    }
    namespace erd
    {
        class EdgeRegistrationDeciderInterface;
    }
    namespace gso
    {
        class BaseGSO3D;
    }

    namespace sensors
    {
        class MSIS;
    }
}

namespace karst_slam{
/** Vector of valid NRD module classes. Any new NRD class should be add here.
  */
static const std::vector<std::string> nrd_classes = {"MSIS_NRD3D",
                                                     "MpIC_NRD3D"};

/** Vector of valid ERD module classes. Any new ERD class should be add here.
  */
static const std::vector<std::string> erd_classes = {"MpIC_NRD3D"};

/** Vector of valid GSO module classes. Any new GSO class should be add here.
  */
static const std::vector<std::string> gso_classes = {"GaussNewtonGSO3D",
                                                     "DoglegGSO3D",
                                                     "LevenbergMarquardtGtsamGSO3D",
                                                     "NonLinearConjugateGradientGSO3D",
                                                     "IsamGSO3D",
                                                     "Isam2GSO3D"};

/** Template function to get the vector valid modules depending on the module type (NRD, ERD or GSO)
  */
template<class T>
const std::vector<std::string>& getValidModules();

// Partial specializations
template<>
const std::vector<std::string>& getValidModules<karst_slam::nrd::NodeRegistrationDeciderInterface>(){return nrd_classes;} // NRD
template<>
const std::vector<std::string>& getValidModules<karst_slam::erd::EdgeRegistrationDeciderInterface>(){return erd_classes;} // ERD
template<>
const std::vector<std::string>& getValidModules<karst_slam::gso::BaseGSO3D>(){return gso_classes;} // GSO

} // end namespace
#endif // MODULE_REGISTRATION_H
