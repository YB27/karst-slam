#ifndef BASEGTSAMGSO3D_H
#define BASEGTSAMGSO3D_H

#include "BaseGSO3D.h"
#include "karst_slam/graph/PoseGraph.h"
#include "mrpt-gtsam/wrapperGTSAM/conversionGraphs.h"
#include "mrpt-gtsam/wrapperGTSAM/conversionOptimizerParams.h"

//  Macro used to define the specialized type of parameters for inherited optimizer class using params_T::type
#define DEFINE_PARAMETER_TYPE_DECL template<class U> struct params_T{struct type;};

//  Define the specialized version of params_T defined in the macro above
//  Should appear before all new optimizer class definition
#define DEFINE_PARAMETER_TYPE_SPE(CLASS,PARAMTYPE) class CLASS; \
                                      template<> \
                                      struct params_T<CLASS> \
                                      {using type = PARAMTYPE;};


namespace karst_slam{namespace gso{

/** Clean-up version (separate visual-related stuffs,...) of CBaseGtsamGSO from mrpt-gtsam in 3D case only
 */
DEFINE_PARAMETER_TYPE_DECL

template<class DERIVED>
class BaseGtsamGSO3D : public BaseGSO3D
{
public:
    explicit BaseGtsamGSO3D(const std::string& name) : BaseGSO3D(name)
    {}

    BaseGtsamGSO3D(const std::string& name,
                   const std::string& configFile) : BaseGSO3D(name, configFile)
    {}
    BaseGtsamGSO3D(const std::string& name,
                   const mrpt::utils::CConfigFile& configFile) : BaseGSO3D(name, configFile)
    {}

    ~BaseGtsamGSO3D(){}

    inline typename params_T<DERIVED>::type getParameters() const {return m_params;}

protected:
    virtual void loadOptimizerParams(const mrpt::utils::CConfigFile& source);
    virtual void loadOptimizerParams_(const mrpt::utils::CConfigFile& source) = 0;
    virtual void printParams_() const = 0;

    // Generic parameters common to all GTSAM optimizer except those derived from GTSAM
    inline void loadNonLinearParams(const mrpt::utils::CConfigFile& source);
    void printNonLinearParams() const;

    virtual void execOptimization(const std::set<mrpt::utils::TNodeID>* nodes_to_optimize = nullptr) override;
    virtual gtsam::Values execOptimization_(const mrpt_gtsam::wrapperGTSAM::gtsamGraph& graph, const std::set<mrpt::utils::TNodeID>* nodes_to_optimize = nullptr) = 0;

    typename params_T<DERIVED>::type m_params;
};

#include "BaseGtsamGSO3D_impl.h"

}} //end namespaces
#endif // BASEGTSAMGSO3D_H
