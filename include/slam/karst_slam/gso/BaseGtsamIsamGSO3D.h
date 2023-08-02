#ifndef BASEGTSAMISAMGSO3D_H
#define BASEGTSAMISAMGSO3D_H

#include "BaseGtsamGSO3D.h"

namespace karst_slam{namespace gso{
DEFINE_PARAMETER_TYPE_SPE(BaseGtsamIsamGSO3D, gtsam::ISAM2Params)

/** Base class for optimization with the ISAM algo from gtsam */ 
class BaseGtsamIsamGSO3D : public BaseGtsamGSO3D<BaseGtsamIsamGSO3D>
{
public:
    explicit BaseGtsamIsamGSO3D(const std::string& name);
    BaseGtsamIsamGSO3D(const std::string& name,const std::string& configFile);
    BaseGtsamIsamGSO3D(const std::string& name,const mrpt::utils::CConfigFile& configFile);

    virtual ~BaseGtsamIsamGSO3D(){}

    bool updateState(const mrpt::obs::CActionCollectionPtr& action,
                     const mrpt::obs::CSensoryFramePtr& observations,
                     const mrpt::obs::CObservationPtr& observation,
                     mrpt::obs::CSensoryFramePtr generatedObservations = mrpt::obs::CSensoryFramePtr()) override;

protected:
    void optimizeGraph() override;
    void _optimizeGraph(bool is_full_update=false) override;
    virtual gtsam::Values execOptimization_(const mrpt_gtsam::wrapperGTSAM::gtsamGraph& graph,
                                            const std::set<mrpt::utils::TNodeID>* nodes_to_optimize = nullptr) = 0;
};
}} // end namespaces
#endif // BASEGTSAMISAMGSO3D_H
