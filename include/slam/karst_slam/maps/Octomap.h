#ifndef OCTOMAP_H
#define OCTOMAP_H

#include "karst_slam/typedefs.h"
#include <mrpt/maps/COctoMap.h>

// Forward declarations
namespace mrpt
{
    namespace obs{class CSensoryFramePtr;}
    namespace utils{class CConfigFile;}
}
namespace karst_slam{namespace obs{class ObservationMSIS_scan;}}

namespace karst_slam{namespace slam{
class OctoMap : public mrpt::maps::COctoMap
{
public:
    using node_to_obs_t = std::map<mrpt::utils::TNodeID, mrpt::obs::CSensoryFramePtr>;

    OctoMap();
    explicit OctoMap(const mrpt::utils::CConfigFile& config_file);
    explicit OctoMap(const std::string& config_file_name);

    void compressMap();

    // ToDo : Update only parts where nodes pose have been updated
    // In original MRPT implementation, they keep all (!) the laser range scan to rebuild the map ...
    // In fact, the map is not required for localization but can be used by converting part of it back to a less accurate point cloud when required (eg loop closure)
    // --> Can also merge successive points cloud to remove covering parts (should be a part of the graph compressing method by merging nodes)
    //
    // Currently (6/06/19), just always fully rebuild the map to test the full graphSlamEngine.
    void update(const node_to_obs_t& nodes_to_laser_scans3D,
                const std::map<mrpt::utils::TNodeID, pose_t>& nodePoses);

    void loadParams(const mrpt::utils::CConfigFile& config_file);

private:
    bool internal_insertObservation(const mrpt::obs::CObservation *obs, const mrpt::poses::CPose3D *robotPose);
    void internal_insertObservation_MSISscan(const karst_slam::obs::ObservationMSIS_scan* obs, const mrpt::poses::CPose3D *robotPose);
    void update_oneNode(const mrpt::poses::CPose3D& nodePose,
                        const mrpt::obs::CSensoryFramePtr& sf);

};
}} // end namespaces
#endif // OCTOMAP_H
