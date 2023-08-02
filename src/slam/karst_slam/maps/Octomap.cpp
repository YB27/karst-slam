#include "karst_slam/maps/Octomap.h"
#include "karst_slam/obs/ObservationMSIS_scan.h"
#include <mrpt/obs/CSensoryFrame.h>
//#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <mrpt/utils/CConfigFile.h>

using namespace std;
using namespace mrpt::maps;
using namespace mrpt::utils;
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace karst_slam;
using namespace karst_slam::obs;
using namespace karst_slam::slam;

OctoMap::OctoMap() :
    COctoMap()
{}

OctoMap::OctoMap(const CConfigFile &config_file) :
    COctoMap()
{
    loadParams(config_file);
}

OctoMap::OctoMap(const string &config_file_name) :
    OctoMap(CConfigFile(config_file_name))
{}

void OctoMap::loadParams(const CConfigFile& cfg)
{
    MRPT_START
           // CConfigFile cfg (config_file_name);
    const string& section = "OctoMap";
    insertionOptions.setOccupancyThres(cfg.read_float(section,
                                                      "occupancy_threshold",
                                                      0.5,
                                                      false));
    insertionOptions.setProbHit(cfg.read_float(section,
                                               "prob_hit",
                                               0.7,
                                               false));
    insertionOptions.setProbMiss(cfg.read_float(section,
                                                "prob_miss",
                                                0.4,
                                                false));
    insertionOptions.setClampingThresMin(cfg.read_float(section,
                                                        "clamping_thresh_min",
                                                        0.1192,
                                                        false));
    insertionOptions.setClampingThresMax(cfg.read_float(section,
                                                        "clamping_thresh_max",
                                                        0.971,
                                                        false));
    likelihoodOptions.decimation = cfg.read_int(section,
                                                "decimation",
                                                1,
                                                false);
    MRPT_END
}

bool OctoMap::internal_insertObservation(const CObservation *obs, const pose_t/*CPose3D*/ *robotPose)
{
    // CObservation2d/3DRangeScan
    CPose3D robotPose_ypr(*robotPose);
    if(COctoMap::internal_insertObservation(obs,&robotPose_ypr))
        return true;
    // Other
    else if(IS_CLASS(obs,ObservationMSIS_scan))
    {
        // Could have just convert the observationMSIS_scan to a CObservation3DRange and use COctoMap::internal_insertObservation
        // However, it costs the copy of the scan...
        internal_insertObservation_MSISscan(static_cast<const ObservationMSIS_scan*>(obs), robotPose);
        return true;
    }
    // else if(...)

    return false;
}

void OctoMap::internal_insertObservation_MSISscan(const ObservationMSIS_scan* obs,
                                                  const pose_t/*CPose3D*/ *robotPose)
{
    cout << "[OctoMap::internal_insertObservation_MSISscan] Update with MSIS scan observation from pose " << endl;
    cout << *robotPose << endl;
    cout << " !! Currently Ignored !! " << endl;
//    // Same as in COctoMapBase::internal_build_PointCloud_for_observation
//    octomap::point3d sensorPt;
//    octomap::Pointcloud  scan;
//    scan.clear();

//    CPose3D sensorPose(UNINITIALIZED_POSE), sensorPoseOnRobot(UNINITIALIZED_POSE);
//    obs->getSensorPose(sensorPoseOnRobot);
//    sensorPose.composeFrom(*robotPose, sensorPoseOnRobot);
//    sensorPt = octomap::point3d(sensorPoseOnRobot.x(),
//                                sensorPoseOnRobot.y(),
//                                sensorPoseOnRobot.z());

//    scan.reserve(obs->m_nPoints);

//    mrpt::math::CMatrixDouble44  H;
//    robotPose->getHomogeneousMatrix(H);
//    const float	m00 = H.get_unsafe(0,0);
//    const float	m01 = H.get_unsafe(0,1);
//    const float	m02 = H.get_unsafe(0,2);
//    const float	m03 = H.get_unsafe(0,3);
//    const float	m10 = H.get_unsafe(1,0);
//    const float	m11 = H.get_unsafe(1,1);
//    const float	m12 = H.get_unsafe(1,2);
//    const float	m13 = H.get_unsafe(1,3);
//    const float	m20 = H.get_unsafe(2,0);
//    const float	m21 = H.get_unsafe(2,1);
//    const float	m22 = H.get_unsafe(2,2);
//    const float	m23 = H.get_unsafe(2,3);

//    mrpt::math::TPoint3Df pt;
//    for (size_t i=0;i<obs->m_nPoints;i++)
//    {
//        pt.x = obs->m_points(i,0);
//        pt.y = obs->m_points(i,1);
//        pt.z = obs->m_points(i,2);

//        // Valid point?
//        if ( pt.x!=0 || pt.y!=0 || pt.z!=0 )
//        {
//            // Translation:
//            const float gx = m00*pt.x + m01*pt.y + m02*pt.z + m03;
//            const float gy = m10*pt.x + m11*pt.y + m12*pt.z + m13;
//            const float gz = m20*pt.x + m21*pt.y + m22*pt.z + m23;

//            // Add to this map:
//            scan.push_back(gx,gy,gz);
//        }
//    }
}

void OctoMap::update_oneNode(const pose_t/*CPose3D*/& nodePose,
                             const CSensoryFramePtr& sf)
{
   CPose3D nodePose_ypr(nodePose);
   for(CSensoryFrame::const_iterator it = sf->begin() ; it != sf->end(); it++)
        insertObservation(it->pointer(), &nodePose_ypr);
}

void OctoMap::update(const node_to_obs_t &nodes_to_obs,
                     const map<TNodeID, pose_t>& nodePoses)
{

    // ToDo : to be reimplemented in a generic way to update from different observation (3D range laser, MSIS, etc...)
    // for( each node)
    //  update_oneNode(obs)
    //
    // and make templated update_<> (specialized) for each type of obs

    // Code modified from CGraphSlamEngine
    clear(); // To be removed in the futur to allow only partial update

    // traverse all the nodes - add their laser scans (more generally : points cloud) at their corresponding
    // poses
    for (map<TNodeID,CSensoryFramePtr>::const_iterator
         it = nodes_to_obs.begin();
         it != nodes_to_obs.end(); ++it)
    {
        update_oneNode(nodePoses.at(it->first), it->second);
    }

//    // Code modified from CGraphSlamEngine
//    clear(); // To be removed in the futur to allow only partial update

//    // traverse all the nodes - add their laser scans (more generally : points cloud) at their corresponding
//    // poses
//    for (map<TNodeID,CSensoryFramePtr>::const_iterator
//         it = nodes_to_obs.begin();
//         it != nodes_to_obs.end(); ++it) {

//        const TNodeID& curr_node = it->first;

//        // fetch LaserScan
//        const CObservation3DRangeScanPtr& curr_laser_scan = it->second;
//        ASSERTMSG_(curr_laser_scan.present(),
//                   format("LaserScan of nodeID: %lu is not present.",
//                          static_cast<unsigned long>(curr_node)));

//        // Add all to octomap
//        insertObservation(curr_laser_scan.pointer(), &(nodePoses.at(curr_node)));
//    }
}

void OctoMap::compressMap()
{
    // Compress the map by first thresholding the nodes' occupancy to the max likelihood
    // Then, prune the tree
    cout <<"[OctoMap::compressMap] Compress the octomap " << endl;
    PIMPL_GET_REF(OcTree, m_octomap).toMaxLikelihood();
    PIMPL_GET_REF(OcTree, m_octomap).prune();
}

