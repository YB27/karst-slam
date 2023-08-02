//#include <graphSlamEngine_app/CGraphSlamEngineMod.h>
//#include <mrpt/graphs/CNetworkOfPoses.h>

//int testSave3DScenefromGT_TUM()
//{
//    using namespace mrpt::graphslam;
//    using namespace mrpt::graphs;

//    // here set manually the path to files.
//    // Do not include the datasets as a runtime ressource (quite heavy)
//    std::string datasetName = "freiburg1_room", prefixFile = "rgbd_dataset_", prefixFolder = "rawlog_" + prefixFile;
//    std::string basePath    = "/home/yohan/pid-workspace/packages/mrpt-gtsam/share";
//    std::string path = basePath + "/" + prefixFolder + datasetName;
//    std::string imgDataPath = path + "/" + prefixFile + datasetName + "_Images";
//    std::string rawlogFile_loc = prefixFile + datasetName + ".rawlog";

//    std::cout << "GT file : " <<  path + "/groundtruth.txt" << std::endl;
//    std::cout << "Rawlog file : " << path + "/" + rawlogFile_loc << std::endl;
//    CGraphSlamEngineMod<CNetworkOfPoses3DInf>::save3DSceneFromGT(path + "/groundtruth.txt",
//                                                                 "rgbd_tum",
//                                                                 path + "/" + rawlogFile_loc,
//                                                                 basePath + "/odometry_3D_TUM.ini",
//                                                                 imgDataPath,
//                                                                 true, // DrawOctomap
//                                                                 true// DrawPointClouds
//                                                                 );

//    return 0;
//}
