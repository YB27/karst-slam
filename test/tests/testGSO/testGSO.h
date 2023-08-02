#ifndef TESTGSO_H
#define TESTGSO_H

#include "karst_slam/tests/utils/testGenerateGraph.h"
#include "karst_slam/tests/utils/testDefines.h"

#include <mrpt-gtsam/gso/CDoglegGSO.h>
#include <mrpt-gtsam/gso/CGaussNewtonGSO.h>
#include <mrpt-gtsam/gso/CIsam2GSO.h>
#include <mrpt-gtsam/gso/CIsamGSO.h>
#include <mrpt-gtsam/gso/CLevenbergMarquardtGtsamGSO.h>
#include <mrpt-gtsam/gso/CNonLinearConjugateGradientGSO.h>

#include "karst_slam/gso/gso_includes.h"

#include "karst_slam/graph/PoseGraph.h"

#include <assert.h>
#include <pid/rpath.h>

// Just a slightly modified versions of the tests in mrpt-gtsam lib
// In particular :
//  - Removed template related to the graph type
//  - Rename of functions test
//  - Adapted to the karst-slam lib interface (eg. use poseGraph instead of directly mrpt/gtsam graphs)

using namespace mrpt_gtsam::wrapperGTSAM;
using namespace mrpt::graphslam::optimizers;
using namespace gtsam;
using namespace std;
using namespace karst_slam;
using namespace karst_slam::gso;
using namespace karst_slam::graph;
using namespace karst_slam::tests::utils;

template<class GRAPH_OPTIMIZER, class GSO>
int testBasicGSO()
{
    using mrpt_graph_t      = mrpt::graphs::CNetworkOfPoses3DInf;
    using gtsam_value_t     = gtsam_pose_t<mrpt_graph_t::constraint_no_pdf_t>;
    using mrpt_pose_T       = mrpt_pose_t<gtsam_value_t>;
    using mrpt_pose_pdf_t   = mrpt_graph_t::constraint_t;

    // Construct the same graph as in gtsam examples gtsam/examples/Pose2SLAMExample.cpp
    gtsamGraph gtsam_graph = createRealisticGraph<mrpt_graph_t>();

    // Now create the same graph in mrpt format
    shared_ptr<mrpt_graph_t> mrpt_graph = mrptToGTSAM_graphConverter<mrpt_graph_t>::convertBackFullGraph(gtsam_graph);
//    cout << "mrpt graph : " << endl;
//    cout << "-----> Nodes : " << endl;
//    for(const auto& node : mrpt_graph->nodes)
//        cout << node.first << " , " << node.second << endl;
//    cout << "-----> Edges : " << endl;
//    for(const auto& edge : mrpt_graph->edges)
//        cout << "(" << edge.first.first <<  "," << edge.first.second << ")" << " , " << edge.second << endl;
//    cout << endl;

    shared_ptr<PoseGraph> pg = make_shared<PoseGraph>(move(mrpt_graph));

    // Debug
    cout << "Original graph : " << endl;
    gtsam_graph.print();
    cout << "Converted graph(pose graph) : " << endl;
    pg->getConvertedGTSAMGraph().print();

    // Expected optimization values
    GRAPH_OPTIMIZER optimizer(gtsam_graph.getFactorGraph(), gtsam_graph.getValues()); // use default parameters
    Values expectedResult = optimizer.optimize();

    // Optimize
    GSO gso;// Use also default parameters
    gso.setPoseGraphPtr(pg);

    // optimizeGraph() is protected
    // so need to use the public interface to indirectly call it
    mrpt::obs::CActionCollectionPtr actCollecPtr;
    mrpt::obs::CSensoryFramePtr sensPtr;
    mrpt::obs::CObservationPtr obsPtr;
    gso.updateState(actCollecPtr, sensPtr, obsPtr);

    // Convert the updated mrpt graph to compare with expected values
    const gtsamGraph& updatedGraph = pg->getConvertedGTSAMGraph();
    //mrptToGTSAM_graphConverter<mrpt_graph_t>::convertFullGraph(*mrpt_graph, updatedGraph);

    // Compare
    if(expectedResult.equals(updatedGraph.getValues()),1e-4)
    {
        PASSED
        return 0;
    }
    else
    {
        FAILED

        // Print debug info
        cout << "----- Debug Info ------" << endl;
        cout << "--> Expected values : " << endl;
        expectedResult.print();
        cout << "--> Computed values : " << endl;
        updatedGraph.getValues().print();

        return -1;
    }
}

int testIsamGSO()
{
    using mrpt_graph_t      = mrpt::graphs::CNetworkOfPoses3DInf;
    using gtsam_value_t     = gtsam_pose_t<mrpt_graph_t::constraint_no_pdf_t>;
    using mrpt_pose_T       = mrpt_graph_t::constraint_no_pdf_t;
    using mrpt_pose_pdf_t   = mrpt_graph_t::constraint_t;
    using gtsam_value_t     = gtsam_pose_t<mrpt_pose_T>;
    using between_t         = typename mrptToGTSAM_graphConverter<mrpt_graph_t>::between_factor_t;
    using prior_t         = typename mrptToGTSAM_graphConverter<mrpt_graph_t>::prior_factor_t;

    //shared_ptr<mrpt_graph_t> mrpt_graph = std::make_shared<mrpt_graph_t>();
    shared_ptr<PoseGraph> pg = make_shared<PoseGraph>();

    // Wrapped Isam
    int reorderInterval = 3;
    IsamGSO3D isamGSO(PID_PATH("test_config_file.ini"),reorderInterval);
    isamGSO.setPoseGraphPtr(pg);

    // GT
    NonlinearISAM isam(reorderInterval);
    NonlinearFactorGraph graph;
    Values initialEstimate;

    int nIter = 10;
    mrpt_pose_pdf_t relativePose, pose_noise_pdf;
    mrpt_pose_T pose_gt = mrpt_pose_T(),pose_noise;

    vector<double> noise = {-0.2,0.1,-0.1,0.01,0.01,-0.01};

    for (size_t i = 0; i <= nIter; i++)
    {
        cout << "Current iteration : " << i << endl;
        if(i == 0)
        {
            // Add a prior factor with constrained noise model (ie sigmas = 0) as an equivalent to the fixed root node
            //pose_gt = mrpt_pose_T();
            graph.emplace_shared<prior_t>(0,
                                          gtsam_value_t(),
                                          gtsam::noiseModel::Constrained::All(gtsam_value_t::dimension));
            initialEstimate.insert(0, gtsam_value_t());
            //pg->insertNode(0, pose_gt);//mrpt_graph->nodes.insert(std::make_pair(0, pose_gt));

        }
        else
        {
            // Edge / Factor
            createRandomPosePDF<mrpt_pose_pdf_t>(relativePose);
            pose_gt += relativePose.mean;

            pose_noise = pose_gt;
            addNoiseToPose(noise,pose_noise);
            pose_noise_pdf.mean = pose_noise;

            initialEstimate.insert(i,convertPose_mrptToGtsam(pose_noise));
            //pg->insertNode(i, pose_noise);//mrpt_graph->nodes.insert(std::make_pair(i, pose_noise));
            pg->insertNode(pose_noise_pdf, relativePose);

            graph.push_back(boost::make_shared<between_t>(i-1,
                                                          i,
                                                          convertPose_mrptToGtsam(relativePose),
                                                          mrptToGTSAM_graphConverter<mrpt_graph_t>::convertNoiseModel(relativePose))
                            );

            pg->insertEdge(i-1, i, relativePose);//mrpt_graph->insertEdge(i-1, i, relativePose);

            cout << "Current graph /initial estimate" << endl;
            graph.print();
            initialEstimate.print();

            isam.update(graph, initialEstimate);
            Values current_gt_estimate = isam.estimate();

            // optimizeGraph() is protected
            // so need to use the public interface to indirectly call it
            mrpt::obs::CActionCollectionPtr actCollecPtr;
            mrpt::obs::CSensoryFramePtr sensPtr;
            mrpt::obs::CObservationPtr obsPtr;
            isamGSO.updateState(actCollecPtr, sensPtr, obsPtr);

            Values current_estimate = pg->getConvertedGTSAMGraph().getValues();//isamGSO.getConvertedGTSAMGraph().getValues();

            if(!current_gt_estimate.equals(current_estimate))
            {
                cout << "[FAILED]  " << __PRETTY_FUNCTION__ << endl;
                // Print debug info
                cout << "----- Debug Info ------" << endl;
                cout << " i = " << i << endl;
                cout << "--> Expected values : " << endl;
                current_gt_estimate.print();
                std::cout << "--> Computed values : " << std::endl;
                current_estimate.print();
                return -1;
            }
            // Clear the factor graph and values for the next iteration
            graph.resize(0);
            initialEstimate.clear();
        }
    }

    cout << "[PASSED]  " << __PRETTY_FUNCTION__ << endl;
    return 0;
}

int testIsam2GSO()
{
    using mrpt_graph_t      = mrpt::graphs::CNetworkOfPoses3DInf;
    using gtsam_value_t     = gtsam_pose_t<mrpt_graph_t::constraint_no_pdf_t>;
    using mrpt_pose_T       = mrpt_graph_t::constraint_no_pdf_t;
    using mrpt_pose_pdf_t   = mrpt_graph_t::constraint_t;
    using gtsam_value_t     = gtsam_pose_t<mrpt_pose_T>;
    using between_t         = typename mrptToGTSAM_graphConverter<mrpt_graph_t>::between_factor_t;
    using prior_t           = typename mrptToGTSAM_graphConverter<mrpt_graph_t>::prior_factor_t;

    //shared_ptr<mrpt_graph_t> mrpt_graph = std::make_shared<mrpt_graph_t>();
    shared_ptr<PoseGraph> pg = make_shared<PoseGraph>();

    // Wrapped Isam
    Isam2GSO3D isamGSO(PID_PATH("test_config_file.ini"));
    isamGSO.setNLSolverIterations(2);// two iterations
    isamGSO.setPoseGraphPtr(pg);

    // GT
    ISAM2Params params = isamGSO.getParameters();
    ISAM2 isam(params);
    NonlinearFactorGraph graph;
    Values initialEstimate;

    int nIter = 10;
    mrpt_pose_pdf_t relativePose, pose_noise_pdf;
    mrpt_pose_T pose_gt, pose_noise;

    vector<double> noise = {-0.2,0.1,-0.1,0.01,0.01,-0.01};

    for (size_t i = 0; i <= nIter; i++)
    {
        if(i == 0)
        {
            // Add a prior factor with constrained noise model (ie sigmas = 0) as an equivalent to the fixed root node
            //pose_gt = mrpt_pose_T();
            graph.emplace_shared<prior_t>(0,
                                          gtsam_value_t(),
                                          gtsam::noiseModel::Constrained::All(gtsam_value_t::dimension));
            initialEstimate.insert(0, gtsam_value_t());
        }
        else
        {
            // Edge / Factor
            createRandomPosePDF<mrpt_pose_pdf_t>(relativePose);
            pose_gt += relativePose.mean;

            pose_noise = pose_gt;
            addNoiseToPose(noise,pose_noise);
            pose_noise_pdf.mean = pose_noise;

            initialEstimate.insert(i,convertPose_mrptToGtsam(pose_noise));
            pg->insertNode(pose_noise_pdf,relativePose);

            graph.push_back(boost::make_shared<between_t>(i-1,
                                                          i,
                                                          convertPose_mrptToGtsam(relativePose),
                                                          mrptToGTSAM_graphConverter<mrpt_graph_t>::convertNoiseModel(relativePose))
                            );

            pg->insertEdge(i-1, i, relativePose);
            graph.print();
            initialEstimate.print();

            // Two iterations of non linear solver
            isam.update(graph, initialEstimate);
            isam.update();
            Values current_gt_estimate = isam.calculateEstimate();

            cout << "Current gt estimate : " << endl;
            current_gt_estimate.print();

            cout << "Current converted graph : " << endl;
            pg->getConvertedGTSAMGraph().print();

            // optimizeGraph() is protected
            // so need to use the public interface to indirectly call it
            mrpt::obs::CActionCollectionPtr actCollecPtr;
            mrpt::obs::CSensoryFramePtr sensPtr;
            mrpt::obs::CObservationPtr obsPtr;
            isamGSO.updateState(actCollecPtr, sensPtr, obsPtr);

            Values current_estimate = pg->getConvertedGTSAMGraph().getValues();

            if(!current_gt_estimate.equals(current_estimate))
            {
                cout << "[FAILED]  " << __PRETTY_FUNCTION__ << endl;
                // Print debug info
                cout << "----- Debug Info ------" << endl;
                cout << " i = " << i << endl;
                cout << "--> Expected values : " << endl;
                current_gt_estimate.print();
                cout << "--> Computed values : " << endl;
                current_estimate.print();
                return -1;
            }
            // Clear the factor graph and values for the next iteration
            graph.resize(0);
            initialEstimate.clear();
        }
    }

    PASSED
    return 0;
}

#endif // TESTGSO_H
