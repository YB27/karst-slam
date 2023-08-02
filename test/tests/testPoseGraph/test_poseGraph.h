#include "karst_slam/graph/PoseGraph.h"
#include "karst_slam/tests/utils/testDefines.h"
#include "karst_slam/compare.h"
#include "mrpt-gtsam/tests/testGTSAM3D.h"
#include "mrpt-gtsam/tests/testUtils.h"
#include "karst_slam/gso/LevenbergMarquardtGtsamGSO3D.h"
#include "mrpt/obs/CActionCollection.h"

using namespace std;
using namespace karst_slam;
//int testDijkstraWithLastNodePdf()
//{
//    using namespace karst_slam::utils;
//    using namespace karst_slam::graph;
//    using mrpt_graph_t = mrpt::graphs::CNetworkOfPoses3DInf;
//    using pose_pdf_t   = mrpt_graph_t::constraint_t;
//    using pose_t       = mrpt_graph_t::constraint_no_pdf_t;

//    shared_ptr<PoseGraph> pg = make_shared<PoseGraph>();

//    mrpt::math::CMatrixDouble66 cov = Eigen::Matrix<double,6,6>::Identity();

//    // expectedPdf1 -> between node 0 (root) and 3
//    // expectedPdf2 -> between node 2 and 5
//    pose_pdf_t incrementPdf, globalPdf;

//    incrementPdf.cov_inv = cov;
//    incrementPdf.mean = mrpt::poses::CPose3D(1,0,0,0.2,0,0);

//    std::map<mrpt::utils::TNodeID, pose_t> nodes;

//    // Node 1
//    globalPdf = incrementPdf;
//    pg->insertNode(globalPdf,incrementPdf);
//    pg->insertEdge(0,1,incrementPdf);
//    nodes[1] = globalPdf.mean;

//    // Node 2
//    globalPdf    += incrementPdf;
//    pg->insertNode(globalPdf,incrementPdf);
//    pg->insertEdge(1,2,incrementPdf);
//    nodes[2] = globalPdf.mean;

//    // Node 3
//    globalPdf    += incrementPdf;
//    pg->insertNode(globalPdf,incrementPdf);
//    pg->insertEdge(2,3,incrementPdf);
//    nodes[3] = globalPdf.mean;

//    // Node 4
//    globalPdf    += incrementPdf;
//    pg->insertNode(globalPdf,incrementPdf);
//    pg->insertEdge(3,4,incrementPdf);
//    nodes[4] = globalPdf.mean;

//    // Node 5
//    globalPdf    += incrementPdf;
//    pg->insertNode(globalPdf,incrementPdf);
//    pg->insertEdge(4,5,incrementPdf);
//    nodes[5] = globalPdf.mean;

//    pg->dijkstra_nodes_estimate();
//}

int testRelativePosePdf()
{
    using namespace karst_slam::utils;
    using namespace karst_slam::graph;
    using mrpt_graph_t      = mrpt::graphs::CNetworkOfPoses3DInf;
    using pose_pdf_t   = mrpt_graph_t::constraint_t;

    shared_ptr<PoseGraph> pg = make_shared<PoseGraph>();

    mrpt::math::CMatrixDouble66 cov = Eigen::Matrix<double,6,6>::Identity();

    // expectedPdf1 -> between node 0 (root) and 3
    // expectedPdf2 -> between node 2 and 5
    pose_pdf_t incrementPdf, globalPdf;
    pose_pdf_t expectedPdf1, expectedPdf2;
    expectedPdf1.cov_inv.unit();
    expectedPdf1.cov_inv *= 1e6;
    expectedPdf2.cov_inv.unit();
    expectedPdf2.cov_inv *= 1e6;

    incrementPdf.cov_inv = cov;
    incrementPdf.mean = mrpt::poses::CPose3D(1,0,0,0.2,0,0);

    // Node 1
    globalPdf = incrementPdf;
    pg->insertNode(globalPdf,incrementPdf);
    pg->insertEdge(0,1,incrementPdf);

    // Node 2
    globalPdf    += incrementPdf;
    pg->insertNode(globalPdf,incrementPdf);
    pg->insertEdge(1,2,incrementPdf);
    expectedPdf1 = globalPdf;

    // Node 3
    globalPdf    += incrementPdf;
    pg->insertNode(globalPdf,incrementPdf);
    pg->insertEdge(2,3,incrementPdf);
    expectedPdf2 += incrementPdf;

    // Node 4
    globalPdf    += incrementPdf;
    pg->insertNode(globalPdf,incrementPdf);
    pg->insertEdge(3,4,incrementPdf);
    expectedPdf2 += incrementPdf;

    // Node 5
    globalPdf    += incrementPdf;
    pg->insertNode(globalPdf,incrementPdf);
    pg->insertEdge(4,5,incrementPdf);
    expectedPdf2 += incrementPdf;

    pose_pdf_t computedPdf1 = pg->getRelativePosePdf(0,2),
               computedPdf2 = pg->getRelativePosePdf(2,5);

    if(!equalPosePdf(expectedPdf1,computedPdf1) ||
       !equalPosePdf(expectedPdf2,computedPdf2)) // Define a large enough epsilon
    {
        FAILED
        cout << "Expected pdfs : " << endl;
        cout << expectedPdf1 << endl;
        cout << expectedPdf2 << endl;
        cout << "Computated pdfs : " << endl;
        cout << computedPdf1 << endl;
        cout << computedPdf2 << endl;
        return -1;
    }
    else
    {
        PASSED
        return 0;
    }
}

/** Test that marginals are correctly computed */
int testMarginals()
{
    using namespace karst_slam::utils;
    using namespace karst_slam::graph;
    using namespace karst_slam::gso;
    using namespace mrpt::math;
    using mrpt_graph_t = mrpt::graphs::CNetworkOfPoses3DInf;
    using pose_pdf_t = mrpt_graph_t::constraint_t;

    // Expected values obtained with a GTSAM graph
    gtsam::Marginals expected_marginals = testGTSAMMarginals3D();

    // Recall that gtsam express 3D pose as [roll pitch yaw x y z] 
    // so convert back to MRPT format [x y z yaw pitch roll]
    vector<CMatrixDouble66> expected_marginals_covs;
    expected_marginals_covs.reserve(3);
    for(int i = 0; i < 3; i++) 
        expected_marginals_covs.push_back(expected_marginals.marginalCovariance(i+1));

    // Define the same graph with PoseGraph
    shared_ptr<PoseGraph> pg = make_shared<PoseGraph>();
    CMatrixDouble66 odo_cov = Eigen::Matrix<double,6,6>::Identity();
    constexpr double cov_val1 = 0.2*0.2, cov_val2 = 0.1*0.1, cov_val3= 0.001*0.001;
    odo_cov(0,0) = cov_val1; // x
    odo_cov(1,1) = cov_val1; // y
    odo_cov(2,2) = cov_val3; // z
    odo_cov(3,3) = cov_val2; // yaw
    odo_cov(4,4) = cov_val3; // pitch
    odo_cov(5,5) = cov_val3; // roll
    CMatrixDouble66 odo_cov_inv = odo_cov.inv();
    CPose3D odo(2.0, 0., 0., 0., 0., 0.); 

    // Root node is already created so add the two next nodes
    pose_pdf_t incrementPdf;
    incrementPdf.cov_inv = odo_cov_inv;
    incrementPdf.mean = odo;

    // Set pose value different to the GT (given with the incrementPDF)
    // Set the same values as in testGTSAMMarginals3D()
    pg->insertNode(pose_pdf_t(CPose3D(0.5, 0.0, 0.1, 0.2, 0.1, -0.2)), incrementPdf);
    pg->insertEdge(0, 1, incrementPdf);

    pg->insertNode(pose_pdf_t(CPose3D(2.3, 0.1, 0.1, -0.2, -0.1, -0.1)), incrementPdf);
    pg->insertEdge(1, 2, incrementPdf);

    // Optimize the poseGraph with GTSAM LM GSO (as in testGTSAMMarginals3D())
    LevenbergMarquardtGtsamGSO3D gso;
    gso.setPoseGraphPtr(pg);

    mrpt::obs::CActionCollectionPtr dummy_action;
    mrpt::obs::CSensoryFramePtr dummy_observations;
    mrpt::obs::CObservationPtr dummy_observation;
    mrpt::obs::CSensoryFramePtr dummy_generatedObservations;
    gso.updateState(dummy_action, dummy_observations, dummy_observation, dummy_generatedObservations);

    // Get the marginals from PoseGraph
    pg->updateMarginals();
    vector<CMatrixDouble66> pg_marginals;
    pg_marginals.reserve(3);
    for(int i = 0; i < 3; i++)
        pg_marginals.push_back(pg->getNodePosePdf(i).getCovariance());

    // Compare
    bool res = true; 
    for(int i = 0; i < 3; i++)
    {
        if(!compareCovGTSAM_MRPT(pg_marginals[i], expected_marginals_covs[i]))
        {   
            res = false;
            break;
        }
    }

    if(res)
    {
        PASSED
        return 0;
    }
    else
    {
        FAILED

        cout << "++++++++++++++++++++++++++++" << endl;
        cout << "+ Converted graph" << endl;
        cout << "++++++++++++++++++++++++++++" << endl;
        pg->printConvertedGraph();
        cout << "++++++++++++++++++++++++++++" << endl;
        cout << "++++++++++++++++++++++++++++" << endl;

        for(int i = 0; i < 3; i++)
        {   
            cout << "-------------- Node " <<  i << " -------------" << endl;
            cout << " PoseGraph node poses after optimization " << endl;
            cout << pg->getNodePose(i) << endl;

            cout << "-----" << endl;
            cout << "Marginal cov" << endl;
            cout << "Expected : " << endl;
            cout << expected_marginals_covs[i] << endl;
            cout << "Computed via PoseGraph: " << endl;
            cout << pg_marginals[i] << endl;
            cout <<  "-----" << endl;
        }
        return- 1;
    }
}
