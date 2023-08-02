#ifndef TESTGENERATEGRAPH_H
#define TESTGENERATEGRAPH_H

#include "karst_slam/poses.h"
#include <mrpt-gtsam/wrapperGTSAM/conversionPoses.h>
#include <mrpt-gtsam/wrapperGTSAM/conversionGraphs.h>
#include <mrpt-gtsam/wrapperGTSAM/conversionOptimizerParams.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/poses/CPose3DQuatPDFGaussianInf.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/inference/Symbol.h>
#include <cmath>
#include <random>

namespace karst_slam{namespace tests{namespace utils{

template<class MRPT_POSE_T>
void createRandomPose(MRPT_POSE_T& pose);

template<class MRPT_POSE_PDF_T>
void createRandomPosePDF(MRPT_POSE_PDF_T& posepdf);

template<>
void createRandomPose<mrpt::poses::CPose2D>(mrpt::poses::CPose2D& pose)
{
    using namespace std;
    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<> dis(-500, 500);
    pose = mrpt::poses::CPose2D( (double)dis(gen)/100.,
                                 (double)dis(gen)/100.,
                                 mrpt::utils::DEG2RAD(dis(gen)%360)
                                );
}

template<>
void createRandomPose<mrpt::poses::CPose3D>(mrpt::poses::CPose3D& pose)
{
    using namespace std;
    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<> dis(-500, 500);
    double x = (double)dis(gen)/100.;
    double y = (double)dis(gen)/100.;
    double z = (double)dis(gen)/100.;
    double yaw   = mrpt::utils::DEG2RAD(dis(gen)%360);
    double roll  = mrpt::utils::DEG2RAD(dis(gen)%360);
    double pitch = mrpt::utils::DEG2RAD(dis(gen)%360);

    pose = mrpt::poses::CPose3D(x,y,z,yaw,roll,pitch);
}

template<>
void createRandomPosePDF<mrpt::poses::CPosePDFGaussianInf>(mrpt::poses::CPosePDFGaussianInf& posepdf)
{
    using namespace std;
    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<> dis(1, 100);
    mrpt::math::CMatrixDouble33 infMat = mrpt::math::CMatrixDouble33::Zero();
    infMat(0,0) = (double)dis(gen)/1000.;
    infMat(1,1) = (double)dis(gen)/1000.;
    infMat(2,2) = (double)dis(gen)/1000.;

    mrpt::poses::CPose2D pose;
    createRandomPose<mrpt::poses::CPose2D>(pose);
    posepdf = mrpt::poses::CPosePDFGaussianInf(std::move(pose), move(infMat));
}

template<>
void createRandomPosePDF<mrpt::poses::CPose3DPDFGaussianInf>(mrpt::poses::CPose3DPDFGaussianInf& posepdf)
{
    using namespace std;
    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<> dis(1, 100);
    mrpt::math::CMatrixDouble66 infMat = mrpt::math::CMatrixDouble66::Zero();
    infMat(0,0) = (double)dis(gen)/1000.;
    infMat(1,1) = (double)dis(gen)/1000.;
    infMat(2,2) = (double)dis(gen)/1000.;
    infMat(3,3) = (double)dis(gen)/1000.;
    infMat(4,4) = (double)dis(gen)/1000.;
    infMat(5,5) = (double)dis(gen)/1000.;

    mrpt::poses::CPose3D pose;
    createRandomPose<mrpt::poses::CPose3D>(pose);
    posepdf = mrpt::poses::CPose3DPDFGaussianInf(move(pose), move(infMat));
}

template<>
void createRandomPosePDF<mrpt::poses::CPose3DPDFGaussian>(mrpt::poses::CPose3DPDFGaussian& posepdf)
{
    mrpt::poses::CPose3DPDFGaussianInf pose_pdf_inf;
    createRandomPosePDF<mrpt::poses::CPose3DPDFGaussianInf>(pose_pdf_inf);
    posepdf = mrpt::poses::CPose3DPDFGaussian(pose_pdf_inf.mean,
                                              pose_pdf_inf.getCovariance());
}

template<class POSE_T>
void addNoiseToPose(const std::vector<double>& noise,
                    POSE_T& pose) = delete;

template<>
void addNoiseToPose<mrpt::poses::CPose2D>(const std::vector<double>& noise,
                                            mrpt::poses::CPose2D& pose)
{
    pose += mrpt::poses::CPose2D(noise[0], noise[1], noise[2]);
}

template<>
void addNoiseToPose<mrpt::poses::CPose3D>(const std::vector<double>& noise,
                                            mrpt::poses::CPose3D& pose)
{
    pose += mrpt::poses::CPose3D(noise[0], noise[1], noise[2], noise[3], noise[4], noise[5]);
}

template<class MRPT_GRAPH_T, int N = 1, int ITER = 4>
void generateIncrementalGraph(mrpt_gtsam::wrapperGTSAM::gtsamGraph& gtsam_graph,
                              mrpt_gtsam::wrapperGTSAM::gtsamGraph& convertedGtsam_graph,
                              double& totalProcessTime)
{
    using namespace mrpt_gtsam::wrapperGTSAM;

    using mrpt_posepdf_t =  typename mrptToGTSAM_graphConverter<MRPT_GRAPH_T>::mrpt_edge_posePDF_t;
    using mrpt_pose_t    =  typename mrptToGTSAM_graphConverter<MRPT_GRAPH_T>::mrpt_node_pose_t;
    using gtsam_value_t  =  typename mrptToGTSAM_graphConverter<MRPT_GRAPH_T>::gtsam_value_t;
    using prior_t          = typename mrptToGTSAM_graphConverter<MRPT_GRAPH_T>::prior_factor_t;
    using between_t        = typename mrptToGTSAM_graphConverter<MRPT_GRAPH_T>::between_factor_t;

    std::shared_ptr<MRPT_GRAPH_T> mrpt_graph = std::make_shared<MRPT_GRAPH_T>();
    mrptToGTSAM_graphConverter<MRPT_GRAPH_T> graphConverter(mrpt_graph);

    totalProcessTime = 0.;
    mrpt_posepdf_t posepdf; // edge
    mrpt_pose_t pose; // node
    mrpt::utils::TNodeID nextId = 0;

    mrpt::utils::CTicTac tic;

    gtsam_value_t poseGtsam;
    for(int i = 0; i < ITER; i++)
    {
        mrpt::utils::TNodeID lastId = nextId;

        // Create a new node / value
        for(int j = 0; j < N; j++)
        {
            // MRPT node
            createRandomPose<mrpt_pose_t>(pose);
            mrpt_graph->nodes.insert(std::make_pair(nextId, pose));

            // Add equivalent Value to the corresponding gtsam graph
            poseGtsam = convertPose_mrptToGtsam(pose);
            gtsam_graph.addValue(nextId,poseGtsam);

            if(nextId == 0) // Default root node
            {
                gtsam_graph.addFactor(boost::make_shared<prior_t>(0,
                                                                  poseGtsam,
                                                                  gtsam::noiseModel::Constrained::All(gtsam_value_t::dimension)));
            }

            nextId++;
        }

        // Create new edges between the new and the previous nodes
        for(int j = lastId; j < nextId-1; j++)
        {
            // MRPT edge
            createRandomPosePDF<mrpt_posepdf_t>(posepdf);
            mrpt_graph->insertEdge(j, j+1, posepdf);

            // GTSAM factor
            gtsam_graph.addFactor(boost::make_shared<between_t>(j,
                                                                j+1,
                                                                convertPose_mrptToGtsam(posepdf),
                                                                mrptToGTSAM_graphConverter<MRPT_GRAPH_T>::convertNoiseModel(posepdf))
                                  );
        }

        // Update the convertion to gtsam graph
        tic.Tic();
        graphConverter.updateConvertedGraph();
        double convertProcessTime = tic.Tac();
        totalProcessTime += convertProcessTime;
        std::cout << "Update conversion done in " << convertProcessTime << " s" << std::endl;
        convertedGtsam_graph = gtsamGraph(graphConverter.getConvertedGTSAMGraph());
    }
}
template<class MRPT_GRAPH_T, int N>
std::shared_ptr<MRPT_GRAPH_T> generateFullGraph(mrpt_gtsam::wrapperGTSAM::gtsamGraph& gtsam_graph)
{
    using namespace mrpt_gtsam::wrapperGTSAM;

    using mrpt_posepdf_t   = typename mrptToGTSAM_graphConverter<MRPT_GRAPH_T>::mrpt_edge_posePDF_t;
    using mrpt_pose_t      = typename mrptToGTSAM_graphConverter<MRPT_GRAPH_T>::mrpt_node_pose_t;
    using prior_t          = typename mrptToGTSAM_graphConverter<MRPT_GRAPH_T>::prior_factor_t;
    using between_t        = typename mrptToGTSAM_graphConverter<MRPT_GRAPH_T>::between_factor_t;
    using gtsam_t          = typename mrptToGTSAM_graphConverter<MRPT_GRAPH_T>::gtsam_value_t;

    std::shared_ptr<MRPT_GRAPH_T> mrpt_graph = std::make_shared<MRPT_GRAPH_T>();

    mrpt_posepdf_t posepdf; // edge
    mrpt_pose_t pose; // node

    gtsam_t gtsamPose;
    for(int i = 0; i < N; i++)
    {
        // MRPT node
        createRandomPose<mrpt_pose_t>(pose);
        mrpt_graph->nodes.insert(std::make_pair(i, pose));

        // Add equivalent Value to the corresponding gtsam graph
        gtsamPose = convertPose_mrptToGtsam(pose);
        gtsam_graph.addValue(i, gtsamPose);

        // Create new edges between the new and the previous nodes
        if(i > 0)
        {
            // MRPT edge
            createRandomPosePDF<mrpt_posepdf_t>(posepdf);
            mrpt_graph->insertEdge(i-1, i, posepdf);

            // GTSAM factor
            gtsam_graph.addFactor(boost::make_shared<between_t>(i-1,
                                                                i,
                                                                convertPose_mrptToGtsam(posepdf),
                                                                mrptToGTSAM_graphConverter<MRPT_GRAPH_T>::convertNoiseModel(posepdf))
                                  );
        }
        else
        {
            gtsam_graph.addFactor(boost::make_shared<prior_t>(0,
                                                              gtsamPose, //gtsam_t(),
                                                              gtsam::noiseModel::Constrained::All(gtsam_t::dimension)));
        }

    }

    return mrpt_graph;
}

template<class MRPT_GRAPH_T,typename std::enable_if<mrpt_gtsam::wrapperGTSAM::is_2D_pose<typename MRPT_GRAPH_T::constraint_no_pdf_t>::value,int>::type = 0>
mrpt_gtsam::wrapperGTSAM::gtsamGraph createRealisticGraph()
{
    using namespace gtsam;
    using namespace std;

    NonlinearFactorGraph graph;

    // No equivalent to prior factor in mrpt (nodes are not represented by a pdf)
    // May be possible to fake it by adding an edge to and from the same node ?
    // To match the root node of mrpt graph, we put a constrained noise with zero sigmas

    noiseModel::Constrained::shared_ptr priorNoise = noiseModel::Constrained::All(3);
    graph.emplace_shared<PriorFactor<Pose2> >(0, Pose2(0, 0, 0), priorNoise);

    /*
    noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
    graph.emplace_shared<PriorFactor<Pose2> >(1, Pose2(0, 0, 0), priorNoise);
    */

    noiseModel::Diagonal::shared_ptr model = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
    graph.emplace_shared<BetweenFactor<Pose2> >(0, 1, Pose2(2, 0, 0     ), model);
    graph.emplace_shared<BetweenFactor<Pose2> >(1, 2, Pose2(2, 0, M_PI_2), model);
    graph.emplace_shared<BetweenFactor<Pose2> >(2, 3, Pose2(2, 0, M_PI_2), model);
    graph.emplace_shared<BetweenFactor<Pose2> >(3, 4, Pose2(2, 0, M_PI_2), model);
    graph.emplace_shared<BetweenFactor<Pose2> >(4, 1, Pose2(2, 0, M_PI_2), model);

    Values initialEstimate;
    initialEstimate.insert(0, Pose2(0.0, 0.0,  0.0   ));
    initialEstimate.insert(1, Pose2(2.3, 0.1, -0.2   ));
    initialEstimate.insert(2, Pose2(4.1, 0.1,  M_PI_2));
    initialEstimate.insert(3, Pose2(4.0, 2.0,  M_PI));
    initialEstimate.insert(4, Pose2(2.1, 2.1, -M_PI_2));

    return mrpt_gtsam::wrapperGTSAM::gtsamGraph(std::move(graph), std::move(initialEstimate));
}

template<class MRPT_GRAPH_T,typename std::enable_if<!mrpt_gtsam::wrapperGTSAM::is_2D_pose<typename MRPT_GRAPH_T::constraint_no_pdf_t>::value,int>::type = 0>
mrpt_gtsam::wrapperGTSAM::gtsamGraph createRealisticGraph()
{
    using namespace gtsam;
    using namespace std;

    NonlinearFactorGraph graph;

    // No equivalent to prior factor in mrpt (nodes are not represented by a pdf)
    // May be possible to fake it by adding an edge to and from the same node ?
    // To match the root node of mrpt graph, we put a constrained noise with zero sigmas

    noiseModel::Constrained::shared_ptr priorNoise = noiseModel::Constrained::All(6);
    graph.emplace_shared<PriorFactor<Pose3> >(0, Pose3(), priorNoise);

    /*
    noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
    graph.emplace_shared<PriorFactor<Pose2> >(1, Pose2(0, 0, 0), priorNoise);
    */

    noiseModel::Diagonal::shared_ptr model = noiseModel::Diagonal::Sigmas((Vector(6) << 0.2, 0.2, 0.1, 0.01, 0.01, 0.01).finished());
    graph.emplace_shared<BetweenFactor<Pose3> >(0, 1, Pose3(Rot3(), Point3(2, 0, 0)), model);
    graph.emplace_shared<BetweenFactor<Pose3> >(1, 2, Pose3(Rot3::Rz(M_PI_2), Point3(2, 0, 0)), model);
    graph.emplace_shared<BetweenFactor<Pose3> >(2, 3, Pose3(Rot3::Rz(M_PI_2), Point3(2, 0, 0)), model);
    graph.emplace_shared<BetweenFactor<Pose3> >(3, 4, Pose3(Rot3::Rz(M_PI_2), Point3(2, 0, 0)), model);
    graph.emplace_shared<BetweenFactor<Pose3> >(4, 1, Pose3(Rot3::Rz(M_PI_2), Point3(2, 0, 0)), model);

    Values initialEstimate;
    initialEstimate.insert(0, Pose3());
    initialEstimate.insert(1, Pose3(Rot3::Rz(-0.2), Point3(2.3, 0.1, -0.1)));
    initialEstimate.insert(2, Pose3(Rot3::Rz(M_PI_2), Point3(4.1, 0.1, -0.1)));
    initialEstimate.insert(3, Pose3(Rot3::Rz(M_PI), Point3(4.0, 2.0, 0.1)));
    initialEstimate.insert(4, Pose3(Rot3::Rz(-M_PI_2), Point3(2.1, 2.1, 0.1)));

    return mrpt_gtsam::wrapperGTSAM::gtsamGraph(std::move(graph), std::move(initialEstimate));
}
}}} // end namespaces
#endif // TESTGENERATEGRAPH_H
