#include "karst_slam/sensors/Scan.h"
#include "karst_slam/simulation/simulationEngine.h"

#include "karst_slam/simulation/iglModel.h"
#include "karst_slam/scanMerging/gui/scanMergingViewer_mrpt.h"

#include "karst_slam/scanMerging/curvilinearAbscissa_SE3.h"
#include "karst_slam/simulation/observationSimulator_sonar.h"
#include "karst_slam/scanMerging/rangeMapping.h"
#include <igl/embree/EmbreeIntersector.h>
#include <igl/point_mesh_squared_distance.h>
#include <mrpt/system/threads.h>
#include <mrpt/poses.h>
#include <mrpt/utils.h>
#include <mrpt/math.h>
#include <pid/rpath.h>
#include <boost/math/distributions/beta.hpp>

using namespace std;
using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace karst_slam::sensors;
using namespace karst_slam::simulation;
using namespace karst_slam::scanMerging;
using namespace karst_slam::gui;

/**
 * Application used to simulate vertical and horizontal sonar observations in a karst by a moving robot
 * As of 17/01/20, it is used to test the scan merging algorithm
 *
 * Simulation and scan merging parameters are defined in a config file .ini.
 *
 * \todo After the testing, need a refactoring to separate simulation (generation of sonar observations) and scan merging.
 */

void screenshotForPaper(simulationViewer_mrpt& viewer)
{
    // note : MRPT functions saveToFile not working ...
    double zoom_sparse = 15, zoom_sparse_oblique = 20;
    double zoom_dense = 100, zoom_dense_oblique = 110;
    double zoom_rough_sparse = 150, zoom_rough_oblique = 160;

    double zoom = zoom_sparse, zoom_oblique = zoom_sparse_oblique;

    // Top view
    viewer.setCamera(0,90,zoom,
                     3,0,0); // 20,0,0 for rough
    system("gnome-screenshot -w -B -d 1");

    // Side view
    viewer.setCamera(90,0,zoom,
                     3,0,0);
    system("gnome-screenshot -w -B -d 1");

    // In between view
    viewer.setCamera(30,30,zoom_oblique,
                     3,0,0);
    system("gnome-screenshot -w -B -d 1");

    // For the zoom with missing data
    //    viewer.setCamera(0,0,60,
    //                     0,-20,-5);
    //    system("gnome-screenshot -w -B -d 1");
}


void scoreResults(const shared_ptr<dataForScanMerging>& data,
                  const scanMergingResults& res,
                  const shared_ptr<iglModel>& envModel)
{
    using namespace std;
    using namespace mrpt::poses;

    double beamWidth = DEG2RAD(35);

    const vector<vector<pointThetaSimu>>& ptsOnArc = data->horizontalSonarPoints_thetaVals;
    const map<int, vector<distributionTheta>>& distributions = res.distribution;

    int NtotalArc = 0, pt_idx = 0, nThetaSamples;
    vector<double> dist_prior, dist_posterior;
    for(const vector<pointThetaSimu>& arc : ptsOnArc)
    {
        nThetaSamples = arc.size();

        // Compute distances to mesh of all sampled points on the arc
        Eigen::MatrixXd P(nThetaSamples,3), C(nThetaSamples,3), sqr_dist(nThetaSamples,1);
        Eigen::MatrixXi I(nThetaSamples,1);
        int i = 0;
        for(const pointThetaSimu& pt : arc)
        {
            P(i,0) = pt.point.x();
            P(i,1) = pt.point.y();
            P(i,2) = pt.point.z();
            i++;
        }
        igl::point_mesh_squared_distance(P, // Points for which we want to compute distance with mesh
                                         envModel->Vertices, // Mesh vertices
                                         envModel->Faces, // Mesh faces (elements)
                                         sqr_dist, // List of smallest squared distances for each point
                                         I, // Index of primitives for which smallest distance is obtained
                                         C); // List of closest points on the mesh

        // Get the distributions related to the current arc
        if(distributions.find(pt_idx) != distributions.end())
        {
            const vector<distributionTheta>& dists = distributions.at(pt_idx);

            // only compare if not uniform
            if(!(dists.size() == 1 &&
                 dists[0].alpha == 1 &&
                 dists[0].beta == 1))
            {

                int nDistInCurrentArc = dists.size();
                cout << "pt_idx = " << pt_idx << endl;
                cout << "Number of dist relative to the arc : " << nDistInCurrentArc << endl;

                double dist_expectation_prior = 0.;
                // Distance to mesh expectation for uniform distribution (prior)
                // It is just the mean of distances
                for(int i = 0; i < nThetaSamples; i++)
                    dist_expectation_prior += sqrt(sqr_dist(i));
                dist_expectation_prior /= (double)nThetaSamples;
                dist_prior.push_back(dist_expectation_prior);
                cout << " --> dist_prior = " << dist_expectation_prior << endl;

                // Distance to mesh expectation for estimated distribution (posterior)
                // One for each distribution on the arc
                double x, curPdf, totalPdf = 0.;
                for(const distributionTheta& dist : dists)
                {
                    double curDist_expectation_posterior = 0.;
                    totalPdf = 0.;
                    for(int j = 0; j < nThetaSamples; j++)
                    {
                        x = (arc[j].theta + 0.5*beamWidth)/beamWidth;
                        curPdf = boost::math::pdf(boost::math::beta_distribution<>(dist.alpha,dist.beta),x);
                        curDist_expectation_posterior += sqrt(sqr_dist(j))*curPdf;
                        totalPdf += curPdf;
                    }
                    curDist_expectation_posterior /= totalPdf;
                    dist_posterior.push_back(curDist_expectation_posterior);
                    cout << "  --> dist_posterior = " << curDist_expectation_posterior << endl;
                }
            }
            NtotalArc++;
        }

        pt_idx++;
    }

    cout << "---------> Total arcs : " << NtotalArc << endl;
    cout << " --------> Non uniform arc : " << dist_posterior.size() << endl;

    // Save
    ofstream filePrior("/home/breux/Exp/scoreResults_prior.txt"),
            filePosterior("/home/breux/Exp/scoreResults_posterior.txt"),
            fileProportionArc("/home/breux/Exp/prop_arcs.txt");
    if(filePrior.is_open() &&
       filePosterior.is_open() &&
       fileProportionArc.is_open())
    {
        for(int i = 0; i <  dist_posterior.size(); i++)
            filePosterior << dist_posterior[i] << endl;
        for(int i = 0; i <  dist_prior.size(); i++)
             filePrior << dist_prior[i] << endl;

        fileProportionArc << dist_posterior.size() << endl;
        fileProportionArc.close();
        filePrior.close();
        filePosterior.close();
    }
    else
        cout << "Can't save results !!!" << endl;
}

// Generate contours along XY, XZ and XZ planes
// use for figures in paper
void generateModelSlices(const shared_ptr<igl::embree::EmbreeIntersector>& embree,
                         const std::vector<double>& curvilinearAbscissa)
{
    igl::Hit hit;
    int n = curvilinearAbscissa.size();
    int middle = n/2;
    cout << "Middle for generateModelSlices : " << middle << endl;

    vector<double> XY_ordinate, XZ_ordinate, ZY_ordinate,ZY_abscisse;

    // ZY
    Eigen::Vector3f raySource_ZY,
                    rayDir_ZY;
    raySource_ZY << curvilinearAbscissa[middle],0.,0.;
    double yaw_rad, cos_y, sin_y;
    for(int yaw = 0; yaw < 360; yaw ++)
    {
        yaw_rad = DEG2RAD(yaw);
        cos_y = cos(yaw_rad);
        sin_y = sin(yaw_rad);

        rayDir_ZY << 0., cos_y, sin_y;
        embree->intersectRay(raySource_ZY,
                             rayDir_ZY,
                             hit);
        ZY_abscisse.push_back(hit.t*cos_y);
        ZY_ordinate.push_back(hit.t*sin_y);
    }

    Eigen::Vector3f rayDir_XY_up,
                    rayDir_XY_down,
                    rayDir_XZ_up,
                    rayDir_XZ_down;
    rayDir_XY_up << 0.,1.,0;
    rayDir_XY_down << 0., -1.,0.;
    rayDir_XZ_up << 0.,0.,1.;
    rayDir_XZ_down << 0.,0.,-1.;
    for(const double& s : curvilinearAbscissa)
    {
        Eigen::Vector3f raySource;
        raySource << s, 0.,0.;vector<CPose3DPDFGaussian> bodyPoses;

        // XY
        embree->intersectRay(raySource,
                             rayDir_XY_up,
                             hit);
        XY_ordinate.push_back(hit.t);
        embree->intersectRay(raySource,
                             rayDir_XY_down,
                             hit);
        XY_ordinate.push_back(-hit.t);

        // XZ
        embree->intersectRay(raySource,
                             rayDir_XZ_up,
                             hit);
        XZ_ordinate.push_back(hit.t);
        embree->intersectRay(raySource,
                             rayDir_XZ_down,
                             hit);
        XZ_ordinate.push_back(-hit.t);
    }

    // Save to files
    ofstream file_XY("XY_modelSlice.txt"), file_XZ("XZ_modelSlice.txt"), file_ZY("ZY_modelSlice.txt");
    if(file_XY.is_open() &&
       file_XZ.is_open() &&
       file_ZY.is_open())
    {
        double s, y, z;
        for(int i = 0; i< curvilinearAbscissa.size(); i++)
        {
            s = curvilinearAbscissa[i];
            y = XY_ordinate[i];
            z = XZ_ordinate[i];
            file_XY << s << "," << XY_ordinate[2.*i] << "\n";
            file_XY << s << "," << XY_ordinate[2.*i + 1] << "\n";
            file_XZ << s << "," << XZ_ordinate[2.*i]  << "\n";
            file_XZ << s << "," << XZ_ordinate[2.*i + 1] << "\n";
        }

        for(int i = 0; i < ZY_abscisse.size(); i++)
        {
           y = ZY_abscisse[i];
           z = ZY_ordinate[i];
           file_ZY << y << "," << z << "\n";
        }

        file_XY.close();
        file_XZ.close();
        file_ZY.close();
    }
    else
        cout << "Error : can't save model slices files ! " << endl;
}

vector<CPose3DPDFGaussian> loadOdometry(const string& fileName)
{
    vector<CPose3DPDFGaussian> odometry;
    string line, val;
    double ypr[3];
    ifstream file(fileName);
    if(file.is_open())
    {
        getline(file,line); // skip header
        while(getline(file,line))
        {
            stringstream ss(line);
            CPose3DPDFGaussian odo;

            // x, y, z
            for(int i = 0 ; i < 3; i++)
            {
                getline(ss,val,',');
                odo.mean.m_coords[i] = stod(val);
            }

            // Yaw, pitch, roll
            for(int i = 0; i < 3; i++)
            {
                getline(ss,val,',');
                ypr[i] = stod(val);
            }
            odo.mean.setYawPitchRoll(ypr[0], ypr[1], ypr[2]);

            // Covariance
            for(int i = 0; i < 6; i++)
            {
                for(int j = 0; j < 6; j++)
                {
                    getline(ss,val,',');
                    odo.cov(i,j) = stod(val);
                }
            }
            odometry.push_back(odo);

        }
        file.close();
    }
    else
        cout << "Can't open file " << fileName << " !" << endl;

    return odometry;
}

vector<double> loadCurvilinearAbscissa(const string& fileName)
{
    vector<double> curvilinearAbscissa;

    ifstream file(fileName);
    string line;
    if(file.is_open())
    {
        while(getline(file,line))
            curvilinearAbscissa.push_back(stod(line));

        file.close();
    }
    else
        cout << "Can't open file " << fileName << " !" << endl;

    return curvilinearAbscissa;
}

vector<horizontalSonarMeasure> loadHorizontalSonarMeasures(const string& fileName)
{
    vector<horizontalSonarMeasure> measures;
    horizontalSonarMeasure meas;
    double robotPose[6];
    ifstream file(fileName);
    string line, val;
    if(file.is_open())
    {
        getline(file,line); // skip header
        while(getline(file,line))
        {
            stringstream ss(line);

            // scanIdx
            getline(ss, val, ',');
            meas.scanIdx = stoi(val);

            // poseIdxInScan
            getline(ss, val, ',');
            meas.timeStamp = stoi(val);

            //rho
            getline(ss, val, ',');
            meas.rho = stod(val);

            // yaw
            getline(ss, val, ',');
            meas.yaw = stod(val);

            // RobotGlobalPosePdf
            for(int i = 0; i < 6 ; i++)
            {
                getline(ss, val, ',');
                robotPose[i] = stod(val);
            }
            meas.robotGlobalPosePdf.mean = CPose3D(robotPose[0], robotPose[1] , robotPose[2],
                                                   robotPose[3], robotPose[4] , robotPose[5]);
            for(int i = 0; i < 6; i++)
            {
                for(int j = 0; j < 6; j++)
                {
                    getline(ss, val, ',');
                    meas.robotGlobalPosePdf.cov(i,j) = stod(val);
                }
            }
            measures.push_back(meas);

        }
        file.close();
    }
    else
        cout << "Can't open file " << fileName << " !" << endl;

    return measures;
}

vector<CPointPDFGaussian> loadVerticalSonarPoints(const string& fileName)
{
    vector<CPointPDFGaussian> points;
    CPointPDFGaussian pt;
    double pt_mean[3];
    ifstream file(fileName);
    string line, val;
    if(file.is_open())
    {
        getline(file,line); // skip header
        while(getline(file,line))
        {
            stringstream ss(line);

            for(int i = 0; i < 3 ; i++)
            {
                getline(ss, val, ',');
                pt_mean[i] = stod(val);
            }
            pt.mean = CPoint3D(pt_mean[0], pt_mean[1], pt_mean[2]);

            for(int i = 0; i < 3 ;i++)
            {
                for(int j = 0; j < 3; j++)
                {
                    getline(ss, val, ',');
                    pt.cov(i,j) = stod(val);
                }
            }

            points.push_back(pt);
        }
        file.close();
    }
    else
        cout << "Can't open file " << fileName << " !" << endl;

    return points;
}

vector<CPoint3D> loadHorizontalSonarPoints(const string& fileName)
{
    vector<CPoint3D> points;
    double pt[3];
    ifstream file(fileName);
    string line, val;
    if(file.is_open())
    {
        getline(file,line); // skip header
        while(getline(file,line))
        {
            stringstream ss(line);

            for(int i = 0; i < 3 ; i++)
            {
                getline(ss, val, ',');
                pt[i] = stod(val);
            }

            points.push_back(CPoint3D(pt[0], pt[1], pt[2]));
        }
        file.close();
    }
    else
        cout << "Can't open file " << fileName << " !" << endl;

    return points;
}

map<uint64_t, trajectoryPose<CPose3DPDFGaussian>> loadBodyPoses(const string& folder)
{
    map<uint64_t, trajectoryPose<CPose3DPDFGaussian>> res;

    string bodyPosesFileName = folder +  "/bodyPoses.txt",
                bodyPosesFileName_gt = folder +  "/bodyPoses_gt.txt",
                curvilinearAbscissaFileName = folder + "/curvilinearAbscissa.txt";
    ifstream fileBodyPoses(bodyPosesFileName),
                  fileBodyPoses_gt(bodyPosesFileName_gt),
                  fileCurvilinearAbscissa(curvilinearAbscissaFileName);
    
    CPose3DPDFGaussian curPose;
    double pose[6];
    vector<CPose3DPDFGaussian> bodyPoses;
    vector<uint64_t> timeStamps;
    string line, val;
    if(fileBodyPoses.is_open())
    {
        getline(fileBodyPoses,line); // skip header

        while(getline(fileBodyPoses,line))
        {
            stringstream ss(line);

            // time stamp
            getline(ss, val, ',');
            timeStamps.push_back(stoull(val));

            for(int i = 0; i < 6 ; i++)
            {
                getline(ss, val, ',');
                pose[i] = stod(val);
            }
            curPose.mean = CPose3D(pose[0],pose[1],pose[2],
                                   pose[3],pose[4],pose[5]);

            for(int i = 0; i <6 ; i++)
            {
                for(int j = 0; j < 6 ; j++)
                {
                    getline(ss, val, ',');
                    curPose.cov(i,j) = stod(val);
                }
            }

            bodyPoses.push_back(curPose);
        }
        fileBodyPoses.close();
    }

    vector<CPose3DPDFGaussian> bodyPoses_gt;
    if(fileBodyPoses_gt.is_open())
    {
        getline(fileBodyPoses_gt,line); // skip header

        while(getline(fileBodyPoses_gt,line))
        {
            stringstream ss(line);

            getline(ss, val, ','); // skip time stamp
            for(int i = 0; i < 6 ; i++)
            {
                getline(ss, val, ',');
                pose[i] = stod(val);
            }
            curPose.mean = CPose3D(pose[0],pose[1],pose[2],
                                   pose[3],pose[4],pose[5]);

            for(int i = 0; i <6 ; i++)
            {
                for(int j = 0; j < 6 ; j++)
                {
                    getline(ss, val, ',');
                    curPose.cov(i,j) = stod(val);
                }
            }

            bodyPoses_gt.push_back(curPose);
        }
        fileBodyPoses_gt.close();
    }

    vector<double> ca;
    if(fileCurvilinearAbscissa.is_open())
    {
        getline(fileCurvilinearAbscissa,line); // skip header
        while(getline(fileCurvilinearAbscissa,line))
        {
            stringstream ss(line);
            getline(ss, val, ','); // skip time stamp

            getline(ss, val, ',');
            ca.push_back(stod(val));
        }

        fileCurvilinearAbscissa.close();
    }

    if(ca.size() == bodyPoses.size() && bodyPoses.size() == bodyPoses_gt.size())
    {
        for(int i = 0 ; i < ca.size(); i++)
        {
            res[timeStamps[i]] = trajectoryPose<CPose3DPDFGaussian>(bodyPoses[i], bodyPoses_gt[i]);
        }
    }
    else
        cout << "[loadBodyPoses] Error differents sizes for bodyPose, bodyPoses_gt and curvilinear Data" << endl;

    return res;
}


vector<vector<pointThetaSimu>> loadhorizontalSonarThetaPoints(const string& fileName)
{
    vector<vector<pointThetaSimu>> res;
    vector<pointThetaSimu> curArc;
    pointThetaSimu pt;
    ifstream file(fileName);
    string line, val;
    int curArcIdx, prevArcIdx = -1;
    if(file.is_open())
    {
        getline(file,line); // skip header
        while(getline(file,line))
        {
            stringstream ss(line);

            getline(ss, val, ',');
            curArcIdx = stoi(val);

            if(curArcIdx != prevArcIdx &&  prevArcIdx != -1)
            {
                res.push_back(curArc);
                curArc.clear();
            }
            prevArcIdx = curArcIdx;

            for(int i = 0; i < 3; i++)
            {
                getline(ss, val, ',');
                pt.point.m_coords[i] = stod(val);
            }

            getline(ss, val, ',');
            pt.theta = stod(val);

            getline(ss, val, ',');
            pt.theta_idx = stoi(val);

            curArc.push_back(pt);

        }
        file.close();
    }
    else
        cout << "Can't open file " << fileName << " !" << endl;

    return res;
}

shared_ptr<dataForScanMerging> loadData(const CConfigFile& cfg)
{
    shared_ptr<dataForScanMerging> data = make_shared<dataForScanMerging>();
    string dataFolder = cfg.read_string("General", "dataFolder", "", true);
    data->isLearningMeanFunc = cfg.read_bool("GaussianProcess","learnMeanFunction",false, true);

    data->odometry = loadOdometry(dataFolder + "/odometry.txt");
    //data->bodyPoses_curvilinearAbscissa = loadCurvilinearAbscissa(dataFolder + "/curvilinearAbscissa.txt");
    data->horizontalData.load(dataFolder + "/surfaceValidationData.txt");
    data->trainingData.load(dataFolder + "/surfaceTrainingData.txt");
    data->horizontalSonarMeasures = loadHorizontalSonarMeasures(dataFolder + "/horizontalSonarMeasures.txt");
    data->verticalSonarPoints = loadVerticalSonarPoints(dataFolder + "/verticalSonarPoints.txt");
    data->horizontalSonarPoints = loadHorizontalSonarPoints(dataFolder + "/horizontalSonarPoints.txt");
    //data->posesWithAbscissaAtTimeStamp = loadPosesWithAbscissaAtTimeStamp(dataFolder + "/posesWithAbscissaAtTimeStamp.txt");
    data->priorCylinder.load(dataFolder + "/priorCylinder.txt");

    vector<double> verticalSonarPose, horizontalSonarPose;
    cfg.read_vector<vector<double>>("SonarVertical", "sensorPoseOnRobot",vector<double>(),verticalSonarPose, true);
    data->verticalSonarPoseOnRobot = CPose3D(verticalSonarPose[0], verticalSonarPose[1], verticalSonarPose[2],
                                             DEG2RAD(verticalSonarPose[3]), DEG2RAD(verticalSonarPose[4]), DEG2RAD(verticalSonarPose[5]));

    cfg.read_vector<vector<double>>("SonarHorizontal", "sensorPoseOnRobot",vector<double>(),horizontalSonarPose, true);
    data->horizontalSonarPoseOnRobot = CPose3D(horizontalSonarPose[0], horizontalSonarPose[1], horizontalSonarPose[2],
                                             DEG2RAD(horizontalSonarPose[3]), DEG2RAD(horizontalSonarPose[4]), DEG2RAD(horizontalSonarPose[5]));
    data->bodyPoses = loadBodyPoses(dataFolder);
    data->horizontalSonarPoints_thetaVals = loadhorizontalSonarThetaPoints(dataFolder + "/horizontalSonarThetaVals.txt");

    return data;
}

void generateSonarSimulator(const CConfigFile& cfg,
                            shared_ptr<observationSimulator_sonar>& verticalSonarSimulator,
                            shared_ptr<observationSimulator_sonar>& horizontalSonarSimulator)
{
    // Get the random seed generator
    uint32_t randSeed = (uint32_t)cfg.read_int("Simulation", "randomSeed", 0,true);
    cout << "randSeed : " << randSeed << endl;
    mrpt::random::Randomize(randSeed);
    //std::cout<<"Test rng :" << mrpt::random::randomGenerator.drawGaussian1D_normalized() << std::endl;
    //mrpt::random::randomGenerator.randomize(randSeed);

    // Load an environnement model (made with blender for example)
    shared_ptr<iglModel> envModel = make_shared<iglModel>(cfg);

    // Embree is used for ray tracing
    shared_ptr<igl::embree::EmbreeIntersector> embree = make_shared<igl::embree::EmbreeIntersector>();
    embree->init(envModel->Vertices.cast<float>(), envModel->Faces);

    // Construct the virtual sonars
    verticalSonarSimulator = make_shared<observationSimulator_sonar>(embree,
                                                                     "SonarVertical",
                                                                     cfg);

    horizontalSonarSimulator = make_shared<observationSimulator_sonar>(embree,
                                                                       "SonarHorizontal",
                                                                       cfg);
}

/*shared_ptr<dataForScanMerging> simulateData(const CConfigFile& cfg)
{
    //----------------------
    // Simulation parameters
    //----------------------
    shared_ptr<observationSimulator_sonar> verticalSonarSimulator, horizontalSonarSimulator; 
    generateSonarSimulator(cfg, verticalSonarSimulator, horizontalSonarSimulator);

    // Start the simulation !
    cout << "#########################" << endl;
    cout << "Create simulation Engine " << endl;
    cout << "#########################" << endl;
    simulationEngine simEngine(cfg, verticalSonarSimulator, horizontalSonarSimulator);

    return simEngine.generateSimulatedData();
}*/

dataForScanMerging_loopClosure simulateData_loopClosure_allScans(const CConfigFile& cfg,
                                                        shared_ptr<simulationViewer_mrpt> viewer,
                                                        shared_ptr<surfaceGP> surfaceEstimator)
{
    //----------------------
    // Simulation parameters
    //----------------------
    shared_ptr<observationSimulator_sonar> verticalSonarSimulator, horizontalSonarSimulator; 
    generateSonarSimulator(cfg, verticalSonarSimulator, horizontalSonarSimulator);

    // Start the simulation !
    cout << "#########################" << endl;
    cout << "Create simulation Engine " << endl;
    cout << "#########################" << endl;
    simulationEngine simEngine(cfg, verticalSonarSimulator, horizontalSonarSimulator);
    simEngine.setCallbackAtScanGeneration(std::bind(&simulationViewer_mrpt::render, viewer, placeholders::_1, placeholders::_2));

    return simEngine.generateSimulatedData_LoopClosure_allScans(surfaceEstimator); //simEngine.generateSimulatedData_LoopClosure();
}

/*dataForScanMerging_loopClosure simulateData_loopClosure(const CConfigFile& cfg)
{
    //----------------------
    // Simulation parameters
    //----------------------
    shared_ptr<observationSimulator_sonar> verticalSonarSimulator, horizontalSonarSimulator; 
    generateSonarSimulator(cfg, verticalSonarSimulator, horizontalSonarSimulator);

    // Start the simulation !
    cout << "#########################" << endl;
    cout << "Create simulation Engine " << endl;
    cout << "#########################" << endl;
    simulationEngine simEngine(cfg, verticalSonarSimulator, horizontalSonarSimulator);

    return simEngine.generateSimulatedData_LoopClosure();
}*/

scanMergingResults processScanData(const shared_ptr<surfaceGP> & surfaceEstimator,
                                   const string& saveFolder,
                                   bool isSimulation,
                                   const shared_ptr<dataForScanMerging>& data)
{
    scanMergingResults res;
    if(data != nullptr)
    {
        // Estimate the 3D points
        surfaceEstimator->setDataFolder(saveFolder);
        res = surfaceEstimator->estimateDistributions(data);
        data->save(saveFolder, isSimulation);
        data->saveHorizontalMeasureLocalSphericalCoords(saveFolder, res);
    }
    else
    {
        MRPT_UNSCOPED_LOGGER_START;
        MRPT_LOG_ERROR_STREAM("[mergingScanPICP::processScanData] Scan data null ptr");
        MRPT_UNSCOPED_LOGGER_END;
    }   

    return res;
}

/*void mergingScan_successive(const string& dataFolder,
                            const CConfigFile& cfg,
                            bool isSimulation)
{
    shared_ptr<dataForScanMerging> data = nullptr;
    if (isSimulation)
    {
        cout << " ---> Simulate Data ..." << endl;
        data = simulateData(cfg);
        cout << " ---> Simulation Done." << endl;
    }
    else
    {
        cout << " --> Load Data ..." << endl;
        data = loadData(cfg);
    }

    cout << "data->verticalSonarPoints size : " << data->verticalSonarPoints.size() << endl;
    shared_ptr<surfaceGP> surfaceEstimator = make_shared<surfaceGP>(cfg);
    scanMergingResults res = processScanData(surfaceEstimator, dataFolder, isSimulation, data);

    // Measure the effectiveness of the method (for paper)
    //scoreResults(data, envModel);

    // View / Drawing
    // The first version used the libigl viewer but it is quite difficult to custom
    // It is better to use the viewer based on mrpt.
    simulationViewer_mrpt viewer("scanMergingViewer", isSimulation);
    viewer.init(cfg);
    viewer.resize(1920, 1080);
    viewer.render(data, res);

    // Only to generate the contour for paper
    //generateModelSlices(embree, data->bodyPoses_curvilinearAbscissa);

    //screenshotForPaper(viewer);

    // Loop events
    while (!viewer.isExit())
    {
        viewer.queryKeyboardEvents();
        mrpt::system::sleep(10);
    }
}*/

// Merge successive scan AND the final loop closure
void mergingScan_karstDonut(const string& dataFolder,
                            const CConfigFile& cfg)
{
    shared_ptr<simulationViewer_mrpt> viewer = make_shared<simulationViewer_mrpt>("scanMergingViewer", true /* isSimulation */) ;
    viewer->init(cfg);
    viewer->resize(1920, 1080);

    // Construct the surface estimator which use a Gaussian process trained on the vertical sonar measurements
    shared_ptr<surfaceGP> surfaceEstimator = make_shared<surfaceGP>(cfg);

    cout << " ---> Simulate Data ..." << endl;
    dataForScanMerging_loopClosure data = simulateData_loopClosure_allScans(cfg, viewer, surfaceEstimator);
    cout << " ---> Simulation Done." << endl;


    // Estimate the 3D points
    //scanMergingResults res_firstScan = processScanData(surfaceEstimator, 
    //                                                   dataFolder + "/loopClosure/firstScan" /*saveFolder*/, 
    //                                                  true /*isSimulation*/, 
    //                                                   data.firstScanGrp);
    //scanMergingResults res_loopClosureScan = processScanData(surfaceEstimator,
    //                                                        dataFolder + "/loopClosure/loopClosureScan" /*saveFolder*/,
    //                                                         true /*isSimulation*/,
    //                                                         data.loopClosureScanGrp);                                          

    //data.save(dataFolder + "/loopClosure",
    //          res_firstScan,
    //          res_loopClosureScan);

    // View / Drawing
    // The first version used the libigl viewer but it is quite difficult to custom
    // It is better to use the viewer based on mrpt.
    //viewer->render_loopClosure(data, res_firstScan, res_loopClosureScan);

    // Loop events
    while (!viewer->isExit())
    {
        viewer->queryKeyboardEvents();
        mrpt::system::sleep(10);
    }
}

/*void mergingScan_loopClosure(const string& dataFolder,
                             const CConfigFile& cfg)
{
    shared_ptr<simulationViewer_mrpt> viewer = make_shared<simulationViewer_mrpt>("scanMergingViewer", true ) ;
    viewer->init(cfg);
    viewer->resize(1920, 1080);

    cout << " ---> Simulate Data ..." << endl;
    dataForScanMerging_loopClosure data = simulateData_loopClosure(cfg);
    cout << " ---> Simulation Done." << endl;

    // Construct the surface estimator which use a Gaussian process trained on the vertical sonar measurements
    shared_ptr<surfaceGP> surfaceEstimator = make_shared<surfaceGP>(cfg);

    // Estimate the 3D points
    scanMergingResults res_firstScan = processScanData(surfaceEstimator, 
                                                       dataFolder + "/loopClosure/firstScan" , 
                                                       true , 
                                                       data.firstScanGrp);
    scanMergingResults res_loopClosureScan = processScanData(surfaceEstimator,
                                                             dataFolder + "/loopClosure/loopClosureScan" ,
                                                             true ,
                                                             data.loopClosureScanGrp);                                          

    data.save(dataFolder + "/loopClosure",
              res_firstScan,
              res_loopClosureScan);

    // View / Drawing
    // The first version used the libigl viewer but it is quite difficult to custom
    // It is better to use the viewer based on mrpt.
    viewer->render_loopClosure(data, res_firstScan, res_loopClosureScan);

    // Loop events
    while (!viewer->isExit())
    {
        viewer->queryKeyboardEvents();
        mrpt::system::sleep(10);
    }
}*/

int main(int argc, char** argv)
{   
    string cfgName = "simulation_config_file.ini";
    if(argc == 2)
         cfgName = argv[1];

    // Load the configuration file
    // ToDo : file could be set as an argument
    cout << "START Scan Merging !!!!!!!!" << endl;
    cout << "Config file : " << cfgName << endl;
    CConfigFile cfg(PID_PATH(cfgName));

    // Get the data from simulation or from a dataset
    bool isSimulation = cfg.read_bool("General", "useSimulatedKarst",true,true);
    string dataFolder = cfg.read_string("General", "dataFolder", "", true);
    bool isLoopClosure = cfg.read_bool("General", "loopClosure", false, true);
    rangeMapping::configure(cfg);
    
    //if(!isLoopClosure)
    //    mergingScan_successive(dataFolder, cfg, isSimulation);
    //else
    mergingScan_karstDonut(dataFolder, cfg);//mergingScan_loopClosure(dataFolder, cfg);    
}
