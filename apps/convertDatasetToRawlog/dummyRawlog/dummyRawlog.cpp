#include "karst_slam/obs/ObservationMSISBeam.h"
#include "karst_slam/obs/ObservationOdometry.h"
#include "karst_slam/poses.h"
#include <mrpt/obs/CRawlog.h>
#include <mrpt/utils.h>

using namespace std;
using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace karst_slam;
using namespace karst_slam::obs;


int main(int argc, char** argv)
{
    CFileGZOutputStream out_f("/home/yohan/Downloads/full_dataset/dummyDataset.rawlog");

    // Generate a Rawlog file by simulating a simple environnement : a wall placed at a certain distance and the robot moving in translation
    int nAngleStep = 100;
    float anglePerStep = 2.f*M_PI/(float)nAngleStep;
    int nBins = 400;
    int max_range = 20;
    float intensityStep = (float)max_range/(float)nBins;
    float wall_distance = 5.f;
    mrpt::poses::CPose3D sensorPoseOnRobot_horizontal /*(0.1,0,-0.42,0,0,M_PI)*/ (0,0,1,0.5*M_PI,0,0),
                         sensorPoseOnRobot_vertical (0.,0.,0.,0.,0.5*M_PI,0.);
    mrpt::poses::CPose3DPDFGaussian localIncr, curGlobalPose, prevGlobalPose,curGlobalPoseBeam;
    double rollPerStep = 0.5*M_PI/(double)nAngleStep;
    double yawPerStep = 0.25*M_PI/(double)nAngleStep;
    localIncr.mean = mrpt::poses::CPose3D(0.01,0,0,0,0,0);
    localIncr.cov  = 0.001*Eigen::Matrix<double,6,6>::Identity();
    //localIncr.cov(3,3) = localIncr.cov(4,4) = localIncr.cov(5,5) = 0.;
    mrpt::math::CMatrixDouble33 R;
    float curAngleBeam = 0.f;
    double distanceToWall = 0.f;
    curGlobalPoseBeam.mean = curGlobalPose.mean + sensorPoseOnRobot_horizontal;
    int j = 0;
    for(int i = 0 ; i < 3; i++)
    {
        for(int curAngleStep = 0; curAngleStep < nAngleStep; curAngleStep++)
        {
            // Generate the horizontal beam
            ObservationMSISBeamPtr beam = ObservationMSISBeam::Create("Sonar_Micron");

            beam->setMaxRange(max_range);
            curAngleBeam = curAngleStep*anglePerStep;
            beam->setAngle(curAngleBeam);
            beam->setNAngularStep(nAngleStep);

            mrpt::poses::CPose3D sonar_rotation(0,0,0,curAngleBeam,0,0);
            curGlobalPoseBeam.mean = curGlobalPose.mean + sensorPoseOnRobot_horizontal + sonar_rotation;

            vector<int> intensities(nBins,0);
            curGlobalPoseBeam.getMeanVal().getRotationMatrix(R);
            if(R.coeff(0,0) > 1e-3)
            {
                distanceToWall = (wall_distance - curGlobalPoseBeam.getMeanVal().x())/R.coeff(0,0);
                if(distanceToWall <= max_range && distanceToWall > 0)
                {
                    int idx = round(distanceToWall/intensityStep);
                    intensities[idx] = 200;
                }
            }

            //float sinAngle = sin(curAngleBeam - M_PI);
            /*if(curAngleBeam > M_PI && sinAngle > 1e-3)
            {
                float distanceToWall = (wall_distance - curGlobalPose.mean.x())/sinAngle;
                if(distanceToWall <= max_range)
                {
                    int idx = round(distanceToWall/intensityStep);
                    intensities[idx] = 200;
                }
            }*/
            beam->setIntensities(move(intensities));
            beam->setSensorPose(sensorPoseOnRobot_horizontal);

            //beam->getDescriptionAsText(cout);
            cout << "i = " << i << endl;
            cout << "curAngleStep = " << curAngleStep << endl;
            out_f << beam;

            // Test as in underwater dataset where odometry is "sparse"
            //if(j%2 == 1)
            //{
                ObservationOdometryPtr odo = ObservationOdometry::Create();
//                mrpt::poses::CPose3D odo_;
//                localIncr.drawSingleSample(odo_);
//                cout << "odo_ : " << odo_ << endl;
                //mrpt::poses::CPose3DPDFGaussian odo_(curGlobalPose.mean - prevGlobalPose.mean, localIncr.cov);
                //odo->pose_pdf = karst_slam::convertPosePdf<pose_pdf_t>(odo_);//pose_pdf_t(odo_);
                odo->pose_pdf.mean    = curGlobalPose.mean - prevGlobalPose.mean; //pose_t(odo_);
                odo->pose_pdf.cov_inv = localIncr.cov.inverse();
                out_f << odo;
                prevGlobalPose = curGlobalPose;
            //}

//            mrpt::poses::CPose3DPDFGaussian realIncr;
//            realIncr.mean = odo_;
//            realIncr.cov = localIncr.cov;
//            curGlobalPose += realIncr;//localIncr;
            curGlobalPose += localIncr;

            // Generate the vertical beam
            ObservationMSISBeamPtr beam_v = ObservationMSISBeam::Create("Sonar_Seaking");
            beam_v->setMaxRange(max_range);
            curAngleBeam = curAngleStep*anglePerStep;
            beam_v->setAngle(curAngleBeam);
            beam_v->setNAngularStep(nAngleStep);
            beam_v->setSensorPose(sensorPoseOnRobot_vertical);

            vector<int> intensities_v(nBins,0);
            intensities_v[nBins/8] = 100;
            beam_v->setIntensities(move(intensities_v));
            out_f << beam_v;

            j++;
        }
    }
}
