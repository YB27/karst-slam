#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/math/poly_roots.h>
#include "karst_slam/scanMerging/surfaceValidationData.h"
#include "karst_slam/scanMerging/simulatedPointAtTheta.h"
#include "karst_slam/scanMerging/ellipticCylinder.h"
#include "karst_slam/scanMerging/rangeMapping.h"
#include "karst_slam/scanMerging/interpolationSE3.h"
#include <iostream>
#include <fstream>

using namespace std;
using namespace mrpt::poses;
using namespace karst_slam::scanMerging;

surfaceValidationData::surfaceValidationData() : surfaceData<surfaceDatum_arcEstimation>()
{
    this->setMinLoggingLevel(mrpt::utils::LVL_DEBUG);
}

double surfaceValidationData::priorRange(const ellipticCylinder& priorCylinder,
                                         const mrpt::poses::CPose3D& pose,
                                         double yaw)
{
    Eigen::Vector3d dir_loc = Eigen::Vector3d::Zero(),
                    dir     = Eigen::Vector3d::Zero(),
                    point   = Eigen::Vector3d::Zero();
    dir_loc << cos(yaw), sin(yaw), 0.;

    dir = pose.getRotationMatrix()*dir_loc;
    point << pose.x(), pose.y(), pose.z();

    return priorCylinder.getDistanceToInfiniteCylinderAlongDirection(point, dir);
}

void surfaceValidationData::polarCoordsProjOnGuide(const ellipticCylinder& priorCylinder, 
                                                   bool isLearningMeanFunc,
                                                   surfaceDatum& sd)
{
    const Eigen::Matrix3d& cylinderBase   = priorCylinder.getPrincipalBase();
    const Eigen::Vector3d& cylinderCenter = priorCylinder.getCenter();
    Eigen::Vector3d ptAsVec, projPoint, orthoVec;
    double rho;

    sd.pt.getAsVector(ptAsVec);
    projPoint  = cylinderBase.transpose()*(ptAsVec - cylinderCenter);

    sd.s       = projPoint(0);
    // Range when orthogonal to the guide
    orthoVec   = projPoint(1)*cylinderBase.col(1) + projPoint(2)*cylinderBase.col(2);
    rho        = orthoVec.norm();
    
    // If not learning the mean function, 
    // consider the distance to elliptic cylinder as the mean
    if(isLearningMeanFunc)
    {
        sd.r_prior = 0.;
        sd.out_r   = rangeMapping::rangeMap(rho);
    }
    else
    {
        sd.r_prior = rangeMapping::rangeMap(priorCylinder.getDistanceToInfiniteCylinderAlongDirection(cylinderCenter + projPoint(0)*cylinderBase.col(0), orthoVec.normalized()));
        sd.out_r   = rangeMapping::rangeMap(rho) - sd.r_prior;
    }
    sd.yaw     = atan2(projPoint(2),projPoint(1));
}

surfaceValidationData surfaceValidationData::generateFromHorizontalSonar_guide(const vector<vector<pointThetaSimu> > &horizontalSonarPoints_theta,
                                                                               const ellipticCylinder& priorCylinder,
                                                                               bool isLearningMeanFunc,
                                                                               double min_abscissa, double max_abscissa)
{
    // Output
    surfaceValidationData data;
    data.reserve(horizontalSonarPoints_theta.size());
    int point_idx = 0;

    const Eigen::Matrix3d& cylinderBase   = priorCylinder.getPrincipalBase();
    const Eigen::Vector3d& cylinderCenter = priorCylinder.getCenter();
    Eigen::Vector3d projPoint, orthoVec, orthoVec_yaw_plus,  orthoVec_yaw_minus;
    CPose3D cylinderPose         = priorCylinder.getPose(), // Pose such that the point is in the plane YZ
            cylinderPose_s_plus  = priorCylinder.getPose(), 
            cylinderPose_s_minus = priorCylinder.getPose();

    double r_prior;
    int idx_theta = 0;
    double ds = 0.01, dyaw = 1e-3;
    double r_prior_s_plus, r_prior_s_minus, r_prior_yaw_plus,r_prior_yaw_minus;
    bool hasValidTheta = false;
    for(const vector<pointThetaSimu>& pointSonar_theta: horizontalSonarPoints_theta)
    {
        // For each possible theta
        surfaceDatum_arcEstimation datum;
        datum.reserve(pointSonar_theta.size());
        idx_theta     = 0;
        hasValidTheta = false;
        for(const pointThetaSimu& pointSimu : pointSonar_theta)
        {
            // Point sampled on the current arc
            const CPoint3D& pointSonar = pointSimu.point;

            // Projection on the guide
            surfaceDatum sd;
            sd.pt = pointSonar;
            polarCoordsProjOnGuide(priorCylinder, isLearningMeanFunc, sd);
            if(sd.s >= min_abscissa && sd.s <= max_abscissa)
            {
                hasValidTheta = true;
                cylinderPose.m_coords         = cylinderCenter + sd.s*cylinderBase.col(0);
                cylinderPose_s_plus.m_coords  = cylinderCenter + (sd.s + ds)*cylinderBase.col(0);
                cylinderPose_s_minus.m_coords = cylinderCenter + (sd.s - ds)*cylinderBase.col(0);

                orthoVec        = cos(sd.yaw)*cylinderBase.col(1) + sin(sd.yaw)*cylinderBase.col(2);
                r_prior_s_plus  = priorCylinder.getDistanceToInfiniteCylinderAlongDirection(cylinderPose_s_plus.m_coords, orthoVec); 
                r_prior_s_minus = priorCylinder.getDistanceToInfiniteCylinderAlongDirection(cylinderPose_s_minus.m_coords, orthoVec); 

                orthoVec_yaw_plus  = cos(sd.yaw + dyaw)*cylinderBase.col(1) + sin(sd.yaw + dyaw)*cylinderBase.col(2);
                orthoVec_yaw_minus = cos(sd.yaw - dyaw)*cylinderBase.col(1) + sin(sd.yaw - dyaw)*cylinderBase.col(2); 
                r_prior_yaw_plus   = priorCylinder.getDistanceToInfiniteCylinderAlongDirection(cylinderPose.m_coords, orthoVec_yaw_plus);
                r_prior_yaw_minus  = priorCylinder.getDistanceToInfiniteCylinderAlongDirection(cylinderPose.m_coords, orthoVec_yaw_minus);

                surfaceDatum_valueAtTheta sd_est(sd,
                                                pointSimu.theta,
                                                aux_normalData(cylinderPose_s_plus, 
                                                               cylinderPose_s_minus, 
                                                               cylinderPose,
                                                               sd.s + ds,
                                                               sd.s - ds,
                                                               sd.yaw + dyaw,
                                                               sd.yaw - dyaw,
                                                               rangeMapping::rangeMap(r_prior_s_plus),
                                                               rangeMapping::rangeMap(r_prior_s_minus),
                                                               rangeMapping::rangeMap(r_prior_yaw_plus),
                                                               rangeMapping::rangeMap(r_prior_yaw_minus)));
                datum.dataPerTheta.push_back(sd_est);
                datum.idx = point_idx;
            }

        }
        if(hasValidTheta)
            data.addData(datum);
        point_idx++;
    }

    return data;
}

surfaceValidationData surfaceValidationData::generateFromHorizontalSonar(const vector<trajectoryPose<CPose3DPDFGaussian>>& robotTraj,
                                                                         const vector<vector<pointThetaSimu> > &horizontalSonarPoints_theta,
                                                                         const CPose3D& verticalSonarPose,
                                                                         const ellipticCylinder& priorCylinder)
{
    // Output
    surfaceValidationData data;
    data.reserve(horizontalSonarPoints_theta.size());

    // For each data point
    vector<int> interval_idxs;
    double t, s, yaw, r;
    bool intersection_valid;
    int point_idx = 0;
    for(const vector<pointThetaSimu>& pointSonar_theta: horizontalSonarPoints_theta)
    {
        // For each possible theta
        surfaceDatum_arcEstimation datum;
        datum.reserve(pointSonar_theta.size());
        double r_prior;
        int idx_theta = 0;
        bool hasValidIntersection = false; 
        for(const pointThetaSimu& pointSimu : pointSonar_theta)
        {
            // Point sampled on the current arc
            const CPoint3D& pointSonar = pointSimu.point;

            // Find the trajectory interval on which to project the sonar point
            interval_idxs = findInterval(robotTraj, pointSonar, verticalSonarPose);
            if(!interval_idxs.empty())
            {
                for(const int curInterval_idx : interval_idxs)
                {
                    // Linear interpolation and intersection
                    // currently only assume one interval of intersection
                    const CPose3DPDFGaussian& bodyPose_begin = robotTraj[curInterval_idx].pose;
                    const CPose3DPDFGaussian& bodyPose_end   = robotTraj[curInterval_idx+1].pose;

                    t = linearInterpolation_intersection(bodyPose_begin, bodyPose_end, verticalSonarPose, pointSonar);

                    CPose3D interp_pose;
                    intersection_valid = intersectionCurvilinearAbscissa(robotTraj[curInterval_idx].curvilinearAbscissa,//bodyPoses_curvilinearAbscissa[curInterval_idx],
                                                                              bodyPose_begin,
                                                                              bodyPose_end,
                                                                              pointSonar,
                                                                              t,
                                                                              verticalSonarPose,
                                                                              interp_pose,
                                                                              s,
                                                                              yaw,
                                                                              r);
                    if(intersection_valid)
                    {                        
                        // Temporary, for test, compute and save data for further computing normals

                        // For the normal estimation, need to compute the interpolation poses for t+dt, t-dt
                        double dt = 0.2, dyaw = 5e-2,
                               t_plus = t + dt, t_minus = t - dt,
                               yaw_plus = yaw + dyaw, yaw_minus = yaw - dyaw;
                        double s_dt_plus, s_dt_minus,
                               yaw_dt_plus, yaw_dt_minus, r_dt_plus, r_dt_minus;
                        CPose3D interp_pose_dt_plus, interp_pose_dt_minus;

                        int interval_idx_dt_plus = curInterval_idx;
                        if(t_plus > 1.)
                        {
                            interval_idx_dt_plus += 1;
                            t_plus -=1;
                        }

                        int interval_idx_dt_minus = curInterval_idx;
                        if(t_minus < 0.)
                        {
                            if(curInterval_idx == 0)
                                t_minus = 0;
                            else
                            {
                                interval_idx_dt_minus -= 1;
                                t_minus += 1;
                            }
                        }

                        if(interval_idx_dt_plus >= 0 &&  interval_idx_dt_plus < robotTraj.size() - 1 && 
                           interval_idx_dt_minus >= 0 &&  interval_idx_dt_minus < robotTraj.size() - 1)
                        {
                            intersectionCurvilinearAbscissa(robotTraj[interval_idx_dt_plus].curvilinearAbscissa,//bodyPoses_curvilinearAbscissa[interval_idx_dt_plus],
                                                            robotTraj[interval_idx_dt_plus].pose,
                                                            robotTraj[interval_idx_dt_plus+1].pose,
                                                            pointSonar,
                                                            t_plus,
                                                            verticalSonarPose,
                                                            interp_pose_dt_plus,
                                                            s_dt_plus,
                                                            yaw_dt_plus,
                                                            r_dt_plus);
                       
                            intersectionCurvilinearAbscissa(robotTraj[interval_idx_dt_minus].curvilinearAbscissa,
                                                            robotTraj[interval_idx_dt_minus].pose,
                                                            robotTraj[interval_idx_dt_minus+1].pose,
                                                            pointSonar,
                                                            t_minus,
                                                            verticalSonarPose,
                                                            interp_pose_dt_minus,
                                                            s_dt_minus,
                                                            yaw_dt_minus,
                                                            r_dt_minus);

                            // To compute r_prior, we use the vertical sonar pose corresponding at the interpolated robot pose hence inter_pose + verticalsonarPose
                            CPose3D interPose_verticalSonar            = interp_pose + verticalSonarPose,
                                    interp_pose_dt_plus_verticalSonar  = interp_pose_dt_plus + verticalSonarPose,
                                    interp_pose_dt_minus_verticalSonar = interp_pose_dt_minus + verticalSonarPose;
                            r_prior = surfaceValidationData::priorRange(priorCylinder, interPose_verticalSonar, yaw);

                            double r_prior_dt_plus = surfaceValidationData::priorRange(priorCylinder, interp_pose_dt_plus_verticalSonar, yaw),
                                r_prior_dt_minus   = surfaceValidationData::priorRange(priorCylinder, interp_pose_dt_minus_verticalSonar, yaw),
                                r_prior_dyaw_plus  = surfaceValidationData::priorRange(priorCylinder, interPose_verticalSonar, yaw_plus),
                                r_prior_dyaw_minus = surfaceValidationData::priorRange(priorCylinder, interPose_verticalSonar, yaw_minus);

                            surfaceDatum sd(s, yaw, rangeMapping::rangeMap(r) - rangeMapping::rangeMap(r_prior));
                            sd.r_prior = rangeMapping::rangeMap(r_prior);
                            sd.pt = pointSonar;

                            surfaceDatum_valueAtTheta sd_est(sd,
                                                            pointSimu.theta,
                                                            aux_normalData(interp_pose_dt_plus_verticalSonar,
                                                                            interp_pose_dt_minus_verticalSonar,
                                                                            interPose_verticalSonar,
                                                                            s_dt_plus,
                                                                            s_dt_minus,
                                                                            yaw_plus,
                                                                            yaw_minus,
                                                                            rangeMapping::rangeMap(r_prior_dt_plus),
                                                                            rangeMapping::rangeMap(r_prior_dt_minus),
                                                                            rangeMapping::rangeMap(r_prior_dyaw_plus),
                                                                            rangeMapping::rangeMap(r_prior_dyaw_minus)));
                            datum.dataPerTheta.push_back(sd_est);
                            datum.idx = point_idx;

                            hasValidIntersection = true;
                            break;
                        }
                    }
                    else
                    {
                        //MRPT_LOG_WARN_STREAM("[surfaceValidationData::generateFromHorizontalSonar] Invalid intersection for point " << point_idx << " at theta " << idx_theta << " for current interval !!! ");
                        //hasNoValidIntersection = true;
                    }
                }
            }
            else
            {
                //MRPT_LOG_WARN_STREAM("[surfaceValidationData::generateFromHorizontalSonar] For arc corresponding to point idx " << point_idx << " at theta idx "  << idx_theta << ", the arc point is outside the trajectory --> rejected ");
                //hasNoValidIntersection = true;
                //break;
            }

             idx_theta++;
        }

        // Add only if intersections were found for at least one theta
        if(hasValidIntersection)
            data.addData(datum);

        point_idx++;
    }

    return data;
}

vector<int> surfaceValidationData::findInterval(const vector<trajectoryPose<CPose3DPDFGaussian>>& bodyPoses,
                                                const CPoint3D& pointSonar,
                                                const CPose3D& verticalSonarPose)
{
    // Brute force version
    // ToDo : optimize it if needed !
    assert(bodyPoses.size() >= 2);

    vector<int> intervalIdx; // intervalIdx index i means the interval [i,i+1]

    CPose3D verticalGlobalSonarPose = bodyPoses[0].pose.mean + verticalSonarPose;

    Eigen::Matrix<double,3,1> p_sonar = pointSonar.getAsVectorVal();
    Eigen::Matrix<double,3,1> p_prev = CPoint3D(verticalGlobalSonarPose).getAsVectorVal(), p_cur;
    Eigen::Matrix<double,3,1> z_prev = verticalGlobalSonarPose.getRotationMatrix().block<3,1>(0,2),
                              z_cur = Eigen::Matrix<double,3,1>::Zero(); 
    double dotProd_prev = z_prev.dot(p_sonar - p_prev), dotProd_cur;
    int i = 1;

    for(vector<trajectoryPose<CPose3DPDFGaussian>>::const_iterator it = bodyPoses.begin()+1; it != bodyPoses.end(); it++)
    {
        verticalGlobalSonarPose = it->pose.mean + verticalSonarPose;
        p_cur =  CPoint3D(verticalGlobalSonarPose).getAsVectorVal(); 
        z_cur =  verticalGlobalSonarPose.getRotationMatrix().block<3,1>(0,2);

        dotProd_cur = z_cur.dot(p_sonar - p_cur);

        // Different sign ?
        // Check for all possibility or break at first found ?
        if(dotProd_cur*dotProd_prev < 0)
            intervalIdx.push_back(i-1);

        p_prev = p_cur;
        z_prev = z_cur;
        dotProd_prev = dotProd_cur;
        i++;
    }

    return intervalIdx;
}

double surfaceValidationData::linearInterpolation_intersection(const CPose3DPDFGaussian& bodyPose_1,
                                                               const CPose3DPDFGaussian& bodyPose_2,
                                                               const CPose3D& verticalSonarPose,
                                                               const CPoint3D& pointSonar)
{
    // ToDo : Better use quaternion ?
    double delta_yaw   = mrpt::math::wrapToPi(bodyPose_2.mean.yaw() - bodyPose_1.mean.yaw());
    double delta_pitch = mrpt::math::wrapToPi(bodyPose_2.mean.pitch() - bodyPose_1.mean.pitch());
    double delta_roll  = mrpt::math::wrapToPi(bodyPose_2.mean.roll() - bodyPose_1.mean.roll());

    // Intermediate values
    const mrpt::math::CMatrixDouble33& R1 = bodyPose_1.mean.getRotationMatrix();
    mrpt::math::CMatrixDouble33 sigmaR;
    sigmaR << 0. , -delta_roll, delta_pitch,
              delta_roll , 0. , -delta_yaw,
              -delta_pitch, delta_yaw, 0.;
    const Eigen::Vector3d z_sonar = verticalSonarPose.getHomogeneousMatrixVal().block<3,1>(0,2);

    const Eigen::Vector3d R1_sigmaR_z_sonar = R1*sigmaR*z_sonar,
                          R1_z_sonar = R1*z_sonar;

    CPose3D verticalSonarGlobalPose_1 = bodyPose_1.mean + verticalSonarPose,
            verticalSonarGlobalPose_2 = bodyPose_2.mean + verticalSonarPose;
    Eigen::Vector3d delta_p = verticalSonarGlobalPose_2.m_coords - verticalSonarGlobalPose_1.m_coords;
    Eigen::Vector3d v       = verticalSonarGlobalPose_1.m_coords - pointSonar.m_coords;

    // Solve the second order polynom
    double a = delta_p.dot(R1_sigmaR_z_sonar);
    double b = delta_p.dot(R1_z_sonar) + v.dot(R1_sigmaR_z_sonar);
    double c = v.dot(R1_z_sonar);
    double r1 = -1, r2 = -1;

    int nRoot = mrpt::math::solve_poly2(a,b,c,r1,r2);
    if(nRoot > 0)
    {
        if(r1 >= 0. && r1 <= 1.)
            return r1;
        else if(nRoot == 2 && r2 >= 0. && r2 <= 1.)
            return r2;
        else
            return -1;
    }
    else
        return -1;
}

bool surfaceValidationData::poseLinearInterpolation(const CPose3D& pose_prev,
                                                    const CPose3D& pose_next,
                                                    const double t,
                                                    CPose3D& interp)
{
    if(t >= 0. && t <= 1.)
    {
        interp.setFromValues((pose_next.x() - pose_prev.x())*t + pose_prev.x(),
                             (pose_next.y() - pose_prev.y())*t + pose_prev.y(),
                             (pose_next.z() - pose_prev.z())*t + pose_prev.z(),
                             mrpt::math::wrapToPi((pose_next.yaw() - pose_prev.yaw())*t + pose_prev.yaw()),
                             mrpt::math::wrapToPi((pose_next.pitch() - pose_prev.pitch())*t + pose_prev.pitch()),
                             mrpt::math::wrapToPi((pose_next.roll() - pose_prev.roll())*t + pose_prev.roll()));
        return true;
    }
    else
        return false;
}

bool surfaceValidationData::intersectionCurvilinearAbscissa(const double curvilinearAbscissa_prev,
                                                            const CPose3DPDFGaussian& bodyPose_prev,
                                                            const CPose3DPDFGaussian& bodyPose_next,
                                                            const CPoint3D& pointSonar,
                                                            const double t,
                                                            const CPose3D& verticalSonarPose,
                                                            CPose3D& interp_pose,
                                                            double& s,
                                                            double& yaw,
                                                            double& r)
{
    interp_pose = linearInterpolateSE3(bodyPose_next.mean, bodyPose_prev.mean, t);
    s = curvilinearAbscissa_prev + localCurvilinearAbscissa(bodyPose_prev.mean, interp_pose);

    CPose3D interp_poseVerticalSonar = interp_pose + verticalSonarPose;
    const mrpt::math::CMatrixDouble44 T = interp_poseVerticalSonar.getHomogeneousMatrixVal();
    const Eigen::Matrix3d R = T.block<3,3>(0,0);

    CPoint3D p(interp_poseVerticalSonar);
    Eigen::Matrix<double,3,1> pointSonar_inIntersectionPlane = R.transpose()*(pointSonar.getAsVectorVal() - p.getAsVectorVal());

    yaw = mrpt::math::wrapToPi(atan2(pointSonar_inIntersectionPlane(1), pointSonar_inIntersectionPlane(0)));

    r = sqrt(pointSonar_inIntersectionPlane(0)*pointSonar_inIntersectionPlane(0) +
             pointSonar_inIntersectionPlane(1)*pointSonar_inIntersectionPlane(1) +
             pointSonar_inIntersectionPlane(2)*pointSonar_inIntersectionPlane(2));

    return (t >=0 && t <=1);
}

void surfaceValidationData::save(const string& fileName) const
{
    ofstream f(fileName);
    if(f.is_open())
    {
        f << "#point_idx,s,yaw,out_r,theta,r_prior,s_dt_plus, s_dt_minus, yaw_dt_plus, yaw_dt_minus, r_prior_dt_plus,r_prior_dt_minus, r_prior_dyaw_plus, r_prior_dyaw_minus , interPose_dt_plus_sonar, interPose_dt_minus_sonar, interPose_sonar" << endl;
        for(const surfaceDatum_arcEstimation& dataPerTheta : m_data)
        {
            int pt_idx = dataPerTheta.idx;
            for(const surfaceDatum_valueAtTheta& sd_est : dataPerTheta.dataPerTheta)
            {
                f << pt_idx          << ","
                  << sd_est.sd.s     << ","
                  << sd_est.sd.yaw   << ","
                  << sd_est.sd.out_r << ","
                  << sd_est.theta << ","
                  << sd_est.sd.r_prior << ","
                  << sd_est.normalData.s_dt_plus << ","
                  << sd_est.normalData.s_dt_minus << ","
                  << sd_est.normalData.yaw_dt_plus << ","
                  << sd_est.normalData.yaw_dt_minus << ","
                  << sd_est.normalData.r_prior_dt_plus << ","
                  << sd_est.normalData.r_prior_dt_minus << ","
                  << sd_est.normalData.r_prior_dyaw_plus << ","
                  << sd_est.normalData.r_prior_dyaw_minus << ","
                  << sd_est.normalData.interp_pose_dt_plus_sonar.yaw() << ","
                  << sd_est.normalData.interp_pose_dt_plus_sonar.pitch() << ","
                  << sd_est.normalData.interp_pose_dt_plus_sonar.roll() << ","
                  << sd_est.normalData.interp_pose_dt_plus_sonar.x() << ","
                  << sd_est.normalData.interp_pose_dt_plus_sonar.y() << ","
                  << sd_est.normalData.interp_pose_dt_plus_sonar.z() << ","
                  << sd_est.normalData.interp_pose_dt_minus_sonar.yaw() << ","
                  << sd_est.normalData.interp_pose_dt_minus_sonar.pitch() << ","
                  << sd_est.normalData.interp_pose_dt_minus_sonar.roll() << ","
                  << sd_est.normalData.interp_pose_dt_minus_sonar.x() << ","
                  << sd_est.normalData.interp_pose_dt_minus_sonar.y() << ","
                  << sd_est.normalData.interp_pose_dt_minus_sonar.z() << ","
                  << sd_est.normalData.interp_pose_sonar.yaw() << ","
                  << sd_est.normalData.interp_pose_sonar.pitch() << ","
                  << sd_est.normalData.interp_pose_sonar.roll() << ","
                  << sd_est.normalData.interp_pose_sonar.x() << ","
                  << sd_est.normalData.interp_pose_sonar.y() << ","
                  << sd_est.normalData.interp_pose_sonar.z() << "\n";
            }
        }
        f.close();
    }
    else
        MRPT_LOG_ERROR_STREAM("[surfaceValidationData::save] Can't open file " + fileName);
}

void surfaceValidationData::load(const string& fileName)
{
    m_data.clear();

    ifstream file(fileName);
    string line, val;
    double interPose [6];
    int pt_idx, cur_idx = -1;
    surfaceDatum_arcEstimation dataArc;
    if(file.is_open())
    {
         getline(file,line); // skip header
         while(getline(file,line))
         {
            stringstream ss(line);
            surfaceDatum_valueAtTheta sd_est;

            // point idx
            getline(ss,val,',');
            pt_idx = stoi(val);
            if(pt_idx != cur_idx && cur_idx > 0)
            {
                // add the current sd_est
                dataArc.idx = cur_idx;
                m_data.push_back(dataArc);

                dataArc.dataPerTheta.clear();
                cur_idx = pt_idx;
            }

            // s
            getline(ss,val,',');
            sd_est.sd.s = stod(val);

            // yaw
            getline(ss,val,',');
            sd_est.sd.yaw = stod(val);

            // out_r
            getline(ss,val,',');
            sd_est.sd.out_r = stod(val);

            // theta
            getline(ss,val,',');
            sd_est.theta = stod(val);

            // r_prior
            getline(ss,val,',');
            sd_est.sd.r_prior = stod(val);

            // s_dt_plus
            getline(ss,val,',');
            sd_est.normalData.s_dt_plus = stod(val);

            // s_dt_minus
            getline(ss,val,',');
            sd_est.normalData.s_dt_minus = stod(val);

            // yaw_dt_plus
            getline(ss,val,',');
            sd_est.normalData.yaw_dt_plus = stod(val);

            // yaw_dt_minus
            getline(ss,val,',');
            sd_est.normalData.yaw_dt_minus = stod(val);

            // r_prior_dt_plus
            getline(ss,val,',');
            sd_est.normalData.r_prior_dt_plus = stod(val);

            // r_prior_dt_minus
            getline(ss,val,',');
            sd_est.normalData.r_prior_dt_minus = stod(val);

            // r_prior_dyaw_plus
            getline(ss,val,',');
            sd_est.normalData.r_prior_dyaw_plus = stod(val);

            // r_prior_dyaw_minus
            getline(ss,val,',');
            sd_est.normalData.r_prior_dyaw_minus = stod(val);

            // interPose_dt_plus_sonar
            for(int i = 0; i < 6; i++)
            {
                getline(ss,val,',');
                interPose[i] = stod(val);
            }
            sd_est.normalData.interp_pose_dt_plus_sonar = CPose3D(interPose[3], interPose[4], interPose[5],
                                                                  interPose[0], interPose[1], interPose[2]);

            // interPose_dt_minus_sonar
            for(int i = 0; i < 6; i++)
            {
                getline(ss,val,',');
                interPose[i] = stod(val);
            }
            sd_est.normalData.interp_pose_dt_minus_sonar = CPose3D(interPose[3], interPose[4], interPose[5],
                                                                  interPose[0], interPose[1], interPose[2]);

            // interPose_sonar
            for(int i = 0; i < 6; i++)
            {
                getline(ss,val,',');
                interPose[i] = stod(val);
            }
            sd_est.normalData.interp_pose_sonar = CPose3D(interPose[3], interPose[4], interPose[5],
                                                          interPose[0], interPose[1], interPose[2]);

            dataArc.dataPerTheta.push_back(sd_est);
         }
         file.close();
    }
    else
       MRPT_LOG_ERROR_STREAM("[surfaceValidationData::load] Can't open file " << fileName);
}
