#include "karst_slam/scanMerging/interpolationSE3.h"

using namespace std;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace boost::math;
using namespace karst_slam::scanMerging;

CPose3D karst_slam::scanMerging::linearInterpolateSE3(const CPose3D& current_pose,
                                                      const CPose3D& previous_pose,
                                                      double t)
{
    CPose3D invA_mut_B;
    invA_mut_B.inverseComposeFrom(current_pose, previous_pose);

    CArrayDouble<6> log;
    invA_mut_B.ln(log);

    // To avoid problem of conversion type Eigen/Mrpt when using t*log
    for(int i = 0; i < 6; i++)
        log[i] *= t;

    CPose3D incr;
    CPose3D::exp(log, incr);

    return previous_pose + incr;
}

CPose3DPDFGaussian karst_slam::scanMerging::linearInterpolateSE3(const CPose3DPDFGaussian& current_pose,
                                                                 const CPose3DPDFGaussian& previous_pose,
                                                                 double t)
{ 
    CPose3DPDFGaussian interpolate;

    if(t >=0. && t <= 1.)
    {
        interpolate.mean = linearInterpolateSE3(current_pose.mean, previous_pose.mean, t);

        // Also interpolate the covariance
        // Here, use the geodesic based on the Fisher-rao metric
        // TODO : could also propagate uncertainty in se(3), interpolate it there and propagate back the uncertainty ... ?
        const CMatrixDouble66& previous_cov = previous_pose.cov;
        const CMatrixDouble66& current_cov  = current_pose.cov;

        // Case for the first measure when the covariance is null
        if(karst_slam::utils::equalMatrix<CMatrixDouble66>(CMatrixDouble66::Zero(), previous_cov,1e-8))
        {
            // Here suppose diagonal matrix for current_cov !!
            interpolate.cov = t*current_cov;
        }
        else
            interpolate.cov = linearInterpolateCovariance(current_cov, previous_cov, t);
    }
    else
        std::cout << "Failed to interpolate : t should be in [0,1]. t = " << t << std::endl;

    return interpolate;
}

CMatrixDouble66 karst_slam::scanMerging::linearInterpolateCovariance(const CMatrixDouble66& current_cov,
                                                                     const CMatrixDouble66& previous_cov,
                                                                     double t)
{
    // Compute the square root of the covariance matrices
    // Eigen compute the non-integer power of a matrix based on the Schur-Padé algorithm 
    // (A SCHUR–PADÉ ALGORITHM FOR FRACTIONAL POWERS OF A MATRIX, NICHOLAS J. HIGHAM AND LIJING LIN)
    // Formula is given in doc/"On-manifold Probabilistic ICP : Application to Underwater Karst Exploration", eq (64)
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,6,6>> esPrevious(previous_cov);
    Eigen::Matrix<double,6,6> sqrtPrevious = esPrevious.operatorSqrt(), sqrtPreviousInverse = sqrtPrevious.inverse();

    return sqrtPrevious * (sqrtPreviousInverse * current_cov * sqrtPreviousInverse).pow(t) * sqrtPrevious;
}


map<uint64_t,trajectoryPose<CPose3DPDFGaussian>> karst_slam::scanMerging::linearInterpolateTrajectorySE3AtGivenPoses(const vector<uint64_t>& timeStamps,
                                                                                                                     const map<uint64_t, trajectoryPose<CPose3DPDFGaussian>>& poses,
                                                                                                                     bool onlyInterpolatedValues)
{
    map<uint64_t, trajectoryPose<CPose3DPDFGaussian>> interpolatedTraj; 
    if(!onlyInterpolatedValues) 
        interpolatedTraj = poses;

    double t;
    auto poses_it = poses.cbegin(), poses_it_prev = poses_it;

    // Use a set (ordered and no duplicate)
    set<uint64_t> timeStampsSet(timeStamps.begin(), timeStamps.end());
    int denumerator;
    for(uint64_t ts : timeStampsSet)
    {
        // Search for the poses interval corresponding to the current timestamp 
        while(ts >= poses_it->first)
        {
            poses_it_prev = poses_it;
            poses_it++;
        }
        
        t = (double)(ts - poses_it_prev->first) / (double)(poses_it->first - poses_it_prev->first);

        trajectoryPose<CPose3DPDFGaussian> interpPose;
        interpPose.pose    = linearInterpolateSE3(poses_it->second.pose,
                                                  poses_it_prev->second.pose,
                                                  t); 
        interpPose.pose_gt = linearInterpolateSE3(poses_it->second.pose_gt,
                                                  poses_it_prev->second.pose_gt,
                                                  t);
        interpolatedTraj.insert(pair<uint64_t, trajectoryPose<CPose3DPDFGaussian>>(ts, interpPose));
    }   

    return interpolatedTraj; 
}

map<uint64_t,trajectoryPose<CPose3DPDFGaussian>> karst_slam::scanMerging::splineInterpolateTrajectorySE3AtGivenPoses(const vector<uint64_t>& timeStamps,
                                                                                                                     const map<uint64_t, trajectoryPose<CPose3DPDFGaussian>>& poses,
                                                                                                                     bool onlyInterpolatedValues)
{
    // TODO : Refactor to avoid doing same things for pose and pose_gt

    map<uint64_t, trajectoryPose<CPose3DPDFGaussian>> interpolatedTraj;
    if(!onlyInterpolatedValues)
        interpolatedTraj = poses;
        
    const CPose3DPDFGaussian& firstPose    = poses.begin()->second.pose,
                              firstPose_gt = poses.begin()->second.pose_gt;
    set<uint64_t> timeStampsSet(timeStamps.begin(), timeStamps.end());
    int nPoses = poses.size();

    //-----------------------
    // Map each pose to se(3)
    //-----------------------
    vector<CArrayDouble<6>> epsilons, epsilons_gt;
    epsilons.reserve(nPoses);
    epsilons_gt.reserve(nPoses);
    CPose3D invA_mut_B;
    CArrayDouble<6> log;
    
    // Store each component in a vector for the spline interpolation
    vector<vector<double>> vectorPerComponent(6), vectorPerComponent_gt(6);
    vector<double> timeStamp_poses; // Need double for barycentric_rational
    timeStamp_poses.reserve(nPoses);
    for(vector<double>& v : vectorPerComponent)
        v.reserve(nPoses);
    for(vector<double>& v : vectorPerComponent_gt)
        v.reserve(nPoses);

    for(const auto& p : poses)
    {
        invA_mut_B.inverseComposeFrom(p.second.pose.mean, firstPose.mean);
        invA_mut_B.ln(log);
        epsilons.push_back(log);
        for(int i = 0; i < 6 ; i++)
            vectorPerComponent[i].push_back(log[i]);

        invA_mut_B.inverseComposeFrom(p.second.pose_gt.mean, firstPose_gt.mean);
        invA_mut_B.ln(log);
        epsilons_gt.push_back(log);
        for(int i = 0; i < 6 ; i++)
            vectorPerComponent_gt[i].push_back(log[i]);
        
        timeStamp_poses.push_back((double)p.first);
    }

    //-----------------------
    // Compute a spline in se(3)
    //----------------------- 
    vector<barycentric_rational<double>> splinesPerComponent, splinesPerComponent_gt;
    for(int i = 0; i < 6; i++)
    {
        splinesPerComponent.push_back(barycentric_rational<double>(timeStamp_poses.begin(),
                                                                   timeStamp_poses.end(),
                                                                   vectorPerComponent[i].begin()));
        splinesPerComponent_gt.push_back(barycentric_rational<double>(timeStamp_poses.begin(),
                                                                   timeStamp_poses.end(),
                                                                   vectorPerComponent_gt[i].begin()));
    }

    //-------------------------------
    // Evaluate at provided timestamp and back to SE(3)
    //-------------------------------
    CArrayDouble<6> curInterp;
    double t;
    auto poses_it = poses.cbegin(), poses_it_prev = poses_it;
    CPose3D incr, incr_gt;
    trajectoryPose<CPose3DPDFGaussian> interpTrajPose;
    for(uint64_t ts : timeStampsSet)
    {   
        //-----------------------------------
        // Interpolate covariance 
        // Currently use linear interpolation 
        //-----------------------------------
        while(ts > poses_it->first)
        {
            poses_it_prev = poses_it;
            poses_it++;
        }

        t = (double)(ts - poses_it_prev->first) / (double)(poses_it->first - poses_it_prev->first);

        if(karst_slam::utils::equalMatrix<CMatrixDouble66>(CMatrixDouble66::Zero(), poses_it_prev->second.pose.cov,1e-8))
        {
            // Here suppose diagonal matrix for current_cov !!
            interpTrajPose.pose.cov = t*poses_it->second.pose.cov;
        }
        else
            interpTrajPose.pose.cov = linearInterpolateCovariance(poses_it->second.pose.cov, poses_it_prev->second.pose.cov, t);

        interpTrajPose.pose_gt.cov = interpTrajPose.pose.cov;

        //-----------------------
        // Evaluate spline at current timeStamp
        //-----------------------
        for(int i = 0; i < 6; i++)
            curInterp[i] = splinesPerComponent[i](ts);

        //-----------------------
        // Project back to SE(3)
        //-----------------------
        CPose3D::exp(curInterp, incr);
        interpTrajPose.pose.mean = firstPose.mean + incr;

        //-----------------------
        // Evaluate spline at current timeStamp
        //-----------------------
        for(int i = 0; i < 6; i++)
            curInterp[i] = splinesPerComponent_gt[i](ts);

        //-----------------------
        // Project back to SE(3)
        //-----------------------
        CPose3D::exp(curInterp, incr_gt);
        interpTrajPose.pose_gt.mean = firstPose_gt.mean + incr_gt;
        interpTrajPose.timeStamp = ts;

        interpolatedTraj.insert(pair<uint64_t, trajectoryPose<CPose3DPDFGaussian>>(ts, interpTrajPose));
    }
 

    return interpolatedTraj;
}

map<uint64_t, trajectoryPose<CPose3DPDFGaussian>> karst_slam::scanMerging::interpolateTrajectorySE3AtGivenPoses(const vector<uint64_t>& timeStamps,
                                                                                                                const map<uint64_t, trajectoryPose<CPose3DPDFGaussian>>& poses,
                                                                                                                INTERPOLATION_TYPE type,
                                                                                                                bool onlyInterpolatedValues)
{
    std::map<uint64_t, trajectoryPose<CPose3DPDFGaussian>> res;
    switch (type)
    {
    case LINEAR:
    {
        res = linearInterpolateTrajectorySE3AtGivenPoses(timeStamps,
                                                         poses,
                                                         onlyInterpolatedValues);
        break;
    }

    case SPLINE:
    {
        res = splineInterpolateTrajectorySE3AtGivenPoses(timeStamps,
                                                         poses,
                                                         onlyInterpolatedValues);
        break;
    }

    default:
    {
        cout << "[interpolateTrajectorySE3AtGivenPoses] Unknown interpolation type" << endl;
        break;
    }
    }

    // Remove the last pose (only used for interpolation)
    res.erase(poses.rbegin()->first);

    return res;
}
