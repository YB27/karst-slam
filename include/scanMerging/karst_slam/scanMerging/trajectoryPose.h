#ifndef DEF_TRAJECTORYPOSE_H
#define DEF_TRAJECTORYPOSE_H

#include "karst_slam/saveToFiles.h"
#include "curvilinearAbscissa_SE3.h"
#include <map>

namespace karst_slam{ namespace scanMerging { 

/** Convenient structure containing all information concerning robot poses */
template<class POSE>
struct trajectoryPose
{
    trajectoryPose() = default;

    trajectoryPose(const trajectoryPose& otherPose)
    {
        pose = otherPose.pose;
        pose_gt = otherPose.pose_gt;
        curvilinearAbscissa = otherPose.curvilinearAbscissa;
        timeStamp = otherPose.timeStamp;
    }

    trajectoryPose(const POSE& pose_,
                   const POSE& pose_gt_ = POSE())
    {
        pose     = pose_;
        pose_gt  = pose_gt_; 
        curvilinearAbscissa = 0.;
        timeStamp = 0;
    }
    trajectoryPose(const POSE& pose_,
                   double curvilinearAbscissa_,
                   unsigned long long timeStamp_,
                   const POSE& pose_gt_ = POSE())
    {
        pose     = pose_;
        pose_gt  = pose_gt_; 
        curvilinearAbscissa = curvilinearAbscissa_;
        timeStamp = timeStamp_;
    }

    trajectoryPose(POSE&& pose_, 
                   POSE&& pose_gt_ = POSE())
    {
        pose     = std::move(pose_);
        pose_gt  = std::move(pose_gt_);
        curvilinearAbscissa = 0.;
        timeStamp = 0;
    }
    trajectoryPose(POSE&& pose_, 
                   double curvilinearAbscissa_,
                   unsigned long long timeStamp_,
                   POSE&& pose_gt_ = POSE())
    {
        pose     = std::move(pose_);
        pose_gt  = std::move(pose_gt_);
        curvilinearAbscissa = curvilinearAbscissa_;
        timeStamp = timeStamp_;
    }

    POSE pose; //!< 3D robot pose (estimated)
    POSE pose_gt; //!<  3D robot pose (ground-truth) (if available eg for simulation)

    double curvilinearAbscissa = 0.; //!< corresponding curvilinear abscissa in the robot trajectory
    unsigned long long timeStamp = 0; //!< TimeStamp (at least uint64)
};

/** Class representing the robot trajectory */
template<class POSE>
class trajectory
{
public:
    trajectory() = default;
    ~trajectory() = default;

    trajectory(const std::vector<POSE>& traj,
               const std::vector<POSE>& traj_gt) :
                m_traj(traj), m_traj_gt(traj_gt){}

    trajectory(std::vector<POSE>&& traj,
               std::vector<POSE>&& traj_gt):
                m_traj(std::move(traj)), m_traj_gt(std::move(traj_gt)){}

    /** Append another trajectory to the current one */
    void append(const trajectory<POSE>& other)
    {
        append_curvilinearAbscissa(other);
        m_traj.insert(m_traj.end(),
            other.m_traj.begin(),
            other.m_traj.end());
        m_traj_gt.insert(m_traj_gt.end(),
            other.m_traj_gt.begin(),
            other.m_traj_gt.end());
        if(!other.m_timeStamp.empty())
            m_timeStamp.insert(m_timeStamp.end(),
                            other.m_timeStamp.begin(),
                            other.m_timeStamp.end());
    }

    /** \overload */
    void append(trajectory<POSE>&& other)
    {
        append_curvilinearAbscissa(other);
        m_traj.insert(m_traj.end(),
            std::make_move_iterator(other.m_traj.begin()),
            std::make_move_iterator(other.m_traj.end()));
        m_traj_gt.insert(m_traj_gt.end(),
            std::make_move_iterator(other.m_traj_gt.begin()),
            std::make_move_iterator(other.m_traj_gt.end()));
        if(!other.m_timeStamp.empty())
            m_timeStamp.insert(m_timeStamp.end(),
                            std::make_move_iterator(other.m_timeStamp.begin()),
                            std::make_move_iterator(other.m_timeStamp.end()));
    }

    /** Append another trajectory considered as independant from the current one
     * This means that the curvilinear abscissa as appended "as is" and not recomputed along the current trajectory
     * (as it is the case in append())
     *
     * @param other another trajectory to append
     * @param startIdx append from this pose idx
     * @param endIdx append until this pose idx (not included)
     */
    void append_independant(const trajectory<POSE>& other,
                            int startIdx,
                            int endIdx)
    {
        m_traj.insert(m_traj.end(),
            other.m_traj.begin() + startIdx,
            other.m_traj.begin() + endIdx);
        m_traj_gt.insert(m_traj_gt.end(),
            other.m_traj_gt.begin() + startIdx,
            other.m_traj_gt.begin() + endIdx);
        m_curvilinearAbscissa.insert(m_curvilinearAbscissa.end(),
            other.m_curvilinearAbscissa.begin() + startIdx,
            other.m_curvilinearAbscissa.begin() + endIdx);
        if(!other.m_timeStamp.empty())
            m_timeStamp.insert(m_timeStamp.end(),
                            other.m_timeStamp.begin(),
                            other.m_timeStamp.end());
    }

    // Tmp function to keep extractScanMerging app compatibility
    // Should be removed after refactoring 
    // (pb here is that the curvilinearAbscissa is not updated)
    void push_front(const trajectoryPose<POSE>& trajPose)
    {
        m_traj.push_back(trajPose.pose);
        m_traj_gt.push_back(trajPose.pose_gt);
    }

    void push_back(const trajectoryPose<POSE>& trajPose)
    {
        // Must push curvilinear before updating the last m_traj pose
        push_back_curvilinearAbscissa(trajPose.pose);
        m_traj.push_back(trajPose.pose);
        m_traj_gt.push_back(trajPose.pose_gt);
        m_timeStamp.push_back(trajPose.timeStamp);
    }

    void push_back(trajectoryPose<POSE>&& trajPose)
    {
        // Must push curvilinear before updating the last m_traj pose
        push_back_curvilinearAbscissa(trajPose.pose);
        m_traj.push_back(std::move(trajPose.pose));
        m_traj_gt.push_back(std::move(trajPose.pose_gt));
        m_timeStamp.push_back(trajPose.timeStamp);
    }

    void push_back(const trajectoryPose<POSE>& trajPose,
                   double curvilinearAbscissa)
    {
        m_traj.push_back(trajPose.pose);
        m_traj_gt.push_back(trajPose.pose_gt);
        m_curvilinearAbscissa.push_back(curvilinearAbscissa);
        m_timeStamp.push_back(trajPose.timeStamp);
    }

    /**
     * Save the trajectory data to three files ("bodyPoses.txt", "bodyPoses_gt.txt" and "curvilinearAbscissa.txt")
     * in the given folder  
     */
    void saveToFiles (const std::string& folder) const
    {
        karst_slam::utils::savePosesToFile(m_traj, folder + "/bodyPoses.txt");
        karst_slam::utils::savePosesToFile(m_traj_gt, folder + "/bodyPoses_gt.txt");
        std::string curvilinearFile = folder + "/curvilinearAbscissa.txt";
        SAVE_START(curvilinearFile)
            for (const double s : m_curvilinearAbscissa)
                file << s << "\n";
        SAVE_END(curvilinearFile)
    }

    void clear()
    {
        m_traj.clear();
        m_traj_gt.clear();
        m_curvilinearAbscissa.clear();
        m_timeStamp.clear();
    }

    void reserve(size_t n)
    {
        m_traj.reserve(n);
        m_traj_gt.reserve(n);
        m_curvilinearAbscissa.reserve(n);
        m_timeStamp.reserve(n);
    }

    /**
     * Create a map with timestamp as keys and the corresponding poses as values
     */
    std::map<unsigned long long, trajectoryPose<POSE>> getPosesByTimeStamp() const
    {
        std::map<unsigned long long, trajectoryPose<POSE>> map;
        int n = size();
        for(int i = 0; i < n; i++)
            map[m_timeStamp[i]] = trajectoryPose<POSE>(m_traj[i], 
                                                       m_curvilinearAbscissa[i],
                                                       m_timeStamp[i], 
                                                       m_traj_gt[i]);
        return map;
    }

    inline trajectoryPose<POSE> getLastPose() const {return trajectoryPose<POSE>(m_traj.back(), m_traj_gt.back());}

    inline int size() const {return m_traj.size();}
    inline const std::vector<POSE>& getTrajectory() const {return m_traj;}
    inline const std::vector<POSE>& getTrajectory_gt() const {return m_traj_gt;}
    inline const std::vector<double>& getCurvilinearAbscissa() const {return m_curvilinearAbscissa;}
    inline const std::vector<unsigned long long>& getTimeStamp() const {return m_timeStamp;}

protected:
    void push_back_curvilinearAbscissa(const POSE& trajPose)
    {
        if (m_curvilinearAbscissa.empty())
            m_curvilinearAbscissa.push_back(0);
        else
            m_curvilinearAbscissa.push_back(m_curvilinearAbscissa.back() +
                localCurvilinearAbscissa(trajPose, m_traj.back()));
    }
    void append_curvilinearAbscissa(const trajectory<POSE>& other)
    {
        if(m_traj.empty())
            m_curvilinearAbscissa = other.m_curvilinearAbscissa;
        else
        {
            // Get the SE(3) distance between this and "other" 
            double abscissaBetweenTwo_Traj = localCurvilinearAbscissa(other.m_traj.front(), m_traj.back()),
                lastAbscissa = m_curvilinearAbscissa.back() + abscissaBetweenTwo_Traj;

            m_curvilinearAbscissa.push_back(lastAbscissa);
            std::vector<double>::const_iterator start = other.m_curvilinearAbscissa.begin();
            start++;
            for (std::vector<double>::const_iterator& it = start; it != other.m_curvilinearAbscissa.end(); it++)
                m_curvilinearAbscissa.push_back(lastAbscissa + *it);
        }
    }

    // Elements are separated in differents vector internally as it is more efficient
    // than a unique vector with all data set in a struct
    std::vector<POSE> m_traj; //!< Vector of robot 3D poses
    std::vector<POSE> m_traj_gt; //!< Vector of Ground-truth 3D poses (if available)
    std::vector<double> m_curvilinearAbscissa; //!< Vector of curvilinear abscissa 
    std::vector<unsigned long long> m_timeStamp; //!< Vector of timeStamp
};

template <class POSE>
std::ostream& operator << (std::ostream& os, const trajectoryPose<POSE>& trajPose)
{
    os << "pose : " << trajPose.pose <<  " , " << "pose_gt : " << trajPose.pose_gt << std::endl;
    return os;
}

}}
#endif // DEF_TRAJECTORYPOSE_H