#include "karst_slam/gui/MainViewer.h"
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/opengl/CAssimpModel.h>
#include <mrpt/poses.h>
#include <mrpt/utils.h>
#include <mrpt/math.h>
#include <filesystem>
#include <pid/rpath.h>

using namespace std;
using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace karst_slam::gui;

struct trajDisplay
{
    trajDisplay(const TColor& color, int nodeSize, int lineWidth)
    {
        nodes = CPointCloud::Create();
        nodes->setColor_u8(color);
        nodes->setPointSize(nodeSize);
        interpLines = CSetOfLines::Create();
        interpLines->setColor_u8(color);
        interpLines->setLineWidth(lineWidth);
    }

    void insertPose(const CPose3D& pose)
    {
        nodes->insertPoint(pose.x(), pose.y(), pose.z());
        if(nodes->size() > 1)
            interpLines->appendLine(prevPose.x(), prevPose.y(), prevPose.z(),
                                    pose.x(), pose.y(), pose.z());
        prevPose = pose;
    }

    void insertToScene(COpenGLScenePtr& scene) const
    {
        scene->insert(nodes);
        scene->insert(interpLines);
    }

    CPointCloudPtr nodes;
    CSetOfLinesPtr interpLines;
    CPose3D prevPose;
};

struct ICPres
{
    vector<CPose3D> res;
    double z_abs;
    double pitch_abs;
    double roll_abs;
};

class resViewer : public MainViewer
{
public:
     resViewer(const std::string& name):MainViewer(name){}
    ~resViewer() = default;

    void initScene(const mrpt::utils::CConfigFile &cfg) override {};
    void initKeystrokes(const mrpt::utils::CConfigFile& cfg) override {};

    void render(const CConfigFile& cfg, const vector<vector<CPose3D>>& trajs)
    {
        MRPT_START
        COpenGLScenePtr& scene = m_win->get3DSceneAndLock();

        // World frame
        // Axis of the world frame
        CSetOfObjectsPtr worldFrame = stock_objects::CornerXYZ(1.f);
        worldFrame->setName("worldFrame");
        scene->insert(worldFrame);
        cout << "render worldFrame ok" << endl;

        string envModelWireFile = cfg.read_string("Simulation", "envModelWire","",true);
        CAssimpModelPtr envModelWire = CAssimpModel::Create();
        envModelWire->setColor_u8(0.,0.,0.);
        envModelWire->loadScene(envModelWireFile);
        envModelWire->setName("envModelWire");
        scene->insert(envModelWire);
        cout << "render envModelWire ok" << endl;

        int nodeSize = 10, lineWidth = 3;
        vector<TColor> colors = {TColor(255,0,0), // GT - Red
               TColor(255,255,255), // DR - White
               TColor(102, 194, 165), // Girona 2D - dark Green
               TColor(252,141,98), // Gaussian 3D (point to point) - Orange 
               TColor(141,160,203), // Gaussian 3D all Arcs - Dark blue 
               TColor(166,216,84)}; // Gaussian 3D (point to plane) - light green 
        vector<trajDisplay> trajDisplays;
        trajDisplays.reserve(colors.size());
        for(const TColor& color : colors)
            trajDisplays.push_back(trajDisplay(color, nodeSize, lineWidth));

        cout << "Traj size : " << trajs.size() << endl;
        cout << "TrajDisplay size:  " << trajDisplays.size() << endl; 

        /*for(const vector<CPose3D>& poses : trajs)
        { 
            for(int i = 0; i < poses.size(); i++)
                trajDisplays[i].insertPose(poses[i]);
        }*/
        for(int i = 0; i < trajs.size(); i++)
        {
            trajDisplay& td = trajDisplays[i];
            for(const CPose3D& pose: trajs[i])
                td.insertPose(pose);

        }
        cout << "trajDisplays ok" << endl;

        for(const trajDisplay& traj : trajDisplays)
            traj.insertToScene(scene);

        m_win->unlockAccess3DScene();
        m_win->forceRepaint();

        MRPT_END
    }

};

ICPres readICPresults(const string& path)
{
    ICPres icpres;
    fstream file(path);
    string line;
    if(file.is_open())
    {
        // skip header and distances
        for(int i = 0; i < 6; i++)
            getline(file,line); // skip header
        
        string val;
        CPose3D pose;
        double yaw, pitch, roll;
        for(int i = 0; i < 6 ; i++)
        {
            getline(file,line);
            CPose3D pose;
            stringstream ss(line);
            
            // yaw
            getline(ss, val, ',');
            yaw = stod(val);

            // pitch
            getline(ss, val, ',');
            pitch = stod(val);

            // roll
            getline(ss, val, ',');
            roll = stod(val);

            pose.setYawPitchRoll(yaw, pitch, roll);

            // x
            getline(ss, val, ',');
            pose.m_coords[0] = stod(val);

            // y
            getline(ss, val, ',');
            pose.m_coords[1] = stod(val);

            // z
            getline(ss, val, ',');
            pose.m_coords[2] = stod(val);

            icpres.res.push_back(pose);
        }

        // Absolute depth, pitch and roll
        getline(file,line);
        icpres.z_abs = stod(line);

        getline(file,line);
        icpres.pitch_abs = stod(line);

        getline(file,line);
        icpres.roll_abs = stod(line);

        file.close();
    }
    else
        cout << "Failed to open file " << path << endl;

    return icpres;
}

vector<vector<CPose3D>> loadPICPTrajectories_old(const string& dataFolder,
                                             int nTraj)
{
    vector<vector<CPose3D>> res;

    // Get all Folders
    vector<int> folders;
    for(const auto& folder : filesystem::directory_iterator(dataFolder))
    {
        if(filesystem::is_directory(folder.path()))
            folders.push_back(stoi(folder.path().stem()));
    }
    
    // Sort 
    sort(folders.begin(), folders.end());

    string path;
    vector<CPose3D> lastGlobalPoses(nTraj, CPose3D());
    res.push_back(lastGlobalPoses);
    for(const int i : folders)
    {
        path = dataFolder + to_string(i) + "/ICPresults.txt";

        ICPres icpres = readICPresults(path);
        CPose3D p;
        for(int i = 0; i < nTraj; i++)
        {   
            p = lastGlobalPoses[i] + icpres.res[i];
            p.m_coords[2] = icpres.z_abs;
            p.setYawPitchRoll(p.yaw(), icpres.pitch_abs, icpres.roll_abs);
            lastGlobalPoses[i] = move(p);
        }
        res.push_back(lastGlobalPoses);
    }

    cout << "load size : " << res.size() << endl;
    return res;
}

vector<CPose3D> loadTrajectory(const string& file, bool skipFirst)
{
    vector<CPose3D> res;

    ifstream f(file);
    if(f.is_open())
    {
        string line, val;
        getline(f,line); // skip header

        CPose3D p, prev_p;
        double yaw, pitch, roll, curAbsPitch, curAbsRoll, curZAbs;
        while(getline(f, line))
        {
            stringstream ss(line);

            if(skipFirst)
                getline(ss, val, ','); // skip distance to gt
            
            getline(ss, val, ',');
            yaw = stod(val);
            getline(ss, val, ',');
            pitch = stod(val);
            getline(ss, val, ',');
            roll = stod(val);
            
            curAbsPitch = prev_p.pitch() + pitch;
            curAbsRoll  = prev_p.roll() + roll;
            p.setYawPitchRoll(yaw, pitch, roll);

            for(int i = 0; i < 3; i++)
            {
                getline(ss, val, ',');
                p.m_coords[i] = stod(val);
            }
            curZAbs = prev_p.m_coords[2] + p.m_coords[2];

            CPose3D res_ = prev_p + p;
            res_.m_coords[2] = curZAbs;
            res_.setYawPitchRoll(res_.yaw(), curAbsPitch, curAbsRoll);
            res.push_back(res_);
            prev_p = res_;

        }
        f.close();
    }

    return res;
}

vector<vector<CPose3D>> loadPICPTrajectories(const string& dataFolder)
{
    vector<string> files {"q_gt.txt", "dist_init_chi2_0.5.txt", "dist2D_chi2_0.5.txt", "dist3D_chi2_0.5.txt", 
                          "dist3D_allArcs_chi2_0.5.txt", "dist3D_plane_startPoint_chi2_0.5.txt"};
    vector<vector<CPose3D>> res;
    res.push_back(loadTrajectory(dataFolder + files[0], false));
    for(int i = 1; i < files.size();i++)
        res.push_back(loadTrajectory(dataFolder + files[i], true));

    return res;
}

int main(int argc, char** argv)
{   
    string dataFolder = "/home/breux/surfaceMeasureData/expKarstDonut/";
    int nTraj = 6;
    vector<vector<CPose3D>> trajs = loadPICPTrajectories(dataFolder);

    CConfigFile cfg(PID_PATH("simulation_config_file.ini"));
    resViewer v("Viewer");
    v.init(cfg);
    v.resize(1900, 1080);
    v.render(cfg, trajs);
    v.eventLoop();
}
