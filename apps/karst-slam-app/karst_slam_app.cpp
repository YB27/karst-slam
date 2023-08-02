#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/utils/CLoadableOptions.h>

#include "karst_slam/graphSlamEngine/GraphSlamEngine.h"
#include "karst_slam/graphSlamEngine/gui/GraphSlamEngineViewer.h"
#include "karst_slam/sensors/SensorFactory.h"
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/COutputLogger.h>

#include <mrpt/system/threads.h>

#include "karst_slam/obs/obs_includes.h"

#include <string>
#include <map>
#include <vector>
#include <sstream>
#include <cerrno>
#include <thread>
#include <atomic>

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::graphs;
using namespace mrpt::math;
using namespace mrpt::opengl;
using namespace mrpt::utils;
using namespace karst_slam;
using namespace karst_slam::sensors;
using namespace karst_slam::slam;
using namespace karst_slam::gui;
using namespace karst_slam::obs;
using namespace karst_slam::scanMerging;

/** Parameters used for this applications
 */
struct generalParameters
{
    bool enableVisuals = true; //!< If true, enable GUI display
    bool saveFinalMap; //!< If true, save the final map (currently unused, initially for Octomap)
    bool pauseAtEachEntry; //!< If true, pause the program at each rawlog entry
    uint64_t startTimestamp = 0; //!< Program starts reading Rawlog data from this timestamp
    string rawlogFile; //!< Name of the Rawlog file containing all the data
};

/** Check if a file exists */
inline bool fileExists(const char* fileName) { return static_cast<bool>(ifstream(fileName)); }

/** Load the .ini file containing configuration parameters */
bool loadIniFile(int argc, char** argv, const shared_ptr<COutputLogger>& logger, CConfigFile& configFile)
{
    if (argc < 2)
    {
        logger->logFmt(LVL_ERROR, "No config file was provided. Abort");
        return false;
    }
    else
    {
        // CConfigFile doesn't check existence of the file ....
        if (!fileExists(argv[1]))
        {
            logger->logFmt(LVL_ERROR, "Can't find the config file provided : %s", argv[1]);
            return false;
        }

        // Read the required ERD / NRD / GSO
        configFile.setFileName(argv[1]);
        return true;
    }
}

/** Read the configuration file */
void readGeneralConfiguration(const CConfigFile& cfg,
    generalParameters& params)
{
    params.enableVisuals    = cfg.read_bool("GeneralConfiguration", "enableVisuals", true, false);
    params.rawlogFile       = cfg.read_string("GeneralConfiguration", "rawlogFile", "", true);
    params.pauseAtEachEntry = cfg.read_bool("GeneralConfiguration", "pauseAtEachEntry", true, false);
    params.saveFinalMap     = cfg.read_bool("GeneralConfiguration", "save_final_map", false, true);
    params.startTimestamp   = cfg.read_uint64_t("GeneralConfiguration", "startTimestamp", 0, true);
    rangeMapping::configure(cfg);
}

/** Register custom classes (for MRPT compatibility) */
void registerCustomClasses()
{
    // ToDo :  May this be directly exported by shared lib ?
    mrpt::utils::registerClass(CLASS_ID(ObservationMSIS_scan));
    mrpt::utils::registerClass(CLASS_ID(ObservationMSISBeam));
    mrpt::utils::registerClass(CLASS_ID(ObservationIMUWithUncertainty));
    mrpt::utils::registerClass(CLASS_ID(ObservationDVL));
    mrpt::utils::registerClass(CLASS_ID(ObservationOdometry));
    mrpt::utils::registerClass(CLASS_ID(ObservationDepth));
    registerAllPendingClasses();
}

/** Main function of the program executed in its own thread 
 * 
 * @param engine GraphSlam engine executing the slam algorithm
 * @param rawlogFile File containing all the input (sensors) log data in MRPT Rawlog format
 * @param isPauseAtEachEntry If true, stop the executation at step
 * @param startTimestamp Skip the rawlog data until this timestamp
 * @param saveFinalMap If true, save the map at the end of execution (currently unused)
 * @param execute_finished Flag indication the end of the execution
 */ 
void execute(shared_ptr<GraphSlamEngine>& engine,
    const string& rawlogFile,
    bool isPauseAtEachEntry,
    uint64_t startTimestamp,
    bool saveFinalMap,
    atomic<bool>& execute_finished)
{
    CActionCollectionPtr action;
    CSensoryFramePtr observations;
    CObservationPtr observation;
    size_t curr_rawlog_entry = 0;
    CFileGZInputStream rawlogFileStream(rawlogFile);

    // Loop through the data file 
    while (CRawlog::getActionObservationPairOrObservation(
        rawlogFileStream,
        action,
        observations,
        observation,
        curr_rawlog_entry) /*&& !request_to_stop*/)
    {
        //cout << "---------------------------------------------------" << endl;
        //cout << "---------------------------------------------------" << endl;
        //cout << " -----------> Rawlog Entry : " << curr_rawlog_entry << endl;
        //cout << "---------------------------------------------------" << endl;
        //cout << "---------------------------------------------------" << endl;
        //cout << endl;

        //cout << "----> Observation : " << endl;
        //observation->getDescriptionAsText(cout);

        //------------------------------------------------
        // DEBUG : Use simple odometry
        //observation->timestamp = curr_rawlog_entry+1;
        /*if(IS_CLASS(observation, ObservationOdometry))
        {
            ObservationOdometryPtr obsOdo = static_cast<ObservationOdometryPtr>(observation);
            double dx = (curr_rawlog_entry%2) ? 0.3 : 0.1;
            obsOdo->pose_pdf = pose_pdf_t(CPose3D(dx, 0.,0.,0.01), CMatrixDouble66::Identity());
        }*/
        //------------------------------------------------
        
        // Start at defined timestamp
        if(observation->timestamp >= startTimestamp)
        {
            // Add small variances to z, pitch and roll 
            if(IS_CLASS(observation, ObservationOdometry))
            {
                // Consider pitch,roll and z as perfectly known
                ObservationOdometryPtr obsOdo = static_cast<ObservationOdometryPtr>(observation);
                obsOdo->pose_pdf.cov_inv(2,2) = 1e18;
                obsOdo->pose_pdf.cov_inv(4,4) = 1e18;
                obsOdo->pose_pdf.cov_inv(5,5) = 1e18;
            }

            engine->executeOneStep(action, observations, observation);
            
            // Press a key on the console to break the pause
            if (isPauseAtEachEntry)
                mrpt::system::pause();
        }

        //cout << "---------------------------------------------------" << endl;
        //cout << "---------------------------------------------------" << endl;
        //cout << endl;

        //request_to_stop = viewer.isExit();
    }

    /*if(saveFinalMap)
    {
        cout << "Save the final Map ... " << endl;
        engine->saveMapToFile("Map.xyz");
        engine->saveAllScans("Scans.txt");
    }*/
    execute_finished = true;
}


// Main
// ////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
    registerCustomClasses();

    // Initializing the logger instance
    shared_ptr<COutputLogger> logger = make_shared<COutputLogger>("karst-slam-app");
    logger->logging_enable_keep_record = true;
    logger->setMinLoggingLevel(LVL_DEBUG);
    try {
        // Load the .ini file
        CConfigFile configFile;
        if (!loadIniFile(argc, argv, logger, configFile))
            return -1;

        // Read configuration
        generalParameters g_params;
        readGeneralConfiguration(configFile, g_params);

        // Generate sensors as defined in the config file
        map<string, shared_ptr<Sensor>> sensors = createSensors(configFile);
        cout << "Created sensors : " << endl;
        for (const auto& pair : sensors)
            cout << "  - " << pair.first << endl;

        // Create the graphSlamEngine from the config file
        // Throw an exception if invalid parameters
        shared_ptr<GraphSlamEngine> engine = make_shared<GraphSlamEngine>(configFile, sensors, logger);

        // GUI
        GraphSlamEngineViewer viewer(configFile);
        if (g_params.enableVisuals)
            engine->attach(&viewer);

        atomic<bool> execute_finished(false);

        // Check that everything is correctly initialized
        if (engine->checkInit())
        {
            // Start !
            std::thread t(execute,
                std::ref(engine),
                std::ref(g_params.rawlogFile),
                g_params.pauseAtEachEntry,
                g_params.startTimestamp,
                g_params.saveFinalMap,
                std::ref(execute_finished));

            // Event loop
            while (!execute_finished)
            {
                viewer.queryKeyboardEvents();
                mrpt::system::sleep(10);
            }
            t.join();
            return 0;
        }
        else
            cout << "GraphSlamEngine has not been correctly initialized. Aborted." << endl;
    }
    catch (exception& e) {
        logger->logFmt(LVL_ERROR, "Program finished due to an exception!!\n%s\n",
            e.what());
        printf("%s", e.what());
        mrpt::system::pause();
        return -1;
    }
    catch (...)
    {
        logger->logFmt(LVL_ERROR, "Program finished due to an untyped exception!!");
        mrpt::system::pause();
        return -1;
    }

    return 0;
}
