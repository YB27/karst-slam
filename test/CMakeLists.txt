
#declare your tests here
PID_Component(TEST_APPLICATION NAME check_gso
              DIRECTORY tests/testGSO
              CXX_STANDARD 14
              EXPORT test-utils karst-slam-slam-lib pid-rpath/rpathlib boost/boost-thread
              INTERNAL COMPILER_OPTIONS -Wall -Werror=return-type -Wuninitialized
              RUNTIME_RESOURCES test_config_file.ini)

PID_Component(TEST_APPLICATION NAME check_obs
              DIRECTORY tests/testObs
              CXX_STANDARD 14
              EXPORT test-utils karst-slam-sensors-lib pid-rpath/rpathlib
              INTERNAL COMPILER_OPTIONS -Wall -Werror=return-type -Wuninitialized
              RUNTIME_RESOURCES test_config_file.ini)


PID_Component(TEST_APPLICATION NAME check_poseGraph
    DIRECTORY tests/testPoseGraph
    CXX_STANDARD 14
    EXPORT test-utils karst-slam-graph-lib karst-slam-slam-lib mrpt-gtsam/test_utils boost/boost-thread
    INTERNAL COMPILER_OPTIONS -Wall -Werror=return-type -Wuninitialized)

PID_Component(TEST_APPLICATION NAME check_scanMerging
    DIRECTORY tests/testScanMerging
    CXX_STANDARD 14
    EXPORT test-utils karst-slam-scanMerging-lib boost/boost-thread
    INTERNAL COMPILER_OPTIONS -Wall -Werror=return-type -Wuninitialized)

PID_Component(TEST_APPLICATION NAME check_simulation
    DIRECTORY tests/testSimulation
    CXX_STANDARD 14
    EXPORT test-utils karst-slam-simulation-lib
    INTERNAL COMPILER_OPTIONS -Wall -Werror=return-type -Wuninitialized)

# Application to be run externally to test the graphSlamEngineViewer
PID_Component(TEST_APPLICATION NAME check_viewer
              DIRECTORY tests/testViewer
              CXX_STANDARD 14
              EXPORT test-utils karst-slam-slam-lib pid-rpath/rpathlib boost/boost-thread
              INTERNAL COMPILER_OPTIONS -Wall -Werror=return-type -Wuninitialized
              RUNTIME_RESOURCES test_config_file.ini testSimpleMap.gz)

run_PID_Test (NAME checking-basic-gso COMPONENT check_gso ARGUMENTS "check_basicGSO")
run_PID_Test (NAME checking-isam-gso COMPONENT check_gso ARGUMENTS "check_isamGSO")
run_PID_Test (NAME checking-isam2-gso COMPONENT check_gso ARGUMENTS "check_isam2GSO")
#run_PID_Test (NAME checking-MSIS-polarToCart COMPONENT check_obs ARGUMENTS "check_polarToCart")
run_PID_Test (NAME checking-MSIS-intensitiesToCart COMPONENT check_obs ARGUMENTS "check_intensitiesToCart")
run_PID_Test (NAME checking-MSIS-fullScan COMPONENT check_obs ARGUMENTS "check_generateFullScan")
run_PID_Test (NAME checking-gui-utils COMPONENT check_viewer ARGUMENTS "check_EigenMatToVecConversion")
run_PID_Test (NAME checking-poseGraph-relativePosePdf COMPONENT check_poseGraph ARGUMENTS "check_relativePosePdf")
run_PID_Test (NAME checking-marginals COMPONENT check_poseGraph ARGUMENTS "check_marginals")
run_PID_Test (NAME checking-ellipticCylinder COMPONENT check_scanMerging ARGUMENTS "check_ellipticCylinder")
run_PID_Test (NAME checking-observationSimulatorSonar COMPONENT check_simulation ARGUMENTS "check_observationSimulatorSonar")

# Only run automatically those tests in release as it is too long in debug (cf Eigen)
IF(CMAKE_BUILD_TYPE STREQUAL "Release")
  #run_PID_Test (NAME checking-icp-minimizeLM COMPONENT check_icp ARGUMENTS "check_minimizeLM")
  #run_PID_Test (NAME checking-icp-dataAssociation COMPONENT check_icp ARGUMENTS "check_dataAssociation")
ENDIF(CMAKE_BUILD_TYPE STREQUAL "Release")


