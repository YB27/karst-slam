
#declare application components
PID_Component(APPLICATION NAME karst-slam-app
              DIRECTORY karst-slam-app
              CXX_STANDARD 17
              EXPORT karst_slam/karst-slam-slam-lib boost/boost-thread
              RUNTIME_RESOURCES test_config_file_ini simplebluerov2.obj simplebluerov2.mtl)

PID_Component(APPLICATION NAME convertToRawlog_underwaterCaves
              DIRECTORY convertDatasetToRawlog/underwaterCaves
              CXX_STANDARD 17
              EXPORT mrpt/mrpt-base karst_slam/karst-slam-gironaDataSetLoader-lib)

PID_Component(APPLICATION NAME dummyRawlog
              DIRECTORY convertDatasetToRawlog/dummyRawlog
              CXX_STANDARD 17
              EXPORT mrpt/mrpt-base karst_slam/karst-slam-obs-lib)

PID_Component(APPLICATION NAME mergingScanPICP
              DIRECTORY mergingScanPICP
              CXX_STANDARD 17
              EXPORT karst_slam/karst-slam-simulation-lib pid-rpath/rpathlib
              RUNTIME_RESOURCES simulation_config_file.ini girona_config_file.ini)
PID_Component_Dependency(karst-slam-app INCLUDE_DIRS suitesparse_INCLUDE_DIRS LINKS SHARED ${suitesparse_LINK_OPTIONS})

PID_Component(APPLICATION NAME extractScansForScanMerging
              DIRECTORY extractScansForScanMerging
              CXX_STANDARD 17
              EXPORT mrpt/mrpt-base karst_slam/karst-slam-gironaDataSetLoader-lib karst_slam/karst-slam-scanMerging-lib boost/boost-thread pid-rpath/rpathlib
              RUNTIME_RESOURCES girona_config_file.ini test_config_file.ini)

PID_Component(APPLICATION NAME PICPresultsViewer
              DIRECTORY PICPresultsViewer
              CXX_STANDARD 17
              EXPORT mrpt/mrpt-base karst_slam/karst-slam-simulation-lib pid-rpath/rpathlib
	      RUNTIME_RESOURCES simulation_config_file.ini
	      INTERNAL LINKS stdc++fs)
