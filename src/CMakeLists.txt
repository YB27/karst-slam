
#declare library components
PID_Component(HEADER NAME test-utils
              DIRECTORY tests
              CXX_STANDARD 17
              EXPORT mrpt-gtsam/mrpt-gtsam-conversion-lib
              INTERNAL COMPILER_OPTIONS -Wall -Werror=return-type -Wuninitialized)

PID_Component(SHARED_LIB NAME karst-slam-common-lib
              DIRECTORY common
              CXX_STANDARD 17
              EXPORT mrpt/mrpt-graphslam mrpt/mrpt-slam mrpt/mrpt-base
              INTERNAL COMPILER_OPTIONS -Wall -Werror=return-type -Wuninitialized)

PID_Component(SHARED_LIB NAME karst-slam-graph-lib
              DIRECTORY graph
              CXX_STANDARD 17
              EXPORT  karst-slam-common-lib mrpt-gtsam/mrpt-gtsam-conversion-lib
              INTERNAL COMPILER_OPTIONS -Wall -Werror=return-type -Wuninitialized)

PID_Component(SHARED_LIB NAME karst-slam-gui-lib
              DIRECTORY gui
              CXX_STANDARD 17
              EXPORT mrpt/mrpt-opengl mrpt/mrpt-gui mrpt/mrpt-graphslam
              INTERNAL COMPILER_OPTIONS -Wall -Werror=return-type -Wuninitialized)

PID_Component(SHARED_LIB NAME karst-slam-obs-lib
              DIRECTORY obs
              CXX_STANDARD 17
              EXPORT karst-slam-common-lib karst-slam-graph-lib mrpt/mrpt-obs mrpt/mrpt-slam
              INTERNAL COMPILER_OPTIONS -Wall -Werror=return-type -Wuninitialized)

PID_Component(SHARED_LIB NAME karst-slam-sensors-lib
              DIRECTORY sensors
              CXX_STANDARD 17
              EXPORT karst-slam-obs-lib karst-slam-gui-lib
              INTERNAL COMPILER_OPTIONS -Wall -Werror=return-type -Wuninitialized)

#PID_Component(SHARED_LIB NAME karst-slam-icp-lib
#              DIRECTORY icp
#              CXX_STANDARD 17
#              EXPORT karst-slam-common-lib karst-slam-obs-lib karst-slam-gui-lib sophus/sophus
#              INTERNAL COMPILER_OPTIONS -Wall -Werror=return-type -Wuninitialized)
#PID_Component_Dependency(karst-slam-icp-lib INCLUDE_DIRS suitesparse_INCLUDE_DIRS LINKS SHARED ${suitesparse_LINK_OPTIONS})

PID_Component(SHARED_LIB NAME karst-slam-scanMerging-lib
              DIRECTORY scanMerging
              CXX_STANDARD 17
              EXPORT mrpt/mrpt-base karst_slam/karst-slam-sensors-lib  karst_slam/karst-slam-gui-lib boost/boost-math #grassmann-averages-pca/libgrassmanAveragesPCA
              INTERNAL COMPILER_OPTIONS -Wall -Werror=return-type -Wuninitialized)
PID_Component_Dependency(COMPONENT karst-slam-scanMerging-lib EXPORT INCLUDE_DIRS /home/yohan/Grassmann-Averages-PCA LINKS STATIC /home/yohan/Grassmann-Averages-PCA/build/libgrassmann_averages.a)

PID_Component(SHARED_LIB NAME karst-slam-slam-lib
              DIRECTORY slam
              CXX_STANDARD 17
              EXPORT mrpt-gtsam/mrpt-gtsam-gso-lib karst-slam-sensors-lib karst-slam-scanMerging-lib
              #EXPORT karst-slam-erd-lib karst-slam-nrd-lib karst-slam-gso-lib karst-slam-obs-lib
              INTERNAL COMPILER_OPTIONS -Wall -Werror=return-type -Wuninitialized)

PID_Component(SHARED_LIB NAME karst-slam-simulation-lib
              DIRECTORY simulation
              CXX_STANDARD 17
              EXPORT mrpt/mrpt-base libigl/libigl_opengl_glfw_imgui libigl/libigl_embree karst_slam/karst-slam-sensors-lib  karst_slam/karst-slam-scanMerging-lib boost/boost-thread
              INTERNAL COMPILER_OPTIONS -Wall -Werror=return-type -Wuninitialized)

PID_Component(SHARED_LIB NAME karst-slam-gironaDataSetLoader-lib
              DIRECTORY gironaDataSetLoader
              CXX_STANDARD 17
              EXPORT mrpt/mrpt-base karst_slam/karst-slam-obs-lib
              INTERNAL COMPILER_OPTIONS -Wall -Werror=return-type -Wuninitialized)
