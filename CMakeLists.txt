CMAKE_MINIMUM_REQUIRED(VERSION 3.0.2)
set(WORKSPACE_DIR ${CMAKE_SOURCE_DIR}/../.. CACHE PATH "root of the PID workspace directory")
list(APPEND CMAKE_MODULE_PATH ${WORKSPACE_DIR}/share/cmake/system) # using generic scripts/modules of the workspace
include(Package_Definition NO_POLICY_SCOPE)

project(karst_slam)


PID_Package(
			AUTHOR      Yohan Breux
			ADDRESS git@gite.lirmm.fr:explore-rov/localization/karst_slam.git

			YEAR        2019
			LICENSE     CeCILL
			DESCRIPTION "This project aims at implementing a SLAM adapted to Karst exploration"
			VERSION     0.1.0
		)

check_PID_Platform(CONFIGURATION suitesparse)

#now finding packages
PID_Dependency(pid-rpath)
PID_Dependency(mrpt-gtsam)
PID_Dependency(mrpt)
PID_Dependency(sophus)
PID_Dependency(libigl) # for simulation (ray tracing on 3D model)
#PID_Dependency(grassmann-averages-pca EXTERNAL)
PID_Dependency(boost) # needed for Grassman_PCA and spline interpolation

build_PID_Package()
