# Requirements for the Python scripts
Python required version : 3.7.4>
The python scripts requires the following librairies :
* scipy 1.7.2>
* numpy 1.17.4
* matplotlib 3.1.1>
* GPy (use the fork https://gite.lirmm.fr/explore-rov/localization/GPy.git where Gaussian Processes with censored data have been added)
* mayavi (for 3D display) 4.7.2
* climin 0.1a1

# Programs usage
* Main program : karst-slam-app. This program currently loads a dataset and start a PoseGraph SLAM algorithm based on using two
perpendicular acoustic sonar. The command for executing it is as follow :
	karst-slam-app <path-to-config-file>
where the default configuration file is share/resources/test_config_file.ini. 
This configuration file contains 
+ Configuration for the MRPT PoseGraph
+ Configuration for the Graph Optimizer from GTSAM
+ Configuration for the scan merging/matching (related to the research paper in /doc)
+ Other configuration (sensors, GUI, keystroke, ...)
For more details, see the comments in the configuration file. 

* mergingScanPICP. This program is used to generate data and testing the scan merging/matching algorithms in simulated environments (no SLAM here). The command for executing it is as follow :
	mergingScanPICP <path-to-config-file>
where the default configuration file is share/resources/simulation_config_file.ini.
Note that this config file contains only a subset of parameters in share/resources/test_config_file.ini related to the scan merging/matching algorithms.


commenter les .ini
merger sur integration
