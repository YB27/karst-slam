import numpy as np
import scipy
import os

""" 
    Convenient functions for writting/loading to/from files 
    Mainly used for research test / paper data
"""

def writePoseFunc(file, q):
    """
        Write a 6D pose in euler + 3d to a file 

        Parameters
        ----------
        file : str
               file to save the pose
        q : (6) array
            pose in euler + 3d
    """
    file.write(str(q[0]) + "," + str(q[1]) + "," + str(q[2]) + "," +
               str(q[3]) + "," + str(q[4]) + "," + str(q[5]) + "\n")

def writePosePDF(file, q):
    """
        Write a 6D pose pdf in euler + 3d to a file

        Parameters
        ----------
        file : str
               file to save the pose
        q : (6) array
            pose in euler + 3d
    """

    file.write(str(q["pose_mean"][0]) + "," + str(q["pose_mean"][1]) + "," + str(q["pose_mean"][2]) + "," +
               str(q["pose_mean"][3]) + "," + str(q["pose_mean"][4]) + "," + str(q["pose_mean"][5]))
    n = q["pose_cov"].shape[0]
    for i in range(0,n):
        for j in range(0,n):
            file.write("," + str(q["pose_cov"][i,j]))
    file.write("\n")

def getSubFoldersSortedList(input_folder):
    """
        Get an alphebetically sorted list of directories in the input folder

        Parameters
        ----------
        input_folder : str
                       folder
        
        Returns
        -------
        folders_sorted : list of strings
                         Sorted list of subfolders in folder
    """
    folders_int = [int(x) for x in os.listdir(input_folder) if
                   os.path.isdir(os.path.join(input_folder, x))] 
    folders_sorted = [str(x) for x in sorted(folders_int)]
    return folders_sorted

def loadDistancesInFolder_singleFile(file):
    """
        Load pose distances data (resulting from experiments). 
        Here we suppose that each algo results are saved in the same file

        Parameters
        ----------
        file : str
              file to load 
        
        Returns
        -------
        distances : list of list of float
                    list of results for each algorithm type
    """

    distances = [[], [], [], [], []]
    with(open(file, "r")) as file:
        file.readline() #header
        for line in file:
            line_parsed = line.split(',')
            for i in range(0, len(distances)):
                 distances[i].append(float(line_parsed[i]))
    return distances

def loadDistancesPosesFromFile(fileName):
    """
        Load distances and poses from file (results from experiments)

        Parameters
        ----------
        fileName : str
                   file to load

        Returns 
        -------
        distances : list of float
                    loaded distances
        poses : list of list of float
                loaded list of poses (in euler + 3d)
    """

    distances = []
    poses = []
    with open(fileName, "r") as file:
        file.readline()  # skip header
        for line in file:
            line_parsed = line.split(",")
            distances.append(float(line_parsed[0]))
            pose = []
            for i in range(1,7):
                pose.append(float(line_parsed[i]))
            poses.append(pose)
    return distances, poses

def loadDistancesInFolder(folder, suffix):
    """
        Load distances data (results from experiments) inside a folder

        Parameters
        ----------
        folder : str
                 folder where are saved the distances results
        suffix : str
                 suffix string to be added to each data file name
        
        Returns
        -------
        distances : list of list of float
                    list of distances for each algo
    """

    suffixTxt = suffix + ".txt"
    files = ["dist_init" + suffixTxt, "dist2D" + suffixTxt, "dist3D" + suffixTxt, "dist3D_allArcs"+ suffixTxt,
             #"dist3D_plane"+ suffixTxt,
             "dist3D_plane_startPoint"+ suffixTxt]
    distances = [[], [], [], [], []]
    i = 0
    for f in files:
        distances[i],_ = loadDistancesPosesFromFile(folder + "/" + f)
        i+=1

    return distances

def readNormal_(file):
    """
        Load one normal data of the estimated surface.

        Parameters
        ----------
        file : str
               file to load

        Returns
        -------
        normal_mean : (3) array
                      normal mean
        normal_cov : (3,3) ndarray
                     covariance matrix of the normal 
    """

    line = file.readline()
    line_parsed = line.split(',')
    normal_mean = np.array(
        [float(line_parsed[0]), float(line_parsed[1]), float(line_parsed[2])])
    normal_cov = []
    for j in range(3, 12):
        normal_cov.append(float(line_parsed[j]))
    normal_cov = np.array(normal_cov).reshape(3, 3)
    return normal_mean, normal_cov

def loadNormalsFromFile(estimatedNormalsFile, nonUniformIdxs):
    """
        Load normal data from a file exprresed in the first frame of two sequential scans

        Parameters
        ----------
        estimatedNormalsFile : str
                               file containing the normals data
        nonUniformIdxs : list of list
                         contains two list of non uniform distribution indexes for the first ande second scan

        Returns
        -------
        normals_firstScanFrame : dict
                                 Dict with "mean"/"cov" keys with respectively (n,3) ndarray and (n,3,3) ndarray values
                                 n = len(nonUniformIdxs[0])
        normals_secondScanFrame : dict
                                 Dict with "mean"/"cov" keys with respectively (m,3) ndarray and (m,3,3) ndarray values
                                 m = len(nonUniformIdxs[1])                        
    """

    nFirstScanSize = len(nonUniformIdxs[0])
    nSecondScanSize = len(nonUniformIdxs[1])
    normals_firstScanFrame = {"mean": np.empty((nFirstScanSize, 3)), "cov": np.empty((nFirstScanSize, 3, 3))}
    normals_secondScanFrame = {"mean": np.empty((nSecondScanSize, 3)), "cov": np.empty((nSecondScanSize, 3, 3))}
    with open(estimatedNormalsFile, "r") as file:
        file.readline()  # skip the headers
        for i in range(0, nFirstScanSize):
            normal_mean, normal_cov = readNormal_(file)
            normals_firstScanFrame["mean"][i, :] = normal_mean
            normals_firstScanFrame["cov"][i, :, :] = normal_cov

        for i in range(0, nSecondScanSize):
            normal_mean, normal_cov = readNormal_(file)
            normals_secondScanFrame["mean"][i, :] = normal_mean
            normals_secondScanFrame["cov"][i, :, :] = normal_cov

    return normals_firstScanFrame, normals_secondScanFrame

def normalsInScanReferenceFrame(normals_inFirstFrame, refFrame):
    """
        Get the normals coords (initially expressed in the first frame of a sequence of two scans)
        in a specified reference frame. 
        The normals coords are replaced in-place.

        Parameters
        ----------
        normals_inFirstFrame : dict
                               dict with "mean"/"cov" keys with respectively (n,3) ndarray and (n,3,3) ndarray values
                               This input is modified by the function.
        refFrame : dict
                   reference frame pose withe "pose_mean"/"pose_cov" key 
    """

    rot_inv = scipy.spatial.transform.Rotation.from_euler('ZYX', refFrame['pose_mean'][0:3]).inv()
    R = rot_inv.as_matrix()
    normals_inFirstFrame["mean"] = rot_inv.apply(np.array(normals_inFirstFrame["mean"]))
    normals_inFirstFrame["cov"] = np.einsum("ij,kjl->kil", R,
                                              np.einsum("kij,lj->kil", normals_inFirstFrame["cov"], R))

def loadNormals(estimatedNormalsFile, firstScan_refFrame, secondScan_refFrame, nonUniformIdxs):
    """
        Load normals data from a file expressed in the reference frame of their respective scan

        Parameters
        ----------
        estimatedNormalsFile : str
                               File containing the normal data
        firstScan_refFrame : dict
                             first scan reference frame pdf
        secondScan_refFrame : dict
                             second scan reference frame pdf 
        nonUniformIdxs : list of list
                         contains two list of non uniform distribution indexes for the first ande second scan
        
        Returns
        -------
        normals_firstScanFrame : dict
                                 Dict with "mean"/"cov" keys with respectively (n,3) ndarray and (n,3,3) ndarray values
                                 n = len(nonUniformIdxs[0])
        normals_secondScanFrame : dict
                                 Dict with "mean"/"cov" keys with respectively (m,3) ndarray and (m,3,3) ndarray values
                                 m = len(nonUniformIdxs[1])  
    """

    normals_firstScanFrame, normals_secondScanFrame = loadNormalsFromFile(estimatedNormalsFile, nonUniformIdxs)

    # Express the normals in the first/second scan reference frame
    normalsInScanReferenceFrame(normals_firstScanFrame, firstScan_refFrame)
    normalsInScanReferenceFrame(normals_secondScanFrame, secondScan_refFrame)

    return normals_firstScanFrame, normals_secondScanFrame

def readPoseMean_yprxyz(line_parsed):
    """
        Load a mean pose from a parsed line in yprxyz format

        Parameters
        ----------
        line_parsed : list of str
                      parsed line containning the mean pose

        Returns
        -------
        poseMean : (6) array
                    mean pose
    """

    poseMean = np.empty((6,))
    for i in range(0,6):
        poseMean[i] = float(line_parsed[i])
    return poseMean

def readPoseMean_notimestamp(line_parsed):
    """
        Load a mean pose from a parsed line with no timestamp
        
        Parameters
        ----------
        line_parsed : list of str
                      parsed line containning the mean pose

        Returns
        -------
        poseMean : (6) array
                    mean pose
    """

    poseMean = np.empty((6,))
    poseMean[0] = float(line_parsed[3])
    poseMean[1] = float(line_parsed[4])
    poseMean[2] = float(line_parsed[5])
    poseMean[3] = float(line_parsed[0])
    poseMean[4] = float(line_parsed[1])
    poseMean[5] = float(line_parsed[2])
    return poseMean

def readPoseMean(line_parsed):
    """
        Load a mean pose from a parsed line
        
        Parameters
        ----------
        line_parsed : list of str
                      parsed line containning the mean pose

        Returns
        -------
        poseMean : (6) array
                    mean pose
    """
    poseMean = np.empty((6,))
    poseMean[0] = float(line_parsed[4])
    poseMean[1] = float(line_parsed[5])
    poseMean[2] = float(line_parsed[6])
    poseMean[3] = float(line_parsed[1])
    poseMean[4] = float(line_parsed[2])
    poseMean[5] = float(line_parsed[3])
    return poseMean

def readPoseCov_yprxyz_(line_parsed, startIdx, endIdx):
    """
        Load a pose pdf covariance expressed in yprxyz from a parsed line
        (internal usage)

        Parameters
        ----------
        line_parsed : list of str
                      parsed line containing the pose covariance vectorized
        startIdx : int
                   index at which starts the covariance in the parsed line
        endIdx : int
                 index at which ends the covariance in the parsed line

        Returns
        -------
        cov : (6,6) ndarray
              pose pdf covariance matrix
    """

    cov = []
    for i in range(startIdx, endIdx):
        cov.append(float(line_parsed[i]))
    return np.array(cov).reshape(6,6)

def readPoseCov_yprxyz(line_parsed, isTimestamp):
    """
        Load a pose pdf covariance expressed in yprxyz from a parsed line

        Parameters
        ----------
        line_parsed : list of str
                      parsed line containing the pose covariance vectorized
        isTimestamp : bool
                      Indicate if the parsed line contains timestamp info

        Returns
        -------
        cov : (6,6) ndarray
              pose pdf covariance matrix
    """

    if(isTimestamp):
        return readPoseCov_yprxyz_(line_parsed,7,43)
    else:
        return readPoseCov_yprxyz_(line_parsed,6,42)

def readPoseCov(line_parsed, isTimestamp):
    """
        Load a pose pdf covariance expressed from a parsed line 

        Parameters
        ----------
        line_parsed : list of str
                      parsed line containing the pose covariance vectorized
        isTimestamp : bool
                      Indicate if the parsed line contains timestamp info

        Returns
        -------
        cov : (6,6) ndarray
              pose pdf covariance matrix
    """

    cov_yprxyz = readPoseCov_yprxyz(line_parsed, isTimestamp)
    poseCov = np.array(cov_yprxyz).reshape(6,6) 
    xyzCov = poseCov[0:3, 0:3].copy()
    off_block = poseCov[0:3, 3:6].copy()
    poseCov[0:3, 0:3] = poseCov[3:6, 3:6]
    poseCov[3:6, 3:6] = xyzCov
    poseCov[0:3, 3:6] = poseCov[3:6, 0:3]
    poseCov[3:6, 0:3] = off_block

    return poseCov

def readPosePDF(line):
    """
        Load a pose pdf from a file line

        Parameters
        ----------
        line : str
               line

        Returns
        -------
        timeStamp : int
                    timestamp of the pose pdf in the line
        pose : dict
               loaded pose pdf from the line
    """

    line_parsed = line.split(',')

    timeStamp = int(line_parsed[0])
    pose = {'pose_mean': readPoseMean(line_parsed), 'pose_cov': readPoseCov(line_parsed, isTimestamp=True)}

    return timeStamp, pose

def loadTrajectoryFromFile(fileName):
    """
        Load a trajectory from a file

        Parameters
        ----------
        fileName : str
                   file containing the trajectory

        Returns
        -------
        trajectory : dict
                     trajectory as a dict with timestamp as key and pose pdf as value
    """

    trajectory = {}
    with open(fileName) as file:
        file.readline() # skip the headers
        for line in file:
            timeStamp, pose_pdf = readPosePDF(line)
            trajectory[timeStamp] = pose_pdf

    return trajectory

def loadHorizontalSonarSimulatedData(horizontalSonarDataFile, firstScan_poses_relative, secondScan_poses_relative, 
                                     rho_std, psi_std, b):
    """
        Load simulated horizontal sonar data from a file

        Parameters
        ----------
        horizontalSonarDataFile : str
                                  file containing the horizontal sonar data
        firstScan_poses_relative : dict
                                   dict with timestamp as key and first scan poses expressed in the first scan reference frame as value                          
        secondScan_poses_relative : dict
                                   dict with timestamp as key and second scan poses expressed in the second scan reference frame as value
        rho_std : float
                  standart deviation for rho (sonar range)
        psi_std : float
                  stander deviation for psi (sonar rotation angle)
        b : float
            overture angle of the sonar

        Returns 
        -------
        firstScanMeasures : dict
                            dict with ['rho'], ['theta'], ['psi'], ['pose_mean'], ['pose_cov] and ['arcIdx'] keys
                            Corresponds to data related to first scan measures expressed in local spherical coords 
    
        secondScanMeasures : dict
                            dict with ['rho'], ['theta'], ['psi'], ['pose_mean'], ['pose_cov] and ['arcIdx'] keys
                            Corresponds to data related to second scan measures expressed in local spherical coords 
    
        nonUniformIdxs : list of list
                         For each scan, list of indexes corresponding to non-uniform distributions
    """

    scanMeas = [[] for i in range(2)]
    nonUniformIdxs = [[],[]]
    idx = 0
    with open(horizontalSonarDataFile, "r") as file:
        file.readline()  # skip the headers
        for line in file:
            line_parsed = line.split(',')
            scan_idx = int(line_parsed[0])
            poseInScanIdx = int(line_parsed[5])
            arcIdx = int(line_parsed[6])
            if (scan_idx == 0):
                pose = firstScan_poses_relative[poseInScanIdx]
            else:
                pose = secondScan_poses_relative[poseInScanIdx]

            alpha = float(line_parsed[3])
            beta = float(line_parsed[4])
            scanMeas[scan_idx].append({'rho': {'mean': float(line_parsed[1]), 'std': rho_std},
                                        'theta': {'alpha': alpha, 'beta': beta,
                                                  'b': b},
                                        'psi': {'mean': float(line_parsed[2]), 'std': psi_std},
                                        'pose_mean': pose['pose_mean'], 'pose_cov': pose['pose_cov'],
                                        'arcIdx': arcIdx})
            if(alpha > 1 and beta > 1):
                nonUniformIdxs[scan_idx].append(idx)
            idx += 1

    return scanMeas[0], scanMeas[1], nonUniformIdxs

def convertForArrayVersion(scanMeas, addNoise=False):
    """
        Convert the input scanMeas with format 'list of dict' into the format 'dict of list'

        Parameters
        ----------
        scanMeas : list of dict
                   input to be converted
        addNoise : bool
                   If true, add noise to the pose covariance

        Returns
        -------
        scanMeas_forArray : dict of list
                            converted data
    """
    
    scanSize = len(scanMeas)
    scanMeas_forArray = {"rho": {"mean": np.empty((scanSize,)), "std": np.empty((scanSize,))},
                         "theta": {"alpha": np.empty((scanSize,)), "beta": np.empty((scanSize,)),
                                   "b": np.empty((scanSize,))},
                         "psi": {"mean": np.empty((scanSize,)), "std": np.empty((scanSize,))},
                         "pose_mean": np.empty((scanSize, 6)), "pose_cov": np.empty((scanSize, 6, 6))}

    # Test : Add gaussian noise to all points to compensate the misalignment between clouds 
    eps = 0.01
    noise_cov = eps*np.eye(6)

    for k in range(0,len(scanMeas)):
        scanMeas_forArray["rho"]["mean"][k] = scanMeas[k]["rho"]["mean"]
        scanMeas_forArray["rho"]["std"][k] = scanMeas[k]["rho"]["std"]
        scanMeas_forArray["theta"]["alpha"][k] = scanMeas[k]["theta"]["alpha"]
        scanMeas_forArray["theta"]["beta"][k] = scanMeas[k]["theta"]["beta"]
        scanMeas_forArray["theta"]["b"][k] = scanMeas[k]["theta"]["b"]
        scanMeas_forArray["psi"]["mean"][k] = scanMeas[k]["psi"]["mean"]
        scanMeas_forArray["psi"]["std"][k] = scanMeas[k]["psi"]["std"]
        scanMeas_forArray["pose_mean"][k] = scanMeas[k]["pose_mean"]
        if(addNoise):
            scanMeas_forArray["pose_cov"][k] = scanMeas[k]["pose_cov"] + noise_cov
        else:
            scanMeas_forArray["pose_cov"][k] = scanMeas[k]["pose_cov"]
    return scanMeas_forArray