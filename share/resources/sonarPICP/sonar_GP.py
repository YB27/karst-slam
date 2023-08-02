''' Temporary : Use this to use the forked version of GPy'''
import sys
sys.path.insert(1, '/home/yohan/GPy')

import GPy
import numpy as np
import math
import os
import time
import matplotlib.pyplot as plt
import poses_euler
import poses_quat
import scipy
import betaDist
import multiprocessing as mp
import climin

""" 
    Main file for learning the environment surface with a Gaussian process 
    and estimating the elevation angle distributions from a wide-beam sonar 
    as explained in the paper [1].
    Also compute the normals pdf at the estimated surface as explained in [2].

    As this code was mainly iterated from progress in the research, 
    Code strongly need a clean refactoring from scratch !!! 

    References
    ----------
    .. [1]  Breux, Yohan, and Lionel Lapierre. 
         "Elevation Angle Estimations of Wide-Beam Acoustic Sonar Measurements 
          for Autonomous Underwater Karst Exploration." Sensors 20.14 (2020): 4028.
    .. [2]  Breux, Yohan, André Mas, and Lionel Lapierre.
         "On-manifold Probabilistic ICP: Application to Underwater Karst Exploration." (2021) 
    .. [3] Groot, Perry, and Peter JF Lucas. "Gaussian process regression with censored data using expectation propagation." (2012)
"""

# Current folder of this file 
script_folder = os.path.dirname(os.path.realpath(__file__))

# Predefined colors for likelihood plots (used for debug/analysis)
colorsForLikelihoodPlot = ['b', 'g', 'm', 'r', 'k', 'c', 'b', 'g', 'orange', 'teal', 'goldenrod','lightblue', 'darkmagenta', 'lime',
'b', 'g', 'r']

# Parameters for plots
params = {
        'axes.labelsize': 15,
        'font.size': 20,
        'legend.fontsize': 10,
        'xtick.labelsize': 15,
        'ytick.labelsize': 15,
        'text.usetex': False,
        'figure.figsize': [9, 7]
        }
plt.rcParams.update(params)

# Horizontal sonar beam width
beamWidth = math.radians(35)
beamWidth_sqr = beamWidth**2
halfBeamWidth = beamWidth/2.

# Number of step for elevation angle sampling 
nStep = 200 # !!! Should be the same as in c++ engine -> TODO : pass it as argument

# Corresponding angle delta for each step
step_theta = beamWidth/float(nStep)
step_theta_sqr = step_theta**2
inv_2_step_theta = 1./(2.*step_theta)

# Parameters for local maximum detection (cf [1], eq(18) of the paper)
min_nDecreaseStep = 2  # number of successive decreasing steps after a local maximum
min_nIncreaseStep = 2  # number of succesive increasing steps before a local maximum

# Maximum variance. Considered as a uniform distribution above this value 
# Corresponds to sigma_m in the sensor paper ([1], equation (51)) with here k=3 
var_max = (beamWidth**2)/36

# Log-likelihood min threshold
LogLikelihoodThreshold = np.log(0.001)

# Abscisse for plot beta distributions 
theta_x = np.arange(-halfBeamWidth, halfBeamWidth, 0.001)

def var_max_valid(theta_mean, b):
    """ 
        Compute the maximum variance for elevation angle leading to valid Beta distribution. 
        
        Parameters
        ----------
        theta_mean : float
                    mean of the theta distribution
        b : float
            sonar aperture

        Returns 
        -------
        max_var : float
                 maximum valid variance for the theta distribution

        Notes
        -----
        Corresponds to sigma_l in [1], equation (47)(50) 
    """

    theta_mean_abs = np.abs(theta_mean)
    return (b - 2.*theta_mean_abs) * (b + 2.*theta_mean_abs)**2 / (4. * (3.*b + 2.*theta_mean_abs))

def loadTrainingData(fileName):
    """ 
        Function to load the file containing the training (vertical sonar) data
        The format is expected to be as follows:
        
        TODO : The names are bad !!! Should better use "verticalSonarData" instead of "Training" and 
        "HorizontalSonarData" instead of "validation". 
        This notation made sense when only dealing with the training of the Gaussian process. 

        Parameters
        ----------
        fileName : str
                   file to be loaded
        
        Returns
        -------
        inputData : (n,2) array
                    input data (curvilinear abcissa s and sonar rotation angle psi)
        outputData : (n) array
                     output data (sonar range rho)
        priorData : (n) array
                    prior data (sonar range rho)
        censoredData : (n) array of int 
                       indicates if the data is censored or not (ie max range sonar)  
        
        Notes
        -----
        The prior rho range is explained in [1], section 4.2.1
        Censored data are used to take into account that sonar measures have a maximum bound.
    """

    listInputData = []
    listOutputData = []
    listPriorData = []
    listCensoredData = []
    with open(fileName) as file:
        file.readline() # skip the headers
        for line in file:
            line_parsed = line.split(',')
            vals = []
            for i in range(0,2):
                vals.append(float(line_parsed[i]))

            listCensoredData.append(int(line_parsed[4]))
            listInputData.append(vals)
            listOutputData.append([float(line_parsed[2])])
            listPriorData.append([float(line_parsed[3])])

    # Convert to ndarray
    return np.asarray(listInputData), np.asarray(listOutputData), np.asarray(listPriorData), np.asarray(listCensoredData)

def loadValidationData(fileName) :
    """ 
        Function to load the file containing the validation (horizontal sonar) data

        Parameters
        ----------
        fileName : str
                   file to be loaded

        Returns
        -------
        inputData : (n,m) array
                    validation data. 
        
        Notes
        -----
        Contains also data for the normal computation. 
        See the C++ function void surfaceValidationData::save(const string& fileName) const
    """

    listInputData = []
    with open(fileName) as file :
        file.readline() # skip the headers
        for line in file:
            line_parsed = line.split(',')
            vals = []
            for i in range(0,len(line_parsed)):
                vals.append(float(line_parsed[i]))
            listInputData.append(vals)

    # Convert to ndarray
    return np.asarray(listInputData)

def GP_predict_batch(m, input_data, size_batch):
    """ 
        Gaussian process inference by batch 

        Parameters
        ----------
        m : Gaussian Process model
        input data : (n,2) array
                     input data for GP prediction
        size_batch : int
                     batch size

        Returns
        -------
        pred_mean : (n) array
                    predicted mean rho for each input datum
        pred_var : (n) array 
                    predicted variance of rho for each input datum
    """ 

    # Predict the mean and variance range for each point in data_vals 
    # Process it by batch to avoid RAM overflow
    nData = input_data.shape[0]
    nBatch = int(nData) // int(size_batch)
    pred_mean = np.empty((nData, 1))
    pred_var = np.empty((nData, 1))
    startIdx = 0
    endIdx = 0
    for i in range(0, nBatch):
        startIdx = i * size_batch
        endIdx = startIdx + size_batch
        pred_mean[startIdx:endIdx], pred_var[startIdx:endIdx] = m.predict_noiseless(input_data[startIdx:endIdx, :])

    lastBatch = nData - endIdx
    if (lastBatch > 0):
        pred_mean[endIdx:], pred_var[endIdx:] = m.predict_noiseless(np.asarray(input_data[endIdx:]))

    # Flatten 
    pred_mean = pred_mean.flatten()
    pred_var = pred_var.flatten()

    return pred_mean, pred_var

def GP_predict_multithread(m, input_data):
    """ 
        Multithreaded gaussian process inference 

        Parameters
        ----------
        m : Gaussian Process model
        input_data : (n,2) array
                     input data for GP prediction
        
        Returns
        -------
        pred_mean : (n) array
                    predicted mean rho for each input datum
        pred_var : (n) array 
                    predicted variance of rho for each input datum
    """

    nCpu = 3
    nSize = input_data.shape[0]
    size_per_cpu = nSize // nCpu
    input_data_list = []
    startIdx = 0

    print("nCpu  : {}".format(nCpu))
    print("nSize : {}".format(nSize))
    print("size_per_cpu : {}".format(size_per_cpu))
    print("leftovers    : {}".format(nSize - nCpu*size_per_cpu))
    for i in range(0, nCpu-1):
        endIdx = startIdx + size_per_cpu
        print("Start / end Idxs : {},{}".format(startIdx, endIdx))
        input_data_list.append(input_data[startIdx:endIdx,:])
        startIdx = endIdx
    
    # Add leftovers to the last thread
    input_data_list.append(input_data[startIdx:, :])

    pool = mp.Pool(nCpu)

    print("-----------------------------------------------------")
    print(" START MULTIPROCESSING REGRESSION " )
    print("-----------------------------------------------------")
    res = pool.map(m.predict_noiseless, input_data_list)

    print(res)

    # Concatenate the results 
    pred_mean = np.empty((nSize, 1))
    pred_var = np.empty((nSize, 1))
    startIdx = 0
    for mean, var in res:
        endIdx = startIdx + mean.shape[0]
        pred_mean[startIdx : endIdx] = mean
        pred_var[startIdx : endIdx] = var    

    # Flatten
    pred_mean = pred_mean.flatten()
    pred_var = pred_var.flatten()

    return pred_mean, pred_var

def generateSampledEstimatedSurface(m, dataFolder, s_vals, yaw_vals, idx_vals, size_batch):
    """ 
        Function which samples the estimated surface at each pose from the training data
        Simply consider each curvilinear abscissa s in the input data and predict the points for 

        Only used for display purpose in the C++ code

        Parameters
        ----------
        m : Gaussian Process model
        dataFolder : str
                     folder containing the input data on which the GP has been trained
        s_vals : (n) array, output
                 curvilinear abscissa used for generating points on the estimated surface by GP
        idx_vals : (n) array, output
                   corresponding index (or timestamp) in the input data
        size_batch : float
                     batch size

        Returns
        -------
        pred_mean : (n) array
                    predicted mean rho for each input datum
        pred_var : (n) array 
                    predicted variance of rho for each input datum
        low_est : (n) array
                  array of predicted rho - 3*sigma at (s,yaw)
        high_est : (n) array
                   array of predicted rho + 3*sigma at (s,yaw)  
    """
    
    # File containing the curvilinear abscissa data 
    # Only used to sample data points for 3d display of the estimated surface
    # TODO: Not needed to use abscissa corresponding to the robot poses
    # Just sample at regular interval ?
    curvilinearAbscissaFile = dataFolder + '/curvilinearAbscissa.txt'

    # Load time stamp and curvilinear abscissa data
    s_val_input = []
    timeStamps = []
    with open(curvilinearAbscissaFile) as f :
        f.readline() # skip the headers
        for line in f:
            line_parsed = line.split(',')
            timeStamps.append(int(line_parsed[0]))
            s_val_input.append(float(line_parsed[1]))       
    
    # Set the yaw sampling and the corresponding step
    nSample_yaw = 60
    step_yaw = 2. * np.pi / nSample_yaw

    # Create a matrix data_vals where each line is a point (s,yaw) 
    data_vals = []
    for s_val_input, ts in zip(s_val_input, timeStamps):
        curYaw = -np.pi
        for j in range(0, nSample_yaw):
            s_vals.append(s_val_input)
            yaw_vals.append(curYaw)
            datum_vals = []
            datum_vals.append(s_val_input)
            datum_vals.append(curYaw)
            data_vals.append(datum_vals)
            idx_vals.append(ts)
            curYaw = curYaw + step_yaw

    data_vals = np.asarray(data_vals)
    pred_mean, pred_var = GP_predict_batch(m, data_vals, size_batch=size_batch)

    # Get ranges at +- 3*sigma 
    pred_std = np.sqrt(pred_var)
    low_est = pred_mean - 3.*pred_std
    high_est = pred_mean + 3.*pred_std

    return pred_mean, pred_var, low_est, high_est

def generateSampledSurfaceSlices(s_vals_input, yaw_vals):
    """ 
        Function which samples the estimated surface only on planes for computing 2D slice graphs 
        Only used for generating figures 13/14 in [1]

        Parameters
        ----------
        s_vals_input : (n) array
                        array of curvilinear abscissa on a generated surface from the GP
        yaw_vals : (m) array
                    array of sonar rotation angle used to generate surface from the GP
    
        Returns
        -------
        res : (l,5) array
              array of data on a slice of the surface : 
              curvilinear abcissa s, rotation angle psi, predicted rho mean and lower/upper bound at +- 3sigma
    """

    data_vals = []
    s_vals = []
    used_yaw_vals = []

    #decimate data
    for yaw_val in yaw_vals:
        i = 0
        for s_val_input in s_vals_input:
            if(i%8 == 0):
                s_vals.append(s_val_input)
                used_yaw_vals.append(yaw_val)
                data_vals.append([s_val_input,yaw_val])
            i += 1

    # Predict the mean and variance range for each point in data_vals 
    pred_mean, pred_var = m.predict(np.asarray(data_vals))
    #del data_vals

    # Flatten 
    pred_mean = pred_mean.flatten()
    pred_std = np.sqrt(pred_var.flatten())

    upperBound = pred_mean + 3.*pred_std
    lowerBound = pred_mean - 3.*pred_std

    res = np.empty((len(s_vals),))
    res[:,0] = s_vals
    res[:,1] = used_yaw_vals
    res[:,2] = pred_mean
    res[:,3] = lowerBound
    res[:,4] = upperBound

    return res

def saveSamplesSlice(data, planeName, kernel_s, kernel_yaw):
    """
        Save the sampled slice (see generateSampleSlices) 
        Only used for generating figures 13/14 in [1]

        Parameters
        ----------
        planeName : str
                    name of the slice plane
        kernel_s : str
                    name of the kernel on curvilinear abscissa
        kernel_yaw : str
                    name of the kernel on sonar rotation angle
    """

    fileName = script_folder  + "/SlicesData/" + kernel_s + "_" + kernel_yaw + "/" + planeName + ".txt"
    with open(fileName,"w") as file :
        file.write("#s, yaw, mean, low, high \n")
        for pt in data:
	        file.write(str(pt[0]) + "," + str(pt[1]) + "," + str(pt[2]) + "," + str(pt[3]) + "," + str(pt[4]) + "\n")

def generateSampleSlices(s_vals_input,kernel_s, kernel_yaw):
    """ 
        Generate slices of the surface 
        Only used for generating figures 13/14 in [1] 

        Parameters
        ----------
        s_vals_input : (n) array
                        array of curvilinear abscissa on a generated surface from the GP
        kernel_s : str
                    name of the kernel on curvilinear abscissa
        kernel_yaw : str
                    name of the kernel on sonar rotation angle
    """

    print("Generate sample slices ...")
    # XY-plane 
    print(" Generate XYslice ...")
    XYslice = generateSampledSurfaceSlices(s_vals_input, [0.5 * np.pi])
    print(" Save XYslice ...")
    saveSamplesSlice(XYslice, "XYslice",kernel_s, kernel_yaw)

    # XZ-plane 
    print(" Generate XZslice ...")
    XZslice = generateSampledSurfaceSlices(s_vals_input, [0, np.pi])
    print(" Save XZslice ...")
    saveSamplesSlice(XZslice, "XZslice",kernel_s, kernel_yaw)

    # ZY-plane  
    n = len(s_vals_input)
    middle = int(0.5*float(n))
    yaw_vals = np.arange(-np.pi,np.pi,np.deg2rad(1))
    print(" Generate ZYslice ...")
    ZYslice = generateSampledSurfaceSlices([s_vals_input[middle]],yaw_vals)
    print(" Save ZYslice ...")
    saveSamplesSlice(ZYslice, "ZYslice",kernel_s, kernel_yaw)

def displayEstimatedSurface(s_vals, yaw_vals, pred_mean, low_est, high_est):
    """ 
        Function to display the 2D surface in (s,yaw,range) coords of the estimated surface 

        Parameters
        ----------
        s_vals : (n) array
                 array of curvilinear abscissa on a generated surface from the GP
        yaw_vals : (n) array
                    array of sonar rotation angle used to generate surface from the GP
        pred_mean : (n) array
                    array of predicted rho at (s,yaw)
        low_est : (n) array
                  array of predicted rho - 3*sigma at (s,yaw)
        high_est : (n) array
                   array of predicted rho + 3*sigma at (s,yaw)        
    """

    ax = plt.axes(projection="3d")
    ax.scatter(s_vals, yaw_vals, pred_mean, c='g', marker='+', alpha=0.5)
    ax.scatter(s_vals, yaw_vals, low_est, c='r', marker='o', alpha=0.5)
    ax.set_xlabel('s')
    ax.set_ylabel('yaw')
    ax.set_zlabel('range')

def saveEstimatedSurface(dataFolder, idx_vals,s_vals, yaw_vals, pred_mean, low_est, high_est):
    """ 
        Function to save the estimated surface to be further displayed in the c++ 3D viewer

        Parameters
        ----------
        dataFolder : str
                     folder to saved the estimated surface 
        idx_vals : (n) array, output
                   corresponding index (or timestamp) in the input data
        s_vals : (n) array, output
                 curvilinear abscissa used for generating points on the estimated surface by GP   
        yaw_vals : (n) array
                    array of sonar rotation angle used to generate surface from the GP
        pred_mean : (n) array
                    array of predicted rho at (s,yaw)
        low_est : (n) array
                  array of predicted rho - 3*sigma at (s,yaw)
        high_est : (n) array
                   array of predicted rho + 3*sigma at (s,yaw)   
    """

    print("Save estimated surface ...")
    with open(dataFolder + "/res_estimatedSurface.txt", "w") as file:
        for idx, s, yaw, mean, low, high in zip(idx_vals, s_vals, yaw_vals, pred_mean, low_est, high_est):
            file.write(str(idx) + "," + str(s) + "," + str(yaw) + "," + str(mean) + "," + str(low) + "," + str(high) + "\n")

def saveDistributionBeta(dataFolder, data):
    """ 
        Function to save the parameters of estimated beta distribution for each estimated theta 

        Parameters
        ----------
        dataFolder : str
                     folder to save the data
        data : (n,6) array
               contains data on elevation angle (theta) distribution :
                - pt_idx : index of the point corresponding to the elevation angle
                - theta_idx : index of the theta (cf theta are sampled along the sonar arc)
                - alpha : alpha parameter of the beta distribution
                - beta  : beta parameter of the beta distribution
                - r : sonar range rho
                - r_prior : sonar range prior
    """

    with open(dataFolder + "/res_distributionBeta.txt", "w") as file:
        for datum in data:
            ''' datum : pt_idx, theta_idx, alpha, beta , r, r_prior '''
            file.write(str(datum[0]) + "," + str(datum[1]) + "," + str(datum[2]) + "," + str(datum[3]) + ","  +  str(datum[4]) + ","  + str(datum[5]) + "\n")

def plotLikelihoodAndFisherInfo(ThetaVals, probaPerTheta, fisher_info, local_maximum_theta,
                                ranges, ranges_prior, mean_ranges, var_ranges, dsigma_sqr,  dmean_sqr, beamWidth):
    """ 
        Plot the likelihood ([1], equations (47/50)) and fisher information ([1], equations (42-44)) graphs
        similarly to Figure 10.

        Parameters
        ----------
        ThetaVals : (n) array
                    values of sample theta in the sonar arc 
        probaPerTheta : (n) array
                        p(rho^v | theta) likelihood value at each sampled theta
        fisher_info : (n) array
                     fisher information at each sampled theta
        local_maximum_theta : (l,2) array
                              contains each local likelihood maxima for theta (value and sample idx)
        ranges : (n) array
                 range as if 3D point corresponding to each sampled theta was measured by vertical sonar
        ranges_prior : (n) array 
                        prior range at each sampled theta
        mean_ranges : (n) array
                      estimated mean of ranges r^v by GP at each sampled theta
        var_ranges : (n) array
                     estimated variances of ranges r^v by GP at each sampled theta              
        dsigma_sqr : (n-2) array
                     square of the derivative of variance of rho^v 
        dmean_sqr : (n-2) array
                    square of the derivative of mean of rho^v      
        beamWidth : float 
                    beam aperture angle      
    """             
    
    i = 0

    n = len(ThetaVals)
    plt.figure(0)
    plt.subplot(2,1,1)
    plt.ylabel("dmean_sqr")
    plt.plot(ThetaVals[1:n-1], dmean_sqr)

    plt.subplot(2,1,2)
    plt.ylabel("dsigma_sqr")
    plt.plot(ThetaVals[1:n-1], dsigma_sqr)

    plt.figure(1)
    plt.tight_layout()
    plt.subplot(2, 2, 1)
    plt.ylabel('f(r^v | theta) unormalized (pdf)')
    # for data in local_maximum:
    print("local_maximum_theta size {}".format(len(local_maximum_theta)))
    for loc_theta, theta_idx,_ in local_maximum_theta:
        print("theta_idx : {}".format(theta_idx))
        plt.axvline(x=loc_theta, c=colorsForLikelihoodPlot[i], ls='--', lw=2)
        i = i + 1
    plt.ticklabel_format(axis='y', style='sci', scilimits=(0, 5))
    plt.plot(ThetaVals, probaPerTheta, '-k', label="Data", lw=2.5)

    plt.subplot(2, 2, 3)

    plt.ylim(0, 0.1)
    plt.xlabel('Theta (rad)')
    plt.ylabel('Inverse Fisher information')

    i = 0
    # for data in local_maximum:
    for loc_theta,_,_ in local_maximum_theta:
        plt.axvline(x=loc_theta, c=colorsForLikelihoodPlot[i], ls='--', lw=2)
        i = i + 1
    plt.ticklabel_format(axis='y', style='sci', scilimits=(0, 5))
    plt.plot(ThetaVals, fisher_info, '-r', lw=2.5)
   # plt.savefig("fisher_info.eps", bbox_inches='tight')
    
    var_thresholds = [var_max_valid(theta, beamWidth) for theta in ThetaVals]
    plt.plot(ThetaVals, var_thresholds, color='b', ls='--')

    std_ranges = np.sqrt(var_ranges)
    plt.subplot(2, 2, 2)
    plt.xlabel("Theta (rad)")
    plt.ylabel("mean rho^v")
    upperBounds = mean_ranges + 3.*std_ranges
    lowerBounds = mean_ranges - 3.*std_ranges
    plt.plot(ThetaVals, mean_ranges, color='k', label = "r^v mean")
    plt.plot(ThetaVals, lowerBounds, color='b', label = "3-sigma Bounds")
    plt.plot(ThetaVals, upperBounds, color='b')
    plt.fill_between(ThetaVals, lowerBounds, upperBounds, facecolor='lightblue', alpha=0.5)
    plt.plot(ThetaVals, ranges, color='r', label = "Measured r^v")
    plt.legend()
    
    plt.subplot(2, 2, 4)
    plt.xlabel("Theta (rad)")
    plt.ylabel("var_rho^v")
    plt.plot(ThetaVals, std_ranges)
    #plt.plot(ThetaVals, np.log(ranges) - np.log(ranges_prior))
    plt.show()

def computeInverseFisherInformations(mean_ranges, var_ranges):
    """
        Compute the inverse Fisher information 

        Parameters
        ----------
        mean_ranges : (n) array
                      mean of rho^v for each sampled theta
        var_ranges : (n) array
                      variance of rho^v for each sampled theta

        Returns 
        -------
        invFisher_info : (n) array
                         inverse fisher information
        dsigma_r_sqr : (n-2) array
                       square of the derivative of variance of rho^v 
        dmean_sqr : (n-2) array
                    square of the derivative of mean of rho^v

        Notes
        -----
        See [1], equation (44) 
    """

    n = len(mean_ranges)
    invFisher_info = np.zeros((n,))
    invFisher_info[0] = 0

    sigma_ranges = np.sqrt(var_ranges)

    # mean_ranges[i+1], mean_ranges[i-1]
    mean_ranges_plus = mean_ranges[2:]
    mean_ranges_minus = mean_ranges[:n-2]
    sigma_ranges_plus = sigma_ranges[2:]
    sigma_ranges_minus = sigma_ranges[:n-2]

    dsigma_r_sqr = np.power(sigma_ranges_plus - sigma_ranges_minus,2) 
    dmean_r_sqr = np.power(mean_ranges_plus - mean_ranges_minus,2)

    invFisher_info[1:n-1] = step_theta_sqr*var_ranges[1:n-1]/(2.*dsigma_r_sqr + dmean_r_sqr)
    invFisher_info[n-1] = 0

    return invFisher_info, dsigma_r_sqr, dmean_r_sqr

def plotBetaDistribution(distribution_beta):
    """ 
        Plot the given beta distributions (for debug).

        Parameters
        ----------
        distribution_beta : (n,6) array
                            array of beta distribution (which is also an array)
                            - pt_idx : index of the point corresponding to the elevation angle
                            - theta_idx : index of the theta (cf theta are sampled along the sonar arc)
                            - alpha : alpha parameter of the beta distribution
                            - beta  : beta parameter of the beta distribution
                            - r : sonar range rho
                            - r_prior : sonar range prior

        Notes
        -----
        Produce plots as in [1], Figure 10c 
    """

    i = 0
    for dist in distribution_beta:
        y = beta.pdf(theta_x, a=dist[2], b=dist[3], loc=-halfBeamWidth, scale=beamWidth)
        plt.plot(theta_x,y,c=colorsForLikelihoodPlot[i],lw=2.5)
        i += 1
    plt.savefig("beta_distribution.eps", bbox_inches='tight')

def estimateBetaDistributions(m, validationData_input, size_batch, verbose=False, plotBetaDist=False):
    """ 
        Estimate the beta distributions for each horizontal sonar measures (validationData) 

        Parameters
        ----------
        m : Gaussian Process model
        validationData_input : (n,5) array
                               input data from the horizontal sonar measurements (pt_index, s, psi^v, rho^v, range_prior)
        size_batch : int
                     batch size
        verbose : bool
                  if true, display more detailed output/warning. 
        plotBetaDist : bool
                       if true, stop at each beta distribution and plot it  

        Returns
        -------
        distribution_beta : (n,6) array
                            array of beta distribution (which is also an array)
                            - pt_idx : index of the point corresponding to the elevation angle
                            - theta_idx : index of the theta (cf theta are sampled along the sonar arc)
                            - alpha : alpha parameter of the beta distribution
                            - beta  : beta parameter of the beta distribution
                            - r : sonar range rho
                            - r_prior : sonar range prior
        validationData_input : (m,5) array
                                subset of validationData_input for which the theta distribution is not uniform

        Notes
        -----
        Set verbose to true for analysis / debugging. Otherwise, it is preferable to set to false.
    """ 

    distribution_beta = []
    indexesForNormalData = []

    # Parse the data 
    indexes = validationData_input[:,0]
    input_data = validationData_input[:,[1,2]]
    ranges = validationData_input[:,3]
    thetas = validationData_input[:,4]
    ranges_prior = validationData_input[:,5]

    # GP predictions  
    print("input_data size {}".format(input_data.shape))
    start = time.time()
    mean_ranges, var_ranges = GP_predict_batch(m, input_data, size_batch=size_batch)
    comp_time = time.time() - start
    print("GP regression in beta dist in {} s:".format(comp_time))

    # Log-likelihoods
    ln_var_ranges = np.log(var_ranges)
    ln_2_pi_half = 0.5*np.log(2.*np.pi)
    sqr_rmean_minus_r = np.power(mean_ranges - ranges, 2)
    ln_likelihoods = -(sqr_rmean_minus_r/(2.*var_ranges) + 0.5*ln_var_ranges + ln_2_pi_half)
    #probaPerTheta = np.exp(ln_likelihoods)

    # Search for the local maxima
    prev_ln_likelihood = -100000000 # Previous maximum log-likelihood
    decreaseStep = 0 # Number of consecutive decreasing steps
    increaseStep = 0 # Number of consecutive increaseing steps
    isIncrease = False # True if log-likelihood is increasing at the current angle
    localMaxima = [] # [idx, theta_val]
    prev_arc_idx = indexes[0] # Previous arc idx 
    prev_theta = thetas[0] # Previous theta (elevation angle) value
    theta_idx = 0 # Current theta index
    end_last_arc_loop_idx = 0 # Loop index of the ending of last arc 
    current_loop_idx = -1 # Current loop index
    for cur_arc_idx, cur_theta, range, range_prior, cur_ln_likelihood in zip(indexes,
                                                                             thetas,
                                                                             ranges,
                                                                             ranges_prior,
                                                                             ln_likelihoods):
        # Update current loop index
        current_loop_idx += 1

        # Still on the same arc ? (one arc correspond to one horizontal measure (beam)) 
        if(cur_arc_idx == prev_arc_idx):
            # First theta ?
            if(theta_idx - 1 == 0 and prev_ln_likelihood > LogLikelihoodThreshold):
                localMaxima.append([prev_theta, theta_idx - 1, current_loop_idx-1])
            
            # Log-likelihood increasing 
            if (prev_ln_likelihood <= cur_ln_likelihood):
                isIncrease = True
                increaseStep += 1

                # Remove the previous maximum as it is not (== noise)
                if (decreaseStep < min_nDecreaseStep and len(localMaxima) > 0):
                    localMaxima.pop()
            # Log-likelihood decreasing         
            elif (isIncrease):
                # Prev data was a local maximum -> add it to the localMaxima list
                if (prev_ln_likelihood > LogLikelihoodThreshold):
                    if (theta_idx == 0 or increaseStep >= min_nIncreaseStep):
                        localMaxima.append([prev_theta, theta_idx - 1, current_loop_idx-1])
                # Reset the steps
                decreaseStep = 1
                increaseStep = 0
                isIncrease = False
            else:
                decreaseStep += 1
        # Now starting on a new arc -> Process the previous arc data
        else:
            # Compute fisher information
            # TODO only compute fisher information for maximum ?
            #  (originally compute for all points to see how look the curve)
            invFisher_info, dsigma_r_sqr, dmean_r_sqr = computeInverseFisherInformations(
                                                              mean_ranges[end_last_arc_loop_idx:current_loop_idx],
                                                              var_ranges[end_last_arc_loop_idx:current_loop_idx])                                              
            # For each local maxima
            for mean_theta, mean_theta_idx, loop_idx in localMaxima :
                # Compute its variance with the inverse fisher information (see [1], section 4.2.4)
                var_theta = invFisher_info[mean_theta_idx]
                if (var_theta > 0):
                    # Get the maxiumum valid variance threshold (see [1],section 4.2.4 and Figure 9)
                    maximum_var = var_max_valid(mean_theta, beamWidth) 
                    # Valid variance
                    if (var_theta <= maximum_var):
                        # Here we consider the mean theta as the mode
                        alpha_, beta_ = betaDist.alphaBetaFromModeVar(mean_theta, var_theta, beamWidth)

                        # Ignore invalid values of alpha and beta.
                        # Note that we ignore bimodal distribution (ie alpha and beta < 0)
                        # and uniform distribution (alpha = beta)
                        if (alpha_ > 0 and beta_ > 0 and (alpha_ >= 1 or beta_ >= 1)
                           and np.fabs(alpha_ - beta_) > 1e-1):
                            distribution_beta.append([prev_arc_idx, mean_theta_idx, alpha_, beta_, prev_range, prev_range_prior])
                            ''' Keep the corresponding data related to normal computation '''
                            indexesForNormalData.append(loop_idx)
                        else:
                            if(verbose):
                                print("Reject distribution for arc {} with alpha/beta : {}/{}".format(prev_arc_idx,alpha_,beta_))
                    else:
                        if(verbose):
                            print("Reject arc {} for high variance : {} (max : {})".format(prev_arc_idx, var_theta, maximum_var))
                else:
                    if(verbose):
                        print("Negative variance for arcIdx {}".format(prev_arc_idx))
                if(plotBetaDist):
                    plotBetaDistribution(distribution_beta)

	    ##################################################################
	    # Uncomment to display the likelihood curve for some arcs
            '''
            print("Prev_arc_idx = {}".format(prev_arc_idx))
            curArc_probaPerTheta = probaPerTheta[end_last_arc_loop_idx:current_loop_idx]
            if (len(curArc_probaPerTheta) > 2):
                # Normalize proba per theta
                normalizeProbaPerTheta(curArc_probaPerTheta)

            if(prev_arc_idx == 242):
                plotLikelihoodAndFisherInfo(thetas[end_last_arc_loop_idx:current_loop_idx],
                                            curArc_probaPerTheta,
                                            invFisher_info,
                                            localMaxima,
                                            ranges[end_last_arc_loop_idx:current_loop_idx], 
                                            ranges_prior[end_last_arc_loop_idx:current_loop_idx],
                                            mean_ranges[end_last_arc_loop_idx:current_loop_idx], 
                                            var_ranges[end_last_arc_loop_idx:current_loop_idx],
                                            dsigma_r_sqr, dmean_r_sqr, beamWidth)'''
	    ##################################################################

            end_last_arc_loop_idx = current_loop_idx

            # Here, we finished to process an arc 
            # Reset intermediate values before processing the next arc
            localMaxima = []
            theta_idx = 0

        # Updates
        prev_ln_likelihood = cur_ln_likelihood
        prev_arc_idx = cur_arc_idx
        prev_theta = cur_theta
        prev_range = range
        prev_range_prior = range_prior
        theta_idx = theta_idx + 1

    return distribution_beta, validationData_input[indexesForNormalData, :]
 
def crossProd(a,b):
    """ 
        Cross product a X b for two 3d vectors

        Parameters
        ----------
        a : (3) array
            first vector
        b : (3) array
            second vector

        Returns
        -------
        c : (3) array
            cross product a X b     
    """

    return np.array([a[1]*b[2] - a[2]*b[1],
                     a[2]*b[0] - a[0]*b[2],
                     a[0]*b[1] - a[1]*b[0]])

def computeNormal(a0,a1,a2,a3,b0,b1,
                  rho):
    """ 
        Compute normals at point cloud from ranges rho_s_plus, rho_s_minus, rho_phi_plus, rho_phi_plus
        and the coefficient a0, a1, a2, a3, b0, b1.
        
        Parameters
        ----------
        a0 : (3) array
             term associated with the term rho_s_plus*rho_psi_plus in the normal random variable expression
        a1 : (3) array
             term associated with the term rho_s_minus*rho_psi_minus in the normal random variable expression
        a2 : (3) array
             term associated with the term rho_s_minus*rho_psi_plus in the normal random variable expression
        a3 : (3) array
             term associated with the term rho_s_plus*rho_psi_minus in the normal random variable expression
        b0 : (3) array
             term associated with the term rho_psi_plus in the normal random variable expression
        b1 : (3) array
             term associated with the term rho_psi_minus in the normal random variable expression

        Returns
        -------
        v : (3) array
            random variable expression as defined in [2], equation (69)

        Notes
        -----
        See [2], section 5.3 and equation (69)
    """
    
    return rho[0]*rho[1]*a0 + rho[2]*rho[3]*a1 + \
           rho[2]*rho[1]*a2 + rho[0]*rho[3]*a3 + \
           rho[1]*b0 + rho[3]*b1

def computeNormalMean_unnormalized(a0,a1,a2,a3,b0,b1,
                      pred_rho_mean,
                      pred_rho_cov):
    """ 
        Compute the unnormalized normal mean 
        
        Parameters
        ----------
        a0 : (3) array
             term associated with the term rho_s_plus*rho_psi_plus in the normal random variable expression
        a1 : (3) array
             term associated with the term rho_s_minus*rho_psi_minus in the normal random variable expression
        a2 : (3) array
             term associated with the term rho_s_minus*rho_psi_plus in the normal random variable expression
        a3 : (3) array
             term associated with the term rho_s_plus*rho_psi_minus in the normal random variable expression
        b0 : (3) array
             term associated with the term rho_psi_plus in the normal random variable expression
        b1 : (3) array
             term associated with the term rho_psi_minus in the normal random variable expression
        pred_rho_mean : float
                        mean of the predicted rho (sonar range) by the learned GP surface
        pred_rho_cov : float
                       variance of the predicted rho (sonar range) by the learned GP surface

        Returns
        -------
        v : (3) array
            unnormalized mean normal

        Notes
        -----
        Defined in [2], equation (70) 
    """

    n = computeNormal(a0,a1,a2,a3,b0,b1,
                  pred_rho_mean)

    return n + pred_rho_cov[0][1]*a0 + pred_rho_cov[2][3]*a1 + pred_rho_cov[1][2]*a2 + pred_rho_cov[0][3]*a3

def expectation_order4(cov_uv, cov_uw, cov_uq, cov_vw, cov_vq, cov_wq,
                       mean_u, mean_v, mean_w, mean_q):
    """ 
        Compute the expectation of the 4th order terms E(UVWQ) for 4 gaussian RV U,V,W,Q
        
        Parameters
        ----------
        cov_uv : float 
                 covariance of the RV U and V
        cov_uw : float
                 covariance of the RV U and W
        cov_uq  : float
                  covariance of the RV U and Q
        cov_vw : float
                 covariance of the RV V and W
        cov_vq : float
                 covariance of the RB V and Q
        cov_wq : float
                 covariance of the RB W and Q
        mean_u : float
                 mean of U
        mean_v : float 
                 mean of V
        mean_W : float
                 mean of W
        mean_Q : float
                 mean of Q

        Returns
        -------
        e : float  
            expectation E(UVWQ)

        Notes
        -----
        Defined in [2], equation (71)(74)  
    """
    
    return cov_uv*cov_wq + cov_uw*cov_vq + cov_uq*cov_vw + cov_uv*mean_w*mean_q + cov_uw*mean_v*mean_q + cov_uq*mean_v*mean_w + cov_vw*mean_u*mean_q + cov_vq*mean_u*mean_w + cov_wq*mean_u*mean_v + mean_u*mean_v*mean_w*mean_q

def expectation_order3(cov_uv, cov_uw, cov_vw, mean_u, mean_v, mean_w):
    """ 
        Compute the expectation of the 3th order terms E(UVW) for 3 gaussian RV U,V,W
        
        Parameters
        ----------
        cov_uv : float 
                 covariance of the RV U and V
        cov_uw : float
                 covariance of the RV U and W
        cov_vw : float
                 covariance of the RV V and W
        mean_u : float
                 mean of U
        mean_v : float 
                 mean of V
        mean_W : float
                 mean of W

        Returns
        -------
        e : float  
            expectation E(UVW)

        Notes
        -----
        Defined in [2], equation (71)(73)  
    """

    return cov_uv*mean_w + cov_uw*mean_v + cov_vw*mean_u + mean_u*mean_v*mean_w

def expectation_order2(cov_uv, mean_u, mean_v):
    """ 
        Compute the expectation of the 2th order terms E(UV) for 2 gaussian RV U,V
        
        Parameters
        ----------
        cov_uv : float 
                 covariance of the RV U and V
        mean_u : float
                 mean of U
        mean_v : float 
                 mean of V

        Returns
        -------
        e : float  
            expectation E(UV)


        Notes
        -----
        Defined in [2], equation (71) 
    """ 

    return cov_uv + mean_u*mean_v

def computeNormalVariance(a0,a1,a2,a3,b0,b1,
                          normalMean,
                          mean_rho,
                          cov_rho):
    """ 
        Compute the normal variance. 

        Parameters
        ----------
        a0 : (3) array
             term associated with the term rho_s_plus*rho_psi_plus in the normal random variable expression
        a1 : (3) array
             term associated with the term rho_s_minus*rho_psi_minus in the normal random variable expression
        a2 : (3) array
             term associated with the term rho_s_minus*rho_psi_plus in the normal random variable expression
        a3 : (3) array
             term associated with the term rho_s_plus*rho_psi_minus in the normal random variable expression
        b0 : (3) array
             term associated with the term rho_psi_plus in the normal random variable expression
        b1 : (3) array
             term associated with the term rho_psi_minus in the normal random variable expression
        normalMean : (3) array
                     mean of the normal
        mean_rho : float
                   mean of the predicted rho
        cov_rho : float
                  variance of the predicted rho

        Returns
        -------
        cov_normal : (3,3) ndarray
                     covariance of the normalized normal

        Notes
        -----
        See [2], section 5.3 
    """
    
    mean_u = mean_rho[0]
    mean_v = mean_rho[1]
    mean_w = mean_rho[2]
    mean_q = mean_rho[3]
    cov_uv = cov_rho[0][1] 
    cov_uw = cov_rho[0][2] 
    cov_uq = cov_rho[0][3]
    cov_vw = cov_rho[1][2]
    cov_wq = cov_rho[2][3]
    cov_vq = cov_rho[1][3]
    var_u = cov_rho[0][0]
    var_v = cov_rho[1][1]
    var_w = cov_rho[2][2]
    var_q = cov_rho[3][3]

    # Polynomial coefficients 
    A_ijkl = [np.outer(a0, a0), # A_2200
              np.outer(a1, a1), # A_0022
              np.outer(a2, a2), # A_0220
              np.outer(a3, a3), # A_2002
              np.outer(a0, a3) + np.outer(a3, a0), # A_2101
              np.outer(a0, a2) + np.outer(a2, a0), # A_1210
              np.outer(a1, a2) + np.outer(a2, a1), # A_0121
              np.outer(a1, a3) + np.outer(a3, a1), # A_1012
              np.outer(a0, a1) + np.outer(a1, a0) + np.outer(a2, a3) + np.outer(a3, a2), # A_1111
              np.outer(a0, b0) + np.outer(b0, a0), # A_1200
              np.outer(a2, b0) + np.outer(b0, a2), # A_0210
              np.outer(a1, b1) + np.outer(b1, a1), # A_0012
              np.outer(a3, b1) + np.outer(b1, a3), # A_1002
              np.outer(a1, b0) + np.outer(b0, a1) + np.outer(a2, b1) + np.outer(b1, a2), # A_0111
              np.outer(a0, b1) + np.outer(b1, a0) + np.outer(a3, b0) + np.outer(b0, a3), # A_1101
              np.outer(b0, b0), # A_0200
              np.outer(b1, b1), # A_0002
              np.outer(b0, b1) + np.outer(b1, b0) # A_0101
              ]

    # Expectations each terms in the 4th order polynom in [2], equation (70)
    expectations = [expectation_order4(cov_uv=cov_uv, cov_uw=var_u, cov_uq=cov_uv, cov_vw=cov_uv, cov_vq=var_v, cov_wq=cov_uv,
                                         mean_u=mean_u, mean_v=mean_v, mean_w=mean_u, mean_q=mean_v),  #E_2200
                             expectation_order4(cov_uv=cov_wq, cov_uw=var_w, cov_uq=cov_wq, cov_vw=cov_wq, cov_vq=var_q, cov_wq=cov_wq,
                                         mean_u=mean_w, mean_v=mean_q, mean_w=mean_w, mean_q=mean_q),  #E_0022
                             expectation_order4(cov_uv=var_v, cov_uw=cov_vw, cov_uq=cov_vw, cov_vw=cov_vw, cov_vq=cov_vw, cov_wq=var_w,
                                         mean_u=mean_v, mean_v=mean_v, mean_w=mean_w, mean_q=mean_w),  #E_0220
                             expectation_order4(cov_uv=var_u, cov_uw=cov_uq, cov_uq=cov_uq, cov_vw=cov_uq, cov_vq=cov_uq, cov_wq=var_q,
                                         mean_u=mean_u, mean_v=mean_u, mean_w=mean_q, mean_q=mean_q),  #E_2002
                             expectation_order4(cov_uv=cov_uv, cov_uw=var_u, cov_uq=cov_uq, cov_vw=cov_uv, cov_vq=cov_vq, cov_wq=cov_uq,
                                         mean_u=mean_u, mean_v=mean_v, mean_w=mean_u, mean_q=mean_q),  #E_2101
                             expectation_order4(cov_uv=cov_uv, cov_uw=cov_uw, cov_uq=cov_uv, cov_vw=cov_vw, cov_vq=var_v, cov_wq=cov_vw,
                                         mean_u=mean_u, mean_v=mean_v, mean_w=mean_w, mean_q=mean_v), # E_1210
                             expectation_order4(cov_uv=cov_vw, cov_uw=var_w, cov_uq=cov_wq, cov_vw=cov_vw, cov_vq=cov_vq, cov_wq=cov_wq,
                                         mean_u=mean_w, mean_v=mean_v, mean_w=mean_w, mean_q=mean_q),  # E_0121
                             expectation_order4(cov_uv=cov_uq, cov_uw=cov_uw, cov_uq=cov_uq, cov_vw=cov_wq, cov_vq=var_q, cov_wq=cov_wq,
                                         mean_u=mean_u, mean_v=mean_q, mean_w=mean_w, mean_q=mean_q),  # E_1012
                             expectation_order4(cov_uv=cov_uv, cov_uw=cov_uw, cov_uq=cov_uq, cov_vw=cov_vw, cov_vq=cov_vq, cov_wq=cov_wq,
                                         mean_u=mean_u, mean_v=mean_v, mean_w=mean_w, mean_q=mean_q),  # E_1111
                             expectation_order3(cov_uv=cov_uv, cov_uw=cov_uv, cov_vw=var_v, mean_u=mean_u, mean_v=mean_v, mean_w=mean_v),  # E_1200
                             expectation_order3(cov_uv=var_v, cov_uw=cov_vw, cov_vw=cov_vw, mean_u=mean_v, mean_v=mean_v, mean_w=mean_w), # E_0210
                             expectation_order3(cov_uv=var_q, cov_uw=cov_wq, cov_vw=cov_wq, mean_u=mean_q, mean_v=mean_q, mean_w=mean_w),  # E_0012
                             expectation_order3(cov_uv=var_q, cov_uw=cov_uq, cov_vw=cov_uq, mean_u=mean_q, mean_v=mean_q, mean_w=mean_u),  # E_1002
                             expectation_order3(cov_uv=cov_vw, cov_uw=cov_vq, cov_vw=cov_wq, mean_u=mean_v, mean_v=mean_w, mean_w=mean_q),  # E_0111
                             expectation_order3(cov_uv=cov_uv, cov_uw=cov_uq, cov_vw=cov_vq, mean_u=mean_u, mean_v=mean_v, mean_w=mean_q),  # E_1101
                             expectation_order2(cov_uv=var_v, mean_u=mean_v, mean_v=mean_v),  # E_0200
                             expectation_order2(cov_uv=var_q, mean_u=mean_q, mean_v=mean_q),  # E_0002
                             expectation_order2(cov_uv=cov_vq, mean_u=mean_v, mean_v=mean_q)  # E_0101
                     ]

    # Adds up the polynom terms
    Var = np.zeros((3,3))
    for exp, coeff in zip(expectations, A_ijkl):
        Var += exp*coeff
    Var -= np.outer(normalMean, normalMean)

    # Normalization jacobian
    norm = np.linalg.norm(normalMean)
    norm_sqr = norm**2
    norm_cube = norm**3
    J = (1./norm_cube)*np.array([[norm_sqr - normalMean[0]**2, -normalMean[0]*normalMean[1], -normalMean[0]*normalMean[2]],
                                [-normalMean[0]*normalMean[1], norm_sqr - normalMean[1]**2, -normalMean[1]*normalMean[2]],
                                [-normalMean[0]*normalMean[2], -normalMean[1]*normalMean[2], norm_sqr - normalMean[2]**2]])

    norm_var = J@Var@J.T

    return norm_var

def getDataForPrediction_normal(normals_data):
    """
        Get the "input" values corresponding the four points required for computing the normal pdf of a point

        Parameters
        ----------
        normals_data : (n,10) ndarray
                       contains data for computing the normals 
        
        Returns
        -------
        data_to_predict : (n,4) ndarray
                          four points around each input point for computing its normal

        Notes
        -----
        See [2], figure 5
    """

    # Here "t" is the interpolation value obtained for \hat{\eta}
    # It corresponds to the \hat{t} defined as the solution of [1], equation (28)
    data_to_predict = []
    for data in normals_data:
        yaw = data[2] 
        s = data[1] # Curvilinear abscissa
        s_plus = data[6] # s(t + dt)
        s_minus = data[7] # s(t -dt)
        yaw_plus = data[8]
        yaw_minus = data[9]

        # The four points required for computing the normal 
        # See [2], Figure 5 
        data_to_predict.append([s_plus, yaw])
        data_to_predict.append([s, yaw_plus])
        data_to_predict.append([s_minus, yaw])
        data_to_predict.append([s, yaw_minus])

    return np.array(data_to_predict)

def predictDataForNormal(m, data_to_predict, curData, isLearningMeanFunc):
    """ 
        Infer data further used to compute the normal pdf
        TODO: Manage the range mapping properly !

        Parameters
        ----------
        data_to_predict : (n,4) ndarray
                          four points around each input point for computing its normal
        curData : (n,m) ndarray
                  data used to compute the normal
        isLearningMeanFunc : bool
                             if true, use a MLP for learning the GP prior surface
        
        Returns 
        -------
        pred_rho_mean : (4) array
                        mean of the predicted rho (sonar range) by the learned GP surface for each four points as defined in [2], figure 5 
        pred_rho_cov : (4) array
                       variance of the predicted rho (sonar range) by the learned GP surface for each four points as defined in [2], figure 5
    """ 

    # Gaussian process inference for the 4 points needed to compute the normal
    pred_rho_mean, pred_rho_cov = m.predict_noiseless(data_to_predict, full_cov=True)
    pred_rho_mean = pred_rho_mean.flatten()
    pred_rho_cov = pred_rho_cov.squeeze()

    # Covariance --> cov = J cov_range_mappinh J^T with mapping x -> sqrt(X)
    J = np.zeros((4,4))
    for i in range(0,4):
        J[i,i] = 2.*pred_rho_mean[i]
    pred_rho_cov = J @ pred_rho_cov @ J

    # When using a range mapping func, need to use the inverse mapping !!!
    # Add the priors to estimated rhos
    if(not isLearningMeanFunc):
        r_prior_ds_plus = curData[10]
        r_prior_ds_minus = curData[11]
        r_prior_dyaw_plus = curData[12]
        r_prior_dyaw_minus = curData[13]
        pred_rho_mean[0] += r_prior_ds_plus
        pred_rho_mean[1] += r_prior_dyaw_plus
        pred_rho_mean[2] += r_prior_ds_minus
        pred_rho_mean[3] += r_prior_dyaw_minus
    
    # TODO : for range mapping. Should be changed !!
    #pred_rho_mean[0] = pred_rho_mean[0] ** 2
    #pred_rho_mean[1] = pred_rho_mean[1] ** 2
    #pred_rho_mean[2] = pred_rho_mean[2] ** 2
    #pred_rho_mean[3] = pred_rho_mean[3] ** 2

    return pred_rho_mean, pred_rho_cov

def computeNormalCovCoefficients_general_(interpose_sonar, interpose_dt_plus_sonar,
                                          interpose_dt_minus_sonar, cos_yaw, sin_yaw,
                                          cos_yaw_plus, sin_yaw_plus,
                                          cos_yaw_minus, sin_yaw_minus):
    """ 
        Compute the coefficients involved in the computation of the normal covariance
        
        Parameters
        ----------
        interpose_sonar : (6) array
                          interpolated pose at curvilinear abcissa s(t) (and sonar rotation angle psi) where the sonar took the measure
        interpose_dt_plus_sonar : (6) array
                                  interpolated pose at curvilinear abcissa s(t+dt) (and sonar rotation angle yaw)
        interpose_dt_minus_sonar : (6) array
                                   interpolated pose at curvilinear abcissa s(t-dt) (and sonar rotation angle yaw)
        cos_yaw : float
                  cosinus of yaw
        sin_yaw : float
                  sinus of yaw
        cos_yaw_plus : float
                       cosinus of yaw + dyaw
        sin_yaw_plus : float
                       sinus of yaw + dyaw
        cos_yaw_minus : float
                        cosinus of yaw - dyaw
        sin_yaw_minus : float
                        sinus of yaw - dyaw

        Returns
        -------
        a0 : (3) array
             term associated with the term rho_s_plus*rho_psi_plus in the normal random variable expression
        a1 : (3) array
             term associated with the term rho_s_minus*rho_psi_minus in the normal random variable expression
        a2 : (3) array
             term associated with the term rho_s_minus*rho_psi_plus in the normal random variable expression
        a3 : (3) array
             term associated with the term rho_s_plus*rho_psi_minus in the normal random variable expression
        b0 : (3) array
             term associated with the term rho_psi_plus in the normal random variable expression
        b1 : (3) array
             term associated with the term rho_psi_minus in the normal random variable expression

        Notes
        -----
        See [2], equation (70)
    """

    R = scipy.spatial.transform.Rotation.from_euler('ZYX', interpose_sonar[0:3]).as_matrix()
    R_plus = scipy.spatial.transform.Rotation.from_euler('ZYX', interpose_dt_plus_sonar[0:3]).as_matrix()
    R_minus = scipy.spatial.transform.Rotation.from_euler('ZYX', interpose_dt_minus_sonar[0:3]).as_matrix()
    t_plus = interpose_dt_plus_sonar[3:6]
    t_minus = interpose_dt_minus_sonar[3:6]
    u = np.array([cos_yaw, sin_yaw, 0])
    u_plus = np.array([cos_yaw_plus, sin_yaw_plus, 0])
    u_minus = np.array([cos_yaw_minus, sin_yaw_minus, 0])

    # Intermediate values
    R_plus_u = R_plus @ u
    R_minus_u = R_minus @ u
    R_u_plus = R @ u_plus
    R_u_minus = R @ u_minus
    dt = t_plus - t_minus

    # Coefficients of vv^T in  [2], equation (70)
    a0 = crossProd(R_plus_u, R_u_plus)
    a1 = crossProd(R_minus_u, R_u_minus)
    a2 = -crossProd(R_minus_u, R_u_plus)
    a3 = -crossProd(R_plus_u, R_u_minus)
    b0 = crossProd(dt, R_u_plus)
    b1 = -crossProd(dt, R_u_minus)

    return a0, a1, a2, a3, b0, b1

def computeNormalCovCoefficients_referenceGuide_(interpose_sonar, interpose_dt_plus_sonar,
                                                 interpose_dt_minus_sonar, two_ds, cos_yaw, sin_yaw,
                                                 cos_yaw_plus, sin_yaw_plus,
                                                 cos_yaw_minus, sin_yaw_minus):
    """ 
        Compute the coefficients involved in the computation of the normal covariance
        in the case where using the prior cylinder axis as a reference guide for curvilinear abscissa.
        
        Parameters
        ----------
        interpose_sonar : (6) array
                          interpolated pose at curvilinear abcissa s(t) (and sonar rotation angle psi) where the sonar took the measure
        interpose_dt_plus_sonar : (6) array
                                  interpolated pose at curvilinear abcissa s(t+dt) (and sonar rotation angle yaw)
        interpose_dt_minus_sonar : (6) array
                                   interpolated pose at curvilinear abcissa s(t-dt) (and sonar rotation angle yaw)
        cos_yaw : float
                  cosinus of yaw
        sin_yaw : float
                  sinus of yaw
        cos_yaw_plus : float
                       cosinus of yaw + dyaw
        sin_yaw_plus : float
                       sinus of yaw + dyaw
        cos_yaw_minus : float
                        cosinus of yaw - dyaw
        sin_yaw_minus : float
                        sinus of yaw - dyaw

        Returns
        -------
        a0 : (3) array
             term associated with the term rho_s_plus*rho_psi_plus in the normal random variable expression
        a1 : (3) array
             term associated with the term rho_s_minus*rho_psi_minus in the normal random variable expression
        a2 : (3) array
             term associated with the term rho_s_minus*rho_psi_plus in the normal random variable expression
        a3 : (3) array
             term associated with the term rho_s_plus*rho_psi_minus in the normal random variable expression
        b0 : (3) array
             term associated with the term rho_psi_plus in the normal random variable expression
        b1 : (3) array
             term associated with the term rho_psi_minus in the normal random variable expression

        Notes
        -----
        See [2], equation (70) for the general case. Here the computation can be simplified.
    """
    
    R = scipy.spatial.transform.Rotation.from_euler('ZYX', interpose_sonar[0:3]).as_matrix()
    u = np.array([0., cos_yaw, sin_yaw])
    u_plus = np.array([0., cos_yaw_plus, sin_yaw_plus])
    u_minus = np.array([0., cos_yaw_minus, sin_yaw_minus])
        
    a0 = R @ crossProd(u, u_plus)
    a1 = -a0
    a2 = -a0
    a3 = a0
    xc = np.array([1., 0., 0.])
    b0 = two_ds*R@crossProd(xc, u_plus)
    b1 = -two_ds*R@crossProd(xc, u_minus)

    return a0, a1, a2, a3, b0, b1

def computeNormalCovCoefficients(curData, pred_rho_mean, fileDebug, isWithRefGuide):
    """ 
        Compute the coefficients involved in the computation of the normal covariance.
        
        Parameters
        ----------
        curData : (n,m) ndarray
                  data used to compute the normal
        pred_rho_mean : (4) array
                        mean of the predicted rho (sonar range) by the learned GP surface for each four points as defined in [2], figure 5
        fileDebug : str
                    file to save debug data (position of four points used for the normal computation)
        isWithRefGuide : bool
                         if true, use a reference guide for defining curvilinear abscissa

        Returns
        -------
        a0 : (3) array
             term associated with the term rho_s_plus*rho_psi_plus in the normal random variable expression
        a1 : (3) array
             term associated with the term rho_s_minus*rho_psi_minus in the normal random variable expression
        a2 : (3) array
             term associated with the term rho_s_minus*rho_psi_plus in the normal random variable expression
        a3 : (3) array
             term associated with the term rho_s_plus*rho_psi_minus in the normal random variable expression
        b0 : (3) array
             term associated with the term rho_psi_plus in the normal random variable expression
        b1 : (3) array
             term associated with the term rho_psi_minus in the normal random variable expression

        Notes
        -----
        See [2], equation (70)
    """
    
    # Load data
    yaw = curData[2] 
    yaw_plus = curData[8]
    yaw_minus = curData[9]
    s_plus = curData[6] # s(t + dt)
    s_minus = curData[7] # s(t -dt)

    # Corresponds to \eta^{S}_{s_p^+} / \eta^{S}_{s_p^-} in [2], section 5.3 
    interpose_dt_plus_sonar = np.array(curData[14:20], dtype=np.dtype('float64'))
    interpose_dt_minus_sonar = np.array(curData[20:26], dtype=np.dtype('float64'))
    interpose_sonar = np.array(curData[26:32], dtype=np.dtype('float64'))

    # Save the point used for computing the normal (in global robot frame)
    # [s, yaw, rho]
    # TODO: Precompute directly all the Yaws (faster)
    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)
    cos_yaw_plus = np.cos(yaw_plus)
    cos_yaw_minus = np.cos(yaw_minus)
    sin_yaw_plus = np.sin(yaw_plus)
    sin_yaw_minus = np.sin(yaw_minus)

    point1 = poses_euler.composePoseEulerPoint(interpose_dt_plus_sonar, pred_rho_mean[0]*np.array([cos_yaw, sin_yaw, 0]))
    point2 = poses_euler.composePoseEulerPoint(interpose_sonar, pred_rho_mean[1] * np.array([cos_yaw_plus, sin_yaw_plus , 0]))
    point3 = poses_euler.composePoseEulerPoint(interpose_dt_minus_sonar, pred_rho_mean[2] * np.array([cos_yaw , sin_yaw, 0]))
    point4 = poses_euler.composePoseEulerPoint(interpose_sonar, pred_rho_mean[3] * np.array([cos_yaw_minus , sin_yaw_minus , 0]))

    fileDebug.write(str(point1[0]) + "," + str(point1[1]) + "," + str(point1[2]) + "\n")
    fileDebug.write(str(point2[0]) + "," + str(point2[1]) + "," + str(point2[2]) + "\n")
    fileDebug.write(str(point3[0]) + "," + str(point3[1]) + "," + str(point3[2]) + "\n")
    fileDebug.write(str(point4[0]) + "," + str(point4[1]) + "," + str(point4[2]) + "\n")

    if(not isWithRefGuide):
        return computeNormalCovCoefficients_general_(interpose_sonar, interpose_dt_plus_sonar,
                                                     interpose_dt_minus_sonar, cos_yaw, sin_yaw,
                                                     cos_yaw_plus, sin_yaw_plus,
                                                     cos_yaw_minus, sin_yaw_minus)
    else:
        return computeNormalCovCoefficients_referenceGuide_(interpose_sonar, interpose_dt_plus_sonar,
                                                            interpose_dt_minus_sonar, s_plus - s_minus, 
                                                            cos_yaw, sin_yaw,
                                                            cos_yaw_plus, sin_yaw_plus,
                                                            cos_yaw_minus, sin_yaw_minus)

def estimateNormals(dataFolder, m, normals_data, size_batch, isLearningMeanFunc, isWithRefGuide):
    """ 
        Compute the normal pdf at each points of the point cloud.
        
        Parameters
        ----------
        dataFolder : str
                     folder containing the data related to the normals pdf
        m : Gaussian Process model
        normals_data : (n,m) array
                       data used for computing the normals pdf
        size_batch : float
                     batch size
        isLearningMeanFunc : bool
                             if true, use a MLP for learning the GP prior surface
        isWithRefGuide : bool
                         if true, use a reference guide for defining curvilinear abscissa
        
        Returns
        -------
        normals_new : (n) array of dict
                      pdf of the normals

        Notes
        -----
        Explained in [2], section 5.3
        General case (isWithRefGuide == False) using the original method in [1] using the robot trajectory. 
        If isWithRefGuide == True, use the variant shown in [2] which uses the prior cylinder 
        axis as a reference curve cf [2], section 5.2.1
    """

    print("########## estimateNormals #############")
    
    normals_new = []

    # Get the "input" data (curvilinear abscissa, yaw) corresponding to the four points
    # used for computing the normal pdf at a point (see [2], section 5.3)
    data_to_predict = getDataForPrediction_normal(normals_data)

    fileDebug = open(dataFolder + "/debugNormals.txt", "w")

    # For each point 
    for i in range(0,len(normals_data)):
        idx = 4*i
        pred_rho_mean, pred_rho_cov = predictDataForNormal(m, data_to_predict[idx:idx+4, :], normals_data[i], isLearningMeanFunc)

        a0, a1, a2, a3, b0, b1 = computeNormalCovCoefficients(normals_data[i], pred_rho_mean, fileDebug, isWithRefGuide)

        #TODO : Refactoring with better variables (avoid function with 10^10 inputs)
        normal_mean_new_unormalized = computeNormalMean_unnormalized(a0,a1,a2,a3,b0,b1,pred_rho_mean, pred_rho_cov)

        normal_variance = computeNormalVariance(a0,a1,a2,a3,b0,b1,
                                                normal_mean_new_unormalized,
                                                pred_rho_mean,
                                                pred_rho_cov)

        normal_new = {'mean': normal_mean_new_unormalized/np.linalg.norm(normal_mean_new_unormalized) , 'cov' : normal_variance}

        normals_new.append(normal_new)
    fileDebug.close()
    return normals_new

def saveNormals(dataFolder, name, normals):
    """ 
        Save the computed normal pdf to a file (to be further used in the C++ program)

        Parameters
        ----------
        dataFolder : str
                     folder where to save the file
        name : str
               name of the file
        normals : (n) array of dict
                  pdf of the normals 
    """

    print("Save estimated normals ...")
    with open(dataFolder + "/" + name, "w") as file:
        file.write("# mean_x, mean_y, mean_z, cov_11, cov_12, .... ,cov33\n")
        for n in normals:
            for x in n['mean']:
                file.write(str(x) + ",")
            for l in n['cov']:
                for c in l:
                    file.write(str(c) + ",")
            file.write("\n")

def graphRhoAtFixedAngle(m, s_start, s_end, s_step, angle):
    """ 
        Plot a graph of rho function of s (curvilinear abscissa) at a fixed angle (yaw) 
        Mainly for analysis / debugging

        Parameters
        ----------
        m : Gaussian Process model
        s_start : float
                  first curvilinear abscissa
        s_end : float
                last curvilinear abscissa
        s_step : float
                 curvilinear abscissa step
        angle : float
                sonar rotation angle yaw           
    """

    s_vals = np.arange(s_start, s_end, s_step)
    data = np.empty((s_vals.shape[0], 2))
    data[:,0] = s_vals
    data[:,1] = np.full((s_vals.shape[0],), angle) 

    rho_mean, rho_cov = m.predict_noiseless(data)

    rho_mean = rho_mean.flatten()
    rho_std = np.sqrt(rho_cov.flatten())
    rho_upperBound = rho_mean + 3.*rho_std
    rho_lowerBound = rho_mean - 3.*rho_std

    plt.figure()
    plt.xlabel('s')
    plt.ylabel('rho - rho_prior')
    plt.plot(s_vals, rho_mean, color='blue',lw=3,ls='--', alpha=1)
    plt.fill_between(s_vals, rho_upperBound, rho_lowerBound, color='blue', alpha=0.5)
    plt.show()

def generateInducingPts(n, input_data):
    """ 
        Generate inducing points for the Stochastic Variational GP (SVGP) 
        It was first introduced when taking into account the censored data (more points to consider)
        TODO: Also test it in the original case without censored data

        Parameters
        ----------
        n : int
            number of inducing points
        input_data : (m,2) array
                     input data for GP (curvilinear abscissa s and sonar rotation angle yaw)

        Returns
        -------
        inducing_pts : (n,2) array
                       inducing points (s,yaw) for training
    """

    print("Input_data : {}".format(input_data))

    # First, get min/max for the curvilinear abscissa
    min_s = np.min(input_data[:,0])
    max_s = np.max(input_data[:,0])

    # Get random inducing points 
    inducing_pts = np.empty((n, input_data.shape[1]))
    inducing_pts[:,0] = (max_s - min_s)*np.random.rand(n,) + min_s
    inducing_pts[:,1] = 2.*np.pi*np.random.rand(n,) - np.pi

    print("Inducing_pts : {}".format(inducing_pts))

    return inducing_pts

def getParameters(argv):
    """ 
        Get the parameter dict from the script arguments

        Parameters
        ----------
        argv : (n) list
               arguments

        Returns
        -------
        params : dict 
                 parameters for the GP
    """

    params['kernelType_s'] = sys.argv[1]
    params['kernelType_y'] = sys.argv[2]
    params['variance'] = float(sys.argv[3])
    params['lengthscale_s'] = float(sys.argv[4])
    params['lengthscale_yaw'] = float(sys.argv[5])
    params['noise_var'] = float(sys.argv[6])
    params['size_batch'] = int(sys.argv[7])
    params['data_folder'] = sys.argv[8]
    params['isLearningMeanFunc'] = int(sys.argv[9])
    params['isUseGPCensored'] = int(sys.argv[10])
    return params

def generateKernel(params):
    """ 
        Generate the kernel product for the GP. 
        It is the product of kernel on curvilinear abcissa and a kernel on yaw (using chordal distance for angles)

        Parameters
        ----------
        params : dict
                 parameters for the GP
        
        Returns
        -------
        prod_kernel : kernel
                      kernel used for the GP

        Notes
        -----
        The kernels are explained in [1], section 3.4 / 4.2.2
    """

    # Get the kernels type and if their parameters are fixed 
    fix_lengthscale_s = True
    if(params['lengthscale_s'] < 0):
        params['lengthscale_s'] = 0.05
        fix_lengthscale_s = False
    fix_lengthscale_yaw = True
    if(params['lengthscale_yaw'] < 0):
        params['lengthscale_yaw'] = 0.5
        fix_lengthscale_yaw = False
    fix_variance = True
    if(params['variance'] < 0):
        params['variance'] = 1.
        fix_variance = False
    params['fix_noise_var'] = True
    if(params['noise_var'] < 0):
        params['noise_var'] = 1.
        params['fix_noise_var'] = False

    # Kernel on curvilinear abscissa
    if(params['kernelType_s'] == 'exponential'): #  same as matern with v = 1/2
        kernel_s = GPy.kern.Exponential(input_dim=1,variance=params['variance'],lengthscale=params['lengthscale_s'], active_dims=[0]) 
    elif(params['kernelType_s'] == 'matern32'):
        kernel_s = GPy.kern.Matern32(input_dim=1, variance=params['variance'], lengthscale=params['lengthscale_s'], active_dims=[0])
    elif(params['kernelType_s'] == 'matern52'):
        kernel_s = GPy.kern.Matern52(input_dim=1, variance=params['variance'], lengthscale=params['lengthscale_s'], active_dims=[0])
    elif(params['kernelType_s'] == 'rbf'): # same as maternInf
        kernel_s = GPy.kern.RBF(input_dim=1, variance=params['variance'], lengthscale=params['lengthscale_s'], active_dims=[0])
    else :
        print(" ---> Unknown Kernel_s type !")

    # Kernel on yaw angle using chordal distance 
    # Currently, only matern52Chordal seems to be usable. Exponential works but give always too high lengthscale when trained. 
    # C2Wendland fails when trained. 
    if(params['kernelType_y'] == 'exponentialChordal'):
        kernel_yaw = GPy.kern.ExponentialChordal(input_dim=1, variance=params['variance'], lengthscale=params['lengthscale_yaw'] , active_dims=[1])
    elif(params['kernelType_y'] == 'matern52Chordal'):
        kernel_yaw = GPy.kern.Matern52Chordal(input_dim=1, variance=params['variance'], lengthscale=params['lengthscale_yaw'] , active_dims=[1])
    elif(params['kernelType_y'] == 'C2WendlandChordal'):
        kernel_yaw = GPy.kern.C2WendlandChordal(input_dim=1, variance=params['variance'], lengthscale=params['lengthscale_yaw'] , active_dims=[1],tau=4)
    elif(params['kernelType_y'] == 'C2WendlandGeo'):
        kernel_yaw = GPy.kern.C2WendlandGeo(input_dim=1, variance=params['variance'], lengthscale=params['lengthscale_yaw'] ,active_dims=[1], tau=4)
    else:
        print(" ---> Unknown Kernel_yaw type !")

    # Fix some parameters if asked to
    if (fix_lengthscale_s):
        kernel_s.lengthscale.fix()
    if (fix_lengthscale_yaw):
        kernel_yaw.lengthscale.fix()
    if (fix_variance):
        kernel_s.variance.fix()
        kernel_yaw.variance.fix()

    # Kernel product s
    return kernel_s*kernel_yaw

def getCensoredDataIdx(censoredData):
    """
        Get the metada indicating indexes of (upper) censored data

        Parameters
        ----------
        censoredData : (n) array of int 
                       indicates if the data is censored or not (ie max range sonar)  

        Returns
        -------
        y_metadata : dict of indexes array
                     contains indexes for each type of data : upper censored (['upperCensored']),
                     lower censored (['lowerCensored']) and non-censored (['gaussianIndexes'])           
                     Note that in our problem, there are no lower censored data
    """

    # Debug : test with only the nonCensored data
    #censoredIndexes = np.array([])#np.array([idx for idx,val in np.ndenumerate(censoredData) if val == 1])
    #gaussianIndexes = np.array([idx for idx,val in np.ndenumerate(censoredData)]).flatten() #if val == 0])
    ##########################################################################

    censoredIndexes = np.array([idx for idx,val in np.ndenumerate(censoredData) if val == 1]).flatten()
    gaussianIndexes = np.array([idx for idx,val in np.ndenumerate(censoredData) if val == 0]).flatten()

    #print("censoredIndexes : {}".format(censoredIndexes))
    #print("GaussianIndexes : {}".format(gaussianIndexes))

    y_metadata = {"upperCensored": censoredIndexes ,
                  "gaussianIndexes": gaussianIndexes}

    print("y_metadata : {}".format(y_metadata))

    return y_metadata

def GPmodel_censored_(trainingData_input, trainingData_output, y_metadata, 
                      kernel, mean_func, noise_var, fix_noise_var):
    """ 
        Generate the GP model when using censored data
        In particular, generate a SVGP (Stochastic Variational) model adapted to take 
        into account censored data

        Parameters
        ----------
        trainingData_input : (n,2) array
                    input data (curvilinear abcissa s and sonar rotation angle psi)
        trainingData_output : (n) array
                     output data (sonar range rho)
        y_metadata : dict of indexes array
                     contains indexes for each type of data : upper censored (['upperCensored']),
                     lower censored (['lowerCensored']) and non-censored (['gaussianIndexes'])           
                     Note that in our problem, there are no lower censored data
        kernel : kernel 
                 kernel for the GP with censored data
        mean_func : func, optional
                    function defining the prior (if any)
        fix_noise_var : bool
                        if true, the likelihood variance is fixed

        Returns
        -------
        m : gaussian process model 

        Notes
        -----
        See [3]. GPy.core.SVGPCensored is not in the original GPy lib 
        but in my own fork https://gite.lirmm.fr/breux/GPy
    """
    
    # Test : Use GP with a tobit likelihood 
    # (take into account censored data e.g. out-range data)
    n = len(trainingData_input)
    print("N input data : {}".format(n))

    nInducingPts = 20
    batchsize = 20
    inducing_pts = generateInducingPts(nInducingPts, trainingData_input[y_metadata['gaussianIndexes'], :])
    m = GPy.core.SVGPCensored(trainingData_input,
                              trainingData_output,
                              inducing_pts,
                              lowerThreshold=None,
                              upperThreshold=10.,
                              kernel=kernel,
                              lik_var=noise_var,
                              mean_function=mean_func,
                              Y_metadata=y_metadata,
                              batchsize=batchsize)
    if(fix_noise_var):
        m.likelihood.variance.fix()
    return m

def GPmodel_(trainingData_input, trainingData_output, kernel, noise_var, mean_func, fix_noise_var):
    """ 
        Generate a GP model

        Parameters
        ----------
        trainingData_input : (n,2) array
                    input data (curvilinear abcissa s and sonar rotation angle psi)
        trainingData_output : (n) array
                     output data (sonar range rho)
        censoredData : (n) array of int 
                       indicates if the data is censored or not (ie max range sonar)  
        mean_func : func, optional
                    function defining the prior (if any)
        fix_noise_var : bool
                        if true, the likelihood variance is fixed

        Returns
        -------
        m : gaussian process model 

        Notes
        -----
        See [1], section 3.4 and references therein related to GP 
    """
    m = GPy.models.GPRegression(trainingData_input, 
                                 trainingData_output,
                                 kernel, 
                                 noise_var=noise_var,
                                 mean_function=mean_func)
    if (fix_noise_var):
        m.Gaussian_noise.variance.fix()
    return m

def GPoptimization_SVGP_(m):
    """ 
        Optimization for SVGP model
        Based on one of tutorial provided by GPy 
        https://nbviewer.org/github/SheffieldML/notebook/blob/master/GPy/SVI.ipynb 

        Parameters
        ----------
        m : gaussian process model
    """
    
    niter = 5000
    def callback(i):
        # print("Log-likelihood : {}".format(m.log_likelihood()))
        # Stop after 5000 iterations
        if i['n_iter'] > niter:
            return True
        return False
    opt = climin.Adadelta(m.optimizer_array, m.stochastic_grad, step_rate=0.2, momentum=0.9)
    opt.minimize_until(callback)

def GPestimation():
    """ 
        Main function. 
        - Learn the surface (with a GP) with the vertical measurements
        - Estimate the elevation angle distributions of the horizontal measurements
        - Compute the normal pdf at estimated horizontal points.
    """

    # Args validity
    if(len(sys.argv) != 11):
        print("Uncorrect number of arguments. The arguments should be as follow : ")
        print("- Kernel type for curvilinear abscissa")
        print("- Kernel type for yaw ")
        print("- Variance of kernels (Note: corresponds to the sqrt of the variance for product kernel used in GP)")
        print("- Lengthscale of the kernel for curvilinear abscissa")
        print("- Lengthscale of the kernel for yaw")
        print("- Gaussian noise variance")
        print("- Batch size")
        print("- Data folder")
        print("- Learning the mean (prior) function flag")
        print("- Using censored data flag")
        return 
    
    # Parameters
    params = getParameters(sys.argv)

    # File containing the training data for the GP (ie data related to the vertical sonar measurements) 
    trainingFile = params["data_folder"] + '/surfaceTrainingData.txt'

    # File containing the data to be infered by the GP 
    # for estimating the elevation angle distributions of the horizontal sonar measurements 
    validationFile = params["data_folder"] + '/surfaceValidationData.txt'

    # Load the training data
    trainingData_input, trainingData_output, trainingData_prior, censoredData = loadTrainingData(trainingFile)
    
    # Load validation (horizontal sonar) data 
    validationData_input = loadValidationData(validationFile)

    # Kernel product 
    kernel = generateKernel(params)

    # Just for test : use MLP to learn the mean function (instead of the prior elliptic cylinder)
    # From first results, not very good results (would require deeper network -> too much comp. time)
    start = time.time()
    mean_func = None
    if (params['isLearningMeanFunc']):
        print("Learn Mean Function ")
        mean_func = GPy.mappings.MLP(2,1,2)

    # Get indexes for each type of data (upper censored and non-censored)
    y_metadata = getCensoredDataIdx(censoredData)

    # GP estimation taking into account for censored data (ie off-range measures)
    if(params['isUseGPCensored']):
        m = GPmodel_censored_(trainingData_input, trainingData_output, y_metadata,
                              kernel, mean_func, params['noise_var'], params['fix_noise_var'])
    # Original GP estimation (only use the non-censored training input)
    else:
        m = GPmodel_(trainingData_input[y_metadata['gaussianIndexes'], :], trainingData_output, kernel,
                     params['noise_var'], mean_func,  params['fix_noise_var'])
    comp_time = time.time() - start
    print("GP regression in {} s:".format(comp_time))

    # Print the GP model parameters before optimization 
    print("--> GP model before optimization : ")
    print(m[''])

    # Optimization 
    # lbfgs optimizer is the fastest. However, it sometimes ends with an error ABNORMAL TERMINATION IN LNSCH 
    # It means that the algorithm failed to go downhill but the results seems to be still ok
    if(params['isUseGPCensored']):
        GPoptimization_SVGP_(m)
    else:
        m.optimize(optimizer='lbfgs',max_iters=500,messages=True)
    
    # Print the GP model parameters after optimization 
    print("--> GP model after optimization : ")
    print(m[''])

    #### For DEBUG ############
    #graphRhoAtFixedAngle(m=m, s_start = 0., s_end = 0.35, s_step = 0.01, angle=0.5*np.pi)
    ###########################

    # Estimate the elevation angle distributions and corresponding normal data for the horizontal sonar measures
    distribution_beta, normalData = estimateBetaDistributions(m, validationData_input, params['size_batch'])

    # Estimate the normal pdfs at each estimated horizontal sonar points 
    normals_new = estimateNormals(params['data_folder'], m, normalData, 
                                  params['size_batch'], params['isLearningMeanFunc'], isWithRefGuide=True)
    
    # Save results to file (to be further used in the C++ code)
    # TODO: Python is used to quickly test things but at the end, all this code should be done 
    # directly in the C++ code for efficiency
    saveNormals(params['data_folder'],"res_estimatedNormalsNew.txt", normals_new)
    saveDistributionBeta(params['data_folder'], distribution_beta)
    del distribution_beta

    # Generate points from the GP (mean and upper/lower bound surface)
    s_vals, yaw_vals, idx_vals = [], [], []
    pred_mean , pred_var, low_est, high_est = generateSampledEstimatedSurface(m,
                                                                              params['data_folder'],
                                                                              s_vals, yaw_vals, idx_vals, params['size_batch'])

    # Save the surface in file (to be displayed in the c++ 3D viewer)
    saveEstimatedSurface(params['data_folder'], idx_vals, s_vals, yaw_vals, pred_mean, low_est, high_est)

'''------------------------------------------'''
'''---------------- Main --------------------'''
'''------------------------------------------'''
if __name__ == "__main__":
    GPestimation()
