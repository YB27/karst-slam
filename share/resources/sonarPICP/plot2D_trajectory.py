import numpy as np
import data_io
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import brewer2mpl
import poses_euler
import poses2D

""" 
    Functions to draw the projected 2D trajectories obtained 
    with relative poses estimated by pIC algorithms. 
    This code was used to generate Figure 16 in [1]
     
    References
    ----------
    .. [1] Yohan Breux and André Mas and Lionel Lapierre, "On-manifold Probabilistic ICP : Application to Underwater Karst
           Exploration"
""" 


# Different global parameters for the plotting 
plot_params = {
    'axes.labelsize': 15,
    'axes.labelweight': 'bold',
    'font.size': 20,
    'font.weight': 'bold',
    'legend.fontsize': 10,
    'xtick.labelsize': 15,
    'ytick.labelsize': 15,
    'text.usetex': False,
    'figure.figsize': [9, 7],
    'mathtext.default': 'regular'
}
plt.rcParams.update(plot_params)
bmap = brewer2mpl.get_map('Set2', 'qualitative', 7)
colors = bmap.mpl_colors
color_2D = colors[0]
color_3D = colors[1]
color_3D_allArcsColor = colors[2]
color_3D_plane = colors[3]
color_3D_plane_startPoint = colors[4]
color_dr = colors[5]

def get_marker(angle):
    """ 
        Generate a marker representing the robot orientation as a rotated triangle 
        
        Parameters
        ----------
        angle : float
                marker rotation angle
        
        Returns
        -------
        marker : (2,4) array
                 marker represented as four 2D points
    """

    ar = np.array([[-.25,-.5],[.25,-.5],[0,.5],[-.25,-.5]]).T
    rot = np.array([[np.cos(angle),np.sin(angle)],[-np.sin(angle),np.cos(angle)]])
    return np.dot(rot,ar).T

def extractXYYAW_fromTrajectories(trajectories):
    """ 
        Extract the 2D pose from the input trajectories

        Parameters
        ----------
        trajectories : array of trajectories. Each trajectory is an array of posePDF (dict with ["pose_mean"] and ["pose_cov"])
                       robot trajectory

        Returns
        -------
        trajectories_x : array of array of float
                         x positions of trajectories
        trajectories_y : array of array of float
                         y positions of trajectories     
        trajectories_orientation : array of array of float
                                   yaw angular position of trajectories    
        covariances2D : array of array of 3x3 covariance matrices 
                        covariance matrices at each 2D pose of the trajectories        
    """

    trajectories_x = []
    trajectories_y = []
    trajectories_orientation = []
    covariances2D = []
    for traj in trajectories:
        traj_x = []
        traj_y = []
        traj_yaw = []
        cov2D = []
        for posePDF in traj :
            traj_yaw.append(posePDF["pose_mean"][0])
            traj_x.append(posePDF["pose_mean"][3])
            traj_y.append(posePDF["pose_mean"][4])
            cov2D.append(poses2D.conditionalCovariance2D(posePDF["pose_cov"]))

        trajectories_x.append(traj_x)
        trajectories_y.append(traj_y)
        trajectories_orientation.append(traj_yaw)
        covariances2D.append(cov2D)
    
    return trajectories_x, trajectories_y, trajectories_orientation, covariances2D

def plot2D_trajectories(trajectories, labels, colors):
    """ 
        Main function for plotting the 2D trajectories 

        Parameters
        ----------
        labels : array of str  
                 name of the different trajectories to plot
        colors : array of (3) tuple
                 array of RGB tuple (values in [0,1]) for the trajectories color
    """

    # Create the figure
    plt.figure(0)
    plt.xlabel("Y (m)")
    plt.ylabel("X (m)")
    ax = plt.gca()

    # Convert 3D pose to 2D position (ie only x,y)
    # Also take apart the yaw
    trajectories_x, trajectories_y, trajectories_orientation, covariances2D = extractXYYAW_fromTrajectories(trajectories)

    # Plot the trajectories
    linewidth = 2
    markersize= 20
    alpha = 0.8
    plt.plot(trajectories_y[0], trajectories_x[0], linewidth=2, color='k',label=labels[0], alpha=alpha)
    for x,y,yaw in zip(trajectories_x[0], trajectories_y[0], trajectories_orientation[0]):
        plt.plot(y,x, linestyle="None", color='k', marker=get_marker(yaw), linewidth =linewidth, markersize=markersize, alpha=alpha)
    for x,y, traj_yaw, c, l in zip(trajectories_x[1:], trajectories_y[1:], trajectories_orientation[1:], colors[1:], labels[1:]):
        plt.plot(y,x, linestyle="dashed", color=c, linewidth=linewidth,label=l,alpha=alpha)
        for x_,y_, yaw in zip(x,y, traj_yaw):
            plt.plot(y_,x_, linestyle="None", color=c, marker=get_marker(yaw),linewidth=linewidth,markersize=markersize, alpha=alpha)
   
    # Plot uncertainty ellipses 
    for x_vec,y_vec,cov_vec,c in zip([trajectories_x[1], trajectories_x[2], trajectories_x[4]],
                         [trajectories_y[1], trajectories_y[2], trajectories_y[4]],
                         [covariances2D[1], covariances2D[2], covariances2D[4]],
                         [colors[1], colors[2], colors[4]]):
        for x,y,cov in zip(x_vec, y_vec, cov_vec):
            cov_xy = poses2D.conditionalCovarianceXY(cov)
            print("cov2D")
            print(cov)
            print("cov_xy")
            print(cov_xy)
            w, v = np.linalg.eig(cov_xy)
            angle = np.arctan2(v[0,0], v[0,1])
            ellipse = Ellipse((y, x), 6.*np.sqrt(w[1]), 6.*np.sqrt(w[0]), np.rad2deg(angle), edgecolor=c, fill=False)
            ax.add_patch(ellipse)

    plt.legend()
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)    
    plt.axis('equal')

    # Also save the figure to a file
    plt.savefig("/home/breux/dataset_2dTrajectory.eps", bbox_inches='tight')
    plt.show()   

def loadTrajectoriesFiles(folder, files):
    """ 
        Load 3d trajectories from files 

        Parameters
        ----------
        folder : str
                 folder where are stored the trajectories files
        files : array of str
                trajectory file names

        Returns
        -------
        trajectories : array of array of pose pdf
                       trajectories loaded from the files
    """ 

    trajectories = []
    for file in files:
        curTraj = []
        lastPosePDF = {"pose_mean": np.zeros(6,), "pose_cov": np.zeros((6,6))}
        curTraj.append(lastPosePDF)
        with open(folder + file, "r") as f:
             for line in f:
                 line_parsed = line.split(',')
                 curPose_mean = data_io.readPoseMean_yprxyz(line_parsed)
                 curPose_cov = data_io.readPoseCov_yprxyz(line_parsed, isTimestamp=False)
                 lastPosePDF = poses_euler.composePosePDFEuler(lastPosePDF, {"pose_mean": curPose_mean, "pose_cov": curPose_cov})
                 curTraj.append(lastPosePDF)
        trajectories.append(curTraj)
                 
    return trajectories

if __name__ == "__main__":
    folder = "../../../build/release/apps/"
    files = ["pICresults_dr.txt", "pICresults_2DpIC.txt", "pICresults_MpIC.txt", "pICresults_MpICa.txt", "pICresults_MpICp.txt"]
    trajectories = loadTrajectoriesFiles(folder, files)

    labels = ["DR", "2DpIC", "MpIC", "MpICa", "MpICp"]
    plot2D_trajectories(trajectories, labels, colors)
