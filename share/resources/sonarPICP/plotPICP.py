import numpy as np
import matplotlib.pyplot as plt
from dataPICP import *
import os
import data_io
import brewer2mpl

""" 
    Functions for drawing experiments results for the pIC algorithms. 
    Used to draw the figures 8, 10, 11, 12, 13 in [1]

    References
    ---------
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

def computeRatio(distances):
    """ 
        Compute the distance ratio d0/d_opt 
        
        Parameters
        ----------
        distances : array of float
                    array of SE(3) distances. 
                    First value is d0 (initial distance) and following values are the d_opt
                    (final distances after optimization with different algo)

        Returns
        -------
        optDistancesRatioPerType : array of array of ratio d0/d_opt

        Notes
        -----
        See [1], section 6.1
    """

    optDistancesRatioPerType = []
    for dist in distances[1:]:
        optDistancesRatio_curType = []
        for d0, d_opt in zip(distances[0], dist):
            optDistancesRatio_curType.append(d0/d_opt)
        optDistancesRatioPerType.append(optDistancesRatio_curType)
    return optDistancesRatioPerType

def plotDistancesICP_scatter(folder):
    """ 
        Plot the distances obtained by experiments in the form of a scatter plot
        Used to plot the Figures 11/13 in [1]

        Parameters
        ----------
        folder : str
                 folder containing the distances data to plot
    """

    distances  = data_io.loadDistancesInFolder(folder, "_chi2_0.05")
    distances2 = data_io.loadDistancesInFolder(folder, "_chi2_0.5")
    plotDistancesScatterPlot(distances, distances2)

def plotComparisonDistributionsRatioICP(folder):
    """ 
        Plot the box plots for comparing distributions of distance obtained with different pIC algorithms
        Used to plot the figures 10/12 in [1]

        Parameters
        ----------
        folder : str
                 folder containing the distances data to plot
    """

    distances = data_io.loadDistancesInFolder(folder, "_chi2_0.05")
    distances2 = data_io.loadDistancesInFolder(folder, "_chi2_0.5")

    optDistancesRatioPerType  = computeRatio(distances)
    optDistancesRatioPerType2 = computeRatio(distances2)

    plotRatioDistributions(optDistancesRatioPerType, optDistancesRatioPerType2, "/home/breux/distanceICPGraph_scan15.png")

def plotRatioDistributions(optDistancesRatioPerType, optDistancesRatioPerType2, saveFile):
    """ 
        Main function for plotting box plot to represent distributions 

        Parameters
        ----------
        optDistancesRatioPerType : array of array of ratio d0/d_opt
                                   first distribution of distance ratio

        optDistancesRatioPerType2 : array of array of ratio d0/d_opt
                                    second distribution of distance ratio
        saveFile : str
                   figure name to be saved
    """

    plt.figure(0)
    i=1
    bp_list = []
    for optDist1,optDist2 in zip(optDistancesRatioPerType, optDistancesRatioPerType2):
        bp = plt.boxplot([optDist1, optDist2], positions=[i,i+1], widths=0.75, whis=(5, 95),
                            showmeans=False, meanline=True, patch_artist=True, showfliers=False)
        bp_list.append(bp)
        i += 3

    plt.ylabel("Ratio")

    labels = ["2DpIC", "MpIC", r'$MpIC_a$', r'$MpIC_p$']
    colors = [color_2D, color_3D, color_3D_allArcsColor, color_3D_plane_startPoint]#bmap.mpl_colors
    plt.xticks([1.5, 4.5, 7.5, 10.5], labels)
    lineWidth = 4
    for bp,c in zip(bp_list, colors):
        for i in range(0,2):
            bp['boxes'][i].set_facecolor(c)
            bp['boxes'][i].set_color(c)
            plt.setp(bp['boxes'], lw=lineWidth)
            bp['whiskers'][i * 2].set_color(c)
            bp['whiskers'][i * 2 + 1].set_color(c)
            plt.setp(bp['whiskers'], lw=lineWidth, ls='--')
            plt.setp(bp['caps'], lw=lineWidth)
            bp['caps'][i * 2].set_color(c)
            bp['caps'][i * 2 + 1].set_color(c)
            plt.setp(bp['medians'], color='k', lw=lineWidth)

    plt.ylabel(r'$\frac{d_0}{d_{opt}}$', labelpad=50, rotation=0, fontsize='large')
    ax = plt.gca()
    ax.spines['top'].set_visible(False)
    ax.spines['bottom'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['left'].set_visible(False)
    ax.tick_params(axis='x', length=0)
    ax.tick_params(axis='y', length=0)
    ax.grid(axis='y', color="0.6", linestyle='-', linewidth=1)
    ax.set_axisbelow(True)
    ax.set_ylim([0., 2.25])

    plt.savefig(saveFile, bbox_inches='tight')
    plt.show()

def plotDistancesScatterPlot_(plt, distances, labels=["2DpIC", "MpIC", r'$MpIC_a$', r'$MpIC_p$'], colors=colors):
    """ 
        Draw a scatter plot of the distances 

        Parameter
        ---------
        plt : plot handle
        distances : array of array of distances
                    First array is d0 (initial distances) and following values are the d_opt
                    (final distances after optimization with different algo)
    """

    plt.xlabel(r'$d_0$', fontsize='large')
    plt.ylabel(r'$d_{opt}$', labelpad=50, rotation = 0, fontsize='large')
    ax = plt.gca()
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.tick_params(axis='x', length=5)
    ax.tick_params(axis='y', length=5)
    ax.grid(axis='x', color="0.8", linestyle='-', linewidth=1)
    ax.grid(axis='y', color="0.8", linestyle='-', linewidth=1)
    ax.set_axisbelow(True)
    ax.margins(0,0)
    #ax.yaxis.set_major_locator(MaxNLocator(prune='lower'))
    ax.xaxis.set_major_locator(plt.MaxNLocator(4))
    ax.yaxis.set_major_locator(plt.MaxNLocator(4))

    lw = 1
    alpha = 0.75
    scale  = 300.
    markers = ['o', '^', '+', '*']
    for i in range(0, len(labels)):
        plt.scatter(distances[0], distances[i+1], c=colors[i], s=scale, linewidths=lw, alpha = alpha, marker= markers[i])

    plt.autoscale()
    plt.legend(labels, labelspacing=1., loc='upper left')

def plotDistancesScatterPlot(distances, distances2):
    """ 
        Draw the scatter plot figures for two values of chi2.
        Used for Figure 11/13 in [1].

        Parameters
        ----------
        distances : array of array of distances
                    distances for the first chi2 value
                    First array is d0 (initial distances) and following values are the d_opt
                    (final distances after optimization with different algo)

        distances2 : array of array of distances
                    distances for the second chi2 value
                    First array is d0 (initial distances) and following values are the d_opt
                    (final distances after optimization with different algo)
    """

    plt.figure(0)
    plotDistancesScatterPlot_(plt, distances)

    # Draw y=x line 
    ax = plt.gca()
    plt.plot([0, np.max(ax.get_xlim()) + 0.02], [0, np.max(ax.get_xlim()) + 0.02], 'k-', alpha=0.75, zorder=0)
    plt.savefig("/home/breux/distanceScatter_scan15_chi2_0.05.png", bbox_inches='tight')
    xlim = ax.get_xlim()
    ylim = ax.get_ylim()
    print(xlim)
    print(ylim)

    plt.figure(1)
    plotDistancesScatterPlot_(plt, distances2)
    plt.xlim(xlim)
    plt.ylim(ylim)
    ax = plt.gca()

    # Draw y=x line 
    plt.plot([0, np.max(ax.get_xlim()) + 0.02], [0, np.max(ax.get_xlim()) + 0.02], 'k-', alpha=0.75, zorder=0)
    plt.savefig("/home/breux/distanceScatter_scan15_chi2_0.5.png", bbox_inches='tight')

    print(plt.gca().get_xlim())
    print(plt.gca().get_ylim())

def plotGraphs_DistanceWRTIteration(figNumber, dataPICP, labels, linestyles, linewidth, show = True, saveName = None):
    """ 
        Plot graph of distances function of the iterations (may be deprecated, old code) 
    """

    plt.figure(figNumber)

    n = len(dataPICP)
    for i in range(0, n):
        plt.plot(np.arange(len(dataPICP[i].distancesSE3)), dataPICP[i].distancesSE3, color=colors[i], label=labels[i], lw=linewidth, ls=linestyles[i])

    plt.xlabel('Iteration')
    plt.ylabel('Distance to GT')
    plt.legend()

    ax = plt.gca()
    ax.spines['top'].set_visible(False)
    ax.spines['bottom'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['left'].set_visible(False)
    ax.tick_params(axis='x', length=0)
    ax.tick_params(axis='y', length=0)
    ax.grid(axis='y', color="0.6", linestyle='-', linewidth=1)
    ax.set_axisbelow(True)

    if(saveName is not None):
        plt.savefig(saveName + ".eps")

    if (show):
        plt.show()

def plotGraphs_DistanceWRTTime(figNumber, dataPICP, colors, labels, linestyles, linewidth, alpha, show=True, saveName=None):
    """ 
        Plot graphs of distance function of the time in second (may be deprecated, old code) 
    """

    plt.figure(figNumber)

    n = len(dataPICP)
    for i in range(0, n):
        plt.plot(dataPICP[i].path["time"], dataPICP[i].distancesSE3, color=colors[i],
                 label=labels[i],  lw=linewidth, ls=linestyles[i], alpha=alpha)

    plt.title("Gaussian pICP comparison w.r.t representation for optimization")
    plt.xlabel('Time (s)')
    plt.ylabel('Distance to GT')
    plt.legend()
    if (show):
        plt.show()

    if (saveName is not None):
        plt.savefig(saveName + ".eps")

def plotGraphs_CostWRTIteration(figNumber, dataPICP, colors, labels, linestyles, linewidth, alpha, show = True, saveName = None):
    """ 
        Plot graphs of pIC cost function relatively to the iteration (may be deprecated, old code)  
    """

    plt.figure(figNumber)

    n = len(dataPICP)
    for i in range(0, n):
        plt.plot(np.arange(len(dataPICP[i].path["cost"])), dataPICP[i].path["cost"], color=colors[i], label=labels[i],  lw=linewidth, ls=linestyles[i])

    plt.title("Gaussian pICP comparison w.r.t representation for optimization")
    plt.xlabel('Iteration')
    plt.ylabel('Cost')
    plt.legend()

    if(show):
        plt.show()
    if (saveName is not None):
        plt.savefig(saveName + ".eps")

def plotGraphs_CostWRTTime(figNumber, dataPICP, colors, labels, linestyles, linewidth, alpha, show=True, saveName=None):
    """ 
        Plot graphs of pIC cost function relatively to time (may be deprecated, old code)  
    """

    plt.figure(figNumber)

    n = len(dataPICP)
    for i in range(0, n):
        plt.plot(dataPICP[i].path["time"], dataPICP[i].path["cost"], color=colors[i],
                 label=labels[i],  lw=linewidth, ls=linestyles[i])

    plt.title("Gaussian pICP comparison w.r.t representation for optimization")
    plt.xlabel('Time')
    plt.ylabel('Cost')
    plt.legend()

    if (show):
        plt.show()
    if (saveName is not None):
        plt.savefig(saveName + ".eps")

def plotBoxPlot_comparisonDistribution(folders, colors, labels, show = True, saveName = None):
    """ 
        Plot the boxplot distribution of normalized transformation distance d0/d_opt obtained using Euler, Quaternion and on-manifold optimization 

        Parameters
        ----------
        folders : array of str
                  folders in which the distances data are stored
        colors : array of (3) tuple
                 RGB tuple (value in [0,1]) corresponding to each ratio distribution color
        labels : array of str
                 label of each ratio distribution (algo name)
        show : bool
              if true, display the figure
        saveName : str
                   figure name to be saved
    """

    n = len(folders)

    optDistancesRatioPerType = []
    for folder in folders:
        print("--> Folder : {}".format(folder))
        files = os.listdir(folder)
        optDistancesRatio_curType = []
        for file in files:
            data_picp = dataPICP.createFromFile(folder + "/" + file)
            d0 = data_picp.distancesSE3[0]
            d_opt = data_picp.distancesSE3[-1]
            optDistancesRatio_curType.append(d0 / d_opt)
        optDistancesRatioPerType.append(optDistancesRatio_curType)

    bp = plt.boxplot(optDistancesRatioPerType, widths=0.5, whis=(5, 95),
                     showmeans=True, meanline=True, patch_artist=True, showfliers=False)
    plt.title("Optimized distance ratio")
    plt.ylabel("Ratio")

    plt.xticks(np.arange(1, n+1), labels)
    lineWidth = 2
    for i in range(0, n):
        bp['boxes'][i].set_facecolor(colors[i])
        plt.setp(bp['boxes'], lw=lineWidth)
        plt.setp(bp['whiskers'], lw=lineWidth)
        plt.setp(bp['caps'], lw=lineWidth)
        plt.setp(bp['medians'], color='k', lw=lineWidth)
        plt.setp(bp['means'], color='k', lw=lineWidth)

    if (show):
        plt.show()
    if (saveName is not None):
        plt.savefig(saveName + ".eps")

def plotBoxPlot_comparisonRepresentation_(file_d0, file_euler, file_quat, file_se, show = True, saveName = None):
    """ 
        Plot the boxplot distribution of normalized transformation distance d0/d_opt obtained using Euler, Quaternion and on-manifold optimization 
        (internal)

        Parameters
        ----------
        file_d0 : str
                  file containing initiale distances
        file_euler : str
                     file containing distances in case of euler optimization
        file_quat : str
                    file containing distances in case of quaternion optimization
        file_se : str
                  file containing distances in case of on-manifold optimization
        show : bool
               if true, display the figure
        saveName : str
                   figure name to be saved
    """

    bmap = brewer2mpl.get_map('Set2', 'qualitative', 7)
    colors = bmap.mpl_colors

    #Load data
    distances = []
    distances.append(np.genfromtxt(file_d0))
    distances.append(np.genfromtxt(file_euler))
    distances.append(np.genfromtxt(file_quat))
    distances.append(np.genfromtxt(file_se))
    ratio_euler = [d0/d for d0,d in zip(distances[0], distances[1])]
    ratio_quat = [d0/d for d0,d in zip(distances[0], distances[2])]
    ratio_se = [d0/d for d0,d in zip(distances[0], distances[3])]

    # Boxplots for comparing ration distributions
    #colors = ['r', 'g', 'c']
    plt.figure(0)
    bp = plt.boxplot([ratio_euler, ratio_quat, ratio_se], positions=[1, 2, 3], widths=0.5, whis=(5, 95),
                     showmeans=False, meanline=True, patch_artist=True, showfliers=False)
    plt.ylabel(r'$\frac{d_0}{d_{opt}}$', labelpad=50, rotation=0, fontsize='large')
    labels = ['Euler', 'Quat', 'se']
    plt.xticks([1, 2, 3], labels)
    lineWidth = 4
    for i in range(0, 3):
        bp['boxes'][i].set_facecolor(colors[i])
        bp['boxes'][i].set_color(colors[i])
        plt.setp(bp['boxes'], lw=lineWidth)
        bp['whiskers'][i * 2].set_color(colors[i])
        bp['whiskers'][i * 2 + 1].set_color(colors[i])
        plt.setp(bp['whiskers'], lw=lineWidth, ls='--')
        plt.setp(bp['caps'], lw=lineWidth)
        bp['caps'][i * 2].set_color(colors[i])
        bp['caps'][i * 2 + 1].set_color(colors[i])
        plt.setp(bp['medians'], color='k', lw=lineWidth)
        #plt.setp(bp['means'], color='k', lw=2.*lineWidth)

    ax = plt.gca()
    ax.spines['top'].set_visible(False)
    ax.spines['bottom'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['left'].set_visible(False)
    ax.tick_params(axis='x', length=0)
    ax.tick_params(axis='y', length=0)
    ax.grid(axis='y', color="0.6", linestyle='-', linewidth=1)
    ax.set_axisbelow(True)
    #plt.grid(True, axis='y')

    if(saveName is not None):
        plt.savefig(saveName + ".eps", bbox_inches='tight')
    if (show):
        plt.show()

    # Scatter plot d_opt function of d_0
    plt.figure(1)
    plotDistancesScatterPlot_(plt, distances, labels=labels, colors = colors)
    # Draw y=x line
    ax = plt.gca()
    plt.plot([0, np.max(ax.get_xlim()) + 0.02], [0, np.max(ax.get_xlim()) + 0.02], 'k-', alpha=0.75, zorder=0)
    if(saveName is not None):
        plt.savefig(saveName + "_scatterplot.eps", bbox_inches='tight')
    if(show):
        plt.show()

def plotGraphs_sonarPICP_simulation(file_gaussian, files_mypicp, colors, labels):
    """ 
        Plot different graphs from pIC experiments (may be deprecated, old code) 
    """

    linestyles = ['-']
    linewidth = 5
    alpha = 0.6

    data = []
    data.append(dataPICP.createFromFile(file_gaussian))
    for file in files_mypicp:
        data.append(dataPICP.createFromFile(file))
        linestyles.append('--')

    plotGraphs_DistanceWRTTime(0, data, colors, labels, linestyles, linewidth, alpha, show=False)
    plt.title("Comparison of accuracy for Gaussian approx and Beta distribution")
    plt.xlabel("Time (s)")
    plt.ylabel("Error (SE3 distance)")
    plt.legend()

    plotGraphs_DistanceWRTIteration(1, data, colors, labels, linestyles, linewidth, alpha, show=False)
    plt.title("Comparison of accuracy for Gaussian approx and Beta distribution")
    plt.xlabel("Iteration")
    plt.ylabel("Error (SE3 distance)")
    plt.legend()

    plotGraphs_CostWRTTime(2, data, colors, labels, linestyles, linewidth, alpha, show=False)
    plt.title("Cost vs time")
    plt.xlabel("Time (s)")
    plt.ylabel("Cost")
    plt.legend()

    plotGraphs_CostWRTIteration(3, data, colors, labels, linestyles, linewidth, alpha, show=False)
    plt.title("Cost vs time")
    plt.xlabel("Iteration")
    plt.ylabel("Cost")
    plt.legend()

    plt.show()