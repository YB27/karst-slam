import matplotlib.pyplot as plt
import numpy as np
import sys

"""
    Functions used to generate the box/violin results plot (Figure 18) in [1]

    References
    ----------
    .. [1] Breux, Yohan, and Lionel Lapierre. 
           "Elevation Angle Estimations of Wide-Beam Acoustic Sonar Measurements 
            for Autonomous Underwater Karst Exploration." Sensors 20.14 (2020): 4028.
"""

 # Plot parameters 
params = {
        'axes.labelsize': 15,
        'axes.labelweight' : 'bold',
        'font.size': 20,
        'font.weight' : 'bold',
        'legend.fontsize': 10,
        'xtick.labelsize': 15,
        'ytick.labelsize': 15,
        'text.usetex': False,
        'figure.figsize': [9, 7]
    }

plt.rcParams.update(params)

# List of files 
type = sys.argv[1]
folder = "/home/breux/Exp/" + type + "/"
Trial_folder = str(sys.argv[2])
kernels = ["exponential", "matern32", "matern52", "rbf"]
plotType = sys.argv[3] # box or violin

data_list = []
for kernel in kernels:
    cur_list = []
    with open(folder + kernel + "/" + Trial_folder + "/scoreResults_prior.txt") as f:
        data = np.loadtxt(f, dtype='float')
        cur_list.append(np.ravel(data))
    with open(folder + kernel +  "/" + Trial_folder + "/scoreResults_posterior.txt") as f:
        data = np.loadtxt(f, dtype='float')
        cur_list.append(np.ravel(data))
    data_list.append(cur_list)

# Get max value
max_list = []
for list in data_list:
    for data in list:
        max_list.append(np.amax(data))
print(max_list)
max_val = np.amax(np.array(max_list))

# Normalize 
#data_list = [data/max_val for data in data_list]

plt.figure(1)
color_prior = 'lightblue'
colors = ['g', 'r', 'c', 'm']
# Box plot or violin 
if(plotType == "box"):
    i = 1
    bp_list = []
    for data in data_list:
        bp = plt.boxplot(data,positions=[i,i+1], widths = 0.5, showmeans=True, meanline=True,patch_artist=True)
        bp_list.append(bp)
        #setBoxColors(bp)
        i += 3

    # Box plot style 
    for bplot,c in zip(bp_list, colors):
        #plt.setp(bplot['boxes'], lw=2, facecolor=c)
        bplot['boxes'][0].set_facecolor(color_prior)
        bplot['boxes'][1].set_facecolor(c)
        plt.setp(bplot['boxes'], lw=2)
        plt.setp(bplot['whiskers'], lw=2)
        plt.setp(bplot['caps'], lw=2)
        plt.setp(bplot['fliers'][0], markersize = 5,markeredgewidth=2, markerfacecolor=color_prior)
        plt.setp(bplot['fliers'][1], markersize = 5, markeredgewidth=2, markerfacecolor=c)
        plt.setp(bplot['medians'], color='k', lw=2)
        plt.setp(bplot['means'], color='k', lw=2)
#elif(plotType == "violin"):
    i = 1
    vp_list = []
    for data in data_list:
        vp = plt.violinplot(data, positions=[i, i + 1], widths=1, showmeans=True, showmedians=True,showextrema=False)
        vp_list.append(vp)
        i += 3

    ''' Violin plot style '''
    for vplot, c in zip(vp_list, colors):
        # plt.setp(bplot['boxes'], lw=2, facecolor=c)
        vplot['bodies'][0].set_facecolor(color_prior)
        vplot['bodies'][1].set_facecolor(c)
        vplot['bodies'][0].set_alpha(0.5)
        vplot['bodies'][1].set_alpha(0.5)
        vplot['bodies'][0].set_edgecolor(color_prior)
        vplot['bodies'][1].set_edgecolor(c)
        vplot['bodies'][0].set_linewidth(2)
        vplot['bodies'][1].set_linewidth(2)

# Display means and medians values
medians = []
means = []
for data in data_list:
    for d in data:
        medians.append(np.percentile(d,50))
        means.append(np.mean(d))
print("Medians : ")
print(medians)
print("Means : ")
print(means)

plt.ylim(bottom=0)
plt.xticks([1.5,4.5,7.5,10.5],["Exp","Mat32", "Mat52", "Rbf"])
plt.grid(True, axis='y')
plt.ylabel("Error expectation (m)")
plt.savefig("/home/breux/Exp/" + type + "_" + Trial_folder + ".png", bbox_inches='tight')
plt.show()

# Bar histogram for number of valid arcs (relative to exponential)
folder = "/home/breux/Exp/"
type_meas = ["Sparse", "Dense", "Noisy"]
nArcs = []
for kernel in kernels:
    curList = []
    for t in type_meas:
        with open(folder + "/" + t + "/" + kernel + "/" + Trial_folder + "/prop_arcs.txt") as f:
            data = np.loadtxt(f, dtype='float')
            curList.append(data.item())
    nArcs.append(curList)

barwidth = 0.1
r1 = np.arange(3)
r2 = [x + barwidth for x in r1]
r3 = [x + barwidth for x in r2]
r4 = [x + barwidth for x in r3]

plt.figure(2)
edgeColor = 'black'
edgeWidth = 2
plt.bar(r1, nArcs[0], color='g', width = barwidth,edgecolor=edgeColor, linewidth=edgeWidth, label='Exp')
plt.bar(r2, nArcs[1], color='r', width = barwidth,edgecolor=edgeColor, linewidth=edgeWidth, label='Mat32')
plt.bar(r3, nArcs[2], color='c', width = barwidth,edgecolor=edgeColor, linewidth=edgeWidth, label='Mat52')
plt.bar(r4, nArcs[3], color='m', width = barwidth,edgecolor=edgeColor, linewidth=edgeWidth, label='Rbf')
plt.xticks([r + 1.5*barwidth for r in range(3)], type_meas)

plt.ylabel("# Non uniform distributions")
plt.legend()
plt.savefig("/home/breux/Exp/histogram.eps", bbox_inches='tight')