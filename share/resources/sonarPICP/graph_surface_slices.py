import matplotlib
import matplotlib.pyplot as plt
#matplotlib.use('Cairo')
import numpy as np
import sys
import csv
import os

""" 
    Functions used to generate the gaussian process slices (Figure 13/14) in [1]

    References
    ----------
    .. [1] Breux, Yohan, and Lionel Lapierre. 
           "Elevation Angle Estimations of Wide-Beam Acoustic Sonar Measurements 
            for Autonomous Underwater Karst Exploration." Sensors 20.14 (2020): 4028.
"""

script_dir = os.path.dirname(os.path.realpath('__file__'))

# Plot parameters
params = {
        'axes.labelsize': 15,
        'axes.labelweight': 'bold',
        'font.size': 20,
        'legend.fontsize': 15,
        'xtick.labelsize': 25,
        'ytick.labelsize': 25,
        'text.usetex': False,
        'figure.figsize': [9, 7]
    }

plt.rcParams.update(params)

# Type of the experiment (dense, sparse) 
type = sys.argv[1]
kernel_fixed = sys.argv[2]
kernel_fixed_name = sys.argv[3]
plane = sys.argv[4]
if(kernel_fixed == "yaw"):
    fixedParamValueOrFree = sys.argv[5]
if(kernel_fixed == "s"):
    kernel_yaw_type = sys.argv[5]
    if (kernel_yaw_type == "exp"):
        kernel_yaw = ["exponentialChordal_100",
                      "exponentialChordal_50",
                      "exponentialChordal_25"]
    else:
        kernel_yaw = ["matern52Chordal_100",
                      "matern52Chordal_50",
                      "matern52Chordal_25"]

kernel_s = ["exponential", "matern32", "matern52", "rbf"]

if(plane == "XY"):
    x_label = "X"
    y_label = "Y"
elif(plane == "XZ"):
    x_label = "X"
    y_label = "Z"
elif(plane == "ZY"):
    x_label = "Y"
    y_label = "Z"
else:
    print("Unknown plane slice !")
title = plane + " slice"

# Load the corresponding slices files 
sliceFiles = []
fileSuffix = plane + "slice"
if(kernel_fixed == "yaw"):
    ''' One list for each kernel_s (exponential, ...) '''
    color_list = ['g', 'r', 'c', 'm']
    label_list = ['exp', 'mat32', 'mat52', 'rbf']
    for kernel in kernel_s:
        folder = script_dir + "/SlicesData/" + kernel + "_" + kernel_fixed_name
        sliceFiles.append(folder+ "/" + fileSuffix + "_" + type + "_" + fixedParamValueOrFree + ".txt")
elif (kernel_fixed == "s"):
    ''' Two kernels + for each use three lengthscale : exp + l=0.25/0.5/1', mat52Chordal + l =... '''
    color_list = ['g', 'r', 'c'] #ToDo : choose colors
    if(kernel_yaw_type == "exp"):
        label_list = ['exp,l=1', 'exp,l=0.5', 'exp,l=0.25']
    else:
        label_list = ['mat52,l=1', 'mat52,l=0.5', 'mat52,l=0.25']
    for kernel in kernel_yaw:
        folder = script_dir + "/SlicesData/" + kernel_fixed_name + "_" + kernel
        sliceFiles.append(folder+ "/" + fileSuffix + "_" + type + ".txt")
else:
    print("Unknown kernel fixed !")

# Model file 
modelFile = script_dir + "/SlicesData/Model/" + plane + "_modelSlice_" + type + ".txt"

# Load Model data
model_abscisse = []
model_ordinate = []
with open(modelFile) as f:
    model_data = csv.reader(f, delimiter=',')
    for row in model_data:
        if(plane == "ZY" or (plane != "ZY" and float(row[1]) > 0)):
            model_abscisse.append(float(row[0]))
            model_ordinate.append(float(row[1]))

# Load data 
if(plane != "ZY"):
    abscisse_data = []
else:
    abscisse_data = {'mean' : [], 'low' : [], 'high' : []}
ordinate_data = []
firstParse = True
for file in sliceFiles:
    with open(file) as f:
        ordinate_data_list = {'mean': [], 'low': [], 'high': []}
        data = csv.reader(f, delimiter=',')
        for row in data:
            x = float(row[0])
            yaw = float(row[1])
            mean_rho = float(row[2])
            low_rho = float(row[3])
            high_rho = float(row[4])

            cos_yaw = np.cos(yaw)
            sin_yaw = np.sin(yaw)

            if(plane == "ZY" or (plane != "ZY" and yaw > 0)):
                if(firstParse):
                    if(plane != "ZY"):
                        abscisse_data.append(x)
                    else:
                        abscisse_data["mean"].append(sin_yaw*mean_rho)
                        abscisse_data["low"].append(sin_yaw*low_rho)
                        abscisse_data["high"].append(sin_yaw*high_rho)

                if(plane == "XY"):
                    ordinate_data_list["mean"].append(sin_yaw*float(row[2]))
                    ordinate_data_list["low"].append(sin_yaw*float(row[3]))
                    ordinate_data_list["high"].append(sin_yaw*float(row[4]))
                else:
                    ordinate_data_list["mean"].append(-cos_yaw * float(row[2]))
                    ordinate_data_list["low"].append(-cos_yaw * float(row[3]))
                    ordinate_data_list["high"].append(-cos_yaw * float(row[4]))

    ordinate_data.append(ordinate_data_list)
    firstParse = False

# Plot ! 
plt.figure(0)
plt.xlabel(x_label)
plt.ylabel(y_label)

# Set the range when fixing K_s and plotting exp/mat52Chordal in two different plots 
if(kernel_fixed == "s"):
    if(type == "dense"):
        if(plane == "XY"):
            plt.ylim(16,21)
        elif(plane == "XZ"):
            plt.ylim(16,23)
        elif(plane == "ZY"):
            plt.ylim(-20,21)
    else:
        if(plane == "XY"):
            plt.ylim(0.4,1.8)
        elif(plane == "XZ"):
            plt.ylim(0.1,1.8)
        elif(plane == "ZY"):
            plt.xlim(-1.5,1.6)
            plt.ylim(-1.4,1.0)

if(plane != "ZY"):
    for ordinate,color,label in zip(ordinate_data,color_list,label_list):
        plt.fill_between(abscisse_data, ordinate['low'], ordinate['high'], color=color, alpha=0.5)
        plt.plot(abscisse_data, ordinate['low'],color=color,lw=3,ls='--', alpha=1)
        plt.plot(abscisse_data, ordinate['high'],color=color,lw=3,ls='--',label=label, alpha=1)
else:
    for ordinate,color,label in zip(ordinate_data,color_list,label_list):
        plt.fill(np.append(abscisse_data['low'],abscisse_data['high'][::-1]),
                 np.append(ordinate['low'],ordinate['high'][::-1]), color=color,alpha=0.5)
        plt.plot(abscisse_data['low'], ordinate['low'],color=color,lw=3,ls='--', alpha=1)
        plt.plot(abscisse_data['high'], ordinate['high'],color=color,lw=3,ls='--',label=label, alpha=1)

# Model GT data 
plt.plot(model_abscisse, model_ordinate, color='k',label="GT", lw=5,ls='-',markersize=15)
plt.legend(loc='lower right')

if(kernel_fixed == "s"):
    fileName = script_dir + "/Graphs/" + plane + "_" + type + "_" + kernel_fixed + "_" + kernel_yaw_type + ".pdf"
else:
    fileName = script_dir + "/Graphs/" + plane + "_" + type + "_" + kernel_fixed + "_" + fixedParamValueOrFree +".pdf"

plt.savefig(fileName,bbox_inches='tight')